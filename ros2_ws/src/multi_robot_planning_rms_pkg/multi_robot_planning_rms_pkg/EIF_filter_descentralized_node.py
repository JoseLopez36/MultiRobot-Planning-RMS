#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from threading import Lock, Event
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped , Point
from nav_msgs.msg import Odometry

# Importar el modelos
from .EIF_models import g_function, G_jacobian, h_function_n, H_jacobian_n, R_noise_model, Q_noise_model_n

# Importar mensajes de multi_robot_planning_rms_msgs
from gz_uwb_beacon_msgs.msg import EIFInput, EIFOutput
from multi_robot_planning_rms_msgs.msg import ProcessStats

class EIFFilterDescentralizedNode(Node):
    def __init__(self):
        super().__init__('EIF_filter_descentralized_node', automatically_declare_parameters_from_overrides=True, allow_undeclared_parameters=True)
        self.get_logger().info("Iniciando nodo de filtro EIF...")
        params = self._parameters  # Diccionario interno de parámetros
        self.get_logger().debug(f"Parámetros disponibles: {list(params.keys())}")

        # Obtener parámetros del archivo de configuración
        # Parametros del filtro
        self.filter_update_rate = self.get_parameter('filter_update_rate').value
        # Parametros necesarios para los modelos
        self.horizontal_vel = self.get_parameter('horizontal_vel').value 
        self.vertical_vel = self.get_parameter('vertical_vel').value
        self.initial_covariance = self.get_parameter('initial_covariance').value
        self.valid_time_threshold = self.get_parameter('valid_measurement_threshold').value 
        self.beacons_ids = self.get_parameter('beacons.ids').value 
        self.num_beacons = len(self.beacons_ids)
        self.beacons = {}
        for beacon_id in self.beacons_ids:
            position = self.get_parameter(f'beacons.{beacon_id}.position').value
            noise_std = self.get_parameter(f'beacons.{beacon_id}.noise_std').value
            self.beacons[beacon_id] = {"position" : position, "noise_std" : noise_std}                             

        # Modelo de predicción y medición:
        self.g, self.h_n, self.G, self.H_n = g_function, h_function_n, G_jacobian, H_jacobian_n

        # Matrices de ruido:
        self.Q_n = Q_noise_model_n                                                                                     # Ruido de medición
        self.R = R_noise_model(self.horizontal_vel, self.vertical_vel, 1.0 / self.filter_update_rate)                  # Ruido de proceso

        # Variables para la creencia de la localizacion en forma canónica
        self.omega = (1.0/self.initial_covariance)*np.eye(3, dtype=np.float64)                           # Matriz de información
        self.xi    = np.array([[0],[0],[0]],dtype=np.float64)                   # Vector de información 
        self.mu    = np.array([[0],[0],[0]],dtype=np.float64) 
        self.ground_truth  = Point()               # vector media del estado estimado
        self.covariance = self.initial_covariance*np.eye(3, dtype=np.float64)                  

        # Variables de resultado de predicción
        self.omega_pred = np.eye(3, dtype=np.float64)                           
        self.xi_pred    = np.array([[0],[0],[0]],dtype=np.float64)                   
        self.mu_pred    = np.array([[0],[0],[0]],dtype=np.float64)                     

        # Variables de suma para actualización
        self.omega_sum = np.zeros([3,3],dtype=np.float64)                           
        self.xi_sum    = np.array([[0],[0],[0]],dtype=np.float64)                      

        # Subscriptores y publicadores
        self.predict_pub = self.create_publisher(PoseWithCovarianceStamped,f"/{self.get_name()}/predicted_position", self.num_beacons)     
        self.stat_pub = self.create_publisher(ProcessStats,f"/{self.get_name()}/process_stats",10)

        self.truth_sub = self.create_subscription(Odometry,f"/ground_truth/vehicle_odom",self.ground_truth_callback, 30) 
        #if self.beacon_id != "":
        self.beacon_innovation_map = {}
        for beacon_id in self.beacons_ids:
            self.create_subscription(EIFOutput,f"/uwb_beacon/{beacon_id}/eif_output", self.partial_innovation_callback, 10)
            # Initialize with empty dict - innovations will be added when received
        self.eif_output_pub =  self.create_publisher(EIFInput,f"/uwb_beacon/eif_input",10)
            
        # Temporizador para la frecuencia de actualización
        self.timer = self.create_timer(1.0 / self.filter_update_rate, self.estimate_localization)

        self.get_logger().info("Nodo de filtro EIF descentralizado iniciado")

    def partial_innovation_callback(self, beacon_output_msg):
            beacon_id = beacon_output_msg.beacon_id
            xi_n = beacon_output_msg.xi
            omega_n = beacon_output_msg.omega
            current_time = self.get_clock().now()
            # Store innovation with timestamp
            self.beacon_innovation_map[beacon_id] = {
                'xi': xi_n,
                'omega': omega_n,
                'timestamp': current_time
            }
            
    def publish_eif_input(self, mu, mu_pred):
        self.get_logger().info("Enviando mensaje de input")
        input_msg = EIFInput() 
        input_msg.header.stamp = self.get_clock().now().to_msg()
        input_msg.mu = mu.flatten().tolist()
        input_msg.mu_predicted = mu_pred.flatten().tolist()

        self.eif_output_pub.publish(input_msg)

    def clean_expired_innovations(self):
        """Remove innovations that have exceeded the valid time threshold"""
        current_time = self.get_clock().now()
        expired_beacons = []
        
        for beacon_id, innovation_data in self.beacon_innovation_map.items():
            if isinstance(innovation_data, dict) and 'timestamp' in innovation_data:
                time_diff = (current_time - innovation_data['timestamp']).nanoseconds / 1e9
                if time_diff > self.valid_time_threshold:
                    expired_beacons.append(beacon_id)
        
        # Remove expired innovations
        for beacon_id in expired_beacons:
            self.get_logger().info(f"Removiendo innovación expirada del beacon {beacon_id} (timeout: {self.valid_time_threshold}s)")
            del self.beacon_innovation_map[beacon_id]
            
        if len(expired_beacons) > 0:
            self.get_logger().info(f"Se eliminaron {len(expired_beacons)} innovaciones expiradas")

    def estimate_localization(self):
        self.get_logger().info("Estimando localización...")
        start_filter = self.get_clock().now()
        
        # Clean expired innovations first
        self.clean_expired_innovations()
        
        start = self.get_clock().now()
        self.predict()
        predic_time = (self.get_clock().now() - start).nanoseconds / 1e9

        # Mandar infomación a las valizas para que puedan realizar los calculos
        start =  self.get_clock().now()
        self.publish_eif_input(self.mu, self.mu_pred)

        # Check if we have valid beacon data (innovations with proper structure)
        valid_beacons = {}
        for beacon_id, innovation_data in self.beacon_innovation_map.items():
            if isinstance(innovation_data, dict) and 'xi' in innovation_data and 'omega' in innovation_data:
                valid_beacons[beacon_id] = innovation_data
        
        self.get_logger().info(f"Beacons válidos disponibles: {len(valid_beacons)}/{len(self.beacons_ids)}")
        
        if len(valid_beacons) == 0:
            self.get_logger().warning("No hay medidas válidas disponibles, no es posible actualizar predicción")
            # Use prediction as the current estimate
            self.omega = self.omega_pred
            self.xi = self.xi_pred
        else:   
            xi, omega = self.update(valid_beacons)
        update_time = (self.get_clock().now() - start).nanoseconds / 1e9

        try:
            self.covariance = np.linalg.inv(self.omega)
        except np.linalg.LinAlgError:
            self.get_logger().warning("Singular matrix, filter cannot calculate covariance")
        
        
        self.publish_estimation(self.xi, self.covariance) # Publicar estimación de localización
        filter_time = (self.get_clock().now() - start_filter).nanoseconds / 1e9

        self.publish_stat(predic_time,update_time,filter_time,len(valid_beacons),self.omega, self.xi, self.mu,self.ground_truth)

        return self.mu, self.omega, self.xi

    def predict(self):
        try:
            # Parte de predicción del filtro EIF
            # Calculo de la media de la estimación en t-1:
            self.mu = np.linalg.inv(self.omega) @ self.xi

            # Calculo prediciones de omega, xi y mu:
            G = self.G(self.mu) # Jacobiano de la función de predicción
            self.omega_pred = np.linalg.inv( G @ np.linalg.inv(self.omega) @ np.transpose(G) + self.R )     #Matriz de información predicha    
            self.mu_pred = self.g(self.mu)
            self.xi_pred = self.omega_pred @ self.mu_pred                                           #Vector de información predicho
        except np.linalg.LinAlgError:
            self.get_logger().warning("Singular matrix, filter cannot predict")
        return self.xi_pred, self.omega_pred, self.mu_pred

    def update(self, innovation_map):
        # Sumatorios de la actualización de matriz y vector de información con los calculos recibidos
        if len(innovation_map) == 0:
            # No valid measurements, return prediction values
            return self.xi_pred, self.omega_pred
            
        # Extract xi and omega values from valid beacon data with new structure
        xi_calulations = np.array([data['xi'] for data in innovation_map.values()])
        omega_calculations = np.array([data['omega'] for data in innovation_map.values()])
        
        if len(xi_calulations) > 0:
            self.xi_sum = np.sum(xi_calulations,axis=0).reshape((3,1))
            self.omega_sum = np.sum(omega_calculations,axis=0).reshape((3,3))
        else:
            self.xi_sum = np.zeros((3,1), dtype=np.float64)
            self.omega_sum = np.zeros((3,3), dtype=np.float64)

        # Actualizar la creencia de la localización
        self.omega = self.omega_pred + self.omega_sum
        self.xi = self.xi_pred + self.xi_sum

        # Resetear sumas
        self.xi_sum.fill(0)
        self.omega_sum.fill(0)                         

        return self.xi, self.omega

    def publish_estimation(self, xi, covariance):
        # Publicar la estimación de localización
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "global"

        print(covariance)
        mu = covariance @ xi

        pose_msg.pose.pose.position.x = mu[0][0]
        pose_msg.pose.pose.position.y = mu[1][0]
        pose_msg.pose.pose.position.z = mu[2][0]

        full_covariance = np.zeros((6,6))
        full_covariance[:3,:3] = covariance

        pose_msg.pose.covariance = full_covariance.flatten().tolist()

        self.predict_pub.publish(pose_msg)

    def ground_truth_callback(self,odom_msg):
        self.ground_truth = odom_msg.pose.pose.position


    def publish_stat(self,predict_time, update_time, filter_time, number_beacons, omega, xi, mu, gt):
        s = ProcessStats()
        s.header.stamp = self.get_clock().now().to_msg()

        #Prediccion:
        s.predicted_position.x = mu[0][0]
        s.predicted_position.y = mu[1][0]
        s.predicted_position.z = mu[2][0]

        #Ground_truth
        s.ground_truth.x = gt.x
        s.ground_truth.y = gt.y
        s.ground_truth.z = gt.z

        #Tiempos de ejecucion
        s.predict_time = predict_time
        s.update_time = update_time
        s.filter_time = filter_time

        # Numero de medidas/calculos recibidos
        s.measurements_received = number_beacons

        # Informacion:
        s.omega = omega.flatten().tolist()
        s.xi = xi.flatten().tolist()

        self.stat_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    eif_filter_descentralized_node = EIFFilterDescentralizedNode()
    rclpy.spin(eif_filter_descentralized_node)
    eif_filter_descentralized_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()