#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from threading import Lock
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped 
from nav_msgs.msg import Odometry

# Importar el modelos
from .EIF_models import g_function, G_jacobian, h_function_n, H_jacobian_n, R_noise_model, Q_noise_model_n

# Importar mensajes de multi_robot_planning_rms_msgs
from gz_uwb_beacon_msgs.msg import Measurement
from multi_robot_planning_rms_msgs.msg import ProcessStats

class EIFFilterNode(Node):
    def __init__(self):
        super().__init__('EIF_filter_node', automatically_declare_parameters_from_overrides=True, allow_undeclared_parameters=True)
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
        self.valid_measurement_threshold = self.get_parameter('valid_measurement_threshold').value                     # Unidades: ns ! Default 10 ms 
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
        self.omega = (1.0/self.initial_covariance)*np.eye(3, dtype=np.float64)                        # Matriz de información
        self.xi    = np.array([[0],[0],[0]],dtype=np.float64)                                       # Vector de información 
        self.mu    = np.array([[0],[0],[0]],dtype=np.float64)                                       # vector media del estado estimado
        self.covariance = self.initial_covariance*np.eye(3, dtype=np.float64)                       # Matriz de covarianza, calculada para visualización y calculo de mu

        # Variables de resultado de predicción
        self.omega_pred = np.eye(3, dtype=np.float64)                           
        self.xi_pred    = np.array([[0],[0],[0]],dtype=np.float64)                   
        self.mu_pred    = np.array([[0],[0],[0]],dtype=np.float64)                     

        # Variables de suma para actualización
        self.omega_sum = np.zeros([3,3],dtype=np.float64)                           
        self.xi_sum    = np.array([[0],[0],[0]],dtype=np.float64)                      
    
        # Variables para la gestion de mediciones
        self.lock = Lock() # Para proteger el acceso a las mediciones
        self.last_measurements = [[beacon_id, None, None] for beacon_id in self.beacons_ids] # Lista de medidas de balizas [id, distancia, timestamp]

        # Subscriptores y publicadores
        self.predict_pub = self.create_publisher(PoseWithCovarianceStamped,f"/{self.get_name()}/predicted_position", self.num_beacons)   
        self.stat_pub = self.create_publisher(ProcessStats,f"/{self.get_name()}/process_stats",10)

        self.truth_sub = self.create_subscription(Odometry,f"/ground_truth/vehicle_odom",self.ground_truth_callback, 30) 
        #if self.beacon_id != "":
        for beacon_id in self.beacons_ids:
            self.create_subscription(Measurement, f'/uwb_beacon/{beacon_id}/measurement', self.beacon_measurements_callback, 10)
            
        # Temporizador para la frecuencia de actualización
        self.timer = self.create_timer(1.0 / self.filter_update_rate, self.estimate_localization)

        self.get_logger().info("Nodo de filtro EIF iniciado")

    def beacon_measurements_callback(self, beacon_msg):
        beacon_id = beacon_msg.beacon_id
        beacon_distance = beacon_msg.distance
        beacon_timestamp = Time.from_msg(beacon_msg.header.stamp).nanoseconds

        # Guardar la última medida de la baliza
        with self.lock:
            self.last_measurements[beacon_id][1] = beacon_distance
            self.last_measurements[beacon_id][2] = beacon_timestamp

        self.get_logger().debug(f"Medida de baliza {beacon_id}: distancia = {beacon_distance}, timestamp = {beacon_timestamp}")        

    def estimate_localization(self):
        self.get_logger().info("Estimando localización...")
        start_filter = self.get_clock().now()
        start = self.get_clock().now()
        self.predict()
        predic_time = (self.get_clock().now() - start).nanoseconds / 1e9

        start =  self.get_clock().now()
        now = start.nanoseconds        # Mismo tipo de timestamp que el mensaje de la baliza
        z = []
        with self.lock:
            for i in range(len(self.last_measurements)):
                if self.last_measurements[i][2] is not None:
                    if now - self.last_measurements[i][2] < self.valid_measurement_threshold:
                        z.append(self.last_measurements[i][:2]) # Filtrar medidas válidas ()
        
        if len(z) == 0:
            self.get_logger().warning("No hay medidas válidas disponibles, no es posible actualizar predicción")
        else:   
            xi, omega = self.update(z)
        update_time = (self.get_clock().now() - start).nanoseconds / 1e9
        
        try:
            self.covariance = np.linalg.inv(self.omega)
        except np.linalg.LinAlgError:
            self.get_logger().warning("Singular matrix, filter cannot calculate covariance")
        
        
        with self.lock:
            self.publish_estimation(self.xi, self.covariance) # Publicar estimación de localización
        filter_time = (self.get_clock().now() - start_filter).nanoseconds / 1e9

        with self.lock:
            self.publish_stat(predic_time,update_time,filter_time,len(z),self.omega, self.xi, self.mu,self.ground_truth)

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

    def update(self, z):
        try:
            # Sumatorios de la actualización de matriz y vector de información segun el número de medidas disponibles
            for i in range(len(z)):
                beacon_id = z[i][0]        # id != i
                H = self.H_n(self.mu, self.beacons[beacon_id]['position']) 
                Q = self.Q_n(self.beacons[beacon_id]['noise_std'])                  # Ruido de medición estandar de cada baliza
                y = z[i][1] - self.h_n(self.mu, self.beacons[beacon_id]['position']) + H @ self.mu_pred #Innovación
                
                self.omega_sum = self.omega_sum + np.transpose(H) @ np.linalg.inv(Q) @ H
                self.xi_sum = self.xi_sum + np.transpose(H) @ np.linalg.inv(Q) @ y
            
            # Actualizar la creencia de la localización
            self.omega = self.omega_pred + self.omega_sum
            self.xi = self.xi_pred + self.xi_sum

        except np.linalg.LinAlgError:
            self.get_logger().warning("Singular matrix, filter cannot update")
            return
    
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
        with self.lock:
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
    eif_filter_node = EIFFilterNode()
    rclpy.spin(eif_filter_node)
    eif_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()