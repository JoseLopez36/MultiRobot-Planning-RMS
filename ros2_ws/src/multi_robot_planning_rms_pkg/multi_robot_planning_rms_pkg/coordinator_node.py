import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from multi_robot_planning_rms_msgs.srv import DarpPetition
from multi_robot_planning_rms_msgs.msg import Trajectory2D
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleCommand, VehicleStatus
from vision_msgs.msg import Point2D
import math
import numpy as np

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')

        # Parametros del nodo coordinador
        self.declare_parameter('uav_ids', ['px4_1'])
        self.declare_parameter('target_altitude', 5.0)
        self.declare_parameter('acceptance_radius', 0.5)
        self.declare_parameter('grid_origin_lat', 0.0) # Not used yet, assuming local frame
        self.declare_parameter('grid_origin_lon', 0.0)

        # Parametros de la peticion DARP
        self.declare_parameter('min_x', 0)
        self.declare_parameter('max_x', 20)
        self.declare_parameter('min_y', 0)
        self.declare_parameter('max_y', 20)
        self.declare_parameter('obstacle_points_x', [0.0])
        self.declare_parameter('obstacle_points_y', [0.0])
        self.declare_parameter('initial_positions_x', [0.0])
        self.declare_parameter('initial_positions_y', [0.0])

        self.uav_ids = self.get_parameter('uav_ids').get_parameter_value().string_array_value
        self.target_altitude = self.get_parameter('target_altitude').get_parameter_value().double_value
        self.acceptance_radius = self.get_parameter('acceptance_radius').get_parameter_value().double_value
        
        # Cliente DARP
        self.darp_client = self.create_client(DarpPetition, 'darp_service')
        while not self.darp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio DARP no disponible, esperando de nuevo...')
        
        # QoS para PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Gestion de estado de los UAVs
        self.uav_publishers_offboard = {}
        self.uav_publishers_trajectory = {}
        self.uav_publishers_command = {}
        self.uav_subscribers_position = {}
        self.uav_subscribers_status = {}
        self.uav_positions = {}
        self.uav_status = {}
        self.uav_trajectories = {} # Key: uav_id, Value: List of Point2D
        self.uav_current_wp_index = {} # Key: uav_id, Value: int
        self.uav_setpoint_counts = {} # Key: uav_id, Value: int
        
        # self.offboard_setpoint_counter = 0 # Ya no usamos contador global

        for uav_id in self.uav_ids:
            # Publicadores
            self.uav_publishers_offboard[uav_id] = self.create_publisher(
                OffboardControlMode,
                f'/{uav_id}/fmu/in/offboard_control_mode',
                qos_profile
            )
            self.uav_publishers_trajectory[uav_id] = self.create_publisher(
                TrajectorySetpoint,
                f'/{uav_id}/fmu/in/trajectory_setpoint',
                qos_profile
            )
            self.uav_publishers_command[uav_id] = self.create_publisher(
                VehicleCommand,
                f'/{uav_id}/fmu/in/vehicle_command',
                qos_profile
            )
            
            # Suscriptores
            self.uav_subscribers_position[uav_id] = self.create_subscription(
                VehicleLocalPosition,
                f'/{uav_id}/fmu/out/vehicle_local_position',
                lambda msg, uid=uav_id: self.position_callback(msg, uid),
                qos_profile
            )
            self.uav_subscribers_status[uav_id] = self.create_subscription(
                VehicleStatus,
                f'/{uav_id}/fmu/out/vehicle_status',
                lambda msg, uid=uav_id: self.status_callback(msg, uid),
                qos_profile
            )
            
            self.uav_positions[uav_id] = None
            self.uav_status[uav_id] = None
            self.uav_trajectories[uav_id] = []
            self.uav_current_wp_index[uav_id] = 0
            self.uav_setpoint_counts[uav_id] = 0

        # Timer para el bucle de control (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Peticion de solucion DARP
        self.request_darp()

    def position_callback(self, msg, uav_id):
        self.uav_positions[uav_id] = msg

    def status_callback(self, msg, uav_id):
        self.uav_status[uav_id] = msg

    def publish_vehicle_command(self, uav_id, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        if uav_id in self.uav_publishers_command:
            self.uav_publishers_command[uav_id].publish(msg)

    def arm(self, uav_id):
        self.publish_vehicle_command(uav_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f"Enviando comando ARM a {uav_id}")

    def engage_offboard_mode(self, uav_id):
        self.publish_vehicle_command(uav_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(f"Enviando comando OFFBOARD a {uav_id}")

    def request_darp(self):
        self.get_logger().info("Peticion de algoritmo DARP...")
        req = DarpPetition.Request()
        req.min_x = self.get_parameter('min_x').get_parameter_value().integer_value
        req.max_x = self.get_parameter('max_x').get_parameter_value().integer_value
        req.min_y = self.get_parameter('min_y').get_parameter_value().integer_value
        req.max_y = self.get_parameter('max_y').get_parameter_value().integer_value
        req.visualization = True

        # Obstaculos
        obs_x = self.get_parameter('obstacle_points_x').get_parameter_value().double_array_value
        obs_y = self.get_parameter('obstacle_points_y').get_parameter_value().double_array_value
        for x, y in zip(obs_x, obs_y):
            p = Point2D()
            p.x = x
            p.y = y
            req.obstacle_points.append(p)

        # Posiciones iniciales
        init_x = self.get_parameter('initial_positions_x').get_parameter_value().double_array_value
        init_y = self.get_parameter('initial_positions_y').get_parameter_value().double_array_value
        
        # Sanity check: coincidencia de longitud de los UAVs y las posiciones iniciales
        if len(init_x) != len(self.uav_ids):
             self.get_logger().warn(f"Coincidencia fallida entre el numero de UAVs ({len(self.uav_ids)}) y las posiciones iniciales ({len(init_x)})")

        for x, y in zip(init_x, init_y):
            p = Point2D()
            p.x = x
            p.y = y
            req.initial_positions.append(p)

        future = self.darp_client.call_async(req)
        future.add_done_callback(self.darp_callback)

    def darp_callback(self, future):
        try:
            response = future.result()
            if not response.trajectories:
                self.get_logger().error("DARP devolvio trayectorias vacias")
                return

            self.get_logger().info(f"Solucion DARP recibida con {len(response.trajectories)} trayectorias")
            
            for i, traj in enumerate(response.trajectories):
                if i < len(self.uav_ids):
                    uav_id = self.uav_ids[i]
                    self.uav_trajectories[uav_id] = traj.points
                    self.uav_current_wp_index[uav_id] = 0
                    self.get_logger().info(f"Trayectoria asignada de longitud {len(traj.points)} a {uav_id}")
                else:
                    self.get_logger().warn(f"Mas trayectorias que UAVs: Trayectoria {i} ignorada")

        except Exception as e:
            self.get_logger().error(f'Llamada al servicio fallida: {e}')

    def control_loop(self):
        for uav_id in self.uav_ids:
            # 1. Publicar modo de control offboard (Heartbeat)
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            self.uav_publishers_offboard[uav_id].publish(offboard_msg)

            # 2. Verificar si hay trayectoria disponible
            traj_points = self.uav_trajectories.get(uav_id, [])
            if not traj_points:
                continue
            
            # 3. Gestionar armadado y offboard mode
            # Solo intentamos cambiar de modo si tenemos trayectoria y hemos enviado algunos setpoints
            self.uav_setpoint_counts[uav_id] += 1
            
            # Disparamos la secuencia de inicio despues de 1 segundo (20 * 0.05s) enviando setpoints
            if self.uav_setpoint_counts[uav_id] == 20:
                self.engage_offboard_mode(uav_id)
                self.arm(uav_id)

            # 4. Publicar punto de trayectoria
            idx = self.uav_current_wp_index.get(uav_id, 0)
            
            if idx >= len(traj_points):
                # Mantenerse en el ultimo punto
                target_p = traj_points[-1]
            else:
                target_p = traj_points[idx]
                
                # Comprobar si se ha llegado al punto
                current_pos = self.uav_positions.get(uav_id)
                if current_pos and current_pos.xy_valid:
                    # Mapear DARP(x,y) a NED(x,y)
                    # Asumcion: DARP X = Este (y), DARP Y = Norte (x)
                    # Re-leer darp_node:
                    # p.x = col (Este)
                    # p.y = row (Norte)
                    # Asumir mapa estandar: X=Este, Y=Norte
                    # PX4 NED: X=Norte, Y=Este
                    
                    target_ned_x = target_p.y
                    target_ned_y = target_p.x
                    
                    dx = target_ned_x - current_pos.x
                    dy = target_ned_y - current_pos.y
                    dist = math.sqrt(dx*dx + dy*dy)
                    
                    if dist < self.acceptance_radius:
                         self.uav_current_wp_index[uav_id] += 1
                         if self.uav_current_wp_index[uav_id] < len(traj_points):
                             self.get_logger().info(f"{uav_id} ha llegado al punto {idx}. Moviendo a {self.uav_current_wp_index[uav_id]}")
                             target_p = traj_points[self.uav_current_wp_index[uav_id]]

            # Construir mensaje de punto de trayectoria
            setpoint_msg = TrajectorySetpoint()
            setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            # Mapeo de coordenadas:
            # DARP X -> PX4 Y (Este)
            # DARP Y -> PX4 X (Norte)
            setpoint_msg.position[0] = float(target_p.y)  # Norte
            setpoint_msg.position[1] = float(target_p.x)  # Este
            setpoint_msg.position[2] = -float(self.target_altitude) # Abajo (negativo arriba)
            setpoint_msg.yaw = float('nan') # No controlar yaw explicitamente

            self.uav_publishers_trajectory[uav_id].publish(setpoint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

