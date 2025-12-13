import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from multi_robot_planning_rms_msgs.srv import DarpPetition
from multi_robot_planning_rms_msgs.msg import Trajectory2D
from px4_msgs.msg import OffboardControlMode, VehicleLocalPosition, VehicleCommand, VehicleStatus
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Point2D
import math
import numpy as np

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')

        # Parametros de los agentes
        self.declare_parameter('agents.ids', [''])
        self.declare_parameter('agents.initial_positions_x', [0.0])
        self.declare_parameter('agents.initial_positions_y', [0.0])

        # Parametros de la peticion DARP
        self.declare_parameter('tasks.min_x', 0)
        self.declare_parameter('tasks.max_x', 0)
        self.declare_parameter('tasks.min_y', 0)
        self.declare_parameter('tasks.max_y', 0)
        self.declare_parameter('tasks.obstacles_positions_x', [0.0])
        self.declare_parameter('tasks.obstacles_positions_y', [0.0])

        # Otros parametros
        self.declare_parameter('target_altitude', 5.0)
        self.declare_parameter('acceptance_radius', 0.5)

        # Obtener los parametros
        self.agent_ids = self.get_parameter('agents.ids').get_parameter_value().string_array_value
        self.initial_positions_x = self.get_parameter('agents.initial_positions_x').get_parameter_value().double_array_value
        self.initial_positions_y = self.get_parameter('agents.initial_positions_y').get_parameter_value().double_array_value
        self.min_x = self.get_parameter('tasks.min_x').get_parameter_value().integer_value
        self.max_x = self.get_parameter('tasks.max_x').get_parameter_value().integer_value
        self.min_y = self.get_parameter('tasks.min_y').get_parameter_value().integer_value
        self.max_y = self.get_parameter('tasks.max_y').get_parameter_value().integer_value
        self.obstacles_positions_x = self.get_parameter('tasks.obstacles_positions_x').get_parameter_value().double_array_value
        self.obstacles_positions_y = self.get_parameter('tasks.obstacles_positions_y').get_parameter_value().double_array_value
        self.target_altitude = self.get_parameter('target_altitude').get_parameter_value().double_value
        self.acceptance_radius = self.get_parameter('acceptance_radius').get_parameter_value().double_value
        
        # Cliente DARP
        self.darp_client = self.create_client(DarpPetition, 'darp_service')
        while not self.darp_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Servicio DARP no disponible, esperando de nuevo...')
        
        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Gestion de estado de los UAVs
        self.position_subscribers = {}
        self.status_subscribers = {}
        self.trajectory_publishers = {}
        self.offboard_control_mode_publishers = {}
        self.vehicle_command_publishers = {}
        
        self.positions = {}
        self.statuses = {}
        self.trajectories = {} # Key: agent_id, Value: List of Point2D
        self.current_wp_indices = {} # Key: agent_id, Value: int
        self.setpoint_counts = {} # Key: agent_id, Value: int
        self.agent_states = {} # Estados: INIT, TAKEOFF, MISSION, LANDED

        # Iterar sobre los agentes
        for agent_id in self.agent_ids:
            # Suscriptores
            # Suscripción a la posición del vehículo
            self.position_subscribers[agent_id] = self.create_subscription(
                PointStamped,
                f'/{agent_id}/state/position',
                lambda msg, uid=agent_id: self.position_callback(msg, uid),
                qos_profile
            )
            
            # Suscripción al estado del vehículo (para saber si está armado/offboard)
            self.status_subscribers[agent_id] = self.create_subscription(
                VehicleStatus,
                f'/{agent_id}/fmu/out/vehicle_status',
                lambda msg, uid=agent_id: self.status_callback(msg, uid),
                qos_profile
            )

            # Publicadores
            # Publicador de modo offboard (Heartbeat)
            self.offboard_control_mode_publishers[agent_id] = self.create_publisher(
                OffboardControlMode,
                f'/{agent_id}/fmu/in/offboard_control_mode',
                qos_profile
            )
            
            # Publicador de trayectoria (Setpoints)
            self.trajectory_publishers[agent_id] = self.create_publisher(
                PointStamped,
                f'/{agent_id}/control/setpoint',
                qos_profile
            )
            
            # Publicador de comandos del vehículo (Armar, Modos)
            self.vehicle_command_publishers[agent_id] = self.create_publisher(
                VehicleCommand,
                f'/{agent_id}/fmu/in/vehicle_command',
                qos_profile
            )

            self.positions[agent_id] = PointStamped()
            self.statuses[agent_id] = VehicleStatus()
            self.trajectories[agent_id] = []
            self.current_wp_indices[agent_id] = 0
            self.setpoint_counts[agent_id] = 0
            self.agent_states[agent_id] = 'INIT'

        # Timer para el bucle de control (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Peticion de solucion DARP
        self.request_darp()

    def position_callback(self, msg, agent_id):
        self.positions[agent_id] = msg

    def status_callback(self, msg, agent_id):
        self.statuses[agent_id] = msg

    def get_px4_system_id(self, agent_id):
        # Asume formato "px4_X" -> sistema X
        try:
            return int(agent_id.split('_')[-1])
        except:
            return 1

    def publish_vehicle_command(self, agent_id, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        
        sys_id = self.get_px4_system_id(agent_id)
        msg.target_system = sys_id + 1
        msg.target_component = 1
        msg.source_system = sys_id + 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.vehicle_command_publishers[agent_id].publish(msg)

    def publish_offboard_control_mode(self, agent_id):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publishers[agent_id].publish(msg)

    def publish_trajectory_setpoint(self, agent_id, x, y, z):
        # Envía setpoint en marco de referencia global
        msg = PointStamped()
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.trajectory_publishers[agent_id].publish(msg)

    def arm(self, agent_id):
        self.publish_vehicle_command(agent_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f'{agent_id}: Enviando comando ARM')

    def disarm(self, agent_id):
        self.publish_vehicle_command(agent_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info(f'{agent_id}: Enviando comando DISARM')

    def engage_offboard_mode(self, agent_id):
        self.publish_vehicle_command(agent_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(f"{agent_id}: Cambiando a modo OFFBOARD")

    def request_darp(self):
        self.get_logger().info("Peticion de algoritmo DARP...")
        req = DarpPetition.Request()
        req.min_x = self.min_x
        req.max_x = self.max_x
        req.min_y = self.min_y
        req.max_y = self.max_y
        req.visualization = False

        # Obstaculos
        obs_x = self.obstacles_positions_x
        obs_y = self.obstacles_positions_y
        for x, y in zip(obs_x, obs_y):
            p = Point2D()
            p.x = x
            p.y = y
            req.obstacle_points.append(p)

        # Posiciones iniciales
        init_x = self.initial_positions_x
        init_y = self.initial_positions_y
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
                if i < len(self.agent_ids):
                    agent_id = self.agent_ids[i]
                    self.trajectories[agent_id] = traj.points
                    self.current_wp_indices[agent_id] = 0
                    self.get_logger().info(f"Trayectoria asignada de longitud {len(traj.points)} a {agent_id}")
                else:
                    self.get_logger().warn(f"Mas trayectorias que UAVs: Trayectoria {i} ignorada")

        except Exception as e:
            self.get_logger().error(f'Llamada al servicio fallida: {e}')

    def control_loop(self):
        for i, agent_id in enumerate(self.agent_ids):
            # Enviar Heartbeat de Offboard
            self.publish_offboard_control_mode(agent_id)
            
            # Obtener el estado del agente
            state = self.agent_states[agent_id]
            current_pos = self.positions[agent_id]
            current_status = self.statuses[agent_id]
            
            # --- Estado: INIT (Espera inicial y Armado) ---
            if state == 'INIT':
                # Enviar setpoint 0,0,-alt
                self.publish_trajectory_setpoint(agent_id, current_pos.point.x, current_pos.point.y, self.target_altitude)
                
                self.setpoint_counts[agent_id] += 1
                
                # Después de unos ciclos, intentar armar y pasar a offboard
                if self.setpoint_counts[agent_id] > 10:
                    if current_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                        self.engage_offboard_mode(agent_id)
                    
                    if current_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                        self.arm(agent_id)
                        
                    # Si ya está en offboard y armado, pasar a despegue
                    if (current_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and 
                        current_status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                        self.agent_states[agent_id] = 'TAKEOFF'
                        self.get_logger().info(f"{agent_id}: Iniciando TAKEOFF")

            # --- Estado: TAKEOFF (Despegue vertical) ---
            elif state == 'TAKEOFF':
                # Mantener setpoint de altura
                self.publish_trajectory_setpoint(agent_id, current_pos.point.x, current_pos.point.y, self.target_altitude)
                
                # Verificar altura
                if current_pos.point.z >= (self.target_altitude - 0.5): # Margen de 0.5m
                     self.agent_states[agent_id] = 'MISSION'
                     self.get_logger().info(f"{agent_id}: Despegue completado. Iniciando MISION")

            # --- Estado: MISSION (Seguimiento de trayectoria DARP) ---
            elif state == 'MISSION':
                traj_points = self.trajectories.get(agent_id, [])
                if not traj_points:
                    # Si no hay trayectoria, mantener posición actual (hover)
                    self.publish_trajectory_setpoint(agent_id, current_pos.point.x, current_pos.point.y, self.target_altitude)
                    continue

                idx = self.current_wp_indices.get(agent_id, 0)
                
                if idx >= len(traj_points):
                    # Fin de trayectoria: Mantener último punto
                    target_pos = traj_points[-1]
                else:
                    target_pos = traj_points[idx]
                    
                    # Calcular distancia al objetivo
                    dx = float(target_pos.x) - float(current_pos.point.x)
                    dy = float(target_pos.y) - float(current_pos.point.y)
                    d = math.sqrt(dx**2 + dy**2)
                    
                    if d < self.acceptance_radius:
                        self.current_wp_indices[agent_id] += 1
                        if self.current_wp_indices[agent_id] < len(traj_points):
                            self.get_logger().info(f"{agent_id} ha llegado al punto {idx}. Moviendo a {self.current_wp_indices[agent_id]}")
                            target_pos = traj_points[self.current_wp_indices[agent_id]]

                # Publicar punto de trayectoria
                self.publish_trajectory_setpoint(agent_id, target_pos.x, target_pos.y, self.target_altitude)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
