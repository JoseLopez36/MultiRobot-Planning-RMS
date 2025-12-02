import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from multi_robot_planning_rms_msgs.srv import DarpPetition
from multi_robot_planning_rms_msgs.msg import Trajectory2D
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleCommand, VehicleStatus
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
        self.position_subscribers = {}
        self.trajectory_publishers = {}
        self.positions = {}
        self.trajectories = {} # Key: agent_id, Value: List of Point2D
        self.current_wp_indices = {} # Key: agent_id, Value: int
        self.setpoint_counts = {} # Key: agent_id, Value: int

        # Iterar sobre los agentes
        for agent_id in self.agent_ids:
            # Suscriptores
            self.position_subscribers[agent_id] = self.create_subscription(
                PointStamped,
                f'/{agent_id}/state/position',
                lambda msg, uid=agent_id: self.position_callback(msg, uid),
                qos_profile
            )
            # Publicadores
            self.trajectory_publishers[agent_id] = self.create_publisher(
                PointStamped,
                f'/{agent_id}/control/setpoint',
                qos_profile
            )

            self.positions[agent_id] = None
            self.trajectories[agent_id] = []
            self.current_wp_indices[agent_id] = 0
            self.setpoint_counts[agent_id] = 0

        # Timer para el bucle de control (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Peticion de solucion DARP
        self.request_darp()

    def position_callback(self, msg, agent_id):
        self.positions[agent_id] = msg

    def request_darp(self):
        self.get_logger().info("Peticion de algoritmo DARP...")
        req = DarpPetition.Request()
        req.min_x = self.min_x
        req.max_x = self.max_x
        req.min_y = self.min_y
        req.max_y = self.max_y
        req.visualization = True

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
        
        # Sanity check: coincidencia de longitud de los UAVs y las posiciones iniciales
        if len(init_x) != len(self.agent_ids):
             self.get_logger().warn(f"Coincidencia fallida entre el numero de UAVs ({len(self.agent_ids)}) y las posiciones iniciales ({len(init_x)})")

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
        for agent_id in self.agent_ids:
            #  1. Verificar si hay trayectoria disponible
            traj_points = self.trajectories.get(agent_id, [])
            if not traj_points:
                continue
            self.setpoint_counts[agent_id] += 1

            # 2. Publicar punto de trayectoria
            idx = self.current_wp_indices.get(agent_id, 0)
            
            if idx >= len(traj_points):
                # Mantenerse en el ultimo punto
                target_p = traj_points[-1]
            else:
                target_p = traj_points[idx]
                
                # Comprobar si se ha llegado al punto
                current_pos = self.positions.get(agent_id)
                if current_pos is not None:
                    dx = float(target_p.x) - float(current_pos.point.x)
                    dy = float(target_p.y) - float(current_pos.point.y)
                    dist = math.sqrt(dx*dx + dy*dy)
                    
                    if dist < self.acceptance_radius:
                        self.current_wp_indices[agent_id] += 1
                        if self.current_wp_indices[agent_id] < len(traj_points):
                            self.get_logger().info(f"{agent_id} ha llegado al punto {idx}. Moviendo a {self.current_wp_indices[agent_id]}")
                            target_p = traj_points[self.current_wp_indices[agent_id]]

            # Construir mensaje de punto de trayectoria
            setpoint_msg = PointStamped()
            setpoint_msg.point.x = float(target_p.x)
            setpoint_msg.point.y = float(target_p.y)
            setpoint_msg.point.z = float(self.target_altitude)

            self.trajectory_publishers[agent_id].publish(setpoint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

