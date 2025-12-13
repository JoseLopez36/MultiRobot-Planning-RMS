import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Point2D

from std_msgs.msg import Int32MultiArray, MultiArrayDimension

from multi_robot_planning_rms_msgs.srv import DarpPetition
from multi_robot_planning_rms_msgs.msg import Trajectory2D

class PlanningNode(Node):
    def __init__(self):
        super().__init__("planning_node")

        # Parámetros
        self.declare_parameter("agents.ids", [""])
        self.declare_parameter("cell_size", 2.0)
        self.declare_parameter("tasks.min_x", 0)
        self.declare_parameter("tasks.max_x", 0)
        self.declare_parameter("tasks.min_y", 0)
        self.declare_parameter("tasks.max_y", 0)
        self.declare_parameter("tasks.obstacles_positions_x", [0.0])
        self.declare_parameter("tasks.obstacles_positions_y", [0.0])
        self.agent_ids = self.get_parameter("agents.ids").get_parameter_value().string_array_value
        self.cell_size = self.get_parameter("cell_size").get_parameter_value().double_value
        self.min_x = self.get_parameter("tasks.min_x").get_parameter_value().integer_value
        self.max_x = self.get_parameter("tasks.max_x").get_parameter_value().integer_value
        self.min_y = self.get_parameter("tasks.min_y").get_parameter_value().integer_value
        self.max_y = self.get_parameter("tasks.max_y").get_parameter_value().integer_value
        self.obstacles_positions_x = self.get_parameter("tasks.obstacles_positions_x").get_parameter_value().double_array_value
        self.obstacles_positions_y = self.get_parameter("tasks.obstacles_positions_y").get_parameter_value().double_array_value

        # QoS para trayectorias
        trajectory_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS para zonas
        zones_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Suscripciones de posiciones por agente
        self.position_subscribers = {}
        self.positions = {}  # agent_id -> PointStamped
        self.position_received = {}  # agent_id -> bool

        for agent_id in self.agent_ids:
            self.positions[agent_id] = None
            self.position_received[agent_id] = False
            self.position_subscribers[agent_id] = self.create_subscription(
                PointStamped,
                f"/{agent_id}/state/position",
                lambda msg, uid=agent_id: self.position_callback(msg, uid),
                qos_profile
            )

        # Publicadores de trayectorias por agente
        self.trajectory_publishers = {}
        for agent_id in self.agent_ids:
            self.trajectory_publishers[agent_id] = self.create_publisher(
                Trajectory2D, 
                f"/{agent_id}/planning/trajectory",
                trajectory_qos
            )

        # Publicador de zonas
        self.zones_pub = self.create_publisher(
            Int32MultiArray,
            "/planning/zones",
            zones_qos,
        )

        # Cliente de servicio DARP
        self.darp_client = self.create_client(DarpPetition, "darp_service")
        while not self.darp_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("Servicio DARP no disponible, esperando de nuevo...")

        self.plan_requested = False
        self.plan_done = False

        # Timer para solicitar la planificación inicial una vez que se reciben todas las posiciones
        self.timer = self.create_timer(0.5, self.maybe_request_initial_plan)

        self.get_logger().info("Nodo de planificacion iniciado. Esperando posiciones iniciales...")

    def position_callback(self, msg: PointStamped, agent_id: str):
        self.positions[agent_id] = msg
        self.position_received[agent_id] = True

    def all_positions_received(self) -> bool:
        return all(self.position_received.get(agent_id, False) for agent_id in self.agent_ids)

    def maybe_request_initial_plan(self):
        if self.plan_done or self.plan_requested:
            return
        if not self.all_positions_received():
            return

        self.plan_requested = True
        self.request_darp()

    def request_darp(self):
        self.get_logger().info("Peticion de algoritmo DARP (planificacion inicial)...")
        req = DarpPetition.Request()
        req.min_x = int(self.min_x)
        req.max_x = int(self.max_x)
        req.min_y = int(self.min_y)
        req.max_y = int(self.max_y)
        req.visualization = False

        # Obstaculos desde parametros
        for x, y in zip(self.obstacles_positions_x, self.obstacles_positions_y):
            p = Point2D()
            p.x = float(x)
            p.y = float(y)
            req.obstacle_points.append(p)

        # Posiciones iniciales desde suscripciones
        for agent_id in self.agent_ids:
            pos = self.positions.get(agent_id)
            if pos is None:
                continue
            p = Point2D()
            p.x = float(pos.point.x)
            p.y = float(pos.point.y)
            req.initial_positions.append(p)

        future = self.darp_client.call_async(req)
        future.add_done_callback(self.darp_callback)

    def darp_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Llamada al servicio DARP fallida: {e}")
            self.plan_requested = False
            return

        if not response.trajectories:
            self.get_logger().error("DARP devolvio trayectorias vacias")
            self.plan_requested = False
            return

        self.get_logger().info(
            f"Solucion DARP recibida con {len(response.trajectories)} trayectorias"
        )

        # Publicar zonas para visualización (filas/columnas en layout)
        # DARP devuelve una matriz plana en orden de filas: zones[row * cols + col]
        if response.zones:
            extent_x = int(self.max_x) - int(self.min_x)
            extent_y = int(self.max_y) - int(self.min_y)
            cols = int(round(float(extent_x) / float(self.cell_size))) if self.cell_size > 0.0 else 0
            rows = int(round(float(extent_y) / float(self.cell_size))) if self.cell_size > 0.0 else 0

            zones_msg = Int32MultiArray()
            zones_msg.layout.dim = [
                MultiArrayDimension(label="rows", size=max(0, rows), stride=max(0, rows * cols)),
                MultiArrayDimension(label="cols", size=max(0, cols), stride=max(0, cols)),
            ]
            zones_msg.layout.data_offset = 0
            zones_msg.data = list(response.zones)
            self.zones_pub.publish(zones_msg)

        # Publicar trayectorias por agente basado en el orden de indices
        for i, traj in enumerate(response.trajectories):
            if i >= len(self.agent_ids):
                self.get_logger().warn(
                    f"Mas trayectorias que UAVs: Trayectoria {i} ignorada"
                )
                continue
            agent_id = self.agent_ids[i]
            msg = Trajectory2D()
            msg.points = traj.points
            self.trajectory_publishers[agent_id].publish(msg)
            self.get_logger().info(
                f"Publicada trayectoria (len={len(msg.points)}) en /{agent_id}/planning/trajectory"
            )

        self.plan_done = True

def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


