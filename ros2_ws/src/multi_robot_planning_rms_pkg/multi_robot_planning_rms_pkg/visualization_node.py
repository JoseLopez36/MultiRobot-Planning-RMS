import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from multi_robot_planning_rms_msgs.msg import Trajectory2D

from dataclasses import dataclass
from typing import Dict, List, Tuple

@dataclass(frozen=True)
class RGBA:
    r: float
    g: float
    b: float
    a: float


def palette(alpha: float) -> List[RGBA]:
    # Simple, deterministic palette (cycled by agent index)
    return [
        RGBA(1.0, 0.0, 0.0, alpha),  # red
        RGBA(0.0, 1.0, 0.0, alpha),  # green
        RGBA(0.0, 0.6, 1.0, alpha),  # blue-ish
        RGBA(1.0, 1.0, 0.0, alpha),  # yellow
        RGBA(1.0, 0.0, 1.0, alpha),  # magenta
        RGBA(0.0, 1.0, 1.0, alpha),  # cyan
        RGBA(1.0, 0.5, 0.0, alpha),  # orange
        RGBA(0.6, 0.0, 1.0, alpha),  # purple
    ]


class VisualizationNode(Node):
    def __init__(self) -> None:
        super().__init__("visualization_node")

        # Parámetros
        self.declare_parameter("agents.ids", [""])
        self.declare_parameter("markers.line_width", 0.10)
        self.declare_parameter("markers.point_scale", 0.25)
        self.declare_parameter("markers.alpha", 0.9)
        self.agent_ids = self.get_parameter("agents.ids").get_parameter_value().string_array_value
        self.line_width = self.get_parameter("markers.line_width").get_parameter_value().double_value
        self.point_scale = self.get_parameter("markers.point_scale").get_parameter_value().double_value
        self.alpha = self.get_parameter("markers.alpha").get_parameter_value().double_value

        # QoS para trayectorias
        trajectory_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS para marcadores
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publicador de marcadores
        self.marker_pub = self.create_publisher(MarkerArray, "/planning/markers", trajectory_qos)

        # Suscriptores de trayectorias
        self.traj_subs = {}
        self.colors = palette(alpha=self.alpha)
        self.agent_index = {aid: i for i, aid in enumerate(self.agent_ids)}

        for agent_id in self.agent_ids:
            self.traj_subs[agent_id] = self.create_subscription(
                Trajectory2D,
                f"/{agent_id}/planning/trajectory",
                lambda msg, uid=agent_id: self.trajectory_cb(msg, uid),
                trajectory_qos
            )

        self.get_logger().info(f"Nodo de visualizacion iniciado")

    def color_for(self, agent_id: str) -> RGBA:
        idx = self.agent_index.get(agent_id, 0)
        return self.colors[idx % len(self.colors)]

    def make_markers(
        self, agent_id: str, points_xy: List[Tuple[float, float]]
    ) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        c = self.color_for(agent_id)

        ns = f"trajectory/{agent_id}"

        # LINE_STRIP
        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = now
        line.ns = ns
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = float(self.line_width)
        line.color.r = float(c.r)
        line.color.g = float(c.g)
        line.color.b = float(c.b)
        line.color.a = float(c.a)
        line.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in points_xy]

        # SPHERE_LIST
        spheres = Marker()
        spheres.header.frame_id = "map"
        spheres.header.stamp = now
        spheres.ns = ns
        spheres.id = 1
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        spheres.pose.orientation.w = 1.0
        spheres.scale.x = float(self.point_scale)
        spheres.scale.y = float(self.point_scale)
        spheres.scale.z = float(self.point_scale)
        spheres.color.r = float(c.r)
        spheres.color.g = float(c.g)
        spheres.color.b = float(c.b)
        spheres.color.a = float(c.a)
        spheres.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in points_xy]

        out = MarkerArray()
        out.markers = [line, spheres]
        return out

    def trajectory_cb(self, msg: Trajectory2D, agent_id: str) -> None:
        pts = [(float(p.x), float(p.y)) for p in msg.points]
        markers = self.make_markers(agent_id, pts)
        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


