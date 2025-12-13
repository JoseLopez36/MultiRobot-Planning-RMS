import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int32MultiArray

from multi_robot_planning_rms_msgs.msg import Trajectory2D

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

@dataclass(frozen=True)
class RGBA:
    r: float
    g: float
    b: float
    a: float

def palette(alpha: float) -> List[RGBA]:
    return [
        RGBA(1.0, 0.0, 0.0, alpha),  # rojo
        RGBA(0.0, 1.0, 0.0, alpha),  # verde
        RGBA(0.0, 0.6, 1.0, alpha),  # azul
        RGBA(1.0, 1.0, 0.0, alpha),  # amarillo
        RGBA(1.0, 0.0, 1.0, alpha),  # magenta
        RGBA(0.0, 1.0, 1.0, alpha),  # cian
        RGBA(1.0, 0.5, 0.0, alpha),  # naranja
        RGBA(0.6, 0.0, 1.0, alpha),  # púrpura
    ]

class VisualizationNode(Node):
    def __init__(self) -> None:
        super().__init__("visualization_node")

        # Parámetros
        self.declare_parameter("agents.ids", [""])
        self.declare_parameter("drones.position_scale", 0.35)
        self.declare_parameter("drones.setpoint_scale", 0.25)
        self.declare_parameter("drones.arrow_width", 0.08)
        self.declare_parameter("drones.label_scale", 0.4)
        self.declare_parameter("drones.z_offset", 0.05)
        self.declare_parameter("trajectories.line_width", 0.10)
        self.declare_parameter("trajectories.point_scale", 0.25)
        self.declare_parameter("trajectories.alpha", 0.9)
        self.declare_parameter("zones.cell_size", 2.0)
        self.declare_parameter("zones.fill_alpha", 0.25)
        self.declare_parameter("zones.outline_width", 0.05)
        self.declare_parameter("tasks.min_x", 0)
        self.declare_parameter("tasks.min_y", 0)
        self.agent_ids = self.get_parameter("agents.ids").get_parameter_value().string_array_value
        self.position_scale = self.get_parameter("drones.position_scale").get_parameter_value().double_value
        self.setpoint_scale = self.get_parameter("drones.setpoint_scale").get_parameter_value().double_value
        self.arrow_width = self.get_parameter("drones.arrow_width").get_parameter_value().double_value
        self.label_scale = self.get_parameter("drones.label_scale").get_parameter_value().double_value
        self.z_offset = self.get_parameter("drones.z_offset").get_parameter_value().double_value
        self.line_width = self.get_parameter("trajectories.line_width").get_parameter_value().double_value
        self.point_scale = self.get_parameter("trajectories.point_scale").get_parameter_value().double_value
        self.alpha = self.get_parameter("trajectories.alpha").get_parameter_value().double_value
        self.zones_cell_size = self.get_parameter("zones.cell_size").get_parameter_value().double_value
        self.zones_fill_alpha = self.get_parameter("zones.fill_alpha").get_parameter_value().double_value
        self.zones_outline_width = self.get_parameter("zones.outline_width").get_parameter_value().double_value
        self.min_x = float(self.get_parameter("tasks.min_x").get_parameter_value().integer_value)
        self.min_y = float(self.get_parameter("tasks.min_y").get_parameter_value().integer_value)

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

        # QoS para marcadores
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publicador de marcadores
        self.drone_markers_pub = self.create_publisher(MarkerArray, "/visualization/drones", marker_qos)
        self.trajectory_markers_pub = self.create_publisher(MarkerArray, "/visualization/trajectories", marker_qos)
        self.zones_markers_pub = self.create_publisher(MarkerArray, "/visualization/zones", marker_qos)

        # Suscriptores de posiciones y setpoints
        self.positions: Dict[str, Optional[PointStamped]] = {aid: None for aid in self.agent_ids}
        self.setpoints: Dict[str, Optional[PointStamped]] = {aid: None for aid in self.agent_ids}
        self.pos_subs = {}
        self.setpoint_subs = {}

        # Suscriptores de trayectorias
        self.traj_subs = {}
        self.colors = palette(alpha=self.alpha)
        self.agent_index = {aid: i for i, aid in enumerate(self.agent_ids)}

        for agent_id in self.agent_ids:
            self.pos_subs[agent_id] = self.create_subscription(
                PointStamped,
                f"/{agent_id}/state/position",
                lambda msg, uid=agent_id: self.position_cb(msg, uid),
                marker_qos,
            )
            self.setpoint_subs[agent_id] = self.create_subscription(
                PointStamped,
                f"/{agent_id}/control/setpoint",
                lambda msg, uid=agent_id: self.setpoint_cb(msg, uid),
                marker_qos,
            )
            self.traj_subs[agent_id] = self.create_subscription(
                Trajectory2D,
                f"/{agent_id}/planning/trajectory",
                lambda msg, uid=agent_id: self.trajectory_cb(msg, uid),
                trajectory_qos
            )

        # Suscriptor de zonas
        self.zones_sub = self.create_subscription(
            Int32MultiArray,
            "/planning/zones",
            self.zones_cb,
            zones_qos
        )

        self.get_logger().info(f"Nodo de visualizacion iniciado")

    def position_cb(self, msg: PointStamped, agent_id: str) -> None:
        self.positions[agent_id] = msg
        markers = self.make_drone_markers()
        self.drone_markers_pub.publish(markers)

    def setpoint_cb(self, msg: PointStamped, agent_id: str) -> None:
        self.setpoints[agent_id] = msg

    def trajectory_cb(self, msg: Trajectory2D, agent_id: str) -> None:
        pts = [(float(p.x), float(p.y)) for p in msg.points]
        markers = self.make_trajectory_markers(agent_id, pts)
        self.trajectory_markers_pub.publish(markers)

    def zones_cb(self, msg: Int32MultiArray) -> None:
        rows, cols = self.grid_rows_cols(msg)
        if rows <= 0 or cols <= 0:
            self.get_logger().warn("Zones grid received without valid rows/cols in layout")
            return

        expected = rows * cols
        data = list(msg.data)
        if len(data) < expected:
            self.get_logger().warn(
                f"Zones grid size mismatch: expected {expected} but got {len(data)}"
            )
            return
        if len(data) > expected:
            data = data[:expected]

        markers = self.make_zone_markers(rows, cols, data)
        self.zones_markers_pub.publish(markers)

    def make_drone_markers(self) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        out = MarkerArray()

        for agent_id in self.agent_ids:
            pos = self.positions.get(agent_id)
            sp = self.setpoints.get(agent_id)
            c = self.color_for_agent(agent_id)
            ns = f"drone/{agent_id}"

            # Marcador de posición del dron (esfera)
            m_pos = Marker()
            m_pos.header.frame_id = "map"
            m_pos.header.stamp = now
            m_pos.ns = ns
            m_pos.id = 0
            m_pos.type = Marker.SPHERE
            m_pos.action = Marker.ADD if pos is not None else Marker.DELETE
            m_pos.pose.orientation.w = 1.0
            m_pos.scale.x = float(self.position_scale)
            m_pos.scale.y = float(self.position_scale)
            m_pos.scale.z = float(self.position_scale)
            m_pos.color.r = float(c.r)
            m_pos.color.g = float(c.g)
            m_pos.color.b = float(c.b)
            m_pos.color.a = 1.0
            if pos is not None:
                m_pos.pose.position.x = float(pos.point.x)
                m_pos.pose.position.y = float(pos.point.y)
                m_pos.pose.position.z = float(pos.point.z + self.z_offset)
            out.markers.append(m_pos)

            # Marcador de setpoint (esfera pequeña)
            m_sp = Marker()
            m_sp.header.frame_id = "map"
            m_sp.header.stamp = now
            m_sp.ns = ns
            m_sp.id = 1
            m_sp.type = Marker.SPHERE
            m_sp.action = Marker.ADD if sp is not None else Marker.DELETE
            m_sp.pose.orientation.w = 1.0
            m_sp.scale.x = float(self.setpoint_scale)
            m_sp.scale.y = float(self.setpoint_scale)
            m_sp.scale.z = float(self.setpoint_scale)
            m_sp.color.r = 1.0
            m_sp.color.g = 1.0
            m_sp.color.b = 1.0
            m_sp.color.a = 0.95
            if sp is not None:
                m_sp.pose.position.x = float(sp.point.x)
                m_sp.pose.position.y = float(sp.point.y)
                m_sp.pose.position.z = float(sp.point.z + self.z_offset)
            out.markers.append(m_sp)

            # Flecha desde posición a setpoint
            m_arrow = Marker()
            m_arrow.header.frame_id = "map"
            m_arrow.header.stamp = now
            m_arrow.ns = ns
            m_arrow.id = 2
            m_arrow.type = Marker.ARROW
            m_arrow.action = (
                Marker.ADD if (pos is not None and sp is not None) else Marker.DELETE
            )
            m_arrow.pose.orientation.w = 1.0
            m_arrow.scale.x = float(self.arrow_width)
            m_arrow.scale.y = float(self.arrow_width * 2.0)
            m_arrow.scale.z = float(self.arrow_width * 3.0)
            m_arrow.color.r = float(c.r)
            m_arrow.color.g = float(c.g)
            m_arrow.color.b = float(c.b)
            m_arrow.color.a = 0.9
            if pos is not None and sp is not None:
                p0 = Point(x=float(pos.point.x), y=float(pos.point.y), z=float(pos.point.z + self.z_offset))
                p1 = Point(x=float(sp.point.x), y=float(sp.point.y), z=float(sp.point.z + self.z_offset))
                m_arrow.points = [p0, p1]
            out.markers.append(m_arrow)

            # Etiqueta (id de agente + altitud)
            m_text = Marker()
            m_text.header.frame_id = "map"
            m_text.header.stamp = now
            m_text.ns = ns
            m_text.id = 3
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD if pos is not None else Marker.DELETE
            m_text.pose.orientation.w = 1.0
            m_text.scale.z = float(self.label_scale)
            m_text.color.r = 1.0
            m_text.color.g = 1.0
            m_text.color.b = 1.0
            m_text.color.a = 1.0
            if pos is not None:
                m_text.pose.position.x = float(pos.point.x + 0.2)
                m_text.pose.position.y = float(pos.point.y + 0.2)
                m_text.pose.position.z = float(pos.point.z + self.z_offset + 0.6)
                m_text.text = f"{agent_id}"
            out.markers.append(m_text)

        return out

    def make_trajectory_markers(
        self, agent_id: str, points_xy: List[Tuple[float, float]]
    ) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        c = self.color_for_agent(agent_id)

        ns = f"trajectory/{agent_id}"

        # Tipo LINE_STRIP
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

        # Tipo SPHERE_LIST
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

    def make_zone_markers(self, rows: int, cols: int, zones: List[int]) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        min_x, min_y, cell = self.zones_origin_and_cell()
        zc = 0.01
        zh = 0.02

        ns = "zones"

        # Celdas rellenas
        fill = Marker()
        fill.header.frame_id = "map"
        fill.header.stamp = now
        fill.ns = ns
        fill.id = 0
        fill.type = Marker.CUBE_LIST
        fill.action = Marker.ADD
        fill.pose.orientation.w = 1.0
        fill.scale.x = float(cell)
        fill.scale.y = float(cell)
        fill.scale.z = float(zh)

        # Contornos (bordes de celdas entre diferentes zonas)
        outline = Marker()
        outline.header.frame_id = "map"
        outline.header.stamp = now
        outline.ns = ns
        outline.id = 1
        outline.type = Marker.LINE_LIST
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = float(self.zones_outline_width)

        for r in range(rows):
            y = min_y + (float(r) + 0.5) * cell
            for c in range(cols):
                zval = int(zones[r * cols + c])
                if zval <= 0:
                    continue
                x = min_x + (float(c) + 0.5) * cell
                fill.points.append(Point(x=float(x), y=float(y), z=zc))
                fill.colors.append(self.color_for_zone(zval, alpha=float(self.zones_fill_alpha)))

        # Dibujar bordes derecho/inferior para cada celda no cero; tratar el exterior como 0
        zline = zc + 0.5 * zh
        for r in range(rows):
            y0 = min_y + float(r) * cell
            y1 = min_y + float(r + 1) * cell
            for c in range(cols):
                zval = int(zones[r * cols + c])
                if zval <= 0:
                    continue

                # Borde derecho
                zr = int(zones[r * cols + (c + 1)]) if (c + 1) < cols else 0
                if zr != zval:
                    x = min_x + float(c + 1) * cell
                    p0 = Point(x=float(x), y=float(y0), z=float(zline))
                    p1 = Point(x=float(x), y=float(y1), z=float(zline))
                    outline.points.extend([p0, p1])
                    col = self.color_for_zone(zval, alpha=1.0)
                    outline.colors.extend([col, col])

                # Borde inferior
                zd = int(zones[(r + 1) * cols + c]) if (r + 1) < rows else 0
                if zd != zval:
                    y = min_y + float(r + 1) * cell
                    x0 = min_x + float(c) * cell
                    x1 = min_x + float(c + 1) * cell
                    p0 = Point(x=float(x0), y=float(y), z=float(zline))
                    p1 = Point(x=float(x1), y=float(y), z=float(zline))
                    outline.points.extend([p0, p1])
                    col = self.color_for_zone(zval, alpha=1.0)
                    outline.colors.extend([col, col])

                # Borde izquierdo
                if c == 0:
                    x = min_x
                    p0 = Point(x=float(x), y=float(y0), z=float(zline))
                    p1 = Point(x=float(x), y=float(y1), z=float(zline))
                    outline.points.extend([p0, p1])
                    col = self.color_for_zone(zval, alpha=1.0)
                    outline.colors.extend([col, col])

                # Borde superior
                if r == 0:
                    y = min_y
                    x0 = min_x + float(c) * cell
                    x1 = min_x + float(c + 1) * cell
                    p0 = Point(x=float(x0), y=float(y), z=float(zline))
                    p1 = Point(x=float(x1), y=float(y), z=float(zline))
                    outline.points.extend([p0, p1])
                    col = self.color_for_zone(zval, alpha=1.0)
                    outline.colors.extend([col, col])

        out = MarkerArray()
        out.markers = [fill, outline]
        return out

    def grid_rows_cols(self, msg: Int32MultiArray) -> Tuple[int, int]:
        rows = 0
        cols = 0
        try:
            dims = list(msg.layout.dim)
        except Exception:
            dims = []

        # Preferir dimensiones etiquetadas si están presentes
        for d in dims:
            if getattr(d, "label", "") == "rows":
                rows = int(d.size)
            elif getattr(d, "label", "") == "cols":
                cols = int(d.size)

        # Respaldo a dimensiones posicionales
        if (rows <= 0 or cols <= 0) and len(dims) >= 2:
            rows = int(dims[0].size)
            cols = int(dims[1].size)

        return rows, cols

    def zones_origin_and_cell(self) -> Tuple[float, float, float]:
        cell = self.zones_cell_size
        min_x = self.min_x
        min_y = self.min_y

        return float(min_x), float(min_y), float(cell)

    def color_for_agent(self, agent_id: str) -> RGBA:
        idx = self.agent_index.get(agent_id, 0)
        return self.colors[idx % len(self.colors)]

    def color_for_zone(self, zone_value: int, alpha: float) -> ColorRGBA:
        # zone_value: 1..N => índice de agente zone_value-1
        idx = max(0, int(zone_value) - 1)
        base = palette(alpha=alpha)[idx % len(self.colors)]
        c = ColorRGBA()
        c.r = float(base.r)
        c.g = float(base.g)
        c.b = float(base.b)
        c.a = float(base.a)
        return c

def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


