import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Point2D
from multi_robot_planning_rms_msgs.msg import Trajectory2D
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # Parámetros
        self.declare_parameter("agent_id", "")
        self.declare_parameter("target_altitude", 5.0)
        self.declare_parameter("acceptance_radius", 0.5)
        self.agent_id = self.get_parameter("agent_id").get_parameter_value().string_value
        self.target_altitude = self.get_parameter("target_altitude").get_parameter_value().double_value
        self.acceptance_radius = self.get_parameter("acceptance_radius").get_parameter_value().double_value

        # QoS para trayectorias
        trajectory_qos = QoSProfile(
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

        # Estado del agente
        self.position = None  # PointStamped
        self.status = None  # VehicleStatus
        self.trajectory_points = []  # list[vision_msgs/Point2D]
        self.current_index = 0
        self.setpoint_count = 0
        self.agent_state = "INIT"  # INIT, TAKEOFF, MISSION

        # Suscripciones
        self.position_sub = self.create_subscription(
            PointStamped,
            f"/{self.agent_id}/state/position",
            self.position_callback,
            qos_profile
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            f"/{self.agent_id}/fmu/out/vehicle_status",
            self.status_callback,
            qos_profile
        )
        self.traj_sub = self.create_subscription(
            Trajectory2D,
            f"/{self.agent_id}/planning/trajectory",
            self.trajectory_callback,
            trajectory_qos
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            f"/{self.agent_id}/fmu/in/offboard_control_mode",
            qos_profile
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            f"/{self.agent_id}/fmu/in/vehicle_command",
            qos_profile
        )
        self.setpoint_pub = self.create_publisher(
            PointStamped,
            f"/{self.agent_id}/control/setpoint",
            qos_profile
        )

        # Publicador: resto de trayectoria (solo visualización)
        self.remaining_traj_pub = self.create_publisher(
            Trajectory2D,
            f"/{self.agent_id}/planning/trajectory_remaining",
            trajectory_qos,
        )

        # Timer: 10 Hz bucle de control
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"Nodo de control iniciado para {self.agent_id}"
        )

    def position_callback(self, msg: PointStamped):
        self.position = msg

    def status_callback(self, msg: VehicleStatus):
        self.status = msg

    def trajectory_callback(self, msg: Trajectory2D):
        self.trajectory_points = list(msg.points)
        self.current_index = 0

        if msg.points is not None and len(msg.points) > 0:
            self.publish_remaining_trajectory()
            self.get_logger().info(
                f"{self.agent_id}: Nueva trayectoria recibida (len={len(self.trajectory_points)}). Reiniciando seguimiento"
            )
        else:
            self.get_logger().warn(f"Trayectoria vacia para el agente {self.agent_id}")
            self.remaining_traj_pub.publish(Trajectory2D())

    def publish_remaining_trajectory(self):
        if not self.trajectory_points:
            return
        start = max(0, min(int(self.current_index), len(self.trajectory_points)))
        out = Trajectory2D()
        # Re-publicar solo los puntos pendientes
        out.points = [
            Point2D(x=float(p.x), y=float(p.y))
            for p in self.trajectory_points[start:]
        ]
        self.remaining_traj_pub.publish(out)

    def get_px4_system_id(self, agent_id: str) -> int:
        # Asume formato "px4_X" -> sistema X
        try:
            return int(agent_id.split("_")[-1])
        except Exception:
            return 1

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.param3 = float(params.get("param3", 0.0))
        msg.param4 = float(params.get("param4", 0.0))
        msg.param5 = float(params.get("param5", 0.0))
        msg.param6 = float(params.get("param6", 0.0))
        msg.param7 = float(params.get("param7", 0.0))

        sys_id = self.get_px4_system_id(self.agent_id)
        msg.target_system = sys_id + 1
        msg.target_component = 1
        msg.source_system = sys_id + 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.vehicle_command_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = PointStamped()
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.setpoint_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info(f"{self.agent_id}: Enviando comando ARM")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info(f"{self.agent_id}: Cambiando a modo OFFBOARD")

    def control_loop(self):
        # Enviar heartbeat de modo OFFBOARD
        self.publish_offboard_control_mode()

        if self.position is None or self.status is None:
            return

        current_pos = self.position
        current_status = self.status

        # --- INIT ---
        if self.agent_state == "INIT":
            # Enviar setpoints iniciales en XY y altitud objetivo
            self.publish_trajectory_setpoint(
                current_pos.point.x, current_pos.point.y, self.target_altitude
            )
            self.setpoint_count += 1

            if self.setpoint_count > 10:
                if current_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.engage_offboard_mode()

                if current_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                    self.arm()

                if (
                    current_status.nav_state
                    == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    and current_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                ):
                    self.agent_state = "TAKEOFF"
                    self.get_logger().info(f"{self.agent_id}: Iniciando TAKEOFF")

        # --- TAKEOFF ---
        elif self.agent_state == "TAKEOFF":
            self.publish_trajectory_setpoint(
                current_pos.point.x, current_pos.point.y, self.target_altitude
            )

            # Comprobar altitud
            if current_pos.point.z >= (self.target_altitude - 0.5):
                self.agent_state = "MISSION"
                self.get_logger().info(
                    f"{self.agent_id}: Despegue completado. Iniciando MISION"
                )

        # --- MISSION ---
        elif self.agent_state == "MISSION":
            if not self.trajectory_points:
                # No hay planificación: hover
                self.publish_trajectory_setpoint(
                    current_pos.point.x, current_pos.point.y, self.target_altitude
                )
                return

            idx = self.current_index

            if idx >= len(self.trajectory_points):
                target_pos = self.trajectory_points[-1]
            else:
                target_pos = self.trajectory_points[idx]

                dx = float(target_pos.x) - float(current_pos.point.x)
                dy = float(target_pos.y) - float(current_pos.point.y)
                d = math.sqrt(dx * dx + dy * dy)

                if d < self.acceptance_radius:
                    self.current_index += 1
                    if self.current_index < len(self.trajectory_points):
                        self.get_logger().info(
                            f"{self.agent_id} ha llegado al punto {idx}. Moviendo a {self.current_index}"
                        )
                        target_pos = self.trajectory_points[self.current_index]

            self.publish_trajectory_setpoint(
                target_pos.x, target_pos.y, self.target_altitude
            )
            self.publish_remaining_trajectory()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


