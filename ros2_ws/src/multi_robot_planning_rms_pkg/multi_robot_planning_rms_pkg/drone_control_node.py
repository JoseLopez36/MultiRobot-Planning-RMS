#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Importar mensajes de PX4 (px4_msgs)
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import GotoSetpoint
from px4_msgs.msg import VehicleCommand

class DroneControlNode(Node):

    def __init__(self):
        super().__init__('drone_control_node')

        # Log
        self.get_logger().info("Iniciando nodo de control de drone...")

        # Declarar perfil de QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Crear suscriptores
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)
        self.local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)

        # Crear publicadores
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        self.setpoint_pub = self.create_publisher(GotoSetpoint, '/fmu/in/goto_setpoint', qos_profile)
        
        # Obtener parámetros del archivo de configuración
        self.update_rate = self.declare_parameter('update_rate', 10.0).value
        self.horizontal_vel = self.declare_parameter('horizontal_vel', 6.0).value
        self.vertical_vel = self.declare_parameter('vertical_vel', 5.0).value
        self.heading_vel = self.declare_parameter('heading_vel', 1.5).value
        self.takeoff_altitude = self.declare_parameter('takeoff_altitude', 3.0).value       

        # Obtener parámetros de la trayectoria
        trajectory_t = self.declare_parameter('trajectory_t', [0.0]).value
        trajectory_x = self.declare_parameter('trajectory_x', [0.0]).value
        trajectory_y = self.declare_parameter('trajectory_y', [0.0]).value
        trajectory_z = self.declare_parameter('trajectory_z', [0.0]).value

        # Crear la trayectoria
        self.trajectory = []
        for i in range(len(trajectory_t)):
            self.trajectory.append({'t': trajectory_t[i], 'x': trajectory_x[i], 'y': trajectory_y[i], 'z': trajectory_z[i]})

        # Inicializar variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.position = None
        self.command_count = 0

        # Crear timer para la máquina de estados y para la actualización de la posición
        self.elapsed_time = 0.0
        self.last_update_time = self.get_clock().now().to_msg().sec
        self.state_timer = self.create_timer(1.0 / self.update_rate, self.update)
    
        # Log
        self.get_logger().info("Nodo de control de drone iniciado")
 
    def status_callback(self, msg):
        # Obtener estado de navegación
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"Nuevo estado de navegación: {msg.nav_state}")
            self.nav_state = msg.nav_state

    def position_callback(self, msg):
        # Convertir coordenadas de NED a ENU
        self.position = [msg.x, -msg.y, -msg.z]

    def update(self):
        # Actualizar tiempo transcurrido
        dt = self.get_clock().now().to_msg().sec - self.last_update_time
        self.elapsed_time += dt
        self.get_logger().info(f"Tiempo transcurrido: {self.elapsed_time}")
        self.last_update_time = self.get_clock().now().to_msg().sec

        # Verificar si se ha recibido una posición
        if self.position is None:
            self.get_logger().info("Esperando posición...")
            return

        # Publicar señal de heartbeat
        self.publish_heartbeat_signal()

        # Activar modo offboard y armar el drone si se han enviado 10 comandos
        if self.command_count == 10:
            self.handle_offboard()
            self.handle_arming()

        # Despegar si la altitud es menor que el 80% de la altitud de despegue y el estado de navegación es offboard
        if self.position[2] < 0.8 * self.takeoff_altitude and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_setpoint_command(self.position[0], self.position[1], self.takeoff_altitude)
            self.get_logger().info(f"Despegando... Altitud actual/deseada: {self.position[2]}/{self.takeoff_altitude}")

        # Publicar setpoint si el estado de navegación es offboard y existe un setpoint
        elif self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            setpoint = self.get_next_setpoint(self.elapsed_time)
            self.publish_setpoint_command(setpoint[0], setpoint[1], setpoint[2])
            self.get_logger().info(f"Navegando... Posición actual/deseada: {self.position}/{setpoint}")

        if self.command_count < 11:
            self.command_count += 1

    def handle_arming(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Comando de armado enviado")

    def handle_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Modo OFFBOARD activado")
        self.offboard_mode = True

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command       # ID del comando
        msg.target_system = 1       # sistema que debe ejecutar el comando
        msg.target_component = 1    # componente que debe ejecutar el comando, 0 para todos los componentes
        msg.source_system = 1       # sistema que envía el comando
        msg.source_component = 1    # componente que envía el comando
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # tiempo en microsegundos
        self.vehicle_command_pub.publish(msg)

    def publish_heartbeat_signal(self):
        # Publicar comando de control offboard (control de posición)
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)

    def get_next_setpoint(self, elapsed_time):
        # Obtener el siguiente setpoint de la trayectoria
        for point in self.trajectory:
            if point['t'] > elapsed_time:
                return [point['x'], point['y'], point['z']]
        return [0.0, 0.0, 7.0]

    def publish_setpoint_command(self, x, y, z):
        # Posición
        msg = GotoSetpoint()
        msg.position[0] = x
        msg.position[1] = -y
        msg.position[2] = -z
        # Restricciones de velocidad
        msg.flag_set_max_horizontal_speed = True
        msg.max_horizontal_speed = self.horizontal_vel
        msg.flag_set_max_vertical_speed = True
        msg.max_vertical_speed = self.vertical_vel
        # Publicar comando de posición
        self.setpoint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    drone_control_node = DroneControlNode()
    rclpy.spin(drone_control_node)
    drone_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()