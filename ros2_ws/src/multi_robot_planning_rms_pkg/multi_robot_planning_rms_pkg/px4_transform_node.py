#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleCommand
from geometry_msgs.msg import PointStamped, TransformStamped

class PX4TransformNode(Node):

    def __init__(self):
        super().__init__('px4_transform_node')

        # Log
        self.get_logger().info("Iniciando nodo de gestion de transformaciones de PX4...")

        # Obtener parámetros
        self.declare_parameter('agents.ids', [''])
        self.declare_parameter('agents.initial_positions_x', [0.0])
        self.declare_parameter('agents.initial_positions_y', [0.0])
        self.agents_ids = self.get_parameter('agents.ids').get_parameter_value().string_array_value
        self.initial_positions_x = self.get_parameter('agents.initial_positions_x').get_parameter_value().double_array_value
        self.initial_positions_y = self.get_parameter('agents.initial_positions_y').get_parameter_value().double_array_value

        # Crear diccionario de orígenes de marcos locales
        self.local_frame_origins = {}
        for i in range(len(self.agents_ids)):
            self.local_frame_origins[self.agents_ids[i]] = (self.initial_positions_x[i], self.initial_positions_y[i], 0.0)

        # Inicializar diccionarios
        self.position_px4_subscribers = {}
        self.command_px4_publishers = {}
        self.trajectory_setpoint_px4_publishers = {}
        self.trajectory_setpoint_ros2_subscribers = {}
        self.position_ros2_publishers = {}

        # Crear broadcast de transformaciones
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Iterar sobre los UAVs para crear suscriptores y publicadores
        for agent_id in self.agents_ids:
            # Crear suscriptores de PX4
            self.position_px4_subscribers[agent_id] = self.create_subscription(
                VehicleLocalPosition,
                f'/{agent_id}/fmu/out/vehicle_local_position',
                lambda msg, uid=agent_id: self.position_px4_callback(msg, uid),
                qos_profile
            )
            # Crear publicadores para PX4
            self.command_px4_publishers[agent_id] = self.create_publisher(
                VehicleCommand,
                f'/{agent_id}/fmu/in/vehicle_command',
                qos_profile
            )
            self.trajectory_setpoint_px4_publishers[agent_id] = self.create_publisher(
                TrajectorySetpoint,
                f'/{agent_id}/fmu/in/trajectory_setpoint',
                qos_profile
            )
            # Crear suscriptores de ROS2
            self.trajectory_setpoint_ros2_subscribers[agent_id] = self.create_subscription(
                PointStamped,
                f'/{agent_id}/control/setpoint',
                lambda msg, uid=agent_id: self.trajectory_setpoint_ros2_callback(msg, uid),
                qos_profile
            )
            # Crear publicadores para ROS2
            self.position_ros2_publishers[agent_id] = self.create_publisher(
                PointStamped,
                f'/{agent_id}/state/position',
                qos_profile
            )
            
            # Publicar marco local del UAV
            origin_x, origin_y, origin_z = self.local_frame_origins[agent_id]
            self.publish_local_frame(agent_id, origin_x, origin_y, origin_z)
        
        # Publicar marco global
        self.publish_global_frame()
    
        # Log
        self.get_logger().info("Nodo de gestion de transformaciones de PX4 iniciado")

    def position_px4_callback(self, msg, agent_id):
        # Validar que la posición sea válida
        if not (msg.xy_valid and msg.z_valid):
            return
        
        # Transformar de NED a ENU
        # NED: x=North, y=East, z=Down
        # ENU: x=East, y=North, z=Up
        local_ned_x = msg.x  # North
        local_ned_y = msg.y  # East
        local_ned_z = msg.z  # Down (negativo de altitud)
        
        # Transformación NED a ENU para posición
        local_enu_x = local_ned_y   # East
        local_enu_y = local_ned_x   # North
        local_enu_z = -local_ned_z  # Up
        
        # Calcular posicion global
        origin_enu_x, origin_enu_y, origin_enu_z = self.local_frame_origins[agent_id]
        global_enu_x = local_enu_x + origin_enu_x
        global_enu_y = local_enu_y + origin_enu_y
        global_enu_z = local_enu_z + origin_enu_z
        
        # Actualizar posición del UAV
        position_msg = PointStamped()
        position_msg.header.stamp = self.get_clock().now().to_msg()
        position_msg.header.frame_id = 'global'
        position_msg.point.x = global_enu_x
        position_msg.point.y = global_enu_y
        position_msg.point.z = global_enu_z
        
        # Publicar posición en ROS2
        self.position_ros2_publishers[agent_id].publish(position_msg)
        
        # Publicar marcos de transformación
        self.publish_body_frame(agent_id, local_enu_x, local_enu_y, local_enu_z)

    def trajectory_setpoint_ros2_callback(self, msg, agent_id):
        # Callback para recibir setpoints de ROS2 y transformarlos a PX4
        # Obtener coordenadas ENU (Globales)
        global_enu_x = msg.point.x
        global_enu_y = msg.point.y
        global_enu_z = msg.point.z
        
        # Transformar a local
        origin_enu_x, origin_enu_y, origin_enu_z = self.local_frame_origins[agent_id]
        local_enu_x = global_enu_x - origin_enu_x
        local_enu_y = global_enu_y - origin_enu_y
        local_enu_z = global_enu_z - origin_enu_z
            
        # Transformación ENU (Local) a NED
        local_ned_x = local_enu_y   # North
        local_ned_y = local_enu_x   # East
        local_ned_z = -local_enu_z  # Down
        
        # Crear mensaje TrajectorySetpoint para PX4
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microsegundos
        setpoint_msg.position[0] = local_ned_x
        setpoint_msg.position[1] = local_ned_y
        setpoint_msg.position[2] = local_ned_z
        setpoint_msg.yaw = float('nan')
        
        # Publicar setpoint a PX4
        self.trajectory_setpoint_px4_publishers[agent_id].publish(setpoint_msg)

    def publish_global_frame(self):
        # Publicar transformación del frame 'global' (origen)
        # Crear mensaje de transformación
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'global'
        
        # Establecer posición relativa al origen
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Establecer orientación (identidad)
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Publicar transformación
        self.static_broadcaster.sendTransform(t)
            
    def publish_local_frame(self, agent_id, x, y, z):
        # Publicar transformación del frame 'global' (origen) al frame 'local' (marco local del drone)
        # Crear mensaje de transformación
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'global'
        t.child_frame_id = f'{agent_id}/local'
        
        # Establecer posición relativa al origen
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        # Establecer orientación
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Publicar transformación
        self.static_broadcaster.sendTransform(t)      

    def publish_body_frame(self, agent_id, x, y, z):
        # Publicar transformación del frame 'local' al frame 'body' (posición y orientación del UAV)
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f'{agent_id}/local'
        t.child_frame_id = f'{agent_id}/body'
        
        # Establecer posición relativa al marco local
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        # Establecer orientación
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Publicar transformación dinámica
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    px4_transform_node = PX4TransformNode()
    rclpy.spin(px4_transform_node)
    px4_transform_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()