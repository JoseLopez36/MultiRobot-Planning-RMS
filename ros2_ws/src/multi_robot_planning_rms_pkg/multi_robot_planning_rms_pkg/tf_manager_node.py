#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Importar mensajes de ROS2
from nav_msgs.msg import Odometry

class TFManagerNode(Node):

    def __init__(self):
        super().__init__('tf_manager_node')

        # Log
        self.get_logger().info("Iniciando nodo de gestion de transformaciones...")

        # Crear suscriptores
        self.odometry_sub = self.create_subscription(Odometry, '/ground_truth/vehicle_odom', self.odometry_callback, 10)

        # Crear broadcast de transformaciones
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Obtener parámetros del archivo de configuración
        tf_update_rate = self.declare_parameter('tf_update_rate', 20.0).value
        self.beacon_ids = self.declare_parameter('beacons.ids', ['']).value

        # Declarar variables
        self.vehicle_pose = None
        self.local_frame_initialized = False

        # Publicar marco global
        self.publish_global_frame()

        # Publicar marcos de balizas
        for beacon_id in self.beacon_ids:
            position = self.declare_parameter(f'beacons.{beacon_id}.position', [0.0, 0.0, 0.0]).value
            frame_id = self.declare_parameter(f'beacons.{beacon_id}.frame_id', 'global').value
            self.publish_beacon_frame(beacon_id, position, frame_id)

        # Crear timer para la actualización de la transformaciones
        self.tf_timer = self.create_timer(1.0 / tf_update_rate, self.update)
    
        # Log
        self.get_logger().info("Nodo de gestion de transformaciones iniciado")

    def odometry_callback(self, msg):
        # Convertir coordenadas de NED a ENU
        self.vehicle_pose = msg.pose.pose

    def update(self):
        # Publicar transformación del frame 'global' (origen) al frame 'local' (marco local del drone)
        if not self.local_frame_initialized:
            self.publish_local_frame()
            self.local_frame_initialized = True

        # Publicar transformación del frame 'local' (marco local del drone) al frame 'body' (cuerpo del drone)
        if self.local_frame_initialized and self.vehicle_pose is not None:
            self.publish_body_frame()

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

    def publish_beacon_frame(self, beacon_id, position, frame_id):
        # Publicar transformación del frame 'global' (origen)
        # Crear mensaje de transformación
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = beacon_id
        
        # Establecer posición relativa al origen
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        
        # Establecer orientación
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Publicar transformación
        self.static_broadcaster.sendTransform(t)
            
    def publish_local_frame(self):
        # Publicar transformación del frame 'global' (origen) al frame 'local' (marco local del drone)
        # Crear mensaje de transformación
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'global'
        t.child_frame_id = 'local'
        
        # Establecer posición relativa al origen
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Establecer orientación
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Publicar transformación
        self.static_broadcaster.sendTransform(t)      

    def publish_body_frame(self):
        # Crear mensaje de transformación
        t = TransformStamped()
        
        # Establecer headers
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'local'
        t.child_frame_id = 'body'
        
        # Establecer posición relativa al origen (local)
        t.transform.translation.x = float(self.vehicle_pose.position.x)
        t.transform.translation.y = float(self.vehicle_pose.position.y)
        t.transform.translation.z = float(self.vehicle_pose.position.z)
        
        # Establecer orientación
        t.transform.rotation.w = float(self.vehicle_pose.orientation.w)
        t.transform.rotation.x = float(self.vehicle_pose.orientation.x)
        t.transform.rotation.y = float(self.vehicle_pose.orientation.y)
        t.transform.rotation.z = float(self.vehicle_pose.orientation.z)
        
        # Publicar transformación
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    tf_manager_node = TFManagerNode()
    rclpy.spin(tf_manager_node)
    tf_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()