#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import sys

class PX4OdomBridge(Node):
    def __init__(self, px4_ns='px4_1'):
        super().__init__(f'{px4_ns}_odom_bridge')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        topic_in = f'/{px4_ns}/fmu/out/vehicle_odometry'
        topic_out = f'/{px4_ns}/odom'
        # Sufijo para el frame único
        if px4_ns.startswith('px4_'):
            suffix = px4_ns.split('_')[-1]
        else:
            suffix = px4_ns
        self.child_frame_id = f'base_link_{suffix}'
        self.pub = self.create_publisher(Odometry, topic_out, 10)
        self.sub = self.create_subscription(
            VehicleOdometry,
            topic_in,
            self.callback,
            qos_profile
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.last_pub_time = self.get_clock().now()

    def callback(self, msg):
        now = self.get_clock().now()
        # Publicar solo si ha pasado al menos 0.1s desde la última publicación
        if (now - self.last_pub_time).nanoseconds > 1e8:
            self.last_pub_time = now
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = self.child_frame_id
            odom.pose.pose.position.x = float(msg.position[0])
            odom.pose.pose.position.y = float(msg.position[1])
            odom.pose.pose.position.z = float(msg.position[2])
            odom.pose.pose.orientation.x = float(msg.q[0])
            odom.pose.pose.orientation.y = float(msg.q[1])
            odom.pose.pose.orientation.z = float(msg.q[2])
            odom.pose.pose.orientation.w = float(msg.q[3])
            self.pub.publish(odom)

            # Publicar la transformación TF odom->base_link_N
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = float(msg.position[0])
            t.transform.translation.y = float(msg.position[1])
            t.transform.translation.z = float(msg.position[2])
            t.transform.rotation.x = float(msg.q[0])
            t.transform.rotation.y = float(msg.q[1])
            t.transform.rotation.z = float(msg.q[2])
            t.transform.rotation.w = float(msg.q[3])
            self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    px4_ns = 'px4_1'
    if len(sys.argv) > 1:
        px4_ns = sys.argv[1]
    node = PX4OdomBridge(px4_ns)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
