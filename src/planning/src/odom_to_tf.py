#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_bridge')
        # 订阅 ackermann 里程计
        self.sub = self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odometry',
            self.cb,
            10
        )
        self.br = TransformBroadcaster(self)

    def cb(self, odom: Odometry):
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = odom.header.frame_id          # 例如 agent0_odom
        t.child_frame_id = odom.child_frame_id            # 例如 agent0_base_footprint
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(OdomToTF())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
