#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import Quaternion
# from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_bridge')

        # 参数：世界坐标系、child 前缀、要映射的数量
        self.declare_parameter('world_frame_id', 'map')
        self.declare_parameter('child_frame_id', 'agent')

        self.world_frame_id = (
            self.get_parameter('world_frame_id').get_parameter_value().string_value
        )
        self.child_frame_id = (
            self.get_parameter('child_frame_id').get_parameter_value().string_value
        )
        # 可配置初始位姿
        # self.declare_parameter('init_x', 0.0)
        # self.declare_parameter('init_y', 0.0)
        # self.declare_parameter('init_yaw', 0.0)  # rad
        # self.x0 = float(self.get_parameter('init_x').value)
        # self.y0 = float(self.get_parameter('init_y').value)
        # self.yaw0 = float(self.get_parameter('init_yaw').value)
        
        # 预先算好初始四元数 q0
        # self.q0 = quaternion_from_euler(0.0, 0.0, self.yaw0)
        
        # 订阅 ackermann 里程计
        self.sub = self.create_subscription(
            Odometry,
            'ackermann_steering_controller/odometry',
            self.cb,
            10
        )
        self.br = TransformBroadcaster(self)

    def cb(self, odom: Odometry):
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        # t.header.frame_id = odom.header.frame_id          # 例如 agent0_odom
        t.header.frame_id = self.world_frame_id          # 例如 agent0_odom
        # t.child_frame_id = odom.child_frame_id            # 例如 agent0_base_footprint
        t.child_frame_id = self.child_frame_id            # 例如 agent0_base_footprint
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        # # 叠加完整 3D 姿态：q_total = q0 * q_odom
        # q = odom.pose.pose.orientation
        # q_odom = [q.x, q.y, q.z, q.w]
        # q_total = quaternion_multiply(self.q0, q_odom)

        # t.transform.rotation.x = q_total[0]
        # t.transform.rotation.y = q_total[1]
        # t.transform.rotation.z = q_total[2]
        # t.transform.rotation.w = q_total[3]
        
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(OdomToTF())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
