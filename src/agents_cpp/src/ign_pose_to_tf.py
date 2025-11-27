#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# 将 Gazebo /world/empty/pose/info (PoseArray) 转成 TF：
# world -> agent0, world -> agent1, ...
#
# 注意：
#  1) PoseArray 里没有 name / id 信息，只是一个 pose 列表
#  2) 这里假设 PoseArray 中前 N 个 pose 分别对应 agent0, agent1, ...
#  3) 如果顺序变化，需要你根据实际情况调整映射逻辑

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class GazeboPoseToTF(Node):
    def __init__(self):
        super().__init__('gazebo_pose_to_tf')

        # 参数：世界坐标系、child 前缀、要映射的数量
        self.declare_parameter('world_frame_id', 'map')
        self.declare_parameter('child_frame_id', 'agent')
        self.declare_parameter('pose_array_id', 0)

        self.world_frame_id = (
            self.get_parameter('world_frame_id').get_parameter_value().string_value
        )
        self.child_frame_id = (
            self.get_parameter('child_frame_id').get_parameter_value().string_value
        )
        self.pose_array_id = (
            self.get_parameter('pose_array_id').get_parameter_value().integer_value
        )
        self.get_logger().info(
            f'GazeboPoseToTF started, world_frame_id={self.world_frame_id}, '
            f'child_frame_id={self.child_frame_id}, '
            f'pose_id={self.pose_array_id}. '
        )

        self.gazebo_odom_ = Odometry()
        self.gazebo_pose = Pose()
        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅 PoseArray
        # 话题名和 world 名要和你当前世界一致（这里是 empty）
        topic_name = '/world/empty/pose/info'
        self.subscription = self.create_subscription(
            PoseArray,
            topic_name,
            self.pose_array_callback,
            10
        )
        self.ign_odom_puber_ = self.create_publisher(Odometry, 'ign_odom', 10)
        
        self.ackermann_odom_sub = self.create_subscription(
            Odometry,
            'ackermann_steering_controller/odometry',
            self.ackermann_pose_callback,
            10
        )

    def pose_array_callback(self, msg: PoseArray):
        # self.get_logger().info('---------- Received PoseArray message from ign!!! ----------')
        self.gazebo_pose = msg.poses[self.pose_array_id]
        # self.get_logger().info(f'Gazebo pose {self.pose_array_id}: {self.gazebo_pose}')

    def ackermann_pose_callback(self, odom: Odometry):
        if self.gazebo_pose.position.x == 0 and self.gazebo_pose.position.y == 0 and self.gazebo_pose.position.z == 0:
            return
        # self.get_logger().info('---------- Received Ackermann Odometry message from ign!!! ----------')
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        # t.header.frame_id = odom.header.frame_id          # 例如 agent0_odom
        t.header.frame_id = self.world_frame_id          # 例如 agent0_odom
        # t.child_frame_id = odom.child_frame_id            # 例如 agent0_base_footprint
        t.child_frame_id = self.child_frame_id            # 例如 agent0_base_footprint
        t.transform.translation.x = self.gazebo_pose.position.x
        t.transform.translation.y = self.gazebo_pose.position.y
        t.transform.translation.z = self.gazebo_pose.position.z
        t.transform.rotation = self.gazebo_pose.orientation
        # # 叠加完整 3D 姿态：q_total = q0 * q_odom
        # q = odom.pose.pose.orientation
        # q_odom = [q.x, q.y, q.z, q.w]
        # q_total = quaternion_multiply(self.q0, q_odom)

        # t.transform.rotation.x = q_total[0]
        # t.transform.rotation.y = q_total[1]
        # t.transform.rotation.z = q_total[2]
        # t.transform.rotation.w = q_total[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # 发布odom
        self.gazebo_odom_.header = odom.header
        self.gazebo_odom_.child_frame_id = self.child_frame_id
        self.gazebo_odom_.pose.pose = self.gazebo_pose
        self.gazebo_odom_.twist.twist = odom.twist.twist
        self.ign_odom_puber_.publish(self.gazebo_odom_)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
