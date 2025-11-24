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

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class GazeboPoseToTF(Node):
    def __init__(self):
        super().__init__('gazebo_pose_to_tf')

        # 参数：世界坐标系、child 前缀、要映射的数量
        self.declare_parameter('world_frame_id', 'map')
        self.declare_parameter('child_frame_prefix', 'agent')
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

    def pose_array_callback(self, msg: PoseArray):
        
        self.get_logger().info('---------- Reverived PoseArray message from ign!!! ----------')

        now = self.get_clock().now().to_msg()
        pose = msg.poses[self.pose_array_id]

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.world_frame_id
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)


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
