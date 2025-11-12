#!/usr/bin/env python
"""
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
import yaml


def generate_launch_description():
    ld = LaunchDescription()
    # 获取URDF文件路径
    pkg_path = get_package_share_directory('planning')
    urdf_path = get_package_share_directory('agents_cpp')
    urdf_file = os.path.join(urdf_path, 'urdf', 'robot.urdf.xacro')
    # urdf_file = os.path.join(urdf_path, 'urdf', 'car.urdf.xacro')
    
    demo_tracking_config = os.path.join(
            pkg_path, 'config', 'demo_tracking_config.yaml'
        )

    rviz2 = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(pkg_path, 'rviz/0.rviz')]],
            # output='screen',
            # shell=True,
        )
    ld.add_action(rviz2)

    # 使用命名空间参数
    from launch_ros.parameter_descriptions import ParameterValue
    from launch.substitutions import Command
    
    
    # 2. 创建若干 agent 的命名空间
    # agent_namespaces = ["agent0", "agent1", "agent2", "agent3", "agent4", "agent5", "agent6", "agent7", "agent8", "agent9"]
    agent_namespaces = ["agent0"]
    for agent_namespace in agent_namespaces:
        # urdf前缀
        agent_description = ParameterValue(
            Command(['xacro ', urdf_file, ' robot_namespace:=', agent_namespace]),
            value_type=str
        )
        
        # 1. odom积分器
        # 启动robot_state_publisher
        odom_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher', 
            namespace=agent_namespace,
            parameters=[{'robot_description': agent_description
                        }],
            output='screen',
        )
        # 加载 joint_state_publisher_gui（可拖动关节）
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=agent_namespace,
            output='screen'
        )
        odom_int = Node(
            package='agents_cpp',
            executable='odom_bicycle',
            namespace=agent_namespace,
            name='odom_bicycle',
            parameters=[demo_tracking_config],
            output='screen',
        )
        odom_int_action = TimerAction(
            period=0.0,
            actions=[odom_state_publisher, joint_state_publisher_node, odom_int]
            # actions=[odom_state_publisher, joint_state_publisher_node]
        )
        ld.add_action(odom_int_action)
        
        # 2. 规划节点 - ros2 run 
        planner = Node(
            package='planning',
            executable='demo_mppi_tracking_node',
            name='demo_mppi_tracking_node',
            namespace=agent_namespace,
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug']
        )
        planner_action = TimerAction(
            period=3.0,  # 时间要久一点，不然先发布轨迹rviz2还没顺利启动
            actions=[planner]
        )
        ld.add_action(planner_action)
        

    return ld