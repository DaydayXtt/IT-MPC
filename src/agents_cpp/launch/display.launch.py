from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 包路径
    pkg_path = get_package_share_directory('agents_cpp')
    urdf_file = os.path.join(pkg_path, 'urdf', 'car.urdf.xacro')

    # Launch 参数
    namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='demo_',
        description='Namespace of the robot instance'
    )

    # 将 xacro 转换为 urdf 的命令
    robot_description_config = Command(
        ['xacro ', urdf_file, ' robot_namespace:=', LaunchConfiguration('robot_namespace')]
    )

    # 参数形式传递给 robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # 加载 joint_state_publisher_gui（可拖动关节）
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    # 加载 joint_state_publisher_gui（可拖动关节）
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    # RViz2 可视化
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'display_car.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node
    ])
