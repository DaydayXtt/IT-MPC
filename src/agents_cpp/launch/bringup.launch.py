from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml, tempfile

def generate_launch_description():
    pkg = get_package_share_directory('agents_cpp')
    urdf_file = os.path.join(pkg, 'urdf', 'car.urdf.xacro')
    rviz_file = os.path.join(pkg, 'rviz', 'display_car.rviz')
    ctrl_yaml = os.path.join(pkg, 'config', 'controllers.yaml')

    # Launch 参数
    # default_ns = 'agent0'
    default_ns = ''
    namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value=default_ns,
        description='Namespace of the robot instance'
    )
    robot_ns = LaunchConfiguration('robot_namespace')
    prefix = default_ns + '_'
    # 读取原始 YAML 并注入前缀
    with open(ctrl_yaml, 'r') as f:
        cfg = yaml.safe_load(f)
    def p(name: str) -> str:
        return name if name.startswith(prefix) else prefix + name
    # diff_drive 控制器
    dd = cfg['diff_drive_controller']['ros__parameters']
    dd['left_wheel_names']  = [p(n) for n in dd['left_wheel_names']]
    print("========== diff_drive_controller: ", dd['left_wheel_names'], "================" )
    dd['right_wheel_names'] = [p(n) for n in dd['right_wheel_names']]
    dd['base_frame_id'] = p(dd.get('base_frame_id', 'base_footprint'))
    dd['odom_frame_id'] = p(dd.get('odom_frame_id', 'odom'))

    # bicycle_steering_controller
    # bs = cfg['bicycle_steering_controller']['ros__parameters']
    # bs['front_steering'] = p(bs['front_steering'])
    # bs['rear_wheels'] = [p(n) for n in bs['rear_wheels']]
    # bs['base_frame_id'] = p(bs.get('base_frame_id', 'base_footprint'))
    # bs['odom_frame_id'] = p(bs.get('odom_frame_id', 'odom'))
    
    # ackermann_steering_controller
    as_ = cfg['ackermann_steering_controller']['ros__parameters']
    as_['front_wheels_names'] = [p(n) for n in as_['front_wheels_names']]
    as_['rear_wheels_names'] = [p(n) for n in as_['rear_wheels_names']]
    as_['base_frame_id'] = p(as_.get('base_frame_id', 'base_footprint'))
    as_['odom_frame_id'] = p(as_.get('odom_frame_id', 'odom'))

    # 写临时 YAML
    tmp_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.safe_dump(cfg, tmp_yaml)
    tmp_yaml.flush()
    tmp_yaml_name = tmp_yaml.name

    robot_description = Command(['xacro ', urdf_file, ' robot_namespace:=', robot_ns, '_'])

    # xacro -> robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            "use_sim_time": True,
            # "use_robot_description_topic": True,
            'robot_description': robot_description
        }],
        # namespace=robot_ns,
        output='screen',
    )

    # 控制器管理器
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        # name='controller_manager',  # 这个节点没有name，加上就启动不了！
        parameters=[{'robot_description': robot_description},
                    tmp_yaml_name],
                    # ctrl_yaml],
        # namespace=robot_ns,
        output='screen'
    )

    joint_state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        # arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        # namespace=robot_ns,
        output="screen",
    )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        # arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        # namespace=robot_ns,
        output="screen",
    )
    
    # bicycle_steering_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["bicycle_steering_controller"],
    #     # arguments=["bicycle_steering_controller", "--controller-manager", "/controller_manager"],
    #     output="screen",
    # )

    ackermann_steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller"],
        # arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
        # namespace=robot_ns,
        output="screen",
    )

    odom_to_tf_node = Node(
        package='planning',
        executable='odom_to_tf.py',
        output='screen',
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
    )
    

    
    return LaunchDescription([
        namespace_arg,
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner_node,
        # diff_drive_spawner,
        # bicycle_steering_spawner,
        ackermann_steering_spawner,
        odom_to_tf_node,
        rviz_node,
    ])
