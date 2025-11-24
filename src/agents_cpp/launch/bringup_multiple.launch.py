# ToFix: 读取不到 yaml 的字段
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os, yaml, tempfile

def generate_launch_description():
    pkg = get_package_share_directory('agents_cpp')
    urdf_file = os.path.join(pkg, 'urdf', 'car.urdf.xacro')
    rviz_file = os.path.join(pkg, 'rviz', 'multiple_cars.rviz')
    ctrl_yaml = os.path.join(pkg, 'config', 'controllers.yaml')
    
    # 预设多台机器人在 map 下的初始位姿: (x0, y0, yaw0)
    # yaw 单位为弧度
    robots = {
        "agent0": (0.0, 1.0, 1.57),
        "agent1": (3.0, 2.0, 3.14), 
        # "agent2": (… , … , …),
    }
    
    ld = LaunchDescription()
    for ns, (x0, y0, yaw0) in robots.items():
        
        # Launch 参数
        prefix = ns + '_'
        # 读取原始 YAML 并注入前缀
        with open(ctrl_yaml, 'r') as f:
            cfg = yaml.safe_load(f)
        def p(name: str) -> str:
            return name if name.startswith(prefix) else prefix + name
        
        # 修改配置文件中的相关namespace
        cm_block = cfg.pop("controller_manager")              # controller_manager:
        ak_block = cfg.pop("ackermann_steering_controller")   # ackermann_steering_controller:
        new_cm_key = f"/{ns}/controller_manager"
        new_asc_key = f"/{ns}/ackermann_steering_controller"
        cfg[new_cm_key] = cm_block
        cfg[new_asc_key] = ak_block
        
        as_ = cfg[new_asc_key]['ros__parameters']
        as_['front_wheels_names'] = [p(n) for n in as_['front_wheels_names']]
        as_['rear_wheels_names'] = [p(n) for n in as_['rear_wheels_names']]
        as_['base_frame_id'] = p(as_.get('base_frame_id', 'base_footprint'))
        as_['odom_frame_id'] = p(as_.get('odom_frame_id', 'odom'))
            
        # 写临时 YAML
        tmp_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.safe_dump(cfg, tmp_yaml)
        tmp_yaml.flush()
        tmp_yaml_name = tmp_yaml.name

        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{ns}_map_to_odom_static_tf',
            arguments=[
                str(x0), str(y0), '0',      # translation: x y z
                str(yaw0), '0', '0',        # rotation ZYX
                'map',                      # parent frame
                f'{ns}_odom',               # child frame，例如 agent0_odom
            ],
            output='screen',
        )

        robot_description = Command(['xacro ', urdf_file, 
                                     ' robot_namespace:=', ns,
                                     ' joint_prefix:=', prefix,
                                     ])

        # xacro -> robot_description
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                "use_sim_time": True,
                'robot_description': robot_description
            }],
            output='screen',
        )

        # 控制器管理器
        control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            # name='controller_manager',  # 这个节点没有name，加上就启动不了！
            parameters=[
                {'robot_description': robot_description},
                tmp_yaml_name,
                # ctrl_yaml,
                ],
            output='screen',
            remappings=[
                    ("ackermann_steering_controller/tf_odometry", "/tf"),
                ],
        )

        joint_state_broadcaster_spawner_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster", 
                "-c", f"/{ns}/controller_manager",
                ],
            output="screen",
        )
        ackermann_steering_spawner = Node(
            package="controller_manager",
            executable="spawner", 
            arguments=[
                "ackermann_steering_controller", 
                "-c", 
                f"/{ns}/controller_manager",
                ],
            output="screen",
        )

        odom_to_tf_node = Node(
            package='agents_cpp',
            executable='odom_to_tf.py',
            output='screen'
        )
        
        group = GroupAction([
            PushRosNamespace(ns),
            static_tf,
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster_spawner_node,
            ackermann_steering_spawner,
            # odom_to_tf_node,
        ])
        ld.add_action(group)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
    )
    ld.add_action(rviz_node)

    
    return ld
