# ToFix: 读取不到 yaml 的字段
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
import os, yaml, tempfile

def generate_launch_description():
    pkg = get_package_share_directory('agents_cpp')
    ros_gz_sim_pkg = get_package_share_path('ros_gz_sim')
    
    urdf_file = os.path.join(pkg, 'urdf', 'car.gz_ros2_control.urdf.xacro')
    rviz_file = os.path.join(pkg, 'rviz', 'multiple_cars.rviz')
    ctrl_yaml = os.path.join(pkg, 'config', 'controllers_gz.yaml')
    gazebo_config_path = os.path.join(pkg, 'config', 'gazebo_bridge.yaml')
    # 预设多台机器人在 map 下的初始位姿: (x0, y0, yaw0)
    # yaw 单位为弧度
    robots = {
        "agent0": {"pose": (0.0, 1.0, 1.57), "ign_id": 2},
        "agent1": {"pose": (3.0, 2.0, 3.14), "ign_id": 1},
        # "agent2": {"pose": (...), "tag_id": ...},
    }
    
    ld = LaunchDescription()
    # ① 启动 Gazebo 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_sim_pkg / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gazebo_config_path}],
        output='screen'
    )
    ld.add_action(gz_sim)
    ld.add_action(gz_bridge)
    
    for ns, info in robots.items():
        x0, y0, yaw0 = info["pose"]
        ign_id = info["ign_id"]
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
                # '--x', str(x0),
                '--x', '0',
                # '--y', str(y0),
                '--y', '0',
                '--z', '0',
                # '--yaw', str(yaw0),
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'map',
                '--child-frame-id', f'{ns}_odom',
            ],
            output='screen',
        )

        robot_description = Command(['xacro ', urdf_file, 
                                     ' robot_namespace:=', ns, 
                                     ' joint_prefix:=', prefix, 
                                     ' controllers_yaml:=', tmp_yaml_name,
                                     ])

        # xacro -> robot_description
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                "use_sim_time": True,
                # "use_robot_description_topic": True,
                'robot_description': robot_description,
            }],
            output='screen',
        )
        # ⑥ Gazebo 中生成该机器人模型
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', ns,                          # Gazebo 里的 model name
                '-topic', f'/{ns}/robot_description', # 从 topic 拿 URDF（下面会讲这个问题）
                '-x', str(x0),
                '-y', str(y0),
                '-Y', str(yaw0),
            ],
            output='screen'
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
            output='screen'
        )

        joint_state_broadcaster_spawner_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster", 
                "-c", f"/{ns}/controller_manager",
                "--param-file", tmp_yaml_name,
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
                "--param-file", tmp_yaml_name,
                ],
            output="screen",
        )

        odom_to_tf_node = Node(
            package='agents_cpp',
            executable='odom_to_tf.py',
            output='screen',
            parameters=[
                {'world_frame_id':  f"{ns}_odom"},
                {'child_frame_id': f"{ns}_base_footprint"},
                # {'pose_array_id': ign_id},
            ]
        )
        ign_pose_tf_node = Node(
            package='agents_cpp',
            executable='ign_pose_to_tf.py',
            output='screen',
            parameters=[
                {'world_frame_id':  f"{ns}_odom"},
                # {'world_frame_id':  "map"},
                {'child_frame_id': f"{ns}_base_footprint"},
                {'pose_array_id': ign_id},
            ]
        )

        group = GroupAction([
            PushRosNamespace(ns),
            static_tf,
            robot_state_publisher_node,
            # control_node,
            joint_state_broadcaster_spawner_node,
            ackermann_steering_spawner,
            ign_pose_tf_node,
        ])
        ld.add_action(spawn_entity)
        ld.add_action(group)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
    )
    ld.add_action(rviz_node)

    
    return ld
