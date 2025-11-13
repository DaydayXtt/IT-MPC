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
    rviz_file = os.path.join(pkg, 'rviz', 'display_car.rviz')
    ctrl_yaml = os.path.join(pkg, 'config', 'controllers.yaml')
    # 预设多台机器人在 map 下的初始位姿: (x0, y0, yaw0)
    # yaw 单位为弧度
    robots = {
        "agent0": (0.0, 1.0, 1.57),
        # "agent1": (3.0, 2.0, 1.57), 
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
        
        # 1. 拿出三个顶层块
        cm_block = cfg.pop("controller_manager")              # controller_manager:
        # jsb_block = cfg.pop("joint_state_broadcaster")        # joint_state_broadcaster:
        # ak_block = cfg.pop("ackermann_steering_controller")   # ackermann_steering_controller:

        # 2. 取出 controller_manager 自己的 ros__parameters
        # cm_params = cm_block.setdefault("ros__parameters", {})

        # 3. 把两个控制器挂到 controller_manager.ros__parameters 下面
        #    结构变成：
        #    /agent0/controller_manager:
        #      ros__parameters:
        #        update_rate: ...
        #        use_sim_time: ...
        #        joint_state_broadcaster:
        #          type: ...
        #        ackermann_steering_controller:
        #          type: ...
        #          ...
        # cm_params["joint_state_broadcaster"] = jsb_block["ros__parameters"]
        # cm_params["ackermann_steering_controller"] = ak_block["ros__parameters"]

        # 4. 把顶层 key 从 controller_manager 改名成 /agent0/controller_manager
        new_key = f"/{ns}/controller_manager"
        cfg[new_key] = cm_block
        # as_ = cm_params["ackermann_steering_controller"]
        as_ = cfg[new_key]['ros__parameters']['ackermann_steering_controller']
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
            # 注意 arguments 必须都是字符串
            arguments=[
                str(x0), str(y0), '0',      # translation: x y z
                str(yaw0), '0', '0',        # rotation ZYX
                'map',                      # parent frame
                f'{ns}_odom',               # child frame，例如 agent0_odom
            ],
            output='screen',
        )

        robot_description = Command(['xacro ', urdf_file, ' robot_namespace:=', prefix])

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
            parameters=[{'robot_description': robot_description},
                        tmp_yaml_name],
                        # ctrl_yaml],
            output='screen'
        )

        joint_state_broadcaster_spawner_node = Node(
            package="controller_manager",
            executable="spawner",
            # arguments=["joint_state_broadcaster"],
            arguments=["joint_state_broadcaster", 
                       "--controller-manager", f"/{ns}/controller_manager"],
            output="screen",
        )
        ackermann_steering_spawner = Node(
            package="controller_manager",
            executable="spawner", 
            # arguments=["ackermann_steering_controller"],
            arguments=[f"ackermann_steering_controller", 
                       "--controller-manager", f"/{ns}/controller_manager"],
            output="screen",
        )

        odom_to_tf_node = Node(
            package='agents_cpp',
            executable='odom_to_tf.py',
            output='screen'
        )
        # ld.add_action(static_tf)
        # ld.add_action(robot_state_publisher_node)
        # ld.add_action(control_node)
        # ld.add_action(joint_state_broadcaster_spawner_node)
        # ld.add_action(ackermann_steering_spawner)
        # ld.add_action(odom_to_tf_node)
        group = GroupAction([
            PushRosNamespace(ns),          # 之后的节点都在 /<ns>/ 命名空间里
            static_tf,
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster_spawner_node,
            ackermann_steering_spawner,
            odom_to_tf_node,
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
