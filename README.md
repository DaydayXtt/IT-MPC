# IT-MPC
Information-Theoretic Model Predictive Control 论文复现

## 参考：
[1] Williams G, Drews P, Goldfain B, et al. Information-theoretic model predictive control: Theory and applications to autonomous driving[J]. IEEE Transactions on Robotics, 2018, 34(6): 1603-1622.

## Py demo：
### Tracking:
``` shell
cd {workspace_folder}
python3 src/planning/src/test_demo/mppi/main_mppi.py
```
### Obstacle avoidance:
``` shell
cd {workspace_folder}
python3 src/planning/src/test_demo/avoid_collision/main.py
```

## C++ demo:
### 自定义Bicycle模型
``` shell
ros2 launch planning demo_mppi_tracking.launch.py
```


## 阿克曼模型
### rviz2 + ros2_control
``` shell
ros2 launch agents_cpp bringup_multiple.launch.py
```
- 如果是joint名称没有匹配，则可以启动controller，会有进一步的log提示。不会连 activate controller 都不行

### gazebo + ros2_control
``` shell
ros2 launch agents_cpp bringup_multiple_gazebo.launch.py
```

### 运动话题
- `/agentX/ackermann_steering_controller/reference`
``` shell
ros2 topic pub -r 20 /agentX/ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 3.0}, angular: {z: -1.0}}}"
```

- 打印话题`/tf`发现`ackermann_steering_controller`不会发布`odom`到`base_footprint`的变换，因为ros2_control将`odom`到`base_footprint`的变换发到了话题`ackermann_steering_controller/tf_odometry`上，所以需要加上一个remapping：
``` python
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
```
- 


## ToFix
1. rviz2中显示与gazebo不一致，gazebo中发生碰撞了但rviz2中没有
    - 现象1： `odom_to_tf_node` rviz2中Model0的base_link和odom相对关系是变化的，但是`ign_pose_tf_node`是不动的。
        - 进不去回调函数了
