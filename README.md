# IT-MPC
Information-Theoretic Model Predictive Control 论文复现

## 参考：
[1] Williams G, Drews P, Goldfain B, et al. Information-theoretic model predictive control: Theory and applications to autonomous driving[J]. IEEE Transactions on Robotics, 2018, 34(6): 1603-1622.

## 一、Py demo
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

## 二、C++ demo:
### 1. 自定义Bicycle模型
``` shell
ros2 launch planning demo_mppi_tracking.launch.py
```



### 2. 阿克曼模型
#### rviz2 + ros2_control
``` shell
ros2 launch agents_cpp bringup_multiple.launch.py
```
- 如果是joint名称没有匹配，则可以启动controller，会有进一步的log提示。不会连 activate controller 都不行

#### gazebo + ros2_control
``` shell
ros2 launch agents_cpp bringup_multiple_gazebo.launch.py
```

#### 运动话题
- `/agentX/ackermann_steering_controller/reference`
``` shell
ros2 topic pub -r 20 /agent0/ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.0}, angular: {z: 0.0}}}"
ros2 topic pub -r 20 /agent1/ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.5}, angular: {z: -0.3}}}"
```

## Debug:
1. 打印话题`/tf`发现`ackermann_steering_controller`不会发布`odom`到`base_footprint`的变换，因为ros2_control将`odom`到`base_footprint`的变换发到了话题`ackermann_steering_controller/tf_odometry`上，所以需要加上一个remapping：
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
  - `gazebo` 在 `xacro` 文件中修改
    ``` xml
    <gazebo>                    
      <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <ros>
          <namespace>${ns}</namespace>
          <remapping>ackermann_steering_controller/tf_odometry:=/tf</remapping>
        </ros>
        <parameters>${cy}</parameters>
      </plugin>
    </gazebo>
    ```

## TODO:

### 可视化
- [x] 可视化期望轨迹和最优轨迹
- [x] 可视化采样轨迹集合

### 模型
- [x] 建立 ros2_control 的阿克曼小车模型
- [x] 将odom的初始位置写成ros参数
- [x] 实现多个小车的的模型创建，通过namespace和group
- [x] gazebo显示，加入碰撞属性
### 算法
- [ ] 将MPPI算法应用到基于 ros2_control 的阿克曼小车模型
  - 能否近似，用自行车模型预测，控制量作用到阿克曼小车
- [ ] tracking有点稳态误差，再调试一下
- [ ] 写一个避障的launch文件
- [ ] GPU并行采样轨迹
- [ ] 基于维诺图求追捕点、基于MPPI避障




## ToFix

