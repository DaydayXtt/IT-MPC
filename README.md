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


## 阿克曼模型 `agents_cpp`
### 启动机器人模型
``` shell
ros2 launch agents_cpp bringup.launch.py
```
- 如果是joint名称没有匹配，则可以启动controller，会有进一步的log。不会连 activate controller 都不行

### ackermann
``` shell
ros2 topic pub -r 20 /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.9}, angular: {z: -0.1}}}"
```
- 打印话题`/tf`发现`ackermann_steering_controller`不会发布`odom`到`base_footprint`的变换，所以需要自己写一个小节点`odom_to_tf.py`，在`bringup`的文件中一起启动。

### 多机器人：
``` shell
ros2 launch agents_cpp bringup_ns.launch.py
ros2 topic pub -r 20 /agent0/ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.9}, angular: {z: -0.1}}}"
```

## ToFix
1. 加入namespace的ros2_control存在读取配置文件字段读不到的问题。


