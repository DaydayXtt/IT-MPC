#ifndef BICYCLE_H
#define BICYCLE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace agents_cpp
{
    using namespace std::chrono_literals;
    using namespace Eigen;
    using geometry_msgs::msg::Quaternion;
    using geometry_msgs::msg::TransformStamped;
    using nav_msgs::msg::Odometry;
    using std::placeholders::_1;
    using std_msgs::msg::Float64MultiArray;
    using tf2_ros::TransformBroadcaster;

    class BicycleNode : public rclcpp::Node
    {
    public:
        BicycleNode();
        BicycleNode(double x, double y, double yaw, double v);

        void update_timer();
        void publish_odom(rclcpp::Time now);

        void update(double steer, double accel);
        VectorXd get_state();

    public:
        double max_steer_abs_; // 最大转向角（弧度）
        double max_accel_abs_; // 最大加速度
        double max_vel_abs_;   // 最大速度
        double delta_t_;       // 时间步长

        double x_;     // x坐标
        double y_;     // y坐标
        double yaw_;   // 航向角（弧度）
        double v_;     // 速度
        double steer_; // 转向角（弧度）
        double accel_; // 加速度

    private:
        void control_input_cb(const Float64MultiArray::SharedPtr msg);

    private:
        std::string ns_;

        double wheel_base_; // 轴距
        double vehicle_w_;  // 车辆宽度
        double vehicle_l_;  // 车辆长度
        double lf_;         // 后轴距车头长度
        double lr_;         // 前轴距车尾长度
        double tr_;         // 车轮半径
        double tw_;         // 车轮宽度
        double wd_;         // 轮距

        // 订阅控制量 [steer, accel]
        rclcpp::Subscription<Float64MultiArray>::SharedPtr control_input_sub_;

        rclcpp::Publisher<Odometry>::SharedPtr odom_pub_; // 发布pose

        rclcpp::TimerBase::SharedPtr update_timer_; // 更新状态定时器

        rclcpp::Time run_last_time_;

        TransformStamped t_map_odom_;
        std::shared_ptr<TransformBroadcaster> tf_broadcaster_;
    };
}

#endif