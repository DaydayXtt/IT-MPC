#include "bicycle.h"
namespace agents_cpp
{
    BicycleNode::BicycleNode() : Node("bicycle_node")
    {
        this->ns_ = this->get_namespace();

        // 声明参数（名字，默认值）
        this->declare_parameter<double>("init_x", -15.0);
        this->declare_parameter<double>("init_y", 0.0);
        this->declare_parameter<double>("init_yaw", 0.0);
        this->declare_parameter<double>("init_v", 0.0);
        this->wheel_base_ = 2.5;
        this->vehicle_w_ = 2.0;
        this->vehicle_l_ = 2.0;
        this->lf_ = 3.25;
        this->lr_ = 0.75;
        this->tr_ = 0.4;
        this->tw_ = 0.4;
        this->wd_ = 1.5;
        
        // this->x_ = 0.0;
        // this->y_ = 0.0;
        // this->yaw_ = 0.0;
        // this->v_ = 0.0;
        // 获取参数
        x_  = this->get_parameter("init_x").as_double();
        y_  = this->get_parameter("init_y").as_double();
        yaw_ = this->get_parameter("init_yaw").as_double();
        v_ = this->get_parameter("init_v").as_double();
        RCLCPP_INFO(this->get_logger(), "init_x: %f", x_);
        
        this->max_steer_abs_ = 30.0 / 180.0 * M_PI; // 最大转向角
        this->max_accel_abs_ = 5.0;                 // 最大加速度
        this->max_vel_abs_ = 10.0;                   // 最大速度
        this->delta_t_ = 0.1;                       // 时间步长

        this->steer_ = 0.0;
        this->accel_ = 0.0;

        control_input_sub_ = this->create_subscription<Float64MultiArray>(
            "control_input", 10,
            std::bind(&BicycleNode::control_input_cb, this, _1));
        run_last_time_ = this->now();

        odom_pub_ = this->create_publisher<Odometry>("odom", 10);
        update_timer_ = this->create_wall_timer(10ms, std::bind(&BicycleNode::update_timer, this));
        RCLCPP_INFO(this->get_logger(), "bicycle_node");

        t_map_odom_ = TransformStamped();
        t_map_odom_.header.frame_id = "map";
        t_map_odom_.header.stamp = this->now();
        t_map_odom_.child_frame_id = this->ns_ + "_odom";
        t_map_odom_.transform.translation.x = 0.0; //
        t_map_odom_.transform.translation.y = 0.0; //

        tf_broadcaster_ = std::make_shared<TransformBroadcaster>(this);
        tf_broadcaster_->sendTransform(t_map_odom_);
    }
    BicycleNode::BicycleNode(double x, double y, double yaw, double v)
        : Node("bicycle_node"), x_(x), y_(y), yaw_(yaw), v_(v)
    {
        this->ns_ = this->get_namespace();
        this->wheel_base_ = 2.5;
        this->vehicle_w_ = 2.0;
        this->vehicle_l_ = 2.0;
        this->lf_ = 3.25;
        this->lr_ = 0.75;
        this->tr_ = 0.4;
        this->tw_ = 0.4;
        this->wd_ = 1.5;

        this->max_steer_abs_ = 30.0 / 180.0 * M_PI; // 最大转向角
        this->max_accel_abs_ = 2.0;                 // 最大加速度
        this->max_vel_abs_ = 1.0;                   // 最大速度
        this->delta_t_ = 0.1;                       // 时间步长

        this->steer_ = 0.0;
        this->accel_ = 0.0;

        control_input_sub_ = this->create_subscription<Float64MultiArray>(
            "control_input", 10,
            std::bind(&BicycleNode::control_input_cb, this, _1));
        run_last_time_ = this->now();

        odom_pub_ = this->create_publisher<Odometry>("odom", 10);
        update_timer_ = this->create_wall_timer(10ms, std::bind(&BicycleNode::update_timer, this));

        t_map_odom_ = TransformStamped();
        t_map_odom_.header.frame_id = "map";
        t_map_odom_.header.stamp = this->now();
        t_map_odom_.child_frame_id = this->ns_ + "_odom";
        t_map_odom_.transform.translation.x = 0.0; //
        t_map_odom_.transform.translation.y = 0.0; //
        tf_broadcaster_->sendTransform(t_map_odom_);
    }

    void BicycleNode::update_timer()
    {
        // RCLCPP_INFO(this->get_logger(), "update_timer");
        rclcpp::Time now = this->now();
        this->delta_t_ = (now - run_last_time_).seconds();
        if (this->delta_t_ <= 0)
        {
            run_last_time_ = now;
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "dt = %f", this->delta_t_);

        this->update(this->steer_, this->accel_);
        this->publish_odom(now);

        run_last_time_ = now;
    }

    void BicycleNode::publish_odom(rclcpp::Time now)
    {
        Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = this->ns_ + "_odom";
        odom.child_frame_id = this->ns_ + "_base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        // 创建一个四元数表示绕Z轴旋转yaw_角度的姿态
        // tf2::Vector3(0, 0, 1)指定旋转轴为Z轴
        // yaw_是绕Z轴的旋转角度
        // tf2::toMsg将tf2::Quaternion转换为ROS消息类型Quaternion
        Quaternion quat = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_));
        odom.pose.pose.orientation = quat;
        odom.twist.twist.linear.x = v_ * cos(yaw_);
        odom.twist.twist.linear.y = v_ * sin(yaw_);
        odom.twist.twist.angular.z = v_ / this->wheel_base_ * tan(steer_);
        odom_pub_->publish(odom);

        // TF 变换
        // map->odom
        t_map_odom_.header.stamp = now;
        tf_broadcaster_->sendTransform(t_map_odom_);

        // // odom->base_link
        TransformStamped t_odom_base_link;
        t_odom_base_link.header.stamp = now;
        t_odom_base_link.header.frame_id = this->ns_ + "_odom";
        t_odom_base_link.child_frame_id = this->ns_ + "_base_link";
        t_odom_base_link.transform.translation.x = this->x_;
        t_odom_base_link.transform.translation.y = this->y_;
        t_odom_base_link.transform.translation.z = 0.0;
        t_odom_base_link.transform.rotation = quat;
        tf_broadcaster_->sendTransform(t_odom_base_link);
    }

    void BicycleNode::update(double steer, double accel)
    {
        this->steer_ = std::clamp(steer, -this->max_steer_abs_, this->max_steer_abs_);
        this->accel_ = std::clamp(accel, -this->max_accel_abs_, this->max_accel_abs_);

        double delta_x = v_ * cos(yaw_) * delta_t_;
        double delta_y = v_ * sin(yaw_) * delta_t_;
        double delta_yaw = v_ / this->wheel_base_ * tan(steer_) * delta_t_;
        double delta_v = accel_ * delta_t_;

        x_ += delta_x;
        y_ += delta_y;
        yaw_ += delta_yaw;
        v_ += delta_v;
        v_ = std::clamp(v_, -this->max_vel_abs_, this->max_vel_abs_);
    }
    VectorXd BicycleNode::get_state()
    {
        Vector4d state;
        state << x_, y_, yaw_, v_;
        return state;
    }
    void BicycleNode::control_input_cb(const Float64MultiArray::SharedPtr msg)
    {
        if (odom_pub_ != nullptr && msg->data.size() == 2)
        {
            this->steer_ = msg->data[0];
            this->accel_ = msg->data[1];
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<agents_cpp::BicycleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
