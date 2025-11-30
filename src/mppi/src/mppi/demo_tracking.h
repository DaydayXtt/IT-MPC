#ifndef DEMO_MPPI_TRACKING_H_
#define DEMO_MPPI_TRACKING_H_

#include <iostream>
#include <cmath>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h> // 包含 getYaw 函数
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 转换函数头文件
#include "mppi.h"
#include "config_reader.h"

namespace planning
{
  using namespace Eigen;
  using namespace std::chrono_literals;
  using geometry_msgs::msg::Point;
  using geometry_msgs::msg::Quaternion;
  using nav_msgs::msg::Odometry;
  using std::placeholders::_1;
  using std_msgs::msg::Float64MultiArray;
  using visualization_msgs::msg::Marker;
  using visualization_msgs::msg::MarkerArray;
  class DemoMPPITracking : public rclcpp::Node
  {
  public:
    DemoMPPITracking();
    void run();

    MatrixXd get_ref_traj(double radius = 15.0, int num_points = 100) const;

  private:
    void odom_callback(const Odometry::SharedPtr msg);

    void publish_control_input(const VectorXd &u);

    void publish_ref_traj(const MatrixXd &ref_traj, int &rviz_id);
    void publish_optimal_traj(const MatrixXd &optimal_traj, int &rviz_id);
    void publish_sample_trajs(const std::vector<MatrixXd> &sample_trajs, int &rviz_id);

  private:
    int id_;
    std::unique_ptr<MPPI> mppi_;
    std::unique_ptr<ConfigReader> mppi_reader_;

    MatrixXd ref_traj_;
    VectorXd end_point_;
    VectorXd current_state_;

    // 订阅odom
    Odometry::SharedPtr odom_msg_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    // 发布控制指令
    rclcpp::Publisher<Float64MultiArray>::SharedPtr control_pub_;

    // 发布最优轨迹 (rviz2)
    MarkerArray trajs_;
    rclcpp::Publisher<MarkerArray>::SharedPtr trajs_pub_;

    // 主流程定时器
    rclcpp::TimerBase::SharedPtr main_timer_;
  };
}

#endif