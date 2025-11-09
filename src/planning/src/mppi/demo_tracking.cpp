#include "demo_tracking.h"

namespace planning
{
    DemoMPPITracking::DemoMPPITracking() : Node("demo_mppi_tracking")
    {
        std::string ns = this->get_namespace();
        // RCLCPP_INFO(this->get_logger(), "ns: %s", ns.c_str());
        id_ = ns.back() - '0';
        RCLCPP_INFO(this->get_logger(), "id: %d", id_);
        mppi_reader_ = std::make_unique<ConfigReader>();
        mppi_reader_->read_mppicontroller_config();

        ref_traj_ = get_ref_traj(15.0, 100);
        // std::cout << ref_traj_ << std::endl;
        end_point_ = ref_traj_.row(ref_traj_.rows() - 1).head(2);
        current_state_ = VectorXd::Zero(4);

        mppi_ = std::make_unique<MPPI>(id_, ref_traj_);

        odom_sub_ = this->create_subscription<Odometry>(
            "odom", 10, std::bind(&DemoMPPITracking::odom_callback, this, _1));
        control_pub_ = this->create_publisher<Float64MultiArray>("control_input", 10);

        trajs_pub_ = this->create_publisher<MarkerArray>("trajs", 10);

        main_timer_ = this->create_wall_timer(
            100ms, std::bind(&DemoMPPITracking::run, this));
    }

    void DemoMPPITracking::run()
    {
        int rviz_id = 0;
        if (odom_msg_ == nullptr)
            return;
        trajs_.markers.clear();

        double dx = end_point_(0) - current_state_(0);
        double dy = end_point_(1) - current_state_(1);
        double distance_to_end = std::hypot(dx, dy);

        // RCLCPP_INFO(this->get_logger(), "End state = (%f, %f)",
        //             end_point_(0), end_point_(1));
        RCLCPP_INFO(this->get_logger(), "Current state = (%f, %f, %f, %f)",
                    current_state_(0), current_state_(1), current_state_(2), current_state_(3));
        auto tic = this->now();
        ControlResult res = mppi_->calc_control_input(current_state_);
        auto toc = this->now();
        auto duration = toc - tic;
        RCLCPP_INFO(this->get_logger(), "MPPI calculation time: %f ms",
                    duration.nanoseconds() / 1e6);
        if (res.arrived)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Agent %d Arrived successfully!, \n\t Distance to end: %f",
                        id_, distance_to_end);
        }
        VectorXd optimal_input = res.u0;
        publish_control_input(optimal_input);

        publish_ref_traj(ref_traj_, rviz_id);
        publish_optimal_traj(res.optimal_traj, rviz_id);
        publish_sample_trajs(res.sampled_traj_list, rviz_id);
    }

    void DemoMPPITracking::odom_callback(const Odometry::SharedPtr msg)
    {
        // 读取当前状态
        odom_msg_ = msg;
        Vector2d pos;
        pos << odom_msg_->pose.pose.position.x, odom_msg_->pose.pose.position.y;

        tf2::Quaternion tf_quat;
        tf2::fromMsg(msg->pose.pose.orientation, tf_quat); // 必须包含头文件
        double yaw = tf2::getYaw(tf_quat);
        double v = std::hypot(odom_msg_->twist.twist.linear.x, odom_msg_->twist.twist.linear.y);

        current_state_ << pos(0), pos(1), yaw, v;
    }

    void DemoMPPITracking::publish_control_input(const VectorXd &u)
    {
        Float64MultiArray msg;
        msg.data.resize(u.rows());
        for (int i = 0; i < u.rows(); ++i)
            msg.data[i] = u(i);
        control_pub_->publish(msg);
    }

    void DemoMPPITracking::publish_ref_traj(const MatrixXd &ref_traj, int &rviz_id)
    {
        // RCLCPP_INFO(this->get_logger(), "Publishing reference trajectory...");
        int length = ref_traj.rows();
        // RCLCPP_INFO(this->get_logger(), "Number of points: %d", length);
        Marker ref_traj_marker;
        ref_traj_marker.header.frame_id = "map";
        ref_traj_marker.header.stamp = this->now();
        ref_traj_marker.ns = "ReferenceTrajectory";
        ref_traj_marker.id = rviz_id++;
        ref_traj_marker.type = Marker::LINE_STRIP;
        ref_traj_marker.action = Marker::ADD;
        ref_traj_marker.scale.x = 0.1;
        ref_traj_marker.color.r = 1.0;
        ref_traj_marker.color.g = 0.0;
        ref_traj_marker.color.b = 0.0;
        ref_traj_marker.color.a = 1.0;
        ref_traj_marker.frame_locked = true;
        ref_traj_marker.lifetime = rclcpp::Duration(0, 0);
        for (int i = 0; i < length; ++i)
        {
            Point p;
            p.x = ref_traj(i, 0);
            // RCLCPP_INFO(this->get_logger(), "x: %f", p.x);
            p.y = ref_traj(i, 1);
            // RCLCPP_INFO(this->get_logger(), "y: %f", p.y);
            p.z = 0.0;
            ref_traj_marker.points.emplace_back(p);
        }
        trajs_.markers.emplace_back(ref_traj_marker);
        trajs_pub_->publish(trajs_);
    }

    void DemoMPPITracking::publish_optimal_traj(const MatrixXd &optimal_traj, int &rviz_id)
    {
        // if (optimal_traj.rows() == 0)
        // {
        //     return;
        // }
        // RCLCPP_INFO(this->get_logger(), "Publishing reference trajectory...");
        int length = optimal_traj.rows();
        // RCLCPP_INFO(this->get_logger(), "Number of points: %d", length);
        Marker optimal_traj_marker;
        optimal_traj_marker.header.frame_id = "map";
        optimal_traj_marker.header.stamp = this->now();
        optimal_traj_marker.ns = "OptimalTrajectory";
        optimal_traj_marker.id = rviz_id++;
        optimal_traj_marker.type = Marker::LINE_STRIP;
        optimal_traj_marker.action = Marker::ADD;
        optimal_traj_marker.scale.x = 0.1;
        optimal_traj_marker.color.r = 0.0;
        optimal_traj_marker.color.g = 1.0;
        optimal_traj_marker.color.b = 0.0;
        optimal_traj_marker.color.a = 1.0;
        optimal_traj_marker.frame_locked = true;
        optimal_traj_marker.lifetime = rclcpp::Duration(0, 0);
        for (int i = 0; i < length; ++i)
        {
            Point p;
            p.x = optimal_traj(i, 0);
            // RCLCPP_INFO(this->get_logger(), "x: %f", p.x);
            p.y = optimal_traj(i, 1);
            // RCLCPP_INFO(this->get_logger(), "y: %f", p.y);
            p.z = 0.0;
            optimal_traj_marker.points.emplace_back(p);
        }
        trajs_.markers.emplace_back(optimal_traj_marker);
        trajs_pub_->publish(trajs_);
    }

    void DemoMPPITracking::publish_sample_trajs(const std::vector<MatrixXd> &sample_trajs, int &rviz_id)
    {
        int sample_num = 50;
        int traj_num = sample_trajs.size();
        // if (traj_num == 0)
        // {
        //     return;
        // }
        // RCLCPP_INFO(this->get_logger(), "traj_num: %d", traj_num);
        if (sample_num > traj_num)
        {
            for (int i = 0; i < traj_num; ++i)
            {
                auto traj = sample_trajs[i];
                int length = traj.rows();
                // RCLCPP_INFO(this->get_logger(), "Number of points: %d", length);
                Marker traj_marker;
                traj_marker.header.frame_id = "map";
                traj_marker.header.stamp = this->now();
                traj_marker.ns = "SampledTrajectories";
                traj_marker.id = rviz_id++;
                traj_marker.type = Marker::LINE_STRIP;
                traj_marker.action = Marker::ADD;
                traj_marker.scale.x = 0.1;
                traj_marker.color.r = 0.0;
                traj_marker.color.g = 0.0;
                traj_marker.color.b = 1.0;
                traj_marker.color.a = 0.2;
                traj_marker.frame_locked = true;
                traj_marker.lifetime = rclcpp::Duration(0, 0);
                for (int i = 0; i < length; ++i)
                {
                    Point p;
                    p.x = traj(i, 0);
                    // RCLCPP_INFO(this->get_logger(), "x: %f", p.x);
                    p.y = traj(i, 1);
                    // RCLCPP_INFO(this->get_logger(), "y: %f", p.y);
                    p.z = 0.0;
                    traj_marker.points.emplace_back(p);
                }
                trajs_.markers.emplace_back(traj_marker);
            }
        }
        else
        {
            // 均匀采样
            int step = traj_num / sample_num;
            for (int i = 0; i < sample_num; ++i)
            {
                int idx = i * step;
                auto traj = sample_trajs[idx];
                int length = traj.rows();
                // RCLCPP_INFO(this->get_logger(), "Number of points: %d", length);
                Marker traj_marker;
                traj_marker.header.frame_id = "map";
                traj_marker.header.stamp = this->now();
                traj_marker.ns = "SampledTrajectories";
                traj_marker.id = rviz_id++;
                traj_marker.type = Marker::LINE_STRIP;
                traj_marker.action = Marker::ADD;
                traj_marker.scale.x = 0.1;
                traj_marker.color.r = 0.0;
                traj_marker.color.g = 0.0;
                traj_marker.color.b = 1.0;
                traj_marker.color.a = 0.2;
                traj_marker.frame_locked = true;
                traj_marker.lifetime = rclcpp::Duration(0, 0);
                for (int i = 0; i < length; ++i)
                {
                    Point p;
                    p.x = traj(i, 0);
                    // RCLCPP_INFO(this->get_logger(), "x: %f", p.x);
                    p.y = traj(i, 1);
                    // RCLCPP_INFO(this->get_logger(), "y: %f", p.y);
                    p.z = 0.0;
                    traj_marker.points.emplace_back(p);
                }
                trajs_.markers.emplace_back(traj_marker);
            }
        }
        trajs_pub_->publish(trajs_);
    }

    MatrixXd DemoMPPITracking::get_ref_traj(double radius, int num_points) const
    {
        // 生成角度序列，从π到-π
        VectorXd theta = VectorXd::LinSpaced(num_points, M_PI, -M_PI);

        // 计算参考路径状态
        VectorXd ref_x = radius * theta.array().cos();
        VectorXd ref_y = radius * theta.array().sin();

        // 计算航向角（更精确的数值微分，模拟np.gradient）
        VectorXd ref_yaw(num_points);

        // 处理第一个点（前向差分）
        ref_yaw(0) = std::atan2(ref_y(1) - ref_y(0), ref_x(1) - ref_x(0));

        // 处理中间点（中心差分）
        for (int i = 1; i < num_points - 1; ++i)
        {
            double dx = ref_x(i + 1) - ref_x(i - 1);
            double dy = ref_y(i + 1) - ref_y(i - 1);
            ref_yaw(i) = std::atan2(dy, dx);
        }

        // 处理最后一个点（后向差分）
        ref_yaw(num_points - 1) = std::atan2(
            ref_y(num_points - 1) - ref_y(num_points - 2),
            ref_x(num_points - 1) - ref_x(num_points - 2));

        // 设置恒定速度
        VectorXd ref_v = VectorXd::Constant(num_points, 5.0);

        // 组合成矩阵
        MatrixXd ref_path(num_points, 4);
        ref_path.col(0) = ref_x;
        ref_path.col(1) = ref_y;
        ref_path.col(2) = ref_yaw;
        ref_path.col(3) = ref_v;

        return ref_path;
    }
}
int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    // auto node = std::make_shared<yjnr3::MPC_Node>();
    auto node = std::make_shared<planning::DemoMPPITracking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
