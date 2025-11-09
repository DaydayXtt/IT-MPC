#ifndef MPPI_H_
#define MPPI_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <cmath>
#include <iostream>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "config_reader.h"

namespace planning
{
    using namespace Eigen;

    struct ControlResult
    {
        VectorXd u0;
        MatrixXd u;
        MatrixXd optimal_traj;
        std::vector<MatrixXd> sampled_traj_list;
        bool arrived;
    };
    class MPPI
    {
    public:
        MPPI(int id, const MatrixXd &ref_traj);

        ControlResult calc_control_input(const VectorXd &observed_x);

    public:
        int K_;                    // 采样数量K（生成K条轨迹）

    private:
        MatrixXd _moving_average_filter(const MatrixXd &xx, int window_size);
        VectorXd _calc_weights(const VectorXd &S);
        double _terminal_cost(const VectorXd &x_T);
        double _stage_cost(const VectorXd &x_t);
        VectorXd _next_x(const VectorXd &x_t, const VectorXd &v_t);
        VectorXd _u_clamp(const VectorXd &u);
        std::vector<MatrixXd> _calc_epsilon(const MatrixXd &sigma, int K_, int T, int dim_u);
        std::tuple<int, double, double, double, double> _get_nearest_waypoint(
            double x, double y, bool update_prev_idx = false);
            
    private:
        int id_;                                    // 车辆ID(namespace)
        std::unique_ptr<ConfigReader> mppi_config_; // 参数配置

        // controller参数
        int dim_x_;                // 状态维度 [x,y,yaw,v]
        int dim_u_;                // 控制维度 [steer,accel]
        int T_;                    // 预测时域长度（步数）
        double param_exploration_; // 探索率（0~1，比例越高探索越强）
        double param_lambda_;      // 温度参数（控制权重分布陡峭度）
        double param_alpha_;       // 衰减因子（控制历史信息保留程度）
        double param_gamma_;
        // double sigma1_;            // 控制噪声协方差1
        // double sigma2_;            // 控制噪声协方差2
        MatrixXd sigma_; // 控制噪声协方差矩阵Σ: BB^T（控制探索强度）
        // double state_cost_xy = 50.0_;
        // double state_cost_yaw = 1.0_;
        // double state_cost_v = 20.0_;
        VectorXd stage_cost_weight_; // 阶段成本权重矩阵
        VectorXd input_cost_weight_; // 输入成本权重矩阵
        // double terminal_cost_xy = 50.0_;
        // double terminal_cost_yaw = 1.0_;
        // double terminal_cost_v = 20.0_;
        VectorXd terminal_cost_weight_; // 终端成本权重矩阵
        int max_search_idx_len_;        // 搜索最近参考点索引长度
        bool visualize_sampled_trajs_;  // 是否可视化采样轨迹
        bool visualize_optimal_traj_;   // 是否可视化最优轨迹

        // vehicle参数
        double delta_t_;    // 时间步长
        double wheel_base_; // 轴距
        double max_steer_;
        double max_accel_rate_;
        double max_speed_;

        MatrixXd u_prev; // T x dim_u
        int pre_waypoints_idx;
        int max_search_idx_len;

        // 参考路径
        MatrixXd ref_traj_;    // 参考轨迹，N x 4
        int ref_traj_end_idx_; // 参考轨迹终点索引

        // 随机数生成器
        std::default_random_engine generator;
    };
}
#endif // MPPI_H_