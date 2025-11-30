#include "mppi.h"

namespace planning
{
    MPPI::MPPI(int id, const MatrixXd &ref_traj) : id_(id), ref_traj_(ref_traj)
    {
        mppi_config_ = std::make_unique<ConfigReader>();
        mppi_config_->read_mppicontroller_config();

        dim_x_ = mppi_config_->mppicontroller().dim_x_;
        dim_u_ = mppi_config_->mppicontroller().dim_u_;
        T_ = mppi_config_->mppicontroller().T_;
        K_ = mppi_config_->mppicontroller().K_;
        // std::cout <<  T_ << std::endl;
        param_exploration_ = mppi_config_->mppicontroller().param_exploration_;
        param_lambda_ = mppi_config_->mppicontroller().param_lambda_;
        param_alpha_ = mppi_config_->mppicontroller().param_alpha_;
        param_gamma_ = mppi_config_->mppicontroller().param_gamma_;

        double sigma1_ = mppi_config_->mppicontroller().sigma1_;
        double sigma2_ = mppi_config_->mppicontroller().sigma2_;
        sigma_ = MatrixXd::Zero(dim_u_, dim_u_);
        sigma_.diagonal() << sigma1_, sigma2_;
        std::cout << "sigma_ = " << std::endl << sigma_ << std::endl;


        double state_cost_xy_ = mppi_config_->mppicontroller().state_cost_xy_;
        double state_cost_yaw_ = mppi_config_->mppicontroller().state_cost_yaw_;
        double state_cost_v_ = mppi_config_->mppicontroller().state_cost_v_;
        stage_cost_weight_ = VectorXd::Zero(dim_x_);
        stage_cost_weight_ << state_cost_xy_, state_cost_xy_, state_cost_yaw_, state_cost_v_;

        double input_cost_ = mppi_config_->mppicontroller().input_cost_;
        input_cost_weight_ = VectorXd::Zero(dim_u_);
        input_cost_weight_ << input_cost_, input_cost_;

        double terminal_cost_xy_ = mppi_config_->mppicontroller().terminal_cost_xy_;
        double terminal_cost_yaw_ = mppi_config_->mppicontroller().terminal_cost_yaw_;
        double terminal_cost_v_ = mppi_config_->mppicontroller().terminal_cost_v_;
        terminal_cost_weight_ = VectorXd::Zero(dim_x_);
        terminal_cost_weight_ << terminal_cost_xy_, terminal_cost_xy_, terminal_cost_yaw_, terminal_cost_v_;

        max_search_idx_len_ = mppi_config_->mppicontroller().max_search_idx_len_;
        visualize_sampled_trajs_ = mppi_config_->mppicontroller().visualize_sampled_trajs_;
        visualize_optimal_traj_ = mppi_config_->mppicontroller().visualize_optimal_traj_;

        mppi_config_->read_vehicleparam_config();
        delta_t_ = mppi_config_->vehicleparam().delta_t_;
        wheel_base_ = mppi_config_->vehicleparam().wheel_base_;
        max_steer_ = mppi_config_->vehicleparam().max_steer_angle_ * M_PI / 180.0;
        max_accel_rate_ = mppi_config_->vehicleparam().max_accel_rate_;
        max_speed_ = mppi_config_->vehicleparam().max_speed_;

        // 初始化参考轨迹
        ref_traj_end_idx_ = ref_traj_.rows() - 1;

        // 初始化控制序列
        u_prev = MatrixXd::Zero(T_, dim_u_);
        pre_waypoints_idx = 0;

        // 初始化随机数生成器
        generator = std::default_random_engine(std::random_device{}());
    }

    ControlResult MPPI::calc_control_input(const VectorXd &observed_x)
    {
        MatrixXd u = u_prev;
        VectorXd x0 = observed_x;

        auto nearest_info = _get_nearest_waypoint(x0(0), x0(1), true);
        int nearest_idx = std::get<0>(nearest_info);

        // 判断是否到达终点区域
        bool arrived = (nearest_idx >= ref_traj_end_idx_ - 2);

        if (arrived)
        {
            std::cout << "Reached the end of the reference traj." << std::endl;
            u << 0.0, 0.0;
            return {u.row(0).transpose(), u, 
                    MatrixXd(),
                    std::vector<MatrixXd>(),
                    arrived};
        }

        // 初始化轨迹成本数组
        VectorXd S = VectorXd::Zero(K_);

        // 1. 采样噪声序列
        std::vector<MatrixXd> epsilon = _calc_epsilon(sigma_, K_, T_, dim_u_);

        // 初始化带噪声的控制序列
        std::vector<MatrixXd> v(K_, MatrixXd::Zero(T_, dim_u_));

        // 2. 生成K条采样轨迹并计算成本
        for (int k = 0; k < K_; ++k)
        {
            VectorXd x = x0;
            for (int t = 1; t <= T_; ++t)
            {
                // 根据探索率生成控制序列
                if (k < (1.0 - param_exploration_) * K_)
                {
                    v[k].row(t - 1) = u.row(t - 1) + epsilon[k].row(t - 1);
                }
                else
                {
                    v[k].row(t - 1) = epsilon[k].row(t - 1);
                }

                // 限制控制输入范围
                VectorXd v_clamped = _u_clamp(v[k].row(t - 1).transpose());
                // 更新状态
                x = _next_x(x, v_clamped);
                // 累积成本
                double input_cost = u.row(t-1).cwiseProduct(input_cost_weight_.transpose()).dot(u.row(t-1).transpose());


                // 计算探索成本项
                VectorXd u_t = u.row(t - 1).transpose();
                VectorXd v_kt = v[k].row(t - 1).transpose();
                double exploration_cost = param_gamma_ * u_t.dot(sigma_.llt().solve(v_kt));

                S(k) += _stage_cost(x) + input_cost + exploration_cost;
            }
            // 添加终端成本
            S(k) += _terminal_cost(x);
        }

        // 3. 计算每条轨迹的权重
        VectorXd w = _calc_weights(S);

        // 4. 加权更新控制序列
        MatrixXd w_epsilon = MatrixXd::Zero(T_, dim_u_);
        for (int t = 0; t < T_; ++t)
        {
            for (int k = 0; k < K_; ++k)
            {
                w_epsilon.row(t) += w(k) * epsilon[k].row(t);
            }
        }

        // 控制序列平滑
        w_epsilon = _moving_average_filter(w_epsilon, 10);
        u += w_epsilon;

        // 计算最优轨迹
        MatrixXd optimal_traj;
        if (visualize_optimal_traj_)
        {
            optimal_traj.resize(T_, dim_x_);
            VectorXd x = x0;
            for (int t = 0; t < T_; ++t)
            {
                x = _next_x(x, _u_clamp(u.row(t).transpose()));
                optimal_traj.row(t) = x.transpose();
            }
        }

        // 计算采样轨迹（按成本排序）
        std::vector<MatrixXd> sampled_traj_list;
        if (visualize_sampled_trajs_)
        {
            sampled_traj_list.resize(K_, MatrixXd::Zero(T_, dim_x_));

            // 获取排序索引
            std::vector<int> sorted_idx(K_);
            for (int i = 0; i < K_; ++i)
                sorted_idx[i] = i;
            std::sort(sorted_idx.begin(), sorted_idx.end(),
                      [&S](int i1, int i2)
                      { return S(i1) < S(i2); });

            for (int idx : sorted_idx)
            {
                VectorXd x = x0;
                for (int t = 0; t < T_; ++t)
                {
                    x = _next_x(x, _u_clamp(v[idx].row(t).transpose()));
                    sampled_traj_list[idx].row(t) = x.transpose();
                }
            }
        }

        // 5. 控制序列滚动
        u_prev.topRows(T_ - 1) = u.bottomRows(T_ - 1);
        u_prev.row(T_ - 1) = u.row(T_ - 1);

        return {u.row(0).transpose(), u, optimal_traj, sampled_traj_list, arrived};
    }
    MatrixXd MPPI::_moving_average_filter(const MatrixXd &xx, int window_size)
    {
        MatrixXd xx_mean = xx;
        int n = xx.rows();
        int dim = xx.cols();

        for (int d = 0; d < dim; ++d)
        {
            for (int i = 0; i < n; ++i)
            {
                double sum = 0.0;
                int count = 0;

                for (int j = -window_size / 2; j <= window_size / 2; ++j)
                {
                    int idx = i + j;
                    if (idx >= 0 && idx < n)
                    {
                        sum += xx(idx, d);
                        count++;
                    }
                }

                if (count > 0)
                {
                    xx_mean(i, d) = sum / count;
                }
            }
        }
        return xx_mean;
    }

    VectorXd MPPI::_calc_weights(const VectorXd &S)
    {
        double rho = S.minCoeff();
        double eta = 0.0;

        for (int k = 0; k < K_; ++k)
        {
            eta += std::exp((-1.0 / param_lambda_) * (S(k) - rho));
        }

        VectorXd w = VectorXd::Zero(K_);
        for (int k = 0; k < K_; ++k)
        {
            w(k) = (1.0 / eta) * std::exp((-1.0 / param_lambda_) * (S(k) - rho));
        }
        return w;
    }

    double MPPI::_terminal_cost(const VectorXd &x_T)
    {
        double x = x_T(0), y = x_T(1), yaw = x_T(2), v = x_T(3);
        yaw = std::fmod(yaw + 2.0 * M_PI, 2.0 * M_PI);

        auto nearest_info = _get_nearest_waypoint(x, y);
        double ref_x = std::get<1>(nearest_info);
        double ref_y = std::get<2>(nearest_info);
        double ref_yaw = std::get<3>(nearest_info);
        double ref_v = std::get<4>(nearest_info);

        double terminal_cost = terminal_cost_weight_(0) * std::pow(x - ref_x, 2) +
                               terminal_cost_weight_(1) * std::pow(y - ref_y, 2) +
                               terminal_cost_weight_(2) * std::pow(yaw - ref_yaw, 2) +
                               terminal_cost_weight_(3) * std::pow(v - ref_v, 2);

        return terminal_cost;
    }

    double MPPI::_stage_cost(const VectorXd &x_t)
    {
        double x = x_t(0), y = x_t(1), yaw = x_t(2), v = x_t(3);
        yaw = std::fmod(yaw + 2.0 * M_PI, 2.0 * M_PI);

        auto nearest_info = _get_nearest_waypoint(x, y);
        double ref_x = std::get<1>(nearest_info);
        double ref_y = std::get<2>(nearest_info);
        double ref_yaw = std::get<3>(nearest_info);
        double ref_v = std::get<4>(nearest_info);

        double stage_cost = stage_cost_weight_(0) * std::pow(x - ref_x, 2) +
                            stage_cost_weight_(1) * std::pow(y - ref_y, 2) +
                            stage_cost_weight_(2) * std::pow(yaw - ref_yaw, 2) +
                            stage_cost_weight_(3) * std::pow(v - ref_v, 2);

        return stage_cost;
    }

    VectorXd MPPI::_next_x(const VectorXd &x_t, const VectorXd &v_t)
    {
        double x = x_t(0), y = x_t(1), yaw = x_t(2), v = x_t(3);
        double steer = v_t(0), accel = v_t(1);

        double l = wheel_base_;
        double dt = delta_t_;

        double new_x = x + v * std::cos(yaw) * dt;
        double new_y = y + v * std::sin(yaw) * dt;
        double new_yaw = yaw + v / l * std::tan(steer) * dt;
        double new_v = v + accel * dt;
        new_v = std::clamp(new_v, -max_speed_, max_speed_);

        VectorXd new_state(4);
        new_state << new_x, new_y, new_yaw, new_v;
        return new_state;
    }

    VectorXd MPPI::_u_clamp(const VectorXd &u)
    {
        VectorXd result = u;
        result(0) = std::clamp(result(0), -max_steer_, max_steer_);
        result(1) = std::clamp(result(1), -max_accel_rate_, max_accel_rate_);
        return result;
    }

    std::vector<MatrixXd> MPPI::_calc_epsilon(const MatrixXd &sigma, int K_, int T, int dim_u)
    {
        std::vector<MatrixXd> epsilon;
        epsilon.reserve(K_);

        // 使用Cholesky分解生成相关噪声
        LLT<MatrixXd> lltOfSigma(sigma);
        MatrixXd L = lltOfSigma.matrixL();

        std::normal_distribution<double> distribution(0.0, 1.0);

        for (int k = 0; k < K_; ++k)
        {
            MatrixXd noise = MatrixXd::Zero(T, dim_u);
            for (int t = 0; t < T; ++t)
            {
                VectorXd z(dim_u);
                for (int d = 0; d < dim_u; ++d)
                {
                    z(d) = distribution(generator);
                }
                noise.row(t) = (L * z).transpose();
            }
            epsilon.push_back(noise);
        }
        return epsilon;
    }

    std::tuple<int, double, double, double, double> MPPI::_get_nearest_waypoint(
        double x, double y, bool update_prev_idx)
    {
        int prev_idx = pre_waypoints_idx;
        int search_end_idx = std::min(static_cast<int>(ref_traj_.rows()) - 1,
                                      prev_idx + max_search_idx_len);

        double min_d = std::numeric_limits<double>::max();
        int nearest_idx = prev_idx;

        for (int i = prev_idx; i < search_end_idx; ++i)
        {
            double dx = x - ref_traj_(i, 0);
            double dy = y - ref_traj_(i, 1);
            double d = dx * dx + dy * dy;

            if (d < min_d)
            {
                min_d = d;
                nearest_idx = i;
            }
        }

        double ref_x = ref_traj_(nearest_idx, 0);
        double ref_y = ref_traj_(nearest_idx, 1);
        double ref_yaw = ref_traj_(nearest_idx, 2);
        double ref_v = ref_traj_(nearest_idx, 3);

        if (update_prev_idx)
        {
            pre_waypoints_idx = nearest_idx;
        }

        return std::make_tuple(nearest_idx, ref_x, ref_y, ref_yaw, ref_v);
    }

}