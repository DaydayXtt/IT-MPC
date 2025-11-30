#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unordered_map>

// 设置namespace的好处：防止和别的模块中的函数、变量命名冲突
namespace planning
{
    struct MPPIControllerStruct // MPPI控制器参数结构体
    {
        int dim_x_ = 4;                  // 状态维度 [x,y,yaw,v]
        int dim_u_ = 2;                  // 控制维度 [steer,accel]
        int T_ = 0;                      // 预测时域长度（步数）
        int K_ = 0;                      // 采样数量K（生成K条轨迹）
        double param_exploration_ = 0.0; // 探索率（0~1，比例越高探索越强）
        double param_lambda_ = 100.0;    // 温度参数（控制权重分布陡峭度）
        double param_alpha_ = 0.98;      // 衰减因子（控制历史信息保留程度）
        double param_gamma_ = param_lambda_ * (1.0 - param_alpha_);
        double sigma1_ = 0.075; // 控制噪声协方差1
        double sigma2_ = 2.0;   // 控制噪声协方差2
        // sigma_ = np.array([ [ 0.075, 0.0 ], [ 0.0, 2.0 ] ]);         // 控制噪声协方差矩阵Σ_ = BB^T（控制探索强度）
        double state_cost_xy_ = 50.0;
        double state_cost_yaw_ = 1.0;
        double state_cost_v_ = 20.0;
        // stage_cost_weight_ = np.array([ 50.0, 50.0, 1.0, 20.0 ]);    // 阶段成本权重矩阵
        double input_cost_ = 1.0; // 输入成本权重矩阵
        double terminal_cost_xy_ = 50.0;
        double terminal_cost_yaw_ = 1.0;
        double terminal_cost_v_ = 20.0;
        // terminal_cost_weight_ = np.array([ 50.0, 50.0, 1.0, 20.0 ]); // 终端成本权重矩阵
        int max_search_idx_len_ = 50;         // 搜索最近参考点索引长度
        bool visualize_sampled_trajs_ = true; // 是否可视化采样轨迹
        bool visualize_optimal_traj_ = true;  // 是否可视化最优轨迹
    };

    struct VehicleParamStruct // Vehicle
    {
        double delta_t_ = 0;         // 时间步长
        double wheel_base_ = 0;      // 轴距
        double max_steer_angle_ = 0; // z轴转向最大角度(角度)
        double max_accel_rate_ = 0;  // 最大加速度
        double max_speed_ = 0;       // 最大速度
    };

    class ConfigReader // 配置文件读取器
    {
    public:
        ConfigReader();

        void read_mppicontroller_config();
        inline MPPIControllerStruct mppicontroller() const { return mppicontroller_; } // const作用：不允许修改{}内的变量

        void read_vehicleparam_config();
        inline VehicleParamStruct vehicleparam() const { return vehicleparam_; } // const作用：不允许修改{}内的变量

    private:
        YAML::Node params_; // 配置文件数据，写成员函数的实现时用到

        MPPIControllerStruct mppicontroller_;
        VehicleParamStruct vehicleparam_;
    };
} // namespace planning
#endif // CONFIG_READER_H_
