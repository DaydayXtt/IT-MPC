#include "config_reader.h"

namespace planning
{
    ConfigReader::ConfigReader() // 配置文件读取器
    {
        // 获取 ${workspaceFolder}/install/planning/share/mpc/目录路径
        std::string planning_share_directory = ament_index_cpp::get_package_share_directory("planning");
        // 获取配置文件
        // std::cout << "dir: " << planning_share_directory + "/config/params_.yaml" << std::endl;
        params_ = YAML::LoadFile(planning_share_directory + "/config/params.yaml");
    }

    void ConfigReader::read_mppicontroller_config()
    {
        try // 捕获异常
        {
            mppicontroller_.dim_x_ = params_["mppicontroller"]["dim_x"].as<int>();
            mppicontroller_.dim_u_ = params_["mppicontroller"]["dim_u"].as<int>();
            mppicontroller_.T_ = params_["mppicontroller"]["T"].as<int>();
            mppicontroller_.K_ = params_["mppicontroller"]["number_of_samples"].as<int>();
            mppicontroller_.param_exploration_ = params_["mppicontroller"]["param_exploration"].as<double>();
            mppicontroller_.param_lambda_ = params_["mppicontroller"]["param_lambda"].as<double>();
            mppicontroller_.param_alpha_ = params_["mppicontroller"]["param_alpha"].as<double>();
            mppicontroller_.sigma1_ = params_["mppicontroller"]["sigma1"].as<double>();
            mppicontroller_.sigma2_ = params_["mppicontroller"]["sigma2"].as<double>();
            mppicontroller_.state_cost_xy_ = params_["mppicontroller"]["state_cost_xy"].as<double>();
            mppicontroller_.state_cost_yaw_ = params_["mppicontroller"]["state_cost_yaw"].as<double>();
            mppicontroller_.state_cost_v_ = params_["mppicontroller"]["state_cost_v"].as<double>();
            mppicontroller_.input_cost_ = params_["mppicontroller"]["input_cost"].as<double>();
            mppicontroller_.terminal_cost_xy_ = params_["mppicontroller"]["terminal_cost_xy"].as<double>();
            mppicontroller_.terminal_cost_yaw_ = params_["mppicontroller"]["terminal_cost_yaw"].as<double>();   
            mppicontroller_.terminal_cost_v_ = params_["mppicontroller"]["terminal_cost_v"].as<double>();
            mppicontroller_.max_search_idx_len_ = params_["mppicontroller"]["max_search_idx_len"].as<int>();
            mppicontroller_.visualize_sampled_trajs_ = params_["mppicontroller"]["visualize_sampled_trajs"].as<bool>();
            mppicontroller_.visualize_optimal_traj_ = params_["mppicontroller"]["visualize_optimal_traj"].as<bool>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load mppicontroller config: %s", e.what());
            // std::cerr << e.what() << '\n';
        }
    }
    void ConfigReader::read_vehicleparam_config()
    {
        try // 捕获异常
        {
            vehicleparam_.delta_t_ = params_["vehicleparams"]["delta_t"].as<double>();
            vehicleparam_.wheel_base_ = params_["vehicleparams"]["wheel_base"].as<double>();
            vehicleparam_.max_steer_angle_ = params_["vehicleparams"]["max_steer_angle"].as<double>();
            vehicleparam_.max_accel_rate_ = params_["vehicleparams"]["max_accel_rate"].as<double>();
            vehicleparam_.max_speed_ = params_["vehicleparams"]["max_speed"].as<double>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load vehicleparam config: %s", e.what());
            // std::cerr << e.what() << '\n';
        }
    }



}