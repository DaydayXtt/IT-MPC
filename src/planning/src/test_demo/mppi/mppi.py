import numpy as np
import math

class MPPIControllerParams:
    def __init__(self):
        self.dim_x = 4                                                 # 状态维度 [x,y,yaw,v]
        self.dim_u = 2                                                 # 控制维度 [steer,accel]
        self.T = 20                                                    # 预测时域长度（步数）
        self.number_of_samples_K = 500                                 # 采样数量K（生成K条轨迹）
        self.param_exploration = 0.0                                   # 探索率（0~1，比例越高探索越强）
        self.param_lambda = 100.0                                      # 温度参数（控制权重分布陡峭度）
        self.param_alpha = 0.98                                        # 衰减因子（控制历史信息保留程度）
        self.sigma = np.array([[0.075, 0.0], [0.0, 2.0]])              # 控制噪声协方差矩阵Σ = BB^T（控制探索强度）
        self.stage_cost_weight = np.array([50.0, 50.0, 1.0, 20.0])     # 阶段成本权重矩阵
        self.input_cost_weight = 1.0                                   # 输入成本权重矩阵
        self.terminal_cost_weight = np.array([50.0, 50.0, 1.0, 20.0])  # 终端成本权重矩阵
        self.max_search_idx_len = 50                                   # 搜索最近参考点索引长度
        self.visualize_sampled_trajs = True                            # 是否可视化采样轨迹
        self.visualize_optimal_traj = True                             # 是否可视化最优轨迹


class MPPIController:
    def __init__(self, cparams, vparams, ref_path):
        # MPPI核心参数
        self.dim_x = cparams.dim_x
        self.dim_u = cparams.dim_u
        self.T = cparams.T
        self.K = cparams.number_of_samples_K
        self.param_exploration = cparams.param_exploration
        self.param_lambda = cparams.param_lambda
        self.param_alpha = cparams.param_alpha
        self.param_gamma = self.param_lambda * (1.0 - (self.param_alpha))
        self.sigma = cparams.sigma
        self.input_cost_weight = cparams.input_cost_weight
        self.stage_cost_weight = cparams.stage_cost_weight
        self.terminal_cost_weight = cparams.terminal_cost_weight
        self.visualize_sampled_trajs = cparams.visualize_sampled_trajs
        self.visualize_optimal_traj = cparams.visualize_optimal_traj

        self.u_prev = np.zeros((self.T, self.dim_u))
        self.pre_waypoints_idx = 0
        self.max_search_idx_len = cparams.max_search_idx_len
    
        # 车辆参数
        self.delta_t = vparams.delta_t * 2
        self.wheel_base = vparams.wheel_base
        self.max_steer_abs = vparams.max_steer_abs
        self.max_accel_abs = vparams.max_accel_abs

        # 参考路径信息
        self.ref_path = ref_path
        self.ref_path_end_idx = len(ref_path) - 1# 参考路径终点索引


    def calc_control_input(self, observed_x):
        """计算控制输入"""
        u = self.u_prev
        x0 = observed_x
        nearest_idx, _, _, _, _ = self._get_nearest_waypoint(x0[0], x0[1], update_prev_idx=True)
        
        # 判断是否到达终点区域（仅基于参考路径索引）
        arrived = nearest_idx >= self.ref_path_end_idx - 2# 允许终点前2个点的误差
        
        if arrived:
            print("Reached the end of the reference path.")
            return None, None, None, None, arrived
    
        # 初始化轨迹成本数组
        S = np.zeros((self.K))
        
        # 1. 采样噪声序列
        epsilon = self._calc_epsilon(self.sigma, self.K, self.T, self.dim_u)

        # 初始化带噪声的控制序列
        v = np.zeros((self.K, self.T, self.dim_u))

        # 2. 生成K条采样轨迹并计算成本
        for k in range(self.K):
            x = x0
            for t in range(1, self.T + 1):
                # 根据探索率生成控制序列
                if k < (1.0 - self.param_exploration) * self.K:
                    v[k, t-1] = u[t-1] + epsilon[k, t-1]
                else:
                    v[k, t-1] = epsilon[k, t-1]

                # 限制控制输入范围
                v_clamped = self._u_clamp(v[k, t-1])  
                # 更新状态
                x = self._next_x(x, v_clamped)
                # 累积成本
                S[k] += self._stage_cost(x) + self.input_cost_weight * np.linalg.norm(u[t-1])**2 + \
                    self.param_gamma * u[t-1].T @ np.linalg.inv(self.sigma) @ v[k, t-1]
            # 添加终端成本
            S[k] += self._terminal_cost(x)

        # 3. 计算每条轨迹的权重
        w = self._calc_weights(S)

        # 4. 加权更新控制序列
        w_epsilon = np.zeros((self.T, self.dim_u))
        for t in range(0, self.T):
            for k in range(self.K):
                w_epsilon[t] += w[k] * epsilon[k, t]

        # 控制序列平滑
        w_epsilon = self._moving_average_filter(w_epsilon, 10)
        u += w_epsilon

        # 计算最优轨迹
        optimal_traj = np.zeros((self.T, self.dim_x))
        if self.visualize_optimal_traj:
            x = x0
            for t in range(0, self.T):
                x = self._next_x(x, self._u_clamp(u[t]))
                optimal_traj[t] = x

        # 计算采样轨迹（按成本排序）
        sampled_traj_list = np.zeros((self.K, self.T, self.dim_x))
        sorted_idx = np.argsort(S)  # 按成本升序排序
        if self.visualize_sampled_trajs:
            for k in sorted_idx:
                x = x0
                for t in range(0, self.T):
                    x = self._next_x(x, self._u_clamp(v[k, t]))
                    sampled_traj_list[k, t] = x

        # 5. 控制序列滚动
        self.u_prev[:-1] = u[1:]
        self.u_prev[-1] = u[-1]

        return u[0], u, optimal_traj, sampled_traj_list, arrived


    def _moving_average_filter(self, xx, window_size):
        """移动平均滤波器，平滑控制序列"""
        b = np.ones(window_size) / window_size
        dim = xx.shape[1]
        xx_mean = np.zeros(xx.shape)

        for d in range(dim):
            xx_mean[:, d] = np.convolve(xx[:, d], b, mode="same")
            n_conv = math.ceil(window_size / 2)
            xx_mean[0, d] *= window_size / n_conv
            for i in range(1, n_conv):
                xx_mean[i, d] *= window_size / (i + n_conv)
                xx_mean[-i, d] *= window_size / (i + n_conv - (window_size % 2)) 
        return xx_mean


    def _calc_weights(self, S):
        """计算各采样轨迹的权重"""
        rho = S.min() # 处于计算的稳定性考虑，防止数值上下溢出，且符合指数运算定律保证结果不变
        eta = 0.0
        for k in range(self.K):
            eta += np.exp((-1.0/self.param_lambda) * (S[k] - rho))
        
        w = np.zeros((self.K))
        for k in range(self.K):
            w[k] = (1.0 / eta) * np.exp((-1.0/self.param_lambda) * (S[k] - rho))
        return w


    def _terminal_cost(self, x_T):
        """计算终端成本"""
        x, y, yaw, v = x_T
        yaw = ((yaw + 2.0 * np.pi) % (2.0 * np.pi))

        _, ref_x, ref_y, ref_yaw, ref_v = self._get_nearest_waypoint(x, y)
        terminal_cost = self.terminal_cost_weight[0] * (x - ref_x)**2 + \
                        self.terminal_cost_weight[1] * (y - ref_y)** 2 + \
                        self.terminal_cost_weight[2] * (yaw - ref_yaw)**2 + \
                        self.terminal_cost_weight[3] * (v - ref_v)**2
        
        return terminal_cost
    

    def _stage_cost(self, x_t):
        """计算阶段成本"""
        x, y, yaw, v = x_t
        yaw = ((yaw + 2.0 * np.pi) % (2.0 * np.pi))

        _, ref_x, ref_y, ref_yaw, ref_v = self._get_nearest_waypoint(x, y)
        stage_cost = self.stage_cost_weight[0] * (x - ref_x)**2 + \
                     self.stage_cost_weight[1] * (y - ref_y)** 2 + \
                     self.stage_cost_weight[2] * (yaw - ref_yaw)**2 + \
                     self.stage_cost_weight[3] * (v - ref_v)** 2
        
        return stage_cost


    def _next_x(self, x_t, v_t):
        """计算下一时刻状态（运动学模型）"""
        x, y, yaw, v = x_t
        steer, accel = v_t

        l = self.wheel_base
        dt = self.delta_t

        new_x = x + v * np.cos(yaw) * dt
        new_y = y + v * np.sin(yaw) * dt
        new_yaw = yaw + v / l * np.tan(steer) * dt
        new_v = v + accel * dt

        return np.array([new_x, new_y, new_yaw, new_v])

                
    def _u_clamp(self, u):
        """限制控制输入在可行范围内"""
        u[0] = np.clip(u[0], -self.max_steer_abs, self.max_steer_abs)
        u[1] = np.clip(u[1], -self.max_accel_abs, self.max_accel_abs)
        return u


    def _calc_epsilon(self, sigma, K, T, dim_u):
        """生成高斯噪声序列"""
        mu = np.zeros((dim_u))
        epsilon = np.random.multivariate_normal(mu, sigma, (K, T))
        return epsilon
    

    def _get_nearest_waypoint(self, x, y, update_prev_idx=False):
        """查找最近的参考路径点"""
        prev_idx = self.pre_waypoints_idx
        search_end_idx = min(self.ref_path.shape[0] - 1, prev_idx + self.max_search_idx_len)

        dx = [x - ref_x for ref_x in self.ref_path[prev_idx:search_end_idx, 0]]
        dy = [y - ref_y for ref_y in self.ref_path[prev_idx:search_end_idx, 1]]
        d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]
        min_d = min(d)
        nearest_idx = d.index(min_d) + prev_idx

        ref_x = self.ref_path[nearest_idx, 0]
        ref_y = self.ref_path[nearest_idx, 1]
        ref_yaw = self.ref_path[nearest_idx, 2]
        ref_v = self.ref_path[nearest_idx, 3]

        if update_prev_idx:
            self.pre_waypoints_idx = nearest_idx

        return nearest_idx, ref_x, ref_y, ref_yaw, ref_v 
