import numpy as np


class VehicleParams:
    def __init__(self):
        self.wheel_base = 2.5           # 轴距
        self.max_steer_abs = np.deg2rad(30.0)  # 最大转向角（弧度）
        self.max_accel_abs = 2.0        # 最大加速度
        self.delta_t = 0.1              # 时间步长
        self.vehicle_w = 2.0            # 车辆宽度
        self.vehicle_l = 4.0            # 车辆长度
        self.lf = 3.25                  # 后轴距车头长度
        self.lr = 0.75                  # 前轴距车尾长度
        self.tr = 0.4                   # 车轮半径
        self.tw = 0.4                   # 车轮宽度
        self.wd = 1.5                   # 轮距


class Vehicle:
    """车辆模型类，实现车辆运动学模型和状态更新"""
    def __init__(self, x, y, yaw, v, params: VehicleParams):
        self.params = params
        self.x = x          # x坐标
        self.y = y          # y坐标
        self.yaw = yaw      # 航向角
        self.v = v          # 速度
        self.steer = 0.0    # 转向角
        self.accel = 0.0    # 加速度

    def get_state(self):
        return np.array([self.x, self.y, self.yaw, self.v])
    
    def update(self, u):
        delta_t = self.params.delta_t

        self.steer = np.clip(u[0], -self.params.max_steer_abs, self.params.max_steer_abs)
        self.accel = np.clip(u[1], -self.params.max_accel_abs, self.params.max_accel_abs)
        
        self.x += self.v * np.cos(self.yaw) * delta_t
        self.y += self.v * np.sin(self.yaw) * delta_t
        self.yaw += self.v / self.params.wheel_base * np.tan(self.steer) * delta_t
        self.v += self.accel * delta_t