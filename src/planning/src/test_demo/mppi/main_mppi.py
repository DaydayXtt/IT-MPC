import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.gridspec as gridspec
from vehicle import VehicleParams, Vehicle
from mppi import MPPIControllerParams, MPPIController



def generate_reference_path(radius=15.0, num_points=100):
    theta = np.linspace(np.pi, -np.pi, num_points)  
    
    # 计算参考路径状态
    ref_x = radius * np.cos(theta)
    ref_y = radius * np.sin(theta)
    ref_yaw = np.arctan2(np.gradient(ref_y), np.gradient(ref_x))  
    ref_v = np.full_like(ref_x, 5.0) 
    
    return np.vstack([ref_x, ref_y, ref_yaw, ref_v]).T


def draw_vehicle(x, y, yaw, steer, ax, vehicle_param, color='#2c3e50'):
    vehicle_outline = np.array([
        [-vehicle_param.lr, vehicle_param.lf, vehicle_param.lf, -vehicle_param.lr, -vehicle_param.lr],
        [vehicle_param.vehicle_w / 2, vehicle_param.vehicle_w / 2, 
         -vehicle_param.vehicle_w / 2, -vehicle_param.vehicle_w / 2, vehicle_param.vehicle_w / 2]
    ])

    wheel = np.array([
        [-vehicle_param.tr, vehicle_param.tr, vehicle_param.tr, -vehicle_param.tr, -vehicle_param.tr],
        [vehicle_param.tw / 2, vehicle_param.tw / 2, -vehicle_param.tw / 2, -vehicle_param.tw / 2, vehicle_param.tw / 2]
    ])

    rr_wheel = wheel.copy()  # 右后轮
    rl_wheel = wheel.copy()  # 左后轮
    fr_wheel = wheel.copy()  # 右前轮
    fl_wheel = wheel.copy()  # 左前轮
    rr_wheel[1, :] += vehicle_param.wd / 2
    rl_wheel[1, :] -= vehicle_param.wd / 2

    rot_steer = np.array([
        [np.cos(steer), -np.sin(steer)],
        [np.sin(steer), np.cos(steer)]
    ])
    
    rot_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])

    fr_wheel = np.dot(rot_steer, fr_wheel)
    fl_wheel = np.dot(rot_steer, fl_wheel)
    fr_wheel += np.array([[vehicle_param.wheel_base], [-vehicle_param.wd / 2]])
    fl_wheel += np.array([[vehicle_param.wheel_base], [vehicle_param.wd / 2]])

    fr_wheel = np.dot(rot_yaw, fr_wheel)
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    
    fl_wheel = np.dot(rot_yaw, fl_wheel)
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    
    rr_wheel = np.dot(rot_yaw, rr_wheel)
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    
    rl_wheel = np.dot(rot_yaw, rl_wheel)
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y
    
    vehicle_outline = np.dot(rot_yaw, vehicle_outline)
    vehicle_outline[0, :] += x
    vehicle_outline[1, :] += y

    artists = []
    artists.append(ax.plot(fr_wheel[0, :], fr_wheel[1, :], '#2c3e50', linewidth=2)[0])
    artists.append(ax.plot(rr_wheel[0, :], rr_wheel[1, :], '#2c3e50', linewidth=2)[0])
    artists.append(ax.plot(fl_wheel[0, :], fl_wheel[1, :], '#2c3e50', linewidth=2)[0])
    artists.append(ax.plot(rl_wheel[0, :], rl_wheel[1, :], '#2c3e50', linewidth=2)[0])
    artists.append(ax.fill(vehicle_outline[0, :], vehicle_outline[1, :], color, alpha=0.85)[0])
    
    arrow_length = 1.5
    arrow = ax.arrow(x, y, 
                    arrow_length * np.cos(yaw), 
                    arrow_length * np.sin(yaw), 
                    head_width=0.6, head_length=0.8, 
                    fc='#f1c40f', ec='#f1c40f', linewidth=2)
    artists.append(arrow)
    
    return artists



class VisualizationManager:
    def __init__(self, ref_path, vehicle_params):
        self.ref_path = ref_path
        self.vehicle_params = vehicle_params
        self.ref_x = ref_path[:, 0]
        self.ref_y = ref_path[:, 1]
        
        self.frame_data_list = []
        self.frame_count = 0
        
        self.fig = None
        self.ax_main = None
        self.ax_steer = None
        self.ax_accel = None
        self.ax_vel = None
        self.title_font = None
        self.label_font = None
        self.colors = None
        self.animation = None
        
        self._initialize_style()
        
    def _initialize_style(self):
        plt.style.use('default')
        self.fig = plt.figure(figsize=(16, 8), facecolor='#f8f9fa', dpi=100)
        
        gs_main = gridspec.GridSpec(1, 2, width_ratios=[3, 2], height_ratios=[1])
        gs_main.update(wspace=0.3, hspace=0.1)
        
        self.ax_main = self.fig.add_subplot(gs_main[0, 0])
        
        gs_controls = gridspec.GridSpecFromSubplotSpec(
            3, 1, subplot_spec=gs_main[0, 1],
            hspace=0.25,
            height_ratios=[1, 1, 1]
        )
        self.ax_steer = self.fig.add_subplot(gs_controls[0])    # 转向角
        self.ax_accel = self.fig.add_subplot(gs_controls[1])    # 加速度
        self.ax_vel = self.fig.add_subplot(gs_controls[2])      # 速度

        for ax in [self.ax_main, self.ax_steer, self.ax_accel, self.ax_vel]:
            ax.set_facecolor('#ffffff')
            ax.grid(True, linestyle='--', linewidth=0.7, alpha=0.7)
            ax.tick_params(axis='both', which='major', labelsize=9)
            ax.tick_params(axis='both', which='minor', labelsize=8)
            for spine in ax.spines.values():
                spine.set_edgecolor('#d1d1d1')
                spine.set_linewidth(0.8)

        self.title_font = {'fontsize': 12, 'fontweight': 'bold', 'color': '#2c3e50'}
        self.label_font = {'fontsize': 10, 'fontweight': 'medium', 'color': '#34495e'}
        
        self.colors = {
            'reference': '#3498db',    # 参考路径-蓝色
            'optimal': '#27ae60',      # 最优轨迹-绿色
            'history': '#e67e22',      # 历史轨迹-橙色
            'sampled': '#7f8c8d',      # 采样轨迹-红色
            'steer': '#9b59b6',        # 转向角-紫色
            'accel': '#2980b9',        # 加速度-蓝色
            'velocity': '#16a085',     # 速度-绿色
            'vehicle': '#a9c6de',      # 车辆-浅蓝灰
            'arrow': '#f1c40f'         # 箭头-黄色
        }
    
    def _visualize_reference_path(self, ax, frame_idx=0):
        artists = []
        
        ref_line, = ax.plot(
            self.ref_path[:, 0], self.ref_path[:, 1], '--', 
            color=self.colors['reference'], linewidth=2.5, 
            alpha=0.9, label='Reference Path'
        )
        artists.append(ref_line)
        
        start_marker = ax.plot(
            self.ref_path[0, 0], self.ref_path[0, 1], 'o', 
            markersize=9, markerfacecolor='#2ecc71', 
            markeredgecolor='#27ae60', markeredgewidth=1.5, 
            label='Start'
        )[0]
        artists.append(start_marker)
        
        end_marker = ax.plot(
            self.ref_path[-1, 0], self.ref_path[-1, 1], 'X', 
            markersize=10, markerfacecolor='#e74c3c', 
            markeredgecolor='#c0392b', markeredgewidth=1.5, 
            label='Goal'
        )[0]
        artists.append(end_marker)
        
        if frame_idx == 0:
            legend = ax.legend(
                loc='upper right', fontsize=9, frameon=True, 
                facecolor='white', edgecolor='#d5d8dc', 
                framealpha=0.95
            )
            legend.get_frame().set_linewidth(0.8)
            artists.append(legend)
        
        return artists
    
    def add_frame(self, current_state, optimal_input, optimal_traj, sampled_traj_list,
                 history_x, history_y, history_time, history_steer, history_accel, history_velocity):
        frame_data = {
            'current_state': current_state.copy(),
            'optimal_input': optimal_input.copy(),
            'optimal_traj': optimal_traj.copy() if optimal_traj is not None else None,
            'sampled_traj_list': sampled_traj_list.copy() if sampled_traj_list is not None else None,
            'history_x': history_x.copy(),
            'history_y': history_y.copy(),
            'history_time': history_time.copy(),
            'history_steer': history_steer.copy(),
            'history_accel': history_accel.copy(),
            'history_velocity': history_velocity.copy(),
            'frame_idx': self.frame_count
        }
        self.frame_data_list.append(frame_data)
        self.frame_count += 1
    
    def _update_frame(self, frame_idx):
        if frame_idx >= len(self.frame_data_list):
            return []
        
        data = self.frame_data_list[frame_idx]
        artists = []
        
        self.ax_main.clear()
        self.ax_steer.clear()
        self.ax_accel.clear()
        self.ax_vel.clear()
        
        ref_artists = self._visualize_reference_path(self.ax_main, data['frame_idx'])
        artists.extend(ref_artists)
        
        if data['sampled_traj_list'] is not None and len(data['sampled_traj_list']) > 0:
            for k in range(min(100, len(data['sampled_traj_list']))):  
                traj_x = data['sampled_traj_list'][k, :, 0]
                traj_y = data['sampled_traj_list'][k, :, 1]
                alpha = max(0.0, 0.05 + 0.2 * (1 - k / 20))
                line, = self.ax_main.plot(traj_x, traj_y, color=self.colors['sampled'], 
                                        linewidth=0.8, alpha=alpha)
                artists.append(line)
        
        if data['optimal_traj'] is not None:
            opt_line, = self.ax_main.plot(data['optimal_traj'][:, 0], data['optimal_traj'][:, 1], 
                                        color=self.colors['optimal'], linewidth=3.0, 
                                        alpha=0.9, label='Optimal Trajectory')
            artists.append(opt_line)
        
        hist_line, = self.ax_main.plot(data['history_x'], data['history_y'], color=self.colors['history'], 
                                     linewidth=2.5, linestyle='-', alpha=0.9, 
                                     label='History Path')
        artists.append(hist_line)
        
        vehicle_artists = draw_vehicle(
            data['current_state'][0], data['current_state'][1], data['current_state'][2], 
            data['optimal_input'][0], self.ax_main, self.vehicle_params, color=self.colors['vehicle']
        )
        artists.extend(vehicle_artists)
        
        self.ax_main.set_aspect('equal')
        self.ax_main.set_xlabel('X Position (m)', **self.label_font)
        self.ax_main.set_ylabel('Y Position (m)',** self.label_font)
        self.ax_main.grid(True, linestyle='--', alpha=0.7)
        
        margin = 5.0
        x_min = min(self.ref_x.min(), min(data['history_x'] or [0])) - margin
        x_max = max(self.ref_x.max(), max(data['history_x'] or [0])) + margin
        y_min = min(self.ref_y.min(), min(data['history_y'] or [0])) - margin
        y_max = max(self.ref_y.max(), max(data['history_y'] or [0])) + margin
        self.ax_main.set_xlim(x_min, x_max)
        self.ax_main.set_ylim(y_min, y_max)
        
        steer_line, = self.ax_steer.plot(data['history_time'], data['history_steer'], 
                                       color=self.colors['steer'], linewidth=2.0, alpha=0.9, 
                                       label='Steering')
        artists.append(steer_line)
        
        self.ax_steer.axhline(y=0, color='#7f8c8d', linestyle='-', linewidth=0.8, alpha=0.5)
        
        self.ax_steer.set_title('Steering Angle', **self.title_font, pad=8)
        self.ax_steer.set_ylabel('Angle (rad)',** self.label_font)
        self.ax_steer.set_xlim(0, len(data['history_time']) * self.vehicle_params.delta_t * 1.05 if data['history_time'] else 10)
        self.ax_steer.set_ylim(-self.vehicle_params.max_steer_abs*1.1, self.vehicle_params.max_steer_abs*1.1)
        self.ax_steer.grid(True, linestyle='--', alpha=0.7)

        if data['history_steer']:
            current_steer = data['history_steer'][-1]
            self.ax_steer.annotate(f'{current_steer:.3f} rad', 
                                  xy=(data['history_time'][-1], current_steer),
                                  xytext=(5, 10), textcoords='offset points',
                                  fontsize=9, color=self.colors['steer'])
        
        accel_line, = self.ax_accel.plot(data['history_time'], data['history_accel'], 
                                       color=self.colors['accel'], linewidth=2.0, alpha=0.9, 
                                       label='Acceleration')
        artists.append(accel_line)

        self.ax_accel.axhline(y=0, color='#7f8c8d', linestyle='-', linewidth=0.8, alpha=0.5)
        
        self.ax_accel.set_title('Acceleration', **self.title_font, pad=8)
        self.ax_accel.set_ylabel('Accel (m/s²)',** self.label_font)
        self.ax_accel.set_xlim(0, len(data['history_time']) * self.vehicle_params.delta_t * 1.05 if data['history_time'] else 10)
        self.ax_accel.set_ylim(-self.vehicle_params.max_accel_abs*1.1, self.vehicle_params.max_accel_abs*1.1)
        self.ax_accel.grid(True, linestyle='--', alpha=0.7)
        
        if data['history_accel']:
            current_accel = data['history_accel'][-1]
            self.ax_accel.annotate(f'{current_accel:.3f} m/s²', 
                                  xy=(data['history_time'][-1], current_accel),
                                  xytext=(5, 10), textcoords='offset points',
                                  fontsize=9, color=self.colors['accel'])
        
        vel_line, = self.ax_vel.plot(data['history_time'], data['history_velocity'], 
                                   color=self.colors['velocity'], linewidth=2.0, alpha=0.9, 
                                   label='Velocity')
        artists.append(vel_line)
        
        self.ax_vel.axhline(y=2.0, color='#e74c3c', linestyle='--', linewidth=1.2, alpha=0.7)
        
        self.ax_vel.set_title('Vehicle Speed', **self.title_font, pad=8)
        self.ax_vel.set_ylabel('Speed (m/s)',** self.label_font)
        self.ax_vel.set_xlabel('Time (s)', **self.label_font)
        self.ax_vel.set_xlim(0, len(data['history_time']) * self.vehicle_params.delta_t * 1.05 if data['history_time'] else 10)
        
        if data['history_velocity']:
            y_max = max(4.0, max(data['history_velocity'])*1.2)
        else:
            y_max = 4.0
        self.ax_vel.set_ylim(0, y_max)
        
        self.ax_vel.grid(True, linestyle='--', alpha=0.7)
        
        if data['history_velocity']:
            current_vel = data['history_velocity'][-1]
            self.ax_vel.annotate(f'{current_vel:.3f} m/s', 
                                xy=(data['history_time'][-1], current_vel),
                                xytext=(5, 10), textcoords='offset points',
                                fontsize=9, color=self.colors['velocity'])
        
        if data['frame_idx'] == 0:
            ref_vel_label = self.ax_vel.annotate('Reference Speed: 2.0 m/s', 
                                               xy=(0.98, 0.95), xycoords='axes fraction',
                                               ha='right', va='top', fontsize=9, 
                                               color='#e74c3c')
            artists.append(ref_vel_label)
        
        return artists
    
    def show_animation(self, interval=100, save_gif=False, gif_path='mppi_simulation.gif', fps=10):
        if not self.frame_data_list:
            print("No frames to animate.")
            return None
        
        print(f"Generating animation with {len(self.frame_data_list)} frames...")
        
        self.animation = FuncAnimation(
            self.fig,
            self._update_frame,
            frames=len(self.frame_data_list),
            interval=interval,
            blit=False,  
            repeat=False
        )

        plt.subplots_adjust(left=0.05, right=0.95, top=0.92, bottom=0.08, hspace=0.25, wspace=0.3)
        self.fig.suptitle('Model Predictive Path Integral (MPPI) Controller Simulation', 
                        fontsize=14, fontweight='bold', color='#2c3e50', y=0.98)
    
        if save_gif:
            try:
                writer = PillowWriter(fps=fps)
                self.animation.save(
                    gif_path, 
                    writer=writer, 
                    dpi=150, 
                    savefig_kwargs={'pad_inches': 0.1}
                )
                print(f"Animation saved as GIF: {gif_path}")
            except Exception as e:
                print(f"Failed to save GIF: {str(e)}")
                print("Ensure Pillow is installed: pip install pillow")
        
        plt.show()
        return self.animation


def main():
    ref_path = generate_reference_path(radius=15.0, num_points=100)
    end_point = ref_path[-1, :2]

    vehicle_params = VehicleParams()
    mppi_params = MPPIControllerParams()
    mppi = MPPIController(mppi_params, vehicle_params, ref_path)
    vehicle = Vehicle(-16.0, 0.0, np.deg2rad(100.0), 0, vehicle_params)  # 初始位置在圆左侧

    viz_manager = VisualizationManager(ref_path, vehicle_params)

    # 仿真参数
    max_iterations = 200
    save_animation = True
    gif_file_path = "./src/planning/src/test_demo/mppi/result/mppi_vehicle_simulation.gif"
    interval = 100
    fps = 10

    # 存储历史数据
    history_x = []
    history_y = []
    history_time = []
    history_steer = []
    history_accel = []
    history_velocity = []

    i = 0
    while i < max_iterations:
        # 获取当前状态
        current_state = vehicle.get_state()
        
        # 记录历史数据
        history_x.append(current_state[0])
        history_y.append(current_state[1])
        history_velocity.append(current_state[3])
        current_time = i * vehicle_params.delta_t
        history_time.append(current_time)
        
        distance_to_end = np.hypot(current_state[0] - end_point[0], current_state[1] - end_point[1])
        optimal_input, _, optimal_traj, sampled_traj_list, arrived = mppi.calc_control_input(current_state)
        
        if arrived:
            print(f"\nSuccessfully reached the end of reference path! "
                  f"Distance to end point: {distance_to_end:.3f}m")
            break
        
        history_steer.append(optimal_input[0])
        history_accel.append(optimal_input[1])
        
        viz_manager.add_frame(
            current_state, optimal_input, optimal_traj, sampled_traj_list,
            history_x, history_y, history_time, history_steer, history_accel, history_velocity
        )
        
        i += 1
        print(f"Time: {current_time:>2.2f}[s], x={current_state[0]:>+3.3f}[m], y={current_state[1]:>+3.3f}[m], "
              f"yaw={current_state[2]:>+3.3f}[rad], v={current_state[3]:>+3.3f}[m/s]")
        vehicle.update(optimal_input)
    
    if i >= max_iterations:
        final_distance = np.hypot(vehicle.get_state()[0] - end_point[0], vehicle.get_state()[1] - end_point[1])
        print(f"\nSimulation stopped: Maximum iterations ({max_iterations}) reached.")
        print(f"Final distance to target: {final_distance:.3f}m")

    viz_manager.show_animation(
        interval=interval,
        save_gif=save_animation,
        gif_path=gif_file_path,
        fps=fps
    )



if __name__ == "__main__":
    main()
