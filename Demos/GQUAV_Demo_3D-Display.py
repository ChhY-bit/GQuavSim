#!/usr/bin/env python3
"""
无人机3D运动实时显示程序
实时监听ROS2话题并显示无人机3D运动轨迹和姿态
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import time
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class UAV3DDisplay(Node):
    def __init__(self):
        super().__init__('uav_3d_display')

        # 记录接收时间
        self.start_time = time.time()
        self.msg_count = 0

        # 数据缓冲区
        self.buffer_size = 5000
        self.x_buffer = deque(maxlen=self.buffer_size)
        self.y_buffer = deque(maxlen=self.buffer_size)
        self.z_buffer = deque(maxlen=self.buffer_size)
        self.phi_buffer = deque(maxlen=self.buffer_size)
        self.theta_buffer = deque(maxlen=self.buffer_size)
        self.psi_buffer = deque(maxlen=self.buffer_size)

        # 当前状态
        self.current_state = None

        # ROS2订阅者
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/uav_state',
            self.state_callback,
            10
        )

        # 创建图形
        self.setup_plot()

        self.get_logger().info('无人机3D显示节点已启动')

    def state_callback(self, msg):
        """状态回调函数"""
        # 消息格式：[x, y, z, phi, theta, psi, vx, vy, vz, wx, wy, wz, timestamp]
        data = msg.data

        self.msg_count += 1
        if self.msg_count % 10 == 0:
            self.get_logger().info(f'Received state data: {len(data)} elements')

        if len(data) >= 13:
            self.x_buffer.append(data[0])
            self.y_buffer.append(data[1])
            self.z_buffer.append(data[2])
            self.phi_buffer.append(data[3])
            self.theta_buffer.append(data[4])
            self.psi_buffer.append(data[5])

            self.current_state = {
                'x': data[0], 'y': data[1], 'z': data[2],
                'phi': data[3], 'theta': data[4], 'psi': data[5]
            }

    def rotation_matrix(self, phi, theta, psi):
        """计算欧拉角旋转矩阵"""
        # 滚转 phi (绕x轴)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi), np.cos(phi)]
        ])
        # 俯仰 theta (绕y轴)
        Ry = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
        # 偏航 psi (绕z轴)
        Rz = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])
        return Rz @ Ry @ Rx

    def get_uav_geometry(self, x, y, z, phi, theta, psi):
        """生成四旋翼无人机的几何形状"""
        R = self.rotation_matrix(phi, theta, psi)

        # 无人机参数
        arm_length = 0.3  # 机臂长度
        motor_radius = 0.08  # 电机半径
        body_radius = 0.1  # 中心体半径

        # 四个电机位置（机体坐标系）
        motor_positions = [
            np.array([arm_length, arm_length, 0]),
            np.array([-arm_length, arm_length, 0]),
            np.array([-arm_length, -arm_length, 0]),
            np.array([arm_length, -arm_length, 0])
        ]

        # 转换到世界坐标系
        world_positions = []
        for pos in motor_positions:
            world_pos = R @ pos + np.array([x, y, z])
            world_positions.append(world_pos)

        # 生成机臂线条
        arm_lines = []
        for i in range(4):
            center = np.array([x, y, z])
            arm_lines.append([center, world_positions[i]])

        # 生成电机圆圈（简化为点）
        motor_points = world_positions

        # 生成前进方向箭头（机体x轴正向）
        arrow_start = np.array([x, y, z])
        arrow_dir = R @ np.array([0.4, 0, 0])
        arrow_end = arrow_start + arrow_dir

        return {
            'motors': motor_points,
            'arms': arm_lines,
            'arrow': (arrow_start, arrow_end),
            'center': np.array([x, y, z])
        }

    def setup_plot(self):
        """设置图形界面"""
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'WenQuanYi Micro Hei', 'SimHei', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False

        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title('UAV 3D Motion Real-time Monitor', fontsize=16, fontweight='bold', pad=20)

        # 轨迹线（绿色虚线）
        self.traj_line, = self.ax.plot([], [], [], 'g--', linewidth=1.5, alpha=0.6, label='Trajectory')

        # 无人机部件
        self.motor_scatter = self.ax.scatter([], [], [], c='blue', s=200, alpha=0.8, label='Motors')
        self.center_scatter = self.ax.scatter([], [], [], c='black', s=150, alpha=0.8, label='Center')
        self.arm_lines = []
        for i in range(4):
            line, = self.ax.plot([], [], [], 'k-', linewidth=3, alpha=0.7)
            self.arm_lines.append(line)

        # 前进方向箭头
        self.arrow_line, = self.ax.plot([], [], [], 'r-', linewidth=4, alpha=0.9, label='Heading')

        # 地面网格
        self.setup_ground()

        # 信息文本
        self.info_text = self.fig.text(0.02, 0.95, '', fontsize=10,
                                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                                        verticalalignment='top')

        self.ax.set_xlabel('X (m)', fontsize=12)
        self.ax.set_ylabel('Y (m)', fontsize=12)
        self.ax.set_zlabel('Z (m)', fontsize=12)
        self.ax.legend(loc='lower right', fontsize=10)

        # 初始视图
        self.ax.view_init(elev=30, azim=45)

    def setup_ground(self):
        """绘制地面"""
        xx, yy = np.meshgrid(np.linspace(-5, 5, 21), np.linspace(-5, 5, 21))
        zz = np.zeros_like(xx)
        self.ax.plot_surface(xx, yy, zz, alpha=0.2, color='gray')

        # 添加坐标轴
        self.ax.plot([-5, 5], [0, 0], [0, 0], 'k-', linewidth=0.5, alpha=0.3)
        self.ax.plot([0, 0], [-5, 5], [0, 0], 'k-', linewidth=0.5, alpha=0.3)

    def update_plot(self, frame):
        """更新图形"""
        if self.current_state is None or len(self.x_buffer) == 0:
            return

        x = self.current_state['x']
        y = self.current_state['y']
        z = self.current_state['z']
        phi = self.current_state['phi']
        theta = self.current_state['theta']
        psi = self.current_state['psi']

        # 更新轨迹
        traj_x = list(self.x_buffer)
        traj_y = list(self.y_buffer)
        traj_z = list(self.z_buffer)
        self.traj_line.set_data(traj_x, traj_y)
        self.traj_line.set_3d_properties(traj_z)

        # 获取无人机几何
        geom = self.get_uav_geometry(x, y, z, phi, theta, psi)

        # 更新电机位置
        motors = np.array(geom['motors'])
        self.motor_scatter._offsets3d = (motors[:, 0], motors[:, 1], motors[:, 2])

        # 更新中心点
        self.center_scatter._offsets3d = ([x], [y], [z])

        # 更新机臂
        for i, arm in enumerate(geom['arms']):
            self.arm_lines[i].set_data([arm[0][0], arm[1][0]], [arm[0][1], arm[1][1]])
            self.arm_lines[i].set_3d_properties([arm[0][2], arm[1][2]])

        # 更新方向箭头
        arrow_start, arrow_end = geom['arrow']
        self.arrow_line.set_data([arrow_start[0], arrow_end[0]], [arrow_start[1], arrow_end[1]])
        self.arrow_line.set_3d_properties([arrow_start[2], arrow_end[2]])

        # 动态调整坐标轴范围
        if len(self.x_buffer) > 1:
            all_x = list(self.x_buffer)
            all_y = list(self.y_buffer)
            all_z = list(self.z_buffer)

            x_range = [min(all_x), max(all_x)]
            y_range = [min(all_y), max(all_y)]
            z_range = [min(all_z), max(all_z)]

            # 添加边距
            x_margin = max(1, (x_range[1] - x_range[0]) * 0.2)
            y_margin = max(1, (y_range[1] - y_range[0]) * 0.2)
            z_margin = max(1, (z_range[1] - z_range[0]) * 0.2)

            # 确保z轴包含0（地面）
            z_min = min(0, z_range[0])
            z_max = max(z_range[1], z_min + z_margin * 2)

            self.ax.set_xlim(x_range[0] - x_margin, x_range[1] + x_margin)
            self.ax.set_ylim(y_range[0] - y_margin, y_range[1] + y_margin)
            self.ax.set_zlim(z_min, z_max + z_margin)

        # 更新信息文本
        info = f"""Position:  x={x:.3f} m
            y={y:.3f} m
            z={z:.3f} m

Attitude:  φ={phi:.3f} rad ({np.degrees(phi):.1f}°)
           θ={theta:.3f} rad ({np.degrees(theta):.1f}°)
           ψ={psi:.3f} rad ({np.degrees(psi):.1f}°)

Messages:  {self.msg_count}"""
        self.info_text.set_text(info)

        return [self.traj_line, self.motor_scatter, self.center_scatter,
                self.arrow_line, *self.arm_lines]

    def run(self):
        """运行显示"""
        # 创建动画
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)

        # 显示图形
        plt.show(block=False)

        # 保持ROS节点运行
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                plt.pause(0.01)

                # 检查窗口是否关闭
                if not plt.fignum_exists(self.fig.number):
                    self.get_logger().info('Plot window closed, exiting...')
                    break
        except KeyboardInterrupt:
            pass

    def __del__(self):
        """析构函数"""
        self.get_logger().info('无人机3D显示节点已关闭')


def main():
    rclpy.init()
    display_node = UAV3DDisplay()
    try:
        display_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        display_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
