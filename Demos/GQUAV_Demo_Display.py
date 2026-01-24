#!/usr/bin/env python3
"""
无人机状态实时显示程序
实时监听ROS2话题并显示12个状态量的曲线图
布局：3行4列子图
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time


class UAVStateDisplay(Node):
    def __init__(self):
        super().__init__('uav_state_display')

        # 记录接收时间，用于计算相对时间
        self.start_time = time.time()
        self.msg_count = 0

        # 数据缓冲区（存储最近500个点）
        self.buffer_size = 500
        self.time_buffer = deque(maxlen=self.buffer_size)
        self.x_buffer = deque(maxlen=self.buffer_size)
        self.y_buffer = deque(maxlen=self.buffer_size)
        self.z_buffer = deque(maxlen=self.buffer_size)
        self.vx_buffer = deque(maxlen=self.buffer_size)
        self.vy_buffer = deque(maxlen=self.buffer_size)
        self.vz_buffer = deque(maxlen=self.buffer_size)
        self.phi_buffer = deque(maxlen=self.buffer_size)
        self.theta_buffer = deque(maxlen=self.buffer_size)
        self.psi_buffer = deque(maxlen=self.buffer_size)
        self.wx_buffer = deque(maxlen=self.buffer_size)
        self.wy_buffer = deque(maxlen=self.buffer_size)
        self.wz_buffer = deque(maxlen=self.buffer_size)

        # ROS2订阅者
        self.state_sub = self.create_subscription(
            Float64MultiArray,
            '/uav_state',
            self.state_callback,
            10
        )

        # 创建图形
        self.setup_plot()

        self.get_logger().info('无人机状态显示节点已启动')

    def state_callback(self, msg):
        """状态回调函数"""
        # 消息格式：[x, y, z, phi, theta, psi, vx, vy, vz, wx, wy, wz, timestamp]
        data = msg.data

        # 调试信息
        self.msg_count += 1
        if self.msg_count % 10 == 0:
            self.get_logger().info(f'Received state data: {len(data)} elements')

        if len(data) >= 13:
            # 使用 Python 接收时间作为时间基准（精度更高）
            current_time = time.time()
            relative_time = current_time - self.start_time
            self.time_buffer.append(relative_time)
            self.x_buffer.append(data[0])
            self.y_buffer.append(data[1])
            self.z_buffer.append(data[2])
            self.phi_buffer.append(data[3])
            self.theta_buffer.append(data[4])
            self.psi_buffer.append(data[5])
            self.vx_buffer.append(data[6])
            self.vy_buffer.append(data[7])
            self.vz_buffer.append(data[8])
            self.wx_buffer.append(data[9])
            self.wy_buffer.append(data[10])
            self.wz_buffer.append(data[11])

    def setup_plot(self):
        """设置图形界面"""
        # 设置中文字体
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'WenQuanYi Micro Hei', 'SimHei', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False

        plt.style.use('seaborn-darkgrid')
        self.fig, self.axes = plt.subplots(3, 4, figsize=(16, 9))
        self.fig.suptitle('UAV State Real-time Monitor', fontsize=16, fontweight='bold')

        # 线条对象
        self.lines = []
        colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A']

        # 状态名称和单位
        state_info = [
            ['X Position (m)', 'X Velocity vx (m/s)', 'Roll phi (rad)', 'Angular Rate wx (rad/s)'],
            ['Y Position (m)', 'Y Velocity vy (m/s)', 'Pitch theta (rad)', 'Angular Rate wy (rad/s)'],
            ['Z Position (m)', 'Z Velocity vz (m/s)', 'Yaw psi (rad)', 'Angular Rate wz (rad/s)']
        ]

        # 创建子图
        for i in range(3):
            for j in range(4):
                ax = self.axes[i, j]
                line, = ax.plot([], [], linewidth=2, color=colors[j])
                self.lines.append(line)
                ax.set_title(state_info[i][j], fontsize=10, fontweight='bold')
                ax.set_xlabel('Time (s)', fontsize=8)
                ax.grid(True, alpha=0.3)
                ax.tick_params(labelsize=8)

        plt.tight_layout()
        plt.subplots_adjust(top=0.93)

    def update_plot(self, frame):
        """更新图形"""
        if len(self.time_buffer) == 0:
            return self.lines

        time_data = np.array(self.time_buffer)

        # 更新12条曲线
        buffers = [
            self.x_buffer, self.vx_buffer, self.phi_buffer, self.wx_buffer,
            self.y_buffer, self.vy_buffer, self.theta_buffer, self.wy_buffer,
            self.z_buffer, self.vz_buffer, self.psi_buffer, self.wz_buffer
        ]

        for i, (line, buffer) in enumerate(zip(self.lines, buffers)):
            if len(buffer) > 0:
                y_data = np.array(buffer)
                line.set_data(time_data[-len(y_data):], y_data)

                # 动态调整坐标轴范围
                ax = self.axes[i // 4, i % 4]
                ax.set_xlim(time_data[0], time_data[-1] + 0.1)
                y_min, y_max = np.min(y_data), np.max(y_data)

                # 当数据范围小于阈值时，设置纵坐标下限
                y_range = y_max - y_min
                if abs(y_range) < 0.01:
                    margin = 0.01
                else:
                    margin = y_range * 0.1

                # 确保纵坐标始终包含0
                if y_min > 0:
                    y_min = 0
                elif y_max < 0:
                    y_max = 0

                ax.set_ylim(y_min - margin, y_max + margin)

        return self.lines

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
                    self.get_logger().info('Plot window closed, restarting...')
                    self.fig.canvas.mpl_connect('close_event', lambda event: None)
                    plt.close(self.fig)
                    self.setup_plot()
                    self.anim = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
                    plt.show(block=False)
        except KeyboardInterrupt:
            pass

    def __del__(self):
        """析构函数"""
        self.get_logger().info('无人机状态显示节点已关闭')


def main():
    rclpy.init()
    display_node = UAVStateDisplay()
    try:
        display_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        display_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
