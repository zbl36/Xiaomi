#!/usr/bin/env python3
"""
相机查看工具
启动 QoS relay 节点并打开 rqt_image_view 查看画面
用法：python3.8 view_camera.py
"""

import subprocess
import threading
import sys
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np


class CameraRelay(Node):
    def __init__(self):
        super().__init__('camera_relay')

        qos_sub = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        qos_pub = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.pub_rgb = self.create_publisher(Image, '/camera/image_rgb', qos_pub)
        self.sub_rgb = self.create_subscription(
            Image, '/rgb_camera/image_raw',
            lambda msg: self.pub_rgb.publish(msg), qos_sub)

        self.pub_ai = self.create_publisher(Image, '/camera/image_ai', qos_pub)
        self.sub_ai = self.create_subscription(
            Image, '/ai_camera/image_raw',
            lambda msg: self.pub_ai.publish(msg), qos_sub)

        self.get_logger().info('相机转发启动')
        self.get_logger().info('  /camera/image_rgb  ← RGB相机')
        self.get_logger().info('  /camera/image_ai   ← AI相机')


def main():
    # 启动 relay 节点（后台线程）
    rclpy.init()
    node = CameraRelay()

    relay_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    relay_thread.start()

    print("relay 已启动，正在打开 rqt_image_view...")
    print("请在下拉菜单选择 /camera/image_rgb 或 /camera/image_ai")

    # 启动 rqt_image_view（阻塞直到窗口关闭）
    try:
        subprocess.run(['ros2', 'run', 'rqt_image_view', 'rqt_image_view'])
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
