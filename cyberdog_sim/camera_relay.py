#!/usr/bin/env python3
"""
相机图像 QoS 转发节点
将 Gazebo 发布的 best_effort 图像转为 reliable，供 rqt_image_view 等工具订阅

用法：
    python3.8 camera_relay.py

查看画面：
    ros2 run rqt_image_view rqt_image_view
    # 选择 /camera/image_rgb 或 /camera/image_ai
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy


class CameraRelay(Node):
    def __init__(self):
        super().__init__('camera_relay')

        qos_sub = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_pub = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        # RGB 相机
        self.pub_rgb = self.create_publisher(Image, '/camera/image_rgb', qos_pub)
        self.sub_rgb = self.create_subscription(
            Image, '/rgb_camera/image_raw',
            lambda msg: self.pub_rgb.publish(msg),
            qos_sub
        )

        # AI 相机
        self.pub_ai = self.create_publisher(Image, '/camera/image_ai', qos_pub)
        self.sub_ai = self.create_subscription(
            Image, '/ai_camera/image_raw',
            lambda msg: self.pub_ai.publish(msg),
            qos_sub
        )

        self.get_logger().info('相机转发节点启动')
        self.get_logger().info('  RGB: /rgb_camera/image_raw -> /camera/image_rgb')
        self.get_logger().info('  AI:  /ai_camera/image_raw  -> /camera/image_ai')


def main():
    rclpy.init()
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
