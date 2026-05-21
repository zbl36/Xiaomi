#!/usr/bin/env python3
"""
相机截图工具
订阅 /rgb_camera/image_raw，按空格保存图片，按 q 退出
用法：python3.8 capture_images.py

保存路径：/home/cyberdog_sim/captured_images/
"""

import cv2
import numpy as np
import os
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

SAVE_DIR = '/home/cyberdog_sim/captured_images'


class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture')
        self._frame = None
        self._lock = threading.Lock()
        self._count = 0

        os.makedirs(SAVE_DIR, exist_ok=True)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.create_subscription(Image, '/rgb_camera/image_raw', self._cb, qos)
        self.get_logger().info(f'截图工具启动，保存路径：{SAVE_DIR}')
        self.get_logger().info('按空格键截图，按 q 退出')

    def _cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape((msg.height, msg.width, -1))
        if msg.encoding in ('rgb8', 'RGB8'):
            arr = arr[:, :, ::-1].copy()
        with self._lock:
            self._frame = arr

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def save(self, label=''):
        frame = self.get_frame()
        if frame is None:
            print('还没收到图像，请等待仿真启动')
            return
        self._count += 1
        ts = time.strftime('%H%M%S')
        name = f'{label}_{ts}_{self._count:03d}.jpg' if label else f'img_{ts}_{self._count:03d}.jpg'
        path = os.path.join(SAVE_DIR, name)
        cv2.imwrite(path, frame)
        print(f'已保存：{path}')


def main():
    rclpy.init()
    node = ImageCapture()

    # 后台 spin
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    print('\n截图工具已启动')
    print('命令：')
    print('  直接回车       → 保存截图')
    print('  输入标签+回车  → 保存带标签的截图（如输入 coke 保存为 coke_xxx.jpg）')
    print('  输入 q         → 退出\n')

    try:
        while True:
            label = input('标签（直接回车截图）> ').strip()
            if label.lower() == 'q':
                break
            node.save(label)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(f'\n所有截图保存在：{SAVE_DIR}')


if __name__ == '__main__':
    main()
