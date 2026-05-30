#!/usr/bin/env python3
"""
实时显示机器狗坐标
用 Gazebo 手动拖动机器狗到目标位置，记录坐标

用法：python3.8 check_pos.py
"""

import sys
import time
import threading
import math

sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TF(Node):
    def __init__(self):
        super().__init__('pos_check')
        self.create_subscription(TFMessage, '/tf', self._cb, 10)
    def _cb(self, msg):
        for t in msg.transforms:
            if t.child_frame_id in ('base_link', 'body'):
                x = t.transform.translation.x
                y = t.transform.translation.y
                q = t.transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                print(f'\rx={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°', end='', flush=True)

rclpy.init()
n = TF()
print("实时坐标（Ctrl+C 退出）：")
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
n.destroy_node()
rclpy.shutdown()
print()
