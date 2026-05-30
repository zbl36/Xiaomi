"""
ROS2 位置订阅工具
订阅 /tf 获取机器人世界坐标，用于赛段切换判断
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import threading


class PoseTracker(Node):
    """订阅 /tf，持续更新机器人世界坐标"""

    def __init__(self):
        super().__init__('pose_tracker')
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self._lock = threading.Lock()
        self.create_subscription(TFMessage, '/tf', self._tf_cb, 10)

    def _tf_cb(self, msg):
        import math
        for t in msg.transforms:
            if (t.header.frame_id == 'vodom' and
                    t.child_frame_id == 'base_link'):
                with self._lock:
                    self.x = t.transform.translation.x
                    self.y = t.transform.translation.y
                    self.z = t.transform.translation.z
                    q = t.transform.rotation
                    siny = 2.0 * (q.w * q.z + q.x * q.y)
                    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    self.yaw = math.atan2(siny, cosy)

    def get_pos(self):
        with self._lock:
            return self.x, self.y, self.z

    def get_yaw(self):
        with self._lock:
            return self.yaw


def start_pose_tracker():
    """在后台线程启动ROS2节点，返回tracker对象"""
    rclpy.init()
    tracker = PoseTracker()

    def spin():
        rclpy.spin(tracker)

    t = threading.Thread(target=spin, daemon=True)
    t.start()
    return tracker
