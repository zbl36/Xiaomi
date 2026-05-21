"""
ROS2 相机图像订阅工具
统一封装相机订阅，供各赛段模块使用
订阅 /rgb_camera/image_raw（best_effort QoS），转为 numpy 数组
"""

import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 全局节点，避免重复初始化
_camera_node = None
_camera_thread = None
_node_lock = threading.Lock()


class CameraNode(Node):
    def __init__(self, topic='/rgb_camera/image_raw'):
        super().__init__('camera_subscriber')
        self._frame = None
        self._lock = threading.Lock()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.create_subscription(Image, topic, self._cb, qos)
        self.get_logger().info(f'相机订阅: {topic}')

    def _cb(self, msg):
        # sensor_msgs/Image -> numpy BGR
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape((msg.height, msg.width, -1))
        # ROS 图像默认 RGB，转为 BGR 供 OpenCV 使用
        if msg.encoding in ('rgb8', 'RGB8'):
            arr = arr[:, :, ::-1].copy()
        with self._lock:
            self._frame = arr

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None


class CameraSubscriber:
    """
    轻量级相机订阅封装，各赛段直接实例化使用

    用法：
        cam = CameraSubscriber()
        cam.start()
        frame = cam.get_frame()  # numpy BGR 数组，None 表示还没收到图像
        cam.stop()
    """

    def __init__(self, topic='/rgb_camera/image_raw'):
        self.topic = topic
        self._node = None
        self._thread = None
        self._running = False

    def start(self):
        if self._running:
            return
        if not rclpy.ok():
            rclpy.init()
        self._node = CameraNode(self.topic)
        self._running = True

        def spin():
            while self._running:
                rclpy.spin_once(self._node, timeout_sec=0.01)

        self._thread = threading.Thread(target=spin, daemon=True)
        self._thread.start()

    def get_frame(self):
        if self._node is None:
            return None
        return self._node.get_frame()

    def stop(self):
        self._running = False
