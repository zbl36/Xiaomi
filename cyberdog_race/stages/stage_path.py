"""
赛段一：石径探路 / 赛段三：曲道冲锋
黄线跟踪 + 弯道转向
"""

import cv2
import numpy as np
import threading
import time
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from utils.camera_subscriber import CameraSubscriber

# 黄线HSV范围
YELLOW_LOW  = np.array([25, 80,  200])
YELLOW_HIGH = np.array([35, 255, 255])

# PID参数
Kp = 0.003
Ki = 0.0001
Kd = 0.001

# 速度参数
VX_ROCKROAD = 0.25   # 石板路速度（慢，抬腿高）
VX_SCURVE   = 0.25   # S形弯道速度
STEP_ROCKROAD = 0.12  # 石板路抬腿高度
STEP_NORMAL   = 0.08  # 正常抬腿高度


class StagePath:
    """
    mode='rockroad' : 赛段一，石板路+弯道
    mode='scurve'   : 赛段三，S形弯道
    """

    def __init__(self, ctrl, mode='rockroad'):
        self.ctrl = ctrl
        self.mode = mode
        self.vx = VX_ROCKROAD if mode == 'rockroad' else VX_SCURVE
        self.step_h = STEP_ROCKROAD if mode == 'rockroad' else STEP_NORMAL

        # PID状态
        self._integral = 0.0
        self._last_error = 0.0

        # 相机（使用ROS2 topic）
        self._cam = CameraSubscriber('/rgb_camera/image_raw')
        self._cam.start()

    def _get_frame(self):
        return self._cam.get_frame()

    def _detect_lane_error(self, frame):
        """
        检测黄线，返回赛道中心偏差（正=偏右，负=偏左）
        取图像下半部分分析，更稳定
        """
        h, w = frame.shape[:2]
        roi = frame[h//2:, :]  # 只看下半部分

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)

        # 分左右两侧找黄线
        left_mask  = mask[:, :w//2]
        right_mask = mask[:, w//2:]

        left_cx  = self._mask_center_x(left_mask)
        right_cx = self._mask_center_x(right_mask)

        if left_cx is None and right_cx is None:
            return 0.0  # 看不到黄线，保持直行

        if left_cx is None:
            # 只看到右线，偏右
            lane_center = (w//2 + right_cx) - w * 0.75
        elif right_cx is None:
            # 只看到左线，偏左
            lane_center = left_cx - w * 0.25
        else:
            # 两侧都看到，取中间
            lane_center = (left_cx + (w//2 + right_cx)) / 2.0

        error = lane_center - w / 2.0
        return error

    def _mask_center_x(self, mask):
        """返回mask中最大连通域的X中心，没有则返回None"""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 500:
            return None
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        return M['m10'] / M['m00']

    def _pid(self, error):
        self._integral += error
        self._integral = max(-200, min(200, self._integral))  # 积分限幅
        derivative = error - self._last_error
        self._last_error = error
        return Kp * error + Ki * self._integral + Kd * derivative

    def step(self, current_y):
        """
        主循环调用，每帧执行一次
        返回 True 表示本赛段完成
        """
        frame = self._get_frame()
        if frame is None:
            # 摄像头还没准备好，先直行
            self.ctrl.move(vx=self.vx, step_height=self.step_h)
            return False

        error = self._detect_lane_error(frame)
        vyaw = -self._pid(error)
        vyaw = max(-0.8, min(0.8, vyaw))  # 限幅

        self.ctrl.move_lowhead(
            vx=self.vx,
            vyaw=vyaw,
        )
        return False  # 由主状态机根据Y坐标判断结束

    def cleanup(self):
        self._cam.stop()
        self.ctrl.stop()
        print(f"[{self.mode}] 赛段结束，停止")
