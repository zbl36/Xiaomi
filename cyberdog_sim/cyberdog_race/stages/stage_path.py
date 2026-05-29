"""
赛段一：石径探路 / 赛段三：曲道冲锋
黄线跟踪 + 弯道转向
"""

import cv2
import numpy as np
import threading
import time

# 黄线HSV范围
YELLOW_LOW  = np.array([20, 100, 100])
YELLOW_HIGH = np.array([35, 255, 255])

# PID参数
Kp = 0.003
Ki = 0.0001
Kd = 0.001

# 速度参数
VX_ROCKROAD = -0.20   # 石板路速度（慢，抬腿高）
VX_SCURVE   = 0.25   # S形弯道速度
STEP_ROCKROAD = 0.06  # 石板路抬腿高度
STEP_NORMAL   = 0.03  # 正常抬腿高度

# 石板路起步准备：先原地掉头 180 度，再倒着通过
TURN_PREP_DURATION = 7.0
TURN_PREP_YAW = 0.78


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
        self._prep_done = mode != 'rockroad'
        self._prep_start = None

        # PID状态
        self._integral = 0.0
        self._last_error = 0.0

        # 摄像头
        self._cap = None
        self._frame = None
        self._frame_lock = threading.Lock()
        self._cam_thread = None
        self._running = False

    def _start_camera(self):
        self._cap = cv2.VideoCapture(0)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._running = True

        def grab():
            while self._running:
                ret, frame = self._cap.read()
                if ret:
                    with self._frame_lock:
                        self._frame = frame
                time.sleep(0.033)

        self._cam_thread = threading.Thread(target=grab, daemon=True)
        self._cam_thread.start()

    def _get_frame(self):
        with self._frame_lock:
            return self._frame.copy() if self._frame is not None else None

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
        if self._cap is None:
            self._start_camera()

        if self.mode == 'rockroad' and not self._prep_done:
            if self._prep_start is None:
                self._prep_start = time.monotonic()

            elapsed = time.monotonic() - self._prep_start
            if elapsed < TURN_PREP_DURATION:
                # 先原地掉头，给后续倒走创造更稳定的朝向
                self.ctrl.move(vx=0.0, vyaw=TURN_PREP_YAW, step_height=self.step_h)
                return False

            self.ctrl.stop()
            time.sleep(0.2)
            self._prep_done = True

        frame = self._get_frame()
        if frame is None:
            # 摄像头还没准备好时先停住，避免无视觉情况下继续朝前走
            self.ctrl.stop()
            return False

        error = self._detect_lane_error(frame)
        vyaw = -self._pid(error)
        vyaw = max(-0.8, min(0.8, vyaw))  # 限幅

        self.ctrl.move(
            vx=self.vx,
            vyaw=vyaw,
            step_height=self.step_h
        )
        return False  # 由主状态机根据Y坐标判断结束

    def cleanup(self):
        self._running = False
        if self._cap:
            self._cap.release()
        self.ctrl.stop()
        print(f"[{self.mode}] 赛段结束，停止")
