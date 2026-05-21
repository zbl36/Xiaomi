"""
赛段六：撷金建功
踢足球出口 + 前往终点趴下

world文件：football3 位于 y≈14.7, x≈0.4
"""

import cv2
import numpy as np
import time
import threading
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from utils.camera_subscriber import CameraSubscriber

VX_APPROACH  = 0.25
VX_KICK      = 0.8
Kp_yaw       = 0.004

# 终点圆圈检测（地面白色圆圈）
FINISH_Y     = 15.8   # 终点Y坐标（由主状态机判断）


class StageKick:
    S_FIND_BALL  = 'FIND_BALL'
    S_ALIGN      = 'ALIGN'
    S_KICK       = 'KICK'
    S_GOTO_FINISH= 'GOTO_FINISH'
    S_DONE       = 'DONE'

    def __init__(self, ctrl):
        self.ctrl = ctrl
        self._cam = CameraSubscriber('/rgb_camera/image_raw')
        self._cam.start()
        self._sub_state = self.S_FIND_BALL
        self._timer = 0.0
        self._lost_count = 0

    def _get_frame(self):
        return self._cam.get_frame()

    def _detect_football(self, frame):
        """检测足球（白色圆形）"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv,
                                 np.array([0, 0, 180]),
                                 np.array([180, 40, 255]))
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 300:
            return None
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        return M['m10'] / M['m00'], M['m01'] / M['m00'], area

    def step(self, current_y):
        frame = self._get_frame()
        if frame is None:
            self.ctrl.move(vx=0.1)
            return False

        w = frame.shape[1]

        if self._sub_state == self.S_FIND_BALL:
            ball = self._detect_football(frame)
            if ball:
                print("[赛段六] 发现足球，开始对准")
                self._sub_state = self.S_ALIGN
                self._lost_count = 0
            else:
                # 慢速前进寻找
                self.ctrl.move(vx=0.2)

        elif self._sub_state == self.S_ALIGN:
            ball = self._detect_football(frame)
            if ball:
                cx, cy, area = ball
                error = cx - w / 2.0
                vyaw = -Kp_yaw * error
                self._lost_count = 0

                if area > 5000:
                    # 足够近，开始踢
                    print("[赛段六] 开始踢球！")
                    self._sub_state = self.S_KICK
                    self._timer = time.time()
                else:
                    self.ctrl.move(vx=VX_APPROACH, vyaw=vyaw)
            else:
                self._lost_count += 1
                if self._lost_count > 30:
                    self._sub_state = self.S_FIND_BALL

        elif self._sub_state == self.S_KICK:
            self.ctrl.move(vx=VX_KICK)
            if time.time() - self._timer > 1.0:
                print("[赛段六] 踢球完成，前往终点")
                self.ctrl.stop()
                time.sleep(0.5)
                self._sub_state = self.S_GOTO_FINISH
                self._timer = time.time()

        elif self._sub_state == self.S_GOTO_FINISH:
            # 前进到终点
            self.ctrl.move(vx=0.3)
            # 由主状态机根据Y坐标判断到达终点
            return True

        elif self._sub_state == self.S_DONE:
            return True

        return False

    def finish(self):
        """到达终点，趴下"""
        print("[赛段六] 到达终点，趴下！比赛结束！")
        self.ctrl.lie_down()

    def cleanup(self):
        self._cam.stop()
        self.finish()
        print("[赛段六] 结束")
