"""
赛段二：荒野寻珠
4×4球阵中找橙色球（每行每列各1个），逐一撞击

world文件中球阵坐标（世界坐标）：
  行间距约0.84m，列间距约1.2m
  R1: y≈1.34  R2: y≈2.18  R3: y≈3.02  R4: y≈3.86
  C1: x≈-0.4  C2: x≈0.8   C3: x≈2.0   C4: x≈3.2

橙色球颜色（来自world文件 ambient: 0.92 0.45 0.17）：
  HSV范围 H:10-20, S:150-255, V:150-255
"""

import cv2
import numpy as np
import time
import threading

# 橙色HSV范围
ORANGE_LOW  = np.array([8,  150, 150])
ORANGE_HIGH = np.array([22, 255, 255])

# 控制参数
VX_APPROACH  = 0.25   # 接近球速度
VX_HIT       = 0.5    # 撞击速度
VX_BACK      = -0.3   # 后退速度
Kp_yaw       = 0.004  # 对准PID

# 撞击判定：球在画面中消失或面积突然变小
HIT_AREA_THRESHOLD = 3000  # 球面积超过此值认为已接近


class StageBall:
    def __init__(self, ctrl):
        self.ctrl = ctrl
        self._cap = None
        self._frame = None
        self._frame_lock = threading.Lock()
        self._running = False

        # 状态
        self._sub_state = 'SCAN'   # SCAN / ALIGN / HIT / BACK / DONE
        self._hit_count = 0        # 已撞击球数
        self._target_lost_count = 0
        self._hit_timer = 0.0

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

        threading.Thread(target=grab, daemon=True).start()

    def _get_frame(self):
        with self._frame_lock:
            return self._frame.copy() if self._frame is not None else None

    def _detect_orange_ball(self, frame):
        """
        检测橙色球，返回 (cx, cy, area) 或 None
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_LOW, ORANGE_HIGH)

        # 形态学去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 200:
            return None

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        return cx, cy, area

    def step(self, current_y):
        if self._cap is None:
            self._start_camera()

        frame = self._get_frame()
        if frame is None:
            self.ctrl.move(vx=0.1)
            return False

        w = frame.shape[1]
        ball = self._detect_orange_ball(frame)

        # --- 子状态机 ---
        if self._sub_state == 'SCAN':
            # 慢速前进扫描
            if ball:
                print(f"[赛段二] 发现橙色球！切换到对准模式")
                self._sub_state = 'ALIGN'
                self._target_lost_count = 0
            else:
                self.ctrl.move(vx=0.2, vyaw=0.0)

        elif self._sub_state == 'ALIGN':
            if ball:
                cx, cy, area = ball
                error = cx - w / 2.0
                vyaw = -Kp_yaw * error

                if area >= HIT_AREA_THRESHOLD:
                    # 足够近了，开始撞击
                    print(f"[赛段二] 开始撞击！")
                    self._sub_state = 'HIT'
                    self._hit_timer = time.monotonic()
                else:
                    # 对准并前进
                    self.ctrl.move(vx=VX_APPROACH, vyaw=vyaw)
                self._target_lost_count = 0
            else:
                self._target_lost_count += 1
                if self._target_lost_count > 20:
                    # 目标丢失，回到扫描
                    self._sub_state = 'SCAN'
                    self._target_lost_count = 0

        elif self._sub_state == 'HIT':
            # 全速冲刺撞击
            self.ctrl.move(vx=VX_HIT)
            if time.monotonic() - self._hit_timer > 0.8:
                self._hit_count += 1
                print(f"[赛段二] 撞击完成！已撞 {self._hit_count}/4 个球")
                self._sub_state = 'BACK'
                self._hit_timer = time.monotonic()

        elif self._sub_state == 'BACK':
            # 后退，准备找下一个球
            self.ctrl.move(vx=VX_BACK)
            if time.monotonic() - self._hit_timer > 1.0:
                if self._hit_count >= 4:
                    print("[赛段二] 4个球全部撞完！前进出口")
                    self._sub_state = 'DONE'
                else:
                    self._sub_state = 'SCAN'

        elif self._sub_state == 'DONE':
            # 向出口方向前进（左上角），由主状态机Y坐标判断退出
            self.ctrl.move(vx=0.3, vy=0.2)
            return True

        return False

    def cleanup(self):
        self._running = False
        if self._cap:
            self._cap.release()
        self.ctrl.stop()
        print("[赛段二] 结束")
