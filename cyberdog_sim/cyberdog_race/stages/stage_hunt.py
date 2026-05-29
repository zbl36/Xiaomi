"""
赛段四：深隧寻珍
三个竖向通道中识别目标物体+障碍物，完成交互+语音播报

world文件坐标：
  football2 (足球):  y≈10.8, x≈2.1
  coke (可乐):       y≈11.1, x≈-0.1
  2area_ball (橙球): y≈11.1, x≈0.95
  限高杆：红色，底部高40cm
  障碍物：两个20cm方块
"""

import cv2
import numpy as np
import subprocess
import time
import threading

# 颜色范围
ORANGE_LOW   = np.array([8,  150, 150])
ORANGE_HIGH  = np.array([22, 255, 255])
RED_LOW1     = np.array([0,  150, 150])
RED_HIGH1    = np.array([10, 255, 255])
RED_LOW2     = np.array([170,150, 150])
RED_HIGH2    = np.array([180,255, 255])

# 控制参数
VX_EXPLORE   = 0.2
VX_HIT       = 0.5
VX_BACK      = -0.3
LOW_BODY_H   = -0.06   # 限高杆时降低体高(m)
Kp_yaw       = 0.004


def speak(text):
    """语音播报（非阻塞）"""
    subprocess.Popen(
        ['espeak', '-v', 'zh', '-s', '150', text],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )


class StageHunt:
    # 子状态
    S_ENTER      = 'ENTER'       # 进入横向通道
    S_GOTO_CH1   = 'GOTO_CH1'    # 前往通道1
    S_EXPLORE_CH = 'EXPLORE_CH'  # 探索当前通道
    S_INTERACT   = 'INTERACT'    # 与目标交互
    S_AVOID      = 'AVOID'       # 避障
    S_BACK_MAIN  = 'BACK_MAIN'   # 退回横向通道
    S_EXIT       = 'EXIT'        # 前往出口（独木桥）

    def __init__(self, ctrl):
        self.ctrl = ctrl
        self._cap = None
        self._frame = None
        self._frame_lock = threading.Lock()
        self._running = False

        self._sub_state = self.S_ENTER
        self._channel = 0          # 当前探索的通道编号 1/2/3
        self._tasks_done = set()   # 已完成的任务
        self._timer = 0.0
        self._interact_target = None

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
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_LOW, ORANGE_HIGH)
        return self._largest_contour_info(mask, min_area=300)

    def _detect_red_bar(self, frame):
        """检测红色限高杆"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, RED_LOW1, RED_HIGH1)
        mask2 = cv2.inRange(hsv, RED_LOW2, RED_HIGH2)
        mask = cv2.bitwise_or(mask1, mask2)
        return self._largest_contour_info(mask, min_area=500)

    def _detect_obstacle(self, frame):
        """检测障碍物（灰色方块，通过形状+颜色）"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
        return self._largest_contour_info(thresh, min_area=1000)

    def _detect_football(self, frame):
        """检测足球（白色圆形）"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv,
                                 np.array([0, 0, 200]),
                                 np.array([180, 30, 255]))
        return self._largest_contour_info(white_mask, min_area=300)

    def _detect_coke(self, frame):
        """检测可乐瓶（黑色圆柱）"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        black_mask = cv2.inRange(hsv,
                                 np.array([0, 0, 0]),
                                 np.array([180, 255, 50]))
        return self._largest_contour_info(black_mask, min_area=300)

    def _largest_contour_info(self, mask, min_area=200):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < min_area:
            return None
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        return cx, cy, area

    def _align_and_approach(self, frame, detect_fn, vx=VX_EXPLORE):
        """对准目标并前进，返回(已对准, 目标信息)"""
        w = frame.shape[1]
        result = detect_fn(frame)
        if result is None:
            return False, None
        cx, cy, area = result
        error = cx - w / 2.0
        vyaw = -Kp_yaw * error
        self.ctrl.move(vx=vx, vyaw=vyaw)
        aligned = abs(error) < 30 and area > 2000
        return aligned, result

    def step(self, current_y):
        if self._cap is None:
            self._start_camera()

        frame = self._get_frame()
        if frame is None:
            self.ctrl.move(vx=VX_EXPLORE)
            return False

        # --- 子状态机 ---
        if self._sub_state == self.S_ENTER:
            # 进入横向通道，直行
            self.ctrl.move(vx=VX_EXPLORE)
            self._channel = 1
            self._sub_state = self.S_EXPLORE_CH
            self._timer = time.monotonic()

        elif self._sub_state == self.S_EXPLORE_CH:
            # 在当前通道内探索，检测所有目标和障碍
            found = self._scan_channel(frame)
            if not found:
                # 通道内没发现，前进一段后退回
                if time.monotonic() - self._timer > 3.0:
                    self._sub_state = self.S_BACK_MAIN

        elif self._sub_state == self.S_INTERACT:
            done = self._do_interact(frame)
            if done:
                self._tasks_done.add(self._interact_target)
                print(f"[赛段四] 任务完成: {self._interact_target}，"
                      f"已完成 {len(self._tasks_done)}/3")
                self._sub_state = self.S_BACK_MAIN
                self._timer = time.monotonic()

        elif self._sub_state == self.S_AVOID:
            done = self._do_avoid(frame)
            if done:
                self._sub_state = self.S_EXPLORE_CH
                self._timer = time.monotonic()

        elif self._sub_state == self.S_BACK_MAIN:
            # 后退回横向通道
            self.ctrl.move(vx=VX_BACK)
            if time.monotonic() - self._timer > 1.5:
                if len(self._tasks_done) >= 3:
                    print("[赛段四] 三个任务全部完成！前往独木桥")
                    self._sub_state = self.S_EXIT
                elif self._channel < 3:
                    self._channel += 1
                    print(f"[赛段四] 前往通道 {self._channel}")
                    self._sub_state = self.S_GOTO_CH1
                    self._timer = time.monotonic()
                else:
                    # 三个通道都探索完了
                    self._sub_state = self.S_EXIT

        elif self._sub_state == self.S_GOTO_CH1:
            # 横向移动到下一个通道（向右平移）
            self.ctrl.move(vx=0.0, vy=-0.3)
            if time.monotonic() - self._timer > 2.0:
                self._sub_state = self.S_EXPLORE_CH
                self._timer = time.monotonic()

        elif self._sub_state == self.S_EXIT:
            # 前往右下角独木桥入口
            self.ctrl.move(vx=0.3)
            return True

        return False

    def _scan_channel(self, frame):
        """扫描通道，发现目标或障碍则切换状态"""
        # 优先检测障碍
        red_bar = self._detect_red_bar(frame)
        if red_bar and 'bar' not in self._tasks_done:
            speak('识别到限高杆')
            print("[赛段四] 识别到限高杆")
            self._interact_target = 'bar'
            self._sub_state = self.S_AVOID
            return True

        obstacle = self._detect_obstacle(frame)
        if obstacle and obstacle[2] > 3000 and 'obstacle' not in self._tasks_done:
            speak('识别到无法跨越障碍')
            print("[赛段四] 识别到无法跨越障碍")
            self._interact_target = 'obstacle'
            self._sub_state = self.S_AVOID
            return True

        # 检测目标物体
        if 'orange_ball' not in self._tasks_done:
            ball = self._detect_orange_ball(frame)
            if ball:
                speak('识别到橙色小球')
                print("[赛段四] 识别到橙色小球")
                self._interact_target = 'orange_ball'
                self._sub_state = self.S_INTERACT
                self._timer = time.monotonic()
                return True

        if 'coke' not in self._tasks_done:
            coke = self._detect_coke(frame)
            if coke:
                speak('识别到可乐瓶')
                print("[赛段四] 识别到可乐瓶")
                self._interact_target = 'coke'
                self._sub_state = self.S_INTERACT
                self._timer = time.monotonic()
                return True

        if 'football' not in self._tasks_done:
            fb = self._detect_football(frame)
            if fb:
                speak('识别到足球')
                print("[赛段四] 识别到足球")
                self._interact_target = 'football'
                self._sub_state = self.S_INTERACT
                self._timer = time.monotonic()
                return True

        # 继续前进探索
        self.ctrl.move(vx=VX_EXPLORE)
        return False

    def _do_interact(self, frame):
        """执行与目标的交互，返回True表示完成"""
        if self._interact_target == 'orange_ball':
            aligned, info = self._align_and_approach(
                frame, self._detect_orange_ball)
            if info and info[2] > 4000:
                self.ctrl.move(vx=VX_HIT)
                time.sleep(0.5)
                return True

        elif self._interact_target == 'coke':
            aligned, info = self._align_and_approach(
                frame, self._detect_coke)
            if info and info[2] > 4000:
                self.ctrl.move(vx=VX_HIT)
                time.sleep(0.8)
                return True

        elif self._interact_target == 'football':
            # 足球需要踢入球门，多次撞击
            aligned, info = self._align_and_approach(
                frame, self._detect_football)
            if info and info[2] > 4000:
                self.ctrl.move(vx=VX_HIT)
                time.sleep(1.0)
                return True

        # 超时保护
        if time.monotonic() - self._timer > 10.0:
            print(f"[赛段四] {self._interact_target} 交互超时，跳过")
            return True

        return False

    def _do_avoid(self, frame):
        """执行避障，返回True表示完成"""
        if self._interact_target == 'bar':
            # 限高杆：降低体高通过
            print("[赛段四] 降低体高通过限高杆")
            self.ctrl.move(vx=0.15, body_height=LOW_BODY_H)
            time.sleep(3.0)
            self.ctrl.move(vx=0.0, body_height=0.0)
            self._tasks_done.add('bar')
            return True

        elif self._interact_target == 'obstacle':
            # 障碍物：横向绕行
            print("[赛段四] 绕行障碍物")
            self.ctrl.move(vx=0.0, vy=0.4)
            time.sleep(1.5)
            self.ctrl.move(vx=0.3, vy=0.0)
            time.sleep(1.5)
            self.ctrl.move(vx=0.0, vy=-0.4)
            time.sleep(1.5)
            self._tasks_done.add('obstacle')
            return True

        return True

    def cleanup(self):
        self._running = False
        if self._cap:
            self._cap.release()
        self.ctrl.stop()
        print("[赛段四] 结束")
