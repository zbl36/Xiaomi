#!/usr/bin/env python3
"""
黄线跟踪测试脚本
订阅相机图像，实时显示黄线检测结果，并控制机器狗跟踪黄线
用法：python3.8 test_lane_follow.py
按 s 开始跟踪，按 x 停止，按 q 退出
"""

import cv2
import numpy as np
import threading
import time
import sys
import os

sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')

import lcm
import robot_control_cmd_lcmt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── 黄线 HSV 范围 ──────────────────────────────────────────
YELLOW_LOW  = np.array([20, 100, 100])
YELLOW_HIGH = np.array([35, 255, 255])

# ── PID 参数（可调）────────────────────────────────────────
Kp = 0.004   # 比例
Ki = 0.0001  # 积分
Kd = 0.002   # 微分

# ── 运动参数 ───────────────────────────────────────────────
VX          = 0.25   # 前进速度
PITCH       = 0.20   # 前倾角度
VYAW_MAX    = 0.8    # 最大转向速度

# ── LCM ───────────────────────────────────────────────────
LCM_URL      = "udpm://239.255.76.67:7671?ttl=255"
MODE_LOCOMOTION = 11
GAIT_TROT       = 9

lc   = lcm.LCM(LCM_URL)
msg  = robot_control_cmd_lcmt.robot_control_cmd_lcmt()
life = 0
lock = threading.Lock()

def send(vx=0.0, vyaw=0.0):
    global life
    with lock:
        life = (life + 1) % 127
        msg.mode        = MODE_LOCOMOTION
        msg.gait_id     = GAIT_TROT
        msg.life_count  = life
        msg.duration    = 200
        msg.vel_des     = [vx, 0.0, vyaw]
        msg.rpy_des     = [0.0, PITCH, 0.0]
        msg.pos_des     = [0.0, 0.0, 0.0]
        msg.acc_des     = [0.0] * 6
        msg.ctrl_point  = [0.0] * 3
        msg.foot_pose   = [0.0] * 6
        msg.step_height = [0.08, 0.08]
        msg.contact     = 0
        msg.value       = 0
        lc.publish("robot_control_cmd", msg.encode())


# ── 相机订阅 ───────────────────────────────────────────────
class CamNode(Node):
    def __init__(self):
        super().__init__('lane_test')
        self.frame = None
        self._lock = threading.Lock()
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/rgb_camera/image_raw', self._cb, qos)

    def _cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ('rgb8', 'RGB8'):
            arr = arr[:, :, ::-1].copy()
        with self._lock:
            self.frame = arr

    def get(self):
        with self._lock:
            return self.frame.copy() if self.frame is not None else None


# ── 黄线检测 ───────────────────────────────────────────────
def detect_lane(frame):
    """
    返回 (error, debug_frame)
    error > 0 偏右，error < 0 偏左
    """
    h, w = frame.shape[:2]
    # 只看下半部分
    roi = frame[h // 2:, :]

    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)

    # 形态学去噪
    k    = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)

    # 分左右找黄线
    left_mask  = mask[:, :w // 2]
    right_mask = mask[:, w // 2:]

    left_cx  = _cx(left_mask)
    right_cx = _cx(right_mask)

    debug = roi.copy()
    cv2.imshow('mask', mask)

    if left_cx is None and right_cx is None:
        cv2.putText(debug, 'NO LINE', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return 0.0, debug

    if left_cx is None:
        lane_cx = w // 2 + right_cx - w * 0.25
    elif right_cx is None:
        lane_cx = left_cx + w * 0.25
    else:
        lane_cx = (left_cx + (w // 2 + right_cx)) / 2.0

    error = lane_cx - w / 2.0

    # 画辅助线
    cv2.line(debug, (int(lane_cx), 0), (int(lane_cx), roi.shape[0]), (0, 255, 0), 2)
    cv2.line(debug, (w // 2, 0), (w // 2, roi.shape[0]), (255, 0, 0), 1)
    cv2.putText(debug, f'err={error:.1f}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    return error, debug


def _cx(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 500:
        return None
    M = cv2.moments(c)
    return M['m10'] / M['m00'] if M['m00'] else None


# ── 主循环 ─────────────────────────────────────────────────
def startup():
    """启动流程：恢复站立 → 进入行走模式"""
    print("启动中：恢复站立...")
    with lock:
        life_count = 1
        msg.mode = 12  # recovery
        msg.gait_id = 0
        msg.life_count = life_count
        msg.duration = 2000
        msg.vel_des = [0.0, 0.0, 0.0]
        msg.rpy_des = [0.0, 0.0, 0.0]
        msg.pos_des = [0.0, 0.0, 0.0]
        msg.acc_des = [0.0] * 6
        msg.ctrl_point = [0.0] * 3
        msg.foot_pose = [0.0] * 6
        msg.step_height = [0.08, 0.08]
        msg.contact = 0
        msg.value = 0
        lc.publish("robot_control_cmd", msg.encode())
    time.sleep(2.5)

    print("进入前倾行走模式...")
    send(vx=0.0, vyaw=0.0)
    time.sleep(0.5)
    print("就绪，按 s 开始跟踪\n")
    rclpy.init()
    cam = CamNode()
    spin_t = threading.Thread(target=lambda: rclpy.spin(cam), daemon=True)
    spin_t.start()

    # 自动站立进入行走模式
    startup()

    integral   = 0.0
    last_error = 0.0
    running    = False

    print("黄线跟踪测试")
    print("  s - 开始跟踪")
    print("  x - 停止")
    print("  q - 退出")
    print(f"  PID: Kp={Kp} Ki={Ki} Kd={Kd}  VX={VX}")

    while True:
        frame = cam.get()
        if frame is None:
            time.sleep(0.05)
            continue

        error, debug = detect_lane(frame)

        # PID
        integral   += error
        integral    = max(-300, min(300, integral))
        derivative  = error - last_error
        last_error  = error
        vyaw        = -(Kp * error + Ki * integral + Kd * derivative)
        vyaw        = max(-VYAW_MAX, min(VYAW_MAX, vyaw))

        if running:
            send(vx=VX, vyaw=vyaw)

        # 显示
        cv2.putText(debug, 'RUNNING' if running else 'STOPPED',
                    (debug.shape[1] - 150, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if running else (0, 0, 255), 2)
        cv2.imshow('lane follow', debug)

        key = cv2.waitKey(33) & 0xFF
        if key == ord('s'):
            running = True
            integral = 0.0
            print("开始跟踪")
        elif key == ord('x'):
            running = False
            send(vx=0.0, vyaw=0.0)
            print("停止")
        elif key == ord('q'):
            break

        # 实时调参（方向键调 Kp）
        # 可在此扩展

    send(vx=0.0, vyaw=0.0)
    cv2.destroyAllWindows()
    cam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
