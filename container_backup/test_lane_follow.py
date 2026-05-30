#!/usr/bin/env python3
"""
黄线跟踪 v3
只做两件事：检测黄线 + 发LCM指令
"""

import cv2
import numpy as np
import threading
import time
import sys

sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')

import lcm
import robot_control_cmd_lcmt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── 参数 ───────────────────────────────────────────────────
YELLOW_LOW  = np.array([25, 80, 200])
YELLOW_HIGH = np.array([35, 255, 255])
Kp          = 0.010
VX          = 0.15
VYAW_MAX    = 1.0

# 斜坡检测 HSV 范围（亮灰色，V高S低）
SLOPE_LOW   = np.array([70, 0, 90])
SLOPE_HIGH  = np.array([120, 50, 140])
SLOPE_AREA_THRESH = 5000  # 斜坡面积阈值（像素）

# ── LCM 直接发送 ──────────────────────────────────────────
LCM_URL = "udpm://239.255.76.67:7671?ttl=255"
lc = lcm.LCM(LCM_URL)
count = 0

def cmd(mode, gait=0, vx=0.0, vyaw=0.0):
    global count
    count = (count + 1) % 127
    m = robot_control_cmd_lcmt.robot_control_cmd_lcmt()
    m.mode = mode
    m.gait_id = gait
    m.life_count = count
    m.duration = 0  # 0=持续执行直到收到新指令
    m.vel_des = [vx, 0.0, vyaw]
    m.rpy_des = [0.0, 0.30, 0.0]  # 前倾30度看近处黄线
    m.pos_des = [0.0, 0.0, 0.0]
    m.acc_des = [0.0] * 6
    m.ctrl_point = [0.0] * 3
    m.foot_pose = [0.0] * 6
    m.step_height = [0.08, 0.08]
    m.contact = 0
    m.value = 0
    lc.publish("robot_control_cmd", m.encode())

# ── 相机 ──────────────────────────────────────────────────
frame_now = None

class Cam(Node):
    def __init__(self):
        super().__init__('cam')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/rgb_camera/image_raw', self._cb, qos)
    def _cb(self, msg):
        global frame_now
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ('rgb8', 'RGB8'):
            a = a[:, :, ::-1].copy()
        frame_now = a

# ── 黄线检测（纯函数）────────────────────────────────────
def find_yellow(frame):
    """返回 error（像素偏差）或 None（没检测到）"""
    h, w = frame.shape[:2]
    # 只看最下 1/3
    roi = frame[h * 2 // 3:, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

    left_mask = mask[:, :w//2]
    right_mask = mask[:, w//2:]

    def cx(m):
        c, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not c: return None
        big = max(c, key=cv2.contourArea)
        if cv2.contourArea(big) < 1000: return None
        M = cv2.moments(big)
        return M['m10']/M['m00'] if M['m00'] else None

    lx = cx(left_mask)
    rx = cx(right_mask)

    cv2.imshow('mask', mask)
    cv2.waitKey(1)

    if lx is None and rx is None:
        return None

    if lx is not None and rx is not None:
        # 双边线：取两线中点，走中间
        lane = (lx + (w//2 + rx)) / 2.0
    elif rx is not None:
        # 只看到右线：右线应该在画面最右侧
        actual_rx = w//2 + rx
        target_rx = w * 9 // 10  # 从4/5改为9/10，更远离右线
        lane = w/2.0 + (actual_rx - target_rx)
    else:
        # 只看到左线：左线应该在画面最左侧
        target_lx = w // 10  # 从1/5改为1/10，更远离左线
        lane = w/2.0 + (lx - target_lx)

    return lane - w / 2.0

# ── 主程序 ────────────────────────────────────────────────
def main():
    global frame_now

    rclpy.init()
    node = Cam()
    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()

    # 站立：持续发指令直到站起来
    print("站立...")
    for i in range(30):
        cmd(12)  # recovery
        time.sleep(0.1)

    print("进入行走模式，按 Ctrl+C 退出")
    running = True

    try:
        while True:
            # 没有图像就发保活
            if frame_now is None:
                cmd(11, 9, vx=0.0, vyaw=0.0)
                time.sleep(0.05)
                continue

            # 取最新帧
            frame = frame_now.copy()

            # 检测斜坡（画面下方1/5）
            h, w = frame.shape[:2]
            slope_roi = frame[h*4//5:, :]
            slope_hsv = cv2.cvtColor(slope_roi, cv2.COLOR_BGR2HSV)
            slope_mask = cv2.inRange(slope_hsv, SLOPE_LOW, SLOPE_HIGH)
            slope_area = cv2.countNonZero(slope_mask)
            if slope_area > SLOPE_AREA_THRESH:
                cmd(11, 9, vx=0.0, vyaw=0.0)
                print(f'\n\nS弯结束！检测到斜坡（面积={slope_area}），停止。')
                break

            # 检测黄线
            err = find_yellow(frame)

            # 控制
            if err is not None:
                vyaw = -(Kp * err)
                vyaw = max(-VYAW_MAX, min(VYAW_MAX, vyaw))
                # 偏差太大说明可能看到隔壁赛道，减速
                if abs(err) > 100:
                    cmd(11, 9, vx=0.05, vyaw=vyaw)
                else:
                    cmd(11, 9, vx=VX, vyaw=vyaw)
                label = f'err={err:.1f} vyaw={vyaw:.3f}'
                print(f'\r{label}', end='', flush=True)
            else:
                cmd(11, 9, vx=-0.1, vyaw=0.0)  # 后退刹车
                label = 'NO LINE - BRAKE'
                print(f'\r{label}          ', end='', flush=True)

            # 显示
            h, w = frame.shape[:2]
            disp = frame[h*2//3:, :].copy()
            cv2.putText(disp, label, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # 显示（去掉GUI减少延迟）
            # cv2.imshow('lane follow', disp)
            # cv2.waitKey(1)

    except KeyboardInterrupt:
        pass

    cmd(11, 9, vx=0.0, vyaw=0.0)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
