#!/usr/bin/env python3
"""
比赛主控：石板路 → 手动过渡 → 赛段二+S弯
用法：python3 race_main.py

石板路运行中按 1 停止石板路，进入手动控制
手动控制中按 q 退出，自动启动赛段二(test_ball_hit.py)
"""

import sys
import os
import time
import threading
import termios
import tty
import select
import subprocess
import math

sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')

import lcm
import robot_control_cmd_lcmt
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# LCM
LCM_URL = "udpm://239.255.76.67:7671?ttl=255"
lc = lcm.LCM(LCM_URL)
count = 0
PITCH = 0.30

def cmd(mode, gait=0, vx=0.0, vy=0.0, vyaw=0.0, step_height=0.08):
    global count
    count = (count + 1) % 127
    m = robot_control_cmd_lcmt.robot_control_cmd_lcmt()
    m.mode = mode
    m.gait_id = gait
    m.life_count = count
    m.duration = 0
    m.vel_des = [vx, vy, vyaw]
    m.rpy_des = [0.0, PITCH, 0.0]
    m.pos_des = [0.0, 0.0, 0.0]
    m.acc_des = [0.0] * 6
    m.ctrl_point = [0.0] * 3
    m.foot_pose = [0.0] * 6
    m.step_height = [step_height, step_height]
    m.contact = 0
    m.value = 0
    lc.publish("robot_control_cmd", m.encode())


def get_key(timeout=0.1):
    """非阻塞读取键盘输入"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return None


# TF yaw
current_yaw = None

class YawNode(Node):
    def __init__(self):
        super().__init__('race_main_yaw')
        self.create_subscription(TFMessage, '/tf', self._tf_cb, 10)

    def _tf_cb(self, msg):
        global current_yaw
        for tf in msg.transforms:
            if tf.child_frame_id in ('body', 'base_link'):
                q = tf.transform.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                current_yaw = math.atan2(siny, cosy)
                break


def align_yaw(target_yaw, tolerance=0.05):
    """精确对齐到目标角度"""
    print(f"  对齐方向到 {math.degrees(target_yaw):.1f}°...")
    while current_yaw is None:
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    stable_start = None
    while True:
        err = target_yaw - current_yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi

        if abs(err) < tolerance:
            if stable_start is None:
                stable_start = time.time()
            elif time.time() - stable_start > 1.0:
                cmd(11, 9, vx=0.0)
                break
            cmd(11, 9, vx=0.0, vyaw=0.0)
        else:
            stable_start = None
            vyaw = max(-0.4, min(0.4, 0.8 * err))
            cmd(11, 9, vx=0.0, vyaw=vyaw)

        print(f'\r  yaw={math.degrees(current_yaw):.1f}° 差={math.degrees(err):.1f}°',
              end='', flush=True)
        time.sleep(0.05)
    print(f"\n  对齐完成: {math.degrees(current_yaw):.1f}°")


def manual_control():
    """手动控制模式，WASD移动，q退出"""
    print("\n" + "="*40)
    print("手动控制模式")
    print("  w/s - 前进/后退")
    print("  a/d - 左转/右转")
    print("  z/c - 左平移/右平移")
    print("  空格 - 停止")
    print("  q   - 退出手动，启动赛段二")
    print("="*40)

    vx, vy, vyaw = 0.0, 0.0, 0.0

    while True:
        key = get_key(0.1)

        if key == 'q':
            cmd(11, 9, vx=0.0)
            print("\n退出手动控制，准备启动赛段二...")
            return
        elif key == 'w':
            vx = 0.15
            vy, vyaw = 0.0, 0.0
        elif key == 's':
            vx = -0.15
            vy, vyaw = 0.0, 0.0
        elif key == 'a':
            vyaw = 0.4
            vx, vy = 0.0, 0.0
        elif key == 'd':
            vyaw = -0.4
            vx, vy = 0.0, 0.0
        elif key == 'z':
            vy = 0.12
            vx, vyaw = 0.0, 0.0
        elif key == 'c':
            vy = -0.12
            vx, vyaw = 0.0, 0.0
        elif key == ' ':
            vx, vy, vyaw = 0.0, 0.0, 0.0

        cmd(11, 3, vx=vx, vy=vy, vyaw=vyaw)
        print(f'\r  vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f}', end='', flush=True)


def main():
    # ROS初始化
    rclpy.init()
    node = YawNode()
    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()

    # 站立
    print("站立...")
    for _ in range(25):
        cmd(12)
        time.sleep(0.1)
    for _ in range(30):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    # 预热tf
    print("预热tf...")
    for _ in range(60):
        cmd(11, 9, vx=0.05)
        time.sleep(0.05)
    for _ in range(10):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    print("\n" + "="*40)
    print("赛段一：石板路（倒走）")
    print("="*40)

    # 对齐-180度
    align_yaw(-math.pi, tolerance=0.05)

    # 倒走，按1停止
    print("\n开始倒走... 按 1 停止石板路")
    while True:
        key = get_key(0.05)
        if key == '1':
            print("\n\n石板路停止！")
            break
        cmd(11, 3, vx=-0.20, vyaw=0.0, step_height=0.06)

    # 保持站立不趴下
    for _ in range(10):
        cmd(11, 3, vx=0.0)
        time.sleep(0.05)

    # 手动控制过渡到赛段二入口
    manual_control()

    time.sleep(1.0)

    # 启动赛段二
    print("\n启动赛段二+S弯...")
    os.execvp('python3', ['python3', '/home/cyberdog_sim/test_ball_hit.py'])


if __name__ == '__main__':
    main()
