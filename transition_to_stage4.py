#!/usr/bin/env python3
"""
S弯出口 → 赛段四入口 过渡脚本 v2
基于坐标实时导航，先转向再前进

用法：python3.8 transition_to_stage4.py
"""

import sys
import time
import threading
import math

sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')

import lcm
import robot_control_cmd_lcmt
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

# ── 目标坐标（赛段四入口，需要根据实际调整）──────────────
# 从截图看，赛段四入口在机器狗左前方
# 当前位置 x=13.38 y=12.00，需要往X负方向移动
# ── 目标坐标（/tf坐标系，手动走过去测得）──────────────────
TARGET_X = -0.10  # 赛段四入口X
TARGET_Y = 8.73   # 赛段四入口Y
ARRIVE_DIST = 0.3

# ── LCM ───────────────────────────────────────────────────
LCM_URL = "udpm://239.255.76.67:7671?ttl=255"
lc = lcm.LCM(LCM_URL)
count = 0

def cmd(mode, gait=0, vx=0.0, vy=0.0, vyaw=0.0):
    global count
    count = (count + 1) % 127
    m = robot_control_cmd_lcmt.robot_control_cmd_lcmt()
    m.mode = mode
    m.gait_id = gait
    m.life_count = count
    m.duration = 0
    m.vel_des = [vx, vy, vyaw]
    m.rpy_des = [0.0, 0.0, 0.0]
    m.pos_des = [0.0, 0.0, 0.0]
    m.acc_des = [0.0] * 6
    m.ctrl_point = [0.0] * 3
    m.foot_pose = [0.0] * 6
    m.step_height = [0.08, 0.08]
    m.contact = 0
    m.value = 0
    lc.publish("robot_control_cmd", m.encode())

# ── 位置监控 ──────────────────────────────────────────────
robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0
pos_ready = False

class TFNode(Node):
    def __init__(self):
        super().__init__('transition_tf')
        # 尝试订阅 /tf
        self.create_subscription(TFMessage, '/tf', self._cb, 10)
    def _cb(self, msg):
        global robot_x, robot_y, robot_yaw, pos_ready
        for t in msg.transforms:
            if t.child_frame_id in ('base_link', 'body'):
                robot_x = t.transform.translation.x
                robot_y = t.transform.translation.y
                q = t.transform.rotation
                robot_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                pos_ready = True

# ── 导航函数 ──────────────────────────────────────────────
def turn_to(target_yaw, tolerance=0.1):
    """原地转向到目标航向，稳定3秒后停止"""
    Kp = 1.5
    stable_start = None
    while True:
        err = target_yaw - robot_yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi

        if abs(err) < tolerance:
            if stable_start is None:
                stable_start = time.time()
            elif time.time() - stable_start > 3.0:
                cmd(11, 9, vx=0.0, vyaw=0.0)
                print()
                return
            # 继续发停止指令保持
            cmd(11, 9, vx=0.0, vyaw=0.0)
        else:
            stable_start = None
            vyaw = Kp * err
            vyaw = max(-0.8, min(0.8, vyaw))
            cmd(11, 9, vx=0.0, vyaw=vyaw)

        print(f'\r  转向中: yaw={math.degrees(robot_yaw):.1f}° 目标={math.degrees(target_yaw):.1f}° 差={math.degrees(err):.1f}°', end='', flush=True)
        time.sleep(0.05)

def move_timed(vx, vy, vyaw, sec, label=""):
    """按指定速度移动指定时间"""
    print(f"  {label}：vx={vx} vy={vy} vyaw={vyaw} 持续{sec}秒")
    start = time.time()
    while time.time() - start < sec:
        cmd(11, 9, vx=vx, vy=vy, vyaw=vyaw)
        elapsed = time.time() - start
        print(f'\r  {label} t={elapsed:.1f}/{sec:.1f}s 位置:({robot_x:.2f},{robot_y:.2f})', end='', flush=True)
        time.sleep(0.05)
    cmd(11, 9, vx=0.0, vy=0.0, vyaw=0.0)
    print()

def go_to(tx, ty):
    """走到目标坐标"""
    VX = 0.25
    Kp_yaw = 1.5

    while True:
        dx = tx - robot_x
        dy = ty - robot_y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < ARRIVE_DIST:
            cmd(11, 9, vx=0.0, vyaw=0.0)
            print(f'\n  到达！位置: ({robot_x:.2f}, {robot_y:.2f})')
            return

        # 目标航向
        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - robot_yaw
        while yaw_err > math.pi: yaw_err -= 2*math.pi
        while yaw_err < -math.pi: yaw_err += 2*math.pi

        # 航向偏差大时减速
        if abs(yaw_err) > 0.5:
            vx = 0.05
        else:
            vx = VX

        vyaw = Kp_yaw * yaw_err
        vyaw = max(-0.8, min(0.8, vyaw))
        cmd(11, 9, vx=vx, vyaw=vyaw)
        print(f'\r  位置:({robot_x:.2f},{robot_y:.2f}) 距离:{dist:.2f}m 航向差:{math.degrees(yaw_err):.1f}°', end='', flush=True)
        time.sleep(0.05)

# ── 主程序 ────────────────────────────────────────────────
def main():
    rclpy.init()
    node = TFNode()
    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()

    # 站立
    print("站立...")
    for _ in range(25):
        cmd(12)
        time.sleep(0.1)
    for _ in range(10):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    # 等待位置
    print("等待位置信息...")
    while not pos_ready:
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    print(f"起始位置: ({robot_x:.2f}, {robot_y:.2f}) yaw={math.degrees(robot_yaw):.1f}°")
    print(f"目标位置: ({TARGET_X:.2f}, {TARGET_Y:.2f})")

    try:
        print(f"目标坐标: ({TARGET_X:.2f}, {TARGET_Y:.2f})")

        # 第一步：转向面朝Y轴正方向
        print(f"\n[1/3] 转向面朝Y轴正方向（90°）...")
        turn_to(math.pi / 2)

        # 第二步：走到目标点
        print(f"\n[2/3] 导航到目标点...")
        go_to(TARGET_X, TARGET_Y)

        # 第三步：调整朝向面朝Y轴正方向
        print(f"\n[3/3] 调整朝向面朝赛段四...")
        turn_to(math.pi / 2)

        print(f"\n完成！最终位置: ({robot_x:.2f}, {robot_y:.2f}) yaw={math.degrees(robot_yaw):.1f}°")

    except KeyboardInterrupt:
        cmd(11, 9, vx=0.0, vyaw=0.0)
        print("\n中断")

    cmd(11, 9, vx=0.0, vyaw=0.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
