#!/usr/bin/env python3
"""
相对位移导航脚本
从当前位置出发，按相对位移走到目标点
方便拖动机器狗到任意位置直接测试

用法：python3.8 nav_relative.py
参数在脚本顶部修改
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

# ── 相对位移参数（从当前位置出发）─────────────────────────
# 正X = 右，负X = 左
# 正Y = 前（面朝方向），负Y = 后
# 先转向90度，再左移，再前进
REL_X = -3.2    # 左移3.2m（负值=左）
REL_Y = 1.5     # 前进1.5m（正值=前）
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
        super().__init__('nav_rel_tf')
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
def turn_to(target_yaw, tolerance=0.017):
    """原地转向到目标航向，误差<1度，稳定3秒后停止"""
    Kp = 0.8
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
            cmd(11, 9, vx=0.0, vyaw=0.0)
        else:
            stable_start = None
            vyaw = Kp * err
            vyaw = max(-0.4, min(0.4, vyaw))
            cmd(11, 9, vx=0.0, vyaw=vyaw)

        print(f'\r  转向: yaw={math.degrees(robot_yaw):.1f}° 目标={math.degrees(target_yaw):.1f}° 差={math.degrees(err):.1f}°', end='', flush=True)
        time.sleep(0.05)

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

        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - robot_yaw
        while yaw_err > math.pi: yaw_err -= 2*math.pi
        while yaw_err < -math.pi: yaw_err += 2*math.pi

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

    # 站立 + 进入行走模式（让/tf开始更新）
    print("站立...")
    for _ in range(25):
        cmd(12)
        time.sleep(0.1)
    print("进入行走模式，等待坐标稳定...")
    for _ in range(50):  # 原地踏步5秒，让/tf更新
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    # 等待位置
    print("等待位置信息...")
    while not pos_ready:
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    start_x = robot_x
    start_y = robot_y
    # 计算绝对目标坐标（基于当前位置+相对位移）
    target_x = start_x + REL_X
    target_y = start_y + REL_Y

    print(f"起始位置: ({start_x:.2f}, {start_y:.2f}) yaw={math.degrees(robot_yaw):.1f}°")
    print(f"相对位移: dx={REL_X}, dy={REL_Y}")
    print(f"目标坐标: ({target_x:.2f}, {target_y:.2f})")

    try:
        # 第一步：转向面朝Y轴正方向
        print(f"\n[1/4] 转向面朝Y轴正方向（90°）...")
        turn_to(math.pi / 2)

        # 第二步：纯左平移到目标X（保持朝向不变）
        print(f"\n[2/4] 左平移到 X={target_x:.2f}...")
        while abs(robot_x - target_x) > 0.2:
            dx = target_x - robot_x
            vy = 0.15 if dx < 0 else -0.15  # dx<0目标在左，vy正值=左移
            cmd(11, 9, vx=0.0, vy=vy, vyaw=0.0)
            print(f'\r  平移: x={robot_x:.2f} 目标={target_x:.2f} 差={dx:.2f}m', end='', flush=True)
            time.sleep(0.05)
        cmd(11, 9, vx=0.0, vy=0.0, vyaw=0.0)
        print(f'\n  平移完成: x={robot_x:.2f}')

        # 第三步：纯前进到目标Y
        print(f"\n[3/4] 前进到 Y={target_y:.2f}...")
        while abs(robot_y - target_y) > 0.2:
            dy = target_y - robot_y
            vx = 0.20 if dy > 0 else -0.20
            cmd(11, 9, vx=vx, vy=0.0, vyaw=0.0)
            print(f'\r  前进: y={robot_y:.2f} 目标={target_y:.2f} 差={dy:.2f}m', end='', flush=True)
            time.sleep(0.05)
        cmd(11, 9, vx=0.0, vy=0.0, vyaw=0.0)
        print(f'\n  前进完成: y={robot_y:.2f}')

        # 第四步：调整朝向
        print(f"\n[4/4] 调整朝向面朝赛段四...")
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
