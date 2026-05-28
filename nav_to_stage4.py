#!/usr/bin/env python3
"""
坐标导航：从S弯出口移动到赛段四入口
订阅 /tf 获取机器狗位置，直走到目标坐标后停止

用法：python3.8 nav_to_stage4.py
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── 目标坐标（赛段四入口，根据 race.world 推算）──
# 可乐瓶 x=-0.1 y=11.1，通道入口大约在 y=8.5 附近
TARGET_X = 1.0    # 通道中间 X 坐标（根据实际调整）
TARGET_Y = 8.5    # 赛段四入口 Y 坐标（根据实际调整）
ARRIVE_DIST = 0.3 # 到达判定距离（米）

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

# ── 位置订阅 ──────────────────────────────────────────────
robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0
pos_ready = False

class TFNode(Node):
    def __init__(self):
        super().__init__('nav_tf')
        self.create_subscription(TFMessage, '/tf', self._cb, 10)

    def _cb(self, msg):
        global robot_x, robot_y, robot_yaw, pos_ready
        for t in msg.transforms:
            if t.child_frame_id == 'base_link' or t.child_frame_id == 'body':
                robot_x = t.transform.translation.x
                robot_y = t.transform.translation.y
                # 从四元数提取 yaw
                q = t.transform.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                robot_yaw = math.atan2(siny, cosy)
                pos_ready = True

# ── 导航逻辑 ──────────────────────────────────────────────
def navigate():
    """直走到目标点"""
    VX = 0.20
    Kp_yaw = 2.0  # 航向修正增益

    print(f"目标坐标: ({TARGET_X:.1f}, {TARGET_Y:.1f})")
    print("等待位置信息...")

    while not pos_ready:
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    print(f"当前位置: ({robot_x:.2f}, {robot_y:.2f}), yaw={math.degrees(robot_yaw):.1f}°")

    while True:
        dx = TARGET_X - robot_x
        dy = TARGET_Y - robot_y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < ARRIVE_DIST:
            cmd(11, 9, vx=0.0, vyaw=0.0)
            print(f"\n到达目标！当前位置: ({robot_x:.2f}, {robot_y:.2f})")
            return True

        # 计算目标航向
        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - robot_yaw

        # 归一化到 [-pi, pi]
        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        # 航向偏差大时原地转，小时边走边修正
        if abs(yaw_err) > 0.5:
            # 原地转向对准目标
            vyaw = Kp_yaw * yaw_err
            vyaw = max(-1.0, min(1.0, vyaw))
            cmd(11, 9, vx=0.0, vyaw=vyaw)
        else:
            # 边走边修正
            vyaw = Kp_yaw * yaw_err
            vyaw = max(-0.5, min(0.5, vyaw))
            cmd(11, 9, vx=VX, vyaw=vyaw)

        print(f'\r位置:({robot_x:.2f},{robot_y:.2f}) 距离:{dist:.2f}m 航向差:{math.degrees(yaw_err):.1f}°', end='', flush=True)
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

    print("开始导航到赛段四入口")
    try:
        navigate()
    except KeyboardInterrupt:
        pass

    cmd(11, 9, vx=0.0, vyaw=0.0)
    print("\n导航结束")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
