#!/usr/bin/env python3
"""
赛段二：荒野寻珠 - S形路径撞击橙色球 v2

策略：
1. 从石板路出口出发，调整方向朝Y轴正向
2. 平移至3-4列之间
3. 直走，遇到橙色球就撞，撞完恢复方向和轨道
4. 走到顶部黄线 → 左转90° → 沿右侧黄线直走
5. 走到左侧黄线 → 左转90° → 平移至1-2列之间
6. 直走，遇到橙色球就撞
7. 走到底部黄线 → 准备进入S弯

用法：python3.8 test_ball_hit.py
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
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math

# ── 参数 ───────────────────────────────────────────────────
# 橙色球 HSV（包含蓝色球用于对称检测）
ORANGE_LOW  = np.array([5, 150, 60])
ORANGE_HIGH = np.array([18, 255, 150])
BLUE_LOW    = np.array([95, 100, 50])
BLUE_HIGH   = np.array([115, 255, 200])

# 黄线 HSV
YELLOW_LOW  = np.array([25, 80, 200])
YELLOW_HIGH = np.array([35, 255, 255])

# 控制参数
Kp_yaw      = 0.008
VX          = 0.15    # 前进速度
VX_HIT      = 0.30    # 撞击速度
VY_SHIFT    = 0.12    # 平移速度
VYAW_MAX    = 0.5
PITCH       = 0.30    # 前倾

# 检测阈值
HIT_AREA    = 2500    # 进入撞击模式的面积
YELLOW_AREA = 3000    # 黄线检测面积阈值
LOST_FRAMES = 20      # 球消失帧数

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
    m.rpy_des = [0.0, PITCH, 0.0]
    m.pos_des = [0.0, 0.0, 0.0]
    m.acc_des = [0.0] * 6
    m.ctrl_point = [0.0] * 3
    m.foot_pose = [0.0] * 6
    m.step_height = [0.08, 0.08]
    m.contact = 0
    m.value = 0
    lc.publish("robot_control_cmd", m.encode())

# ── 相机 + TF ─────────────────────────────────────────────
frame_now = None
current_yaw = None

class Cam(Node):
    def __init__(self):
        super().__init__('ball_cam')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/rgb_camera/image_raw', self._cb, qos)
        self.create_subscription(TFMessage, '/tf', self._tf_cb, 10)

    def _cb(self, msg):
        global frame_now
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ('rgb8', 'RGB8'):
            a = a[:, :, ::-1].copy()
        frame_now = a

    def _tf_cb(self, msg):
        global current_yaw
        for tf in msg.transforms:
            if tf.child_frame_id in ('body', 'base_link'):
                q = tf.transform.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                current_yaw = math.atan2(siny, cosy)
                break

# ── 检测函数 ──────────────────────────────────────────────
def detect_orange(frame):
    """检测橙色球，返回 (cx, cy, area) 或 None"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, ORANGE_LOW, ORANGE_HIGH)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < 200:
        return None
    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None
    return M['m10']/M['m00'], M['m01']/M['m00'], area


def detect_yellow_bottom(frame):
    """检测画面底部1/5是否有黄线"""
    h, w = frame.shape[:2]
    roi = frame[h*4//5:, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
    return cv2.countNonZero(mask) > YELLOW_AREA


# ── 动作函数 ──────────────────────────────────────────────
def wait_frame():
    """等待一帧图像"""
    while frame_now is None:
        cmd(11, 9, vx=0.0)
        time.sleep(0.05)
    return frame_now.copy()


def align_yaw(target_yaw=math.pi/2, tolerance=0.017):
    """
    调整机器狗朝向到目标yaw（默认Y轴正向=90°=pi/2）
    tolerance=0.017 rad ≈ 1°，稳定3秒后停止
    """
    print(f"  对齐方向到 {math.degrees(target_yaw):.1f}°...")
    timeout = 0
    while current_yaw is None:
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)
        timeout += 1
        print(f"\r  等待yaw... {timeout*0.1:.1f}s", end='', flush=True)
        if timeout > 100:
            print("\n  [警告] tf超时，跳过对齐")
            return
    print(f"\n  当前yaw={math.degrees(current_yaw):.1f}°")

    Kp = 0.8
    stable_start = None
    while True:
        err = target_yaw - current_yaw
        while err > math.pi: err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi

        if abs(err) < tolerance:
            if stable_start is None:
                stable_start = time.time()
            elif time.time() - stable_start > 3.0:
                cmd(11, 9, vx=0.0)
                break
            cmd(11, 9, vx=0.0, vyaw=0.0)
        else:
            stable_start = None
            vyaw = max(-0.4, min(0.4, Kp * err))
            cmd(11, 9, vx=0.0, vyaw=vyaw)

        print(f'\r  转向: yaw={math.degrees(current_yaw):.1f}° '
              f'目标={math.degrees(target_yaw):.1f}° '
              f'差={math.degrees(err):.1f}°', end='', flush=True)
        time.sleep(0.05)

    print(f"\n  方向对齐完成: {math.degrees(current_yaw):.1f}°")


def detect_all_balls(frame):
    """检测画面中所有球（橙+蓝），返回各球中心x坐标列表"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 橙色
    m1 = cv2.inRange(hsv, ORANGE_LOW, ORANGE_HIGH)
    # 蓝色
    m2 = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)
    mask = cv2.bitwise_or(m1, m2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    for c in contours:
        if cv2.contourArea(c) < 200:
            continue
        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        centers.append(M['m10'] / M['m00'])
    return centers


def shift_to_center():
    """
    左平移直到画面中球的分布左右对称
    判断标准：左半边球数 ≈ 右半边球数，或左右球的平均x接近画面中心
    """
    print("  视觉居中平移...")
    stable_count = 0

    while True:
        frame = wait_frame()
        h, w = frame.shape[:2]
        centers = detect_all_balls(frame)

        if len(centers) >= 2:
            avg_x = sum(centers) / len(centers)
            err = avg_x - w / 2.0
            print(f'\r  球数={len(centers)} avg_x={avg_x:.0f} '
                  f'center={w//2} err={err:.0f}', end='', flush=True)

            if abs(err) < 30:
                stable_count += 1
                if stable_count > 10:
                    print("\n  居中完成！")
                    break
                cmd(11, 9, vx=0.0)
            else:
                stable_count = 0
                # err>0 说明球偏右，需要右平移；err<0 说明球偏左，需要左平移
                vy = -0.08 if err > 0 else 0.08
                cmd(11, 9, vx=0.0, vy=vy)
        else:
            # 球太少，继续左平移
            cmd(11, 9, vx=0.0, vy=VY_SHIFT)
            stable_count = 0

        time.sleep(0.05)

    for _ in range(10):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)


def hit_ball():
    """撞击当前检测到的橙色球，直到球消失"""
    lost = 0

    while True:
        frame = wait_frame()
        result = detect_orange(frame)
        if result:
            cx, cy, area = result
            w = frame.shape[1]
            err = cx - w / 2.0
            vyaw = -(Kp_yaw * err)
            vyaw = max(-VYAW_MAX, min(VYAW_MAX, vyaw))
            lost = 0

            if area > HIT_AREA:
                cmd(11, 9, vx=VX_HIT, vyaw=vyaw)
                print(f'\r  [HIT] area={area:.0f}', end='', flush=True)
            else:
                cmd(11, 9, vx=VX, vyaw=vyaw)
                print(f'\r  [接近] err={err:.1f} area={area:.0f}', end='', flush=True)
        else:
            lost += 1
            cmd(11, 9, vx=VX_HIT, vyaw=0.0)
            if lost > LOST_FRAMES:
                print(f'\n  撞击完成！')
                return
        time.sleep(0.05)


def walk_until_yellow():
    """直走直到检测到画面底部有黄线，再走到黄线消失后停止"""
    print("  直走，等待黄线...")
    yellow_seen = False

    while True:
        frame = wait_frame()

        has_yellow = detect_yellow_bottom(frame)

        if not yellow_seen:
            # 还没看到黄线，检测橙色球边走边撞
            if has_yellow:
                yellow_seen = True
                print("  检测到黄线，继续走过...")
            else:
                # 检测橙色球
                result = detect_orange(frame)
                if result:
                    cx, cy, area = result
                    if area > 300:
                        w = frame.shape[1]
                        ball_side = 'left' if cx < w / 2.0 else 'right'
                        print(f"\n  发现橙色球！area={area:.0f} 在{ball_side}侧")
                        hit_ball()
                        recover_after_hit(ball_side)
                        print("  继续直走...")
                        continue
        else:
            # 已经看到黄线，等黄线消失
            if not has_yellow:
                cmd(11, 9, vx=0.0)
                print("  黄线消失，停止")
                for _ in range(10):
                    cmd(11, 9, vx=0.0)
                    time.sleep(0.1)
                return

        # 正常直走
        cmd(11, 9, vx=VX, vyaw=0.0)
        time.sleep(0.05)


def recover_after_hit(side):
    """撞击后恢复Y轴正方向，先平移一段再视觉居中"""
    # 停止
    for _ in range(10):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    # 用tf恢复Y轴正方向（5度误差即可）
    align_yaw(target_yaw=math.pi/2, tolerance=0.087)

    # 先向反方向平移一小段，远离球
    shift_time = 5.0
    if side == 'left':
        print("  先右平移远离球...")
        for _ in range(int(shift_time / 0.05)):
            cmd(11, 9, vx=0.0, vy=-VY_SHIFT)
            time.sleep(0.05)
    else:
        print("  先左平移远离球...")
        for _ in range(int(shift_time / 0.05)):
            cmd(11, 9, vx=0.0, vy=VY_SHIFT)
            time.sleep(0.05)

    # 停一下
    for _ in range(10):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    # 视觉居中恢复轨道
    shift_to_center()


def turn_left_90():
    """基于tf精确左转90°"""
    print("  左转90°...")
    if current_yaw is None:
        time.sleep(0.5)
    target = current_yaw + math.pi / 2
    # 归一化
    if target > math.pi: target -= 2 * math.pi
    align_yaw(target_yaw=target, tolerance=0.017)
    print("  转向完成")


def shift_left(sec=2.0):
    """左平移指定时间"""
    print(f"  左平移 {sec}秒...")
    for _ in range(int(sec / 0.05)):
        cmd(11, 9, vx=0.0, vy=VY_SHIFT)
        time.sleep(0.05)
    for _ in range(10):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)


def shift_right(sec=2.0):
    """右平移指定时间"""
    print(f"  右平移 {sec}秒...")
    for _ in range(int(sec / 0.05)):
        cmd(11, 9, vx=0.0, vy=-VY_SHIFT)
        time.sleep(0.05)
    for _ in range(10):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)


# ── 主程序 ────────────────────────────────────────────────
def main():
    global frame_now

    rclpy.init()
    node = Cam()
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

    # 预热：发送运动指令让tf开始更新
    print("预热tf...")
    for _ in range(60):
        cmd(11, 9, vx=0.05)
        time.sleep(0.05)
    # 停下来
    for _ in range(10):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)
    while current_yaw is None:
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)
        print("\r  等待tf数据...", end='', flush=True)
    print(f"\n  tf就绪: yaw={math.degrees(current_yaw):.1f}°")

    print("等待相机...")
    while frame_now is None:
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    print("="*40)
    print("赛段二：荒野寻珠")
    print("="*40)

    try:
        # ── 阶段0：对齐Y轴正向 + 视觉居中到3-4列之间 ──
        print("\n[阶段0] 对齐方向 + 平移到3-4列之间")
        align_yaw(target_yaw=math.pi/2, tolerance=0.017)
        shift_to_center()

        # ── 阶段1：走3-4列通道（从下往上）──
        print("\n[阶段1] 走3-4列通道")
        walk_until_yellow()

        # ── 阶段2：左转90° ──
        print("\n[阶段2] 左转90°")
        turn_left_90()

        # ── 阶段3：沿右侧黄线直走到左侧黄线 ──
        print("\n[阶段3] 沿右侧黄线直走")
        walk_until_yellow()

        # ── 阶段4：左转90° ──
        print("\n[阶段4] 左转90°")
        turn_left_90()

        # ── 阶段5：对齐方向 + 平移至1-2列之间 ──
        print("\n[阶段5] 对齐方向 + 平移至1-2列之间")
        align_yaw(target_yaw=-math.pi/2, tolerance=0.017)
        shift_to_center()

        # ── 阶段6：走1-2列通道（从上往下）──
        print("\n[阶段6] 走1-2列通道")
        walk_until_yellow()

        # ── 完成 ──
        print("\n" + "="*40)
        print("赛段二完成！准备进入S弯")
        print("="*40)

    except KeyboardInterrupt:
        pass

    cmd(11, 9, vx=0.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
