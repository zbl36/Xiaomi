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
ORANGE_HIGH = np.array([18, 255, 255])
BLUE_LOW    = np.array([95, 100, 50])
BLUE_HIGH   = np.array([115, 255, 200])

# 黄线 HSV
YELLOW_LOW  = np.array([25, 80, 200])
YELLOW_HIGH = np.array([35, 255, 255])

# 控制参数（加速版）
Kp_yaw      = 0.008
VX          = 0.25    # 前进速度（原0.15）
VX_HIT      = 0.35    # 撞击速度（原0.30）
VY_SHIFT    = 0.18    # 平移速度（原0.12）
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


def detect_yellow_right(frame):
    """检测画面右侧1/5是否有黄线，返回黄线的平均y坐标或None"""
    h, w = frame.shape[:2]
    roi = frame[:, w*4//5:]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
    if cv2.countNonZero(mask) > 500:
        return True
    return False


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
            elif time.time() - stable_start > 1.5:
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
    """检测画面中所有近处球（橙+蓝），过滤远处小球，返回各球中心x坐标列表"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, ORANGE_LOW, ORANGE_HIGH)
    m2 = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)
    mask = cv2.bitwise_or(m1, m2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 400:  # 过滤远处小球
            continue
        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        centers.append(M['m10'] / M['m00'])
    return centers


def shift_to_center(direction='left'):
    """
    向指定方向平移，直到画面中球分布关于中心对称
    判断标准：左侧最近球和右侧最近球到中心的距离差 < 阈值
    """
    vy = VY_SHIFT if direction == 'left' else -VY_SHIFT
    print(f"  {direction}平移居中...")
    stable_count = 0

    while True:
        frame = wait_frame()
        h, w = frame.shape[:2]
        centers = detect_all_balls(frame)
        cx = w / 2.0

        if len(centers) >= 2:
            # 分左右
            left_balls = [x for x in centers if x < cx]
            right_balls = [x for x in centers if x >= cx]

            if left_balls and right_balls:
                # 左侧最靠近中心的球 和 右侧最靠近中心的球
                left_nearest = max(left_balls)   # 最靠右的左球
                right_nearest = min(right_balls)  # 最靠左的右球
                left_dist = cx - left_nearest
                right_dist = right_nearest - cx
                diff = abs(left_dist - right_dist)
                avg_x = sum(centers) / len(centers)
                err = avg_x - cx

                print(f'\r  球数={len(centers)} L_dist={left_dist:.0f} '
                      f'R_dist={right_dist:.0f} diff={diff:.0f} err={err:.0f}',
                      end='', flush=True)

                # 对称条件：左右最近球到中心距离差<30px 且 avg偏移合理
                if diff < 30 and abs(err) < 50:
                    stable_count += 1
                    if stable_count > 6:
                        print("\n  居中完成！")
                        rev_vy = -vy
                        for _ in range(int(0.5 / 0.05)):
                            cmd(11, 9, vx=0.0, vy=rev_vy)
                            time.sleep(0.05)
                        break
                    cmd(11, 9, vx=0.0)
                else:
                    stable_count = 0
                    cmd(11, 9, vx=0.0, vy=vy)
            else:
                # 只有一侧有球，继续平移
                stable_count = 0
                cmd(11, 9, vx=0.0, vy=vy)
                print(f'\r  球数={len(centers)} 只有一侧有球，继续平移',
                      end='', flush=True)
        else:
            cmd(11, 9, vx=0.0, vy=vy)
            stable_count = 0
            print(f'\r  球数={len(centers)} 不足，继续平移', end='', flush=True)

        time.sleep(0.05)

    for _ in range(5):
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


def walk_until_yellow(heading=math.pi/2, max_hits=99, swing=False):
    """
    直走撞球直到黄线或撞够max_hits个球
    swing=True时先左右摇摆扫描（用于12列）
    返回撞球数量
    """
    print("  直走，等待黄线...")
    yellow_seen = False
    hit_count = 0

    if swing:
        print("  左右摇摆扫描...")
        found_in_swing = False
        left_target = heading + math.radians(60)
        right_target = heading - math.radians(60)
        for target in [right_target, heading, left_target]:
            if found_in_swing:
                break
            goal_yaw = target
            while True:
                frame = wait_frame()
                result = detect_orange(frame)
                if result and result[2] > 2500:
                    cx, cy, area = result
                    w = frame.shape[1]
                    side = 'left' if cx < w/2.0 else 'right'
                    print(f"\n  扫描发现球！area={area:.0f} {side}侧")
                    hit_ball()
                    hit_count += 1
                    if hit_count >= max_hits:
                        return hit_count
                    recover_after_hit(side, target_yaw=heading)
                    found_in_swing = True
                    break
                err = goal_yaw - current_yaw
                while err > math.pi: err -= 2*math.pi
                while err < -math.pi: err += 2*math.pi
                if abs(err) < math.radians(5):
                    break
                vyaw = 0.7 if err > 0 else -0.7
                cmd(11, 9, vx=0.0, vyaw=vyaw)
                time.sleep(0.05)
            if target == heading and not found_in_swing:
                align_yaw(target_yaw=heading, tolerance=math.radians(10))
        align_yaw(target_yaw=heading, tolerance=0.087)

    # 正常直走撞球
    while True:
        frame = wait_frame()
        has_yellow = detect_yellow_bottom(frame)

        if not yellow_seen:
            if has_yellow:
                yellow_seen = True
                print("  检测到黄线，继续走过...")
            else:
                result = detect_orange(frame)
                if result and result[2] > 2500:
                    cx, cy, area = result
                    w = frame.shape[1]
                    side = 'left' if cx < w/2.0 else 'right'
                    print(f"\n  发现球！area={area:.0f} {side}侧")
                    hit_ball()
                    hit_count += 1
                    if hit_count >= max_hits:
                        return hit_count
                    recover_after_hit(side, target_yaw=heading)
                    continue
        else:
            if not has_yellow:
                print("  黄线消失，再走一段...")
                for _ in range(int(1.0 / 0.05)):
                    cmd(11, 9, vx=VX, vyaw=0.0)
                    time.sleep(0.05)
                cmd(11, 9, vx=0.0)
                for _ in range(5):
                    cmd(11, 9, vx=0.0)
                    time.sleep(0.1)
                return hit_count

        cmd(11, 9, vx=VX, vyaw=0.0)
        time.sleep(0.05)


def recover_after_hit(side, target_yaw=math.pi/2):
    """撞击后恢复方向，然后平移回居中"""
    # 停止
    for _ in range(5):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)

    # 用tf恢复方向（5度误差即可）
    align_yaw(target_yaw=target_yaw, tolerance=0.087)

    # 撞左边球→右平移居中；撞右边球→左平移居中
    if side == 'left':
        shift_to_center(direction='right')
    else:
        shift_to_center(direction='left')

    # 居中后不再重复对齐（已经对齐过了）


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


def walk_to_yellow_simple():
    """直走直到底部检测到黄线，每走4秒右平移1秒修正偏左"""
    print("  直走，等待底部黄线...")
    walk_duration = 4.0
    shift_duration = 1.0

    while True:
        t0 = time.time()
        found_yellow = False
        while time.time() - t0 < walk_duration:
            frame = wait_frame()
            if detect_yellow_bottom(frame):
                print("  检测到底部黄线")
                found_yellow = True
                break
            cmd(11, 9, vx=VX, vyaw=0.0)
            time.sleep(0.05)

        if found_yellow:
            return

        # 右平移1秒修正偏左
        print("  右平移修正...")
        t0 = time.time()
        while time.time() - t0 < shift_duration:
            cmd(11, 9, vx=0.0, vy=-VY_SHIFT)
            time.sleep(0.05)
        cmd(11, 9, vx=0.0)
        print("  修正完成")


def shift_until_no_yellow():
    """左平移直到底部黄线消失（到达S弯入口）"""
    print("  左平移，等待底部黄线消失...")
    while True:
        frame = wait_frame()
        if not detect_yellow_bottom(frame):
            cmd(11, 9, vx=0.0)
            print("  底部黄线消失，到达S弯入口")
            for _ in range(5):
                cmd(11, 9, vx=0.0)
                time.sleep(0.1)
            return
        cmd(11, 9, vx=0.0, vy=VY_SHIFT)
        time.sleep(0.05)


def walk_along_right_yellow():
    """
    直走，同时检测右下角黄线保持距离
    当左半边底部出现黄线并消失后停止
    """
    print("  直走，等待左半边底部黄线...")
    yellow_bottom_seen = False

    while True:
        frame = wait_frame()
        h, w = frame.shape[:2]

        # 只检测左半边底部1/5
        roi = frame[h*4//5:, :w//2]
        hsv_bottom = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_bottom = cv2.inRange(hsv_bottom, YELLOW_LOW, YELLOW_HIGH)
        has_bottom = cv2.countNonZero(mask_bottom) > YELLOW_AREA

        if not yellow_bottom_seen:
            if has_bottom:
                yellow_bottom_seen = True
                print("\n  左半底部检测到黄线，继续走过...")
        else:
            if not has_bottom:
                cmd(11, 9, vx=0.0)
                print("  黄线消失，停止")
                for _ in range(5):
                    cmd(11, 9, vx=0.0)
                    time.sleep(0.1)
                return

        # 检测右下角黄线来修正方向（保持黄线在右下角）
        right_bottom_roi = frame[h*3//4:, w*3//4:]
        hsv_rb = cv2.cvtColor(right_bottom_roi, cv2.COLOR_BGR2HSV)
        mask_rb = cv2.inRange(hsv_rb, YELLOW_LOW, YELLOW_HIGH)
        rb_pixels = cv2.countNonZero(mask_rb)

        vyaw = 0.0
        if rb_pixels > 300:
            # 右下角黄线太多，偏右了，微调左
            vyaw = 0.08
        elif rb_pixels < 50:
            # 右下角没黄线，偏左了，微调右
            vyaw = -0.05

        cmd(11, 9, vx=VX, vyaw=vyaw)
        time.sleep(0.05)


def shift_left(sec=2.0):
    """左平移指定时间"""
    print(f"  左平移 {sec}秒...")
    for _ in range(int(sec / 0.05)):
        cmd(11, 9, vx=0.0, vy=VY_SHIFT)
        time.sleep(0.05)
    for _ in range(5):
        cmd(11, 9, vx=0.0)
        time.sleep(0.1)


def shift_right(sec=2.0):
    """右平移指定时间"""
    print(f"  右平移 {sec}秒...")
    for _ in range(int(sec / 0.05)):
        cmd(11, 9, vx=0.0, vy=-VY_SHIFT)
        time.sleep(0.05)
    for _ in range(5):
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
    for _ in range(5):
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
        total_hits = 0

        # ── 阶段0：对齐Y轴正向 + 视觉居中到3-4列之间 ──
        print("\n[阶段0] 对齐方向 + 平移到3-4列之间")
        align_yaw(target_yaw=math.pi/2, tolerance=0.017)
        shift_to_center()

        # ── 阶段1：走3-4列通道（从下往上）──
        print("\n[阶段1] 走3-4列通道（上行）")
        hits = walk_until_yellow(heading=math.pi/2, max_hits=4)
        total_hits += hits
        print(f"  累计撞球: {total_hits}")

        # ── 阶段2：对齐180° + 直走到左侧黄线 ──
        print("\n[阶段2] 对齐180° + 横穿")
        align_yaw(target_yaw=math.pi, tolerance=0.017)
        walk_along_right_yellow()

        # ── 阶段3：对齐Y轴负方向 + 居中3-4列 ──
        print("\n[阶段3] 对齐Y轴负方向 + 居中3-4列")
        align_yaw(target_yaw=-math.pi/2, tolerance=0.017)
        shift_to_center()

        # ── 阶段4：走3-4列通道（从上往下）撞球 ──
        print("\n[阶段4] 走3-4列通道（下行）")
        remaining = 4 - total_hits
        hits = walk_until_yellow(heading=-math.pi/2, max_hits=remaining, swing=True)
        total_hits += hits
        print(f"  累计撞球: {total_hits}")

        # 撞够4个球后直接转Y正+左平移居中+找S弯
        if total_hits >= 4:
            print("\n[→S弯] 撞够4球，进入S弯逻辑")
            align_yaw(target_yaw=math.pi/2, tolerance=0.017)
            shift_to_center()

        # ── 阶段5：对齐Y轴正向 + 直走到底部黄线 + 进入S弯 ──
        print("\n[阶段5] 对齐Y轴正向 + 走到底部黄线 + 进入S弯")
        align_yaw(target_yaw=math.pi/2, tolerance=0.017)
        walk_to_yellow_simple()
        # 检测到黄线后再走2秒
        print("  再走2秒...")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            cmd(11, 9, vx=VX, vyaw=0.0)
            time.sleep(0.05)
        cmd(11, 9, vx=0.0)
        # 对齐180度
        print("  对齐180度...")
        align_yaw(target_yaw=math.pi, tolerance=math.radians(5))
        # 直走，等待左半边底部黄线出现并消失
        print("  直走，等待左半边底部黄线...")
        yellow_seen_5 = False
        while True:
            frame = wait_frame()
            h, w = frame.shape[:2]
            roi = frame[h*4//5:, :w//2]
            hsv_b = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask_b = cv2.inRange(hsv_b, YELLOW_LOW, YELLOW_HIGH)
            has_y = cv2.countNonZero(mask_b) > YELLOW_AREA
            if not yellow_seen_5:
                if has_y:
                    yellow_seen_5 = True
                    print("  左半底部检测到黄线，继续走过...")
            else:
                if not has_y:
                    cmd(11, 9, vx=0.0)
                    print("  黄线消失，停止")
                    for _ in range(5):
                        cmd(11, 9, vx=0.0)
                        time.sleep(0.1)
                    break
            cmd(11, 9, vx=VX, vyaw=0.0)
            time.sleep(0.05)
        # 对齐Y轴正向进入S弯
        print("  对齐Y轴正向，进入S弯")
        align_yaw(target_yaw=math.pi/2, tolerance=0.017)

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
