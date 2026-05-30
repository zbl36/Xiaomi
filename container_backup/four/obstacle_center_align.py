#!/usr/bin/env python3
"""
障碍物居中站位

流程：
1. 使用 vision_module 中的 cube 模型识别立方体障碍物。
2. 调整朝向，让障碍物中心落在画面正中心。
3. 居中后小步前进靠近。
4. 连续三次识别不到障碍物后，切入第一组手动动作序列。
"""

import importlib.util
import sys
import threading
import time
from pathlib import Path

import lcm
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

BASE_DIR = Path(__file__).resolve().parent
VISION_DIR = BASE_DIR / "vision_module"
GAIT_DIR = BASE_DIR / "loco_hl_example" / "customized_gait"

sys.path.insert(0, str(VISION_DIR))
sys.path.insert(0, str(GAIT_DIR))
sys.path.insert(0, "/usr/local/lib/python3.8/site-packages")

from robot_control_cmd_lcmt import robot_control_cmd_lcmt


def load_vision_detector_class():
    module_path = VISION_DIR / "vision_module.py"
    if not module_path.exists():
        raise FileNotFoundError(f"找不到视觉模块: {module_path}")

    spec = importlib.util.spec_from_file_location("cyberdog_vision_module", str(module_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.VisionDetector


VisionDetector = load_vision_detector_class()

LCM_URL = "udpm://239.255.76.67:7671?ttl=255"

MODE_STAND = 6
MODE_LOCOMOTION = 11
MODE_RECOVERY = 12
GAIT_TROT = 9

# 障碍物识别与站位参数
OBSTACLE_TARGET_TYPE = "cube"
OBSTACLE_AREA_THRESHOLD = 120     # 过滤太小的障碍物框
OBSTACLE_DEAD_ZONE = 35           # 普通对准死区，像素
OBSTACLE_FINAL_DEAD_ZONE = 18     # 最终站位死区，越小越正

# 控制参数：仍然保持低频步进控制
CONTROL_CYCLE_TIME = 3.0
STOP_SETTLE = 0.5
FORWARD_VX = 0.25                 # 居中后前进速度，可调
FORWARD_DURATION = 0.45           # 居中后每次前进时间，可调
LATERAL_VY_MIN = 0.08             # 小误差左右平移速度，可调
LATERAL_VY_MAX = 0.22             # 大误差左右平移速度，可调
LATERAL_DURATION_MIN = 0.25       # 小误差左右平移时间，可调
LATERAL_DURATION_MAX = 0.65       # 大误差左右平移时间，可调
LATERAL_DIR = -1                   # 左右平移方向反了就改成 -1
LOST_STOP_LIMIT = 2               # 连续几次没找到障碍物后停止程序

# 三次没找到障碍物后的手动步进序列，速度仿照 cyberdog_control.py
MANUAL_VX = 0.4
MANUAL_VY = 0.3
MANUAL_VYAW = 0.6
MANUAL_SEND_GAP = 0.1
MANUAL_STEP_SLEEP = 10.0
MANUAL_SEGMENT_SLEEP = 10.0

# 蓝色球追踪参数
BLUE_TARGET_TYPE = "blue_ball"
BLUE_AREA_THRESHOLD = 80
BLUE_CLOSE_AREA = 3500
BLUE_DEAD_ZONE = 30
BLUE_FINAL_DEAD_ZONE = 15
BLUE_CONFIRM_FRAMES = 4
BLUE_FORWARD_VX = 0.35
BLUE_FORWARD_DURATION = 0.65
BLUE_TURN_VYAW_MIN = 0.10
BLUE_TURN_VYAW_MAX = 0.32
BLUE_TURN_DURATION_MIN = 0.25
BLUE_TURN_DURATION_MAX = 0.75
BLUE_CONTROL_CYCLE_TIME = 3.0
BLUE_YAW_DIR = 1                   # 蓝球追踪转向方向反了就改成 -1

# 蓝球丢失后的转身与障碍物重识别
BLUE_LOST_TURN_VYAW = 0.9
BLUE_LOST_TURN_DURATION = 4.0
BLUE_LOST_AFTER_TURN_SLEEP = 15.0
BLUE_LOST_FORWARD_STEPS = 2
BLUE_LOST_FORWARD_SLEEP = 7.0
RECHECK_OBSTACLE_LOST_LIMIT = 2
RECHECK_OBSTACLE_TURN_VYAW_MIN = 0.10
RECHECK_OBSTACLE_TURN_VYAW_MAX = 0.32
RECHECK_OBSTACLE_TURN_DURATION_MIN = 0.25
RECHECK_OBSTACLE_TURN_DURATION_MAX = 0.75
RECHECK_OBSTACLE_YAW_DIR = 1
RECHECK_OBSTACLE_APPROACH_VX = 0.25
RECHECK_OBSTACLE_APPROACH_DURATION = 0.6


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class LcmController:
    def __init__(self):
        self.lc_cmd = lcm.LCM(LCM_URL)
        self.msg = robot_control_cmd_lcmt()
        self.life_count = 0
        self.lock = threading.Lock()
        self.keepalive_stop = threading.Event()
        self.keepalive_thread = threading.Thread(target=self._keepalive_loop, daemon=True)
        self.keepalive_thread.start()

    def _keepalive_loop(self):
        while not self.keepalive_stop.is_set():
            with self.lock:
                self.lc_cmd.publish("robot_control_cmd", self.msg.encode())
            time.sleep(0.1)

    def send(self, mode, gait_id=0, vx=0.0, vy=0.0, vyaw=0.0,
             duration=500, step_height=0.08):
        with self.lock:
            self.life_count = (self.life_count + 1) % 127
            self.msg.mode = mode
            self.msg.gait_id = gait_id
            self.msg.life_count = self.life_count
            self.msg.duration = duration
            self.msg.contact = 0
            self.msg.value = 0
            self.msg.vel_des = [vx, vy, vyaw]
            self.msg.rpy_des = [0.0, 0.0, 0.0]
            self.msg.pos_des = [0.0, 0.0, 0.0]
            self.msg.acc_des = [0.0] * 6
            self.msg.ctrl_point = [0.0] * 3
            self.msg.foot_pose = [0.0] * 6
            self.msg.step_height = [step_height, step_height]
            self.lc_cmd.publish("robot_control_cmd", self.msg.encode())

    def hold_current(self, duration_s, period=0.2):
        deadline = time.time() + duration_s
        while time.time() < deadline:
            with self.lock:
                self.lc_cmd.publish("robot_control_cmd", self.msg.encode())
            time.sleep(period)

    def motion_for(self, duration_s, vx=0.0, vy=0.0, vyaw=0.0, period=0.25):
        deadline = time.time() + duration_s
        while time.time() < deadline:
            self.send(MODE_LOCOMOTION, GAIT_TROT,
                      vx=vx, vy=vy, vyaw=vyaw,
                      duration=int(period * 1000))
            time.sleep(period)

    def stop(self, settle_s=0.3):
        self.send(MODE_LOCOMOTION, GAIT_TROT, vx=0.0, vy=0.0, vyaw=0.0)
        self.hold_current(settle_s)

    def stand(self, settle_s=0.5):
        self.send(MODE_STAND, duration=500)
        self.hold_current(settle_s)

    def stand_up(self):
        print("[控制] 恢复站立...")
        self.send(MODE_RECOVERY, duration=2000)
        self.hold_current(5.0)
        self.send(MODE_STAND, duration=1000)
        self.hold_current(2.0)
        print("[控制] 已站立")

    def enter_locomotion(self):
        print("[控制] 进入行走模式...")
        self.send(MODE_LOCOMOTION, GAIT_TROT, duration=500)
        self.hold_current(1.0)

    def shutdown(self):
        self.keepalive_stop.set()


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_center_detector")
        self.bridge = CvBridge()
        self.detector = VisionDetector(
            enable_coke=False,
            enable_football=False,
            enable_cube=True,
            enable_ganzi=False,
            enable_blue_ball=True,
            enable_orange_ball=False,
        )
        self.lock = threading.Lock()
        self.obstacle = None
        self.blue_ball = None
        self.image_width = 640

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, "/rgb_camera/image_raw", self._cb, qos)

    def _cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        obstacle = self.detector.get_closest_target(
            frame,
            target_type=OBSTACLE_TARGET_TYPE,
            area_threshold=OBSTACLE_AREA_THRESHOLD,
        )
        blue_ball = self.detector.get_closest_target(
            frame,
            target_type=BLUE_TARGET_TYPE,
            area_threshold=BLUE_AREA_THRESHOLD,
        )
        with self.lock:
            self.image_width = frame.shape[1]
            self.obstacle = obstacle
            self.blue_ball = blue_ball

    def snapshot(self):
        with self.lock:
            return self.obstacle, self.blue_ball, self.image_width


def start_detector():
    rclpy.init()
    node = ObstacleDetector()
    thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    thread.start()
    return node


def align_obstacle_step(ctrl, obstacle, image_width):
    cx, _, area, conf = obstacle
    error = cx - image_width / 2.0

    if abs(error) <= OBSTACLE_DEAD_ZONE:
        print(f"[障碍物] 已居中，前进一步 area={area:.0f}, conf={conf:.2f}")
        ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vx=FORWARD_VX)
        time.sleep(FORWARD_DURATION)
        ctrl.stop(STOP_SETTLE)
        return

    error_ratio = clamp(abs(error) / (image_width / 2.0), 0.0, 1.0)
    vy_mag = LATERAL_VY_MIN + (LATERAL_VY_MAX - LATERAL_VY_MIN) * error_ratio
    duration = LATERAL_DURATION_MIN + (
        LATERAL_DURATION_MAX - LATERAL_DURATION_MIN) * error_ratio
    vy = LATERAL_DIR * (-vy_mag if error < 0 else vy_mag)
    print(f"[障碍物] 左右平移到中间 error={error:.0f}, vy={vy:.2f}, "
          f"duration={duration:.2f}s, area={area:.0f}")
    ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vy=vy)
    time.sleep(duration)
    ctrl.stop(STOP_SETTLE)


def turn_align_obstacle_step(ctrl, obstacle, image_width):
    cx, _, area, conf = obstacle
    error = cx - image_width / 2.0


    error_ratio = clamp(abs(error) / (image_width / 2.0), 0.0, 1.0)
    vyaw_mag = RECHECK_OBSTACLE_TURN_VYAW_MIN + (
        RECHECK_OBSTACLE_TURN_VYAW_MAX - RECHECK_OBSTACLE_TURN_VYAW_MIN) * error_ratio
    duration = RECHECK_OBSTACLE_TURN_DURATION_MIN + (
        RECHECK_OBSTACLE_TURN_DURATION_MAX - RECHECK_OBSTACLE_TURN_DURATION_MIN) * error_ratio
    vyaw = RECHECK_OBSTACLE_YAW_DIR * vyaw_mag if error < 0 else RECHECK_OBSTACLE_YAW_DIR * (-vyaw_mag)
    print(f"[障碍物] 左右旋转正对 error={error:.0f}, vyaw={vyaw:.2f}, "
          f"duration={duration:.2f}s, area={area:.0f}")
    ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=vyaw)
    time.sleep(duration)
    ctrl.stop(STOP_SETTLE)


def send_manual_burst(ctrl, label, send_count, sleep_s, vx=0.0, vy=0.0,
                      vyaw=0.0):
    print(f"[手动序列] {label}: 连发 {send_count} 次，然后保持 {sleep_s}s")
    for index in range(send_count):
        print(f"[手动序列] {label} 第 {index + 1}/{send_count} 次")
        ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vx=vx, vy=vy, vyaw=vyaw)
        time.sleep(MANUAL_SEND_GAP)
    time.sleep(sleep_s)


def run_manual_sequence_after_obstacle_lost(ctrl):
    print("[任务] 原地站立，准备执行固定手动步进序列...")
    ctrl.stand(1.0)
    ctrl.enter_locomotion()

    send_manual_burst(ctrl, "前进", 3, MANUAL_SEGMENT_SLEEP, vx=MANUAL_VX)
    send_manual_burst(ctrl, "左转", 6, MANUAL_SEGMENT_SLEEP, vyaw=MANUAL_VYAW)
    send_manual_burst(ctrl, "前进", 4, MANUAL_SEGMENT_SLEEP, vx=MANUAL_VX)
    send_manual_burst(ctrl, "右转", 6, MANUAL_SEGMENT_SLEEP, vyaw=-MANUAL_VYAW)
    send_manual_burst(ctrl, "前进", 4, MANUAL_SEGMENT_SLEEP, vx=MANUAL_VX)
    send_manual_burst(ctrl, "右转", 5, MANUAL_SEGMENT_SLEEP, vyaw=-MANUAL_VYAW)
    send_manual_burst(ctrl, "前进", 4, MANUAL_SEGMENT_SLEEP, vx=MANUAL_VX)
    send_manual_burst(ctrl, "左转", 5, MANUAL_SEGMENT_SLEEP, vyaw=MANUAL_VYAW)

    ctrl.stop(1.0)
    print("[任务] 固定手动步进序列完成，准备追踪蓝色球")


def run_final_sequence(ctrl):
    print("[任务] 执行转身后的结尾动作序列...")
    ctrl.stand(1.0)
    ctrl.enter_locomotion()
    send_manual_burst(ctrl, "前进", 3, MANUAL_STEP_SLEEP, vx=MANUAL_VX)
    send_manual_burst(ctrl, "左平移", 4, MANUAL_STEP_SLEEP, vy=MANUAL_VY)
    send_manual_burst(ctrl, "原地左转", 13, MANUAL_STEP_SLEEP, vyaw=MANUAL_VYAW)
    send_manual_burst(ctrl, "后退", 1, MANUAL_STEP_SLEEP, vx=-MANUAL_VX)
    ctrl.stop(1.0)
    ctrl.stand(1.0)
    print("[任务] 结尾动作序列完成，任务结束")


def follow_blue_ball_step(ctrl, blue_ball, image_width):
    cx, _, area, conf = blue_ball
    error = cx - image_width / 2.0

    if abs(error) <= BLUE_DEAD_ZONE:
        print(f"[蓝球] 已居中，前进一步 area={area:.0f}, conf={conf:.2f}")
        ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vx=BLUE_FORWARD_VX)
        time.sleep(BLUE_FORWARD_DURATION)
        ctrl.stop(STOP_SETTLE)
        return

    error_ratio = clamp(abs(error) / (image_width / 2.0), 0.0, 1.0)
    vyaw_mag = BLUE_TURN_VYAW_MIN + (
        BLUE_TURN_VYAW_MAX - BLUE_TURN_VYAW_MIN) * error_ratio
    duration = BLUE_TURN_DURATION_MIN + (
        BLUE_TURN_DURATION_MAX - BLUE_TURN_DURATION_MIN) * error_ratio
    vyaw = BLUE_YAW_DIR * vyaw_mag if error < 0 else BLUE_YAW_DIR * (-vyaw_mag)
    print(f"[蓝球] 调整朝向 error={error:.0f}, vyaw={vyaw:.2f}, "
          f"duration={duration:.2f}s, area={area:.0f}")
    ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=vyaw)
    time.sleep(duration)
    ctrl.stop(STOP_SETTLE)


def track_blue_ball(ctrl, detector):
    print("[任务] 开始无限接近蓝色球，第一次丢失蓝球后切换流程...")

    while True:
        _, blue_ball, width = detector.snapshot()
        if blue_ball is None:
            print("[蓝球] 第一次未识别到蓝色球，停止追踪")
            ctrl.stop(STOP_SETTLE)
            return

        follow_blue_ball_step(ctrl, blue_ball, width)
        time.sleep(BLUE_CONTROL_CYCLE_TIME)


def turn_after_blue_lost(ctrl):
    print("[任务] 蓝球丢失，先前进两步...")
    ctrl.stop(0.8)
    for index in range(BLUE_LOST_FORWARD_STEPS):
        print(f"[任务] 蓝球丢失后前进第 {index + 1}/{BLUE_LOST_FORWARD_STEPS} 步")
        ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vx=MANUAL_VX)
        time.sleep(MANUAL_SEND_GAP)
    print(f"[任务] 前进两步完成，等待 {BLUE_LOST_FORWARD_SLEEP:.1f}s")
    time.sleep(BLUE_LOST_FORWARD_SLEEP)

    print("[任务] 执行180度转身...")
    ctrl.motion_for(BLUE_LOST_TURN_DURATION, vyaw=BLUE_LOST_TURN_VYAW)
    ctrl.stop(0.8)
    print(f"[任务] 转身完成，等待 {BLUE_LOST_AFTER_TURN_SLEEP:.1f}s 后识别障碍物")
    time.sleep(BLUE_LOST_AFTER_TURN_SLEEP)


def recheck_obstacle_after_blue_lost(ctrl, detector):
    print("[任务] 开始重新识别并正面对准障碍物...")
    lost_count = 0
    approaching = False

    while True:
        obstacle, _, width = detector.snapshot()
        if obstacle is None:
            lost_count += 1
            print(f"[障碍物] 未识别到 {lost_count}/{RECHECK_OBSTACLE_LOST_LIMIT}")
            ctrl.stop(STOP_SETTLE)
            if lost_count >= RECHECK_OBSTACLE_LOST_LIMIT:
                print("[任务] 连续三次识别不到障碍物，执行末尾动作序列")
                run_final_sequence(ctrl)
                return
            time.sleep(CONTROL_CYCLE_TIME)
            continue

        lost_count = 0
        cx, _, area, conf = obstacle
        error = cx - width / 2.0

        if approaching:
            print(f"[障碍物] 正面对准后直线靠近 area={area:.0f}, "
                  f"conf={conf:.2f}, error={error:.0f}")
            ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vx=RECHECK_OBSTACLE_APPROACH_VX)
            time.sleep(RECHECK_OBSTACLE_APPROACH_DURATION)
            ctrl.stop(STOP_SETTLE)
            time.sleep(CONTROL_CYCLE_TIME)
            continue

        if abs(error) <= OBSTACLE_FINAL_DEAD_ZONE:
            print(f"[障碍物] 已正面对准，直接开始直线靠近 "
                  f"error={error:.0f}, area={area:.0f}, conf={conf:.2f}")
            ctrl.stop(0.8)
            approaching = True
            time.sleep(CONTROL_CYCLE_TIME)
            continue

        turn_align_obstacle_step(ctrl, obstacle, width)
        time.sleep(CONTROL_CYCLE_TIME)


def main():
    ctrl = LcmController()
    detector = start_detector()
    lost_count = 0

    ctrl.stand_up()
    ctrl.enter_locomotion()
    print("[任务] 开始识别障碍物，居中就前进一步，连续三次丢失后切入动作序列...")

    try:
        while True:
            obstacle, _, width = detector.snapshot()

            if obstacle is None:
                lost_count += 1
                print(f"[视觉] 未识别到障碍物 {lost_count}/{LOST_STOP_LIMIT}")
                ctrl.stop(STOP_SETTLE)
                if lost_count >= LOST_STOP_LIMIT:
                    ctrl.stand(1.0)
                    print("[任务] 连续三次未找到障碍物，进入固定路线和蓝球追踪")
                    run_manual_sequence_after_obstacle_lost(ctrl)
                    track_blue_ball(ctrl, detector)
                    turn_after_blue_lost(ctrl)
                    recheck_obstacle_after_blue_lost(ctrl, detector)
                    break
                time.sleep(CONTROL_CYCLE_TIME)
                continue

            lost_count = 0
            align_obstacle_step(ctrl, obstacle, width)
            time.sleep(CONTROL_CYCLE_TIME)

    except KeyboardInterrupt:
        print("\n[中断] 用户停止")
    finally:
        ctrl.stop(0.3)
        ctrl.stand(10.0)
        ctrl.shutdown()
        time.sleep(0.5)
        rclpy.shutdown()
        print("[结束] 程序退出")


if __name__ == "__main__":
    main()
