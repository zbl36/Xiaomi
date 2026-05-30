#!/usr/bin/env python3
"""
可乐瓶跟随 + 限高杆低姿态通过

流程：
1. 正常站立后追踪可乐瓶前进。
2. 如果检测到限高杆足够近，停止并执行 lowposture_forward 自定义步态。
3. 低姿态通过后切回站立，再继续追踪可乐瓶。
"""

import copy
import importlib.util
import math
import sys
import threading
import time
from pathlib import Path

import lcm
import rclpy
import toml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

BASE_DIR = Path(__file__).resolve().parent
VISION_DIR = BASE_DIR / "vision_module"
GAIT_DIR = BASE_DIR / "loco_hl_example" / "customized_gait"
LCM_TYPE_DIR = BASE_DIR / "src" / "cyberdog_locomotion" / "common" / "lcm_type" / "lcm"

sys.path.insert(0, str(VISION_DIR))
sys.path.insert(0, str(GAIT_DIR))
sys.path.insert(0, str(LCM_TYPE_DIR))
sys.path.insert(0, "/usr/local/lib/python3.8/site-packages")

from file_send_lcmt import file_send_lcmt
from robot_control_cmd_lcmt import robot_control_cmd_lcmt


def load_vision_detector_class():
    module_path = VISION_DIR / "vision_module.py"
    if not module_path.exists():
        raise FileNotFoundError(f"找不到视觉模块: {module_path}")

    spec = importlib.util.spec_from_file_location(
        "cyberdog_vision2_module", str(module_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.VisionDetector


VisionDetector = load_vision_detector_class()

LCM_URL = "udpm://239.255.76.67:7671?ttl=255"

MODE_STAND = 6
MODE_LOCOMOTION = 11
MODE_RECOVERY = 12
MODE_USER_GAIT = 62
GAIT_TROT = 9
GAIT_USER = 110

LOWPOSTURE_NAME = "lowposture_forward"
LOWPOSTURE_DURATION_MS = 23040
LOWPOSTURE_RECOVERY_LEAD_S = 0.5

# 可乐跟随参数
COKE_CLOSE_AREA = 45000       # 可乐足够近后停止，可按实际画面调
COKE_AREA_THRESHOLD = 15      # 过滤太小的可乐检测框
COKE_CONF_THRESHOLD = 0.15    # 可乐单独放宽置信度，提高召回率
COKE_IMGSZ = 960              # 提高YOLO输入尺寸，增强远距离小目标识别
COKE_MEMORY_SECONDS = 2.0     # 可乐短时记忆，避免单帧漏检就停住
COKE_DEAD_ZONE = 45           # 可乐中心死区，像素
FOLLOW_CYCLE_TIME = 3.0
FORWARD_VX = 0.32
FORWARD_DURATION = 0.45
TURN_VYAW_MIN = 0.16
TURN_VYAW_MAX = 0.38
TURN_DURATION_MIN = 0.25
TURN_DURATION_MAX = 0.65

# 连续丢失可乐后的恢复动作
COKE_LOST_LIMIT = 2           # 连续几次识别不到可乐后停止追踪
TURN_AROUND_VYAW = 0.9       # 原地转身角速度，可调
TURN_AROUND_DURATION = 4    # 原地转身时间，可调；约180度需按实际调
TURN_TO_BACKWARD_WAIT = 2  # 转身结束后等待多久再后退，可调
BACKWARD_VX = -0.28           # 后退速度，可调
BACKWARD_STEP_DURATION = 0.8  # 每次后退持续时间，可调
BACKWARD_STEP_COUNT = 2       # 后退几步
BACKWARD_STEP_SETTLE = 0.4    # 每步后停稳时间，可调

# 限高杆触发参数
GANZI_CLOSE_AREA = 36000      # 限高杆足够近后触发低姿态，可按实际画面调
GANZI_AREA_THRESHOLD = 300
GANZI_CONFIRM_FRAMES = 3      # 连续识别到近距离限高杆才触发
GANZI_REDETECT_SLEEP = 2.0    # 后退结束后，重新识别限高杆前等待时间
GANZI_DEAD_ZONE = 50          # 限高杆居中死区，像素，可调
GANZI_TURN_VYAW_MIN = 0.16    # 对准限高杆的最小转向速度，可调
GANZI_TURN_VYAW_MAX = 0.38    # 对准限高杆的最大转向速度，可调
GANZI_TURN_DURATION_MIN = 0.25
GANZI_TURN_DURATION_MAX = 0.65
GANZI_APPROACH_VX = 0.22      # 限高杆已居中但不够近时的前进速度，可调
GANZI_APPROACH_DURATION = 0.4 # 限高杆已居中但不够近时，每次前进时间，可调
GANZI_CONTROL_CYCLE_TIME = 3.0 # 二次对准限高杆时，控制指令发送间隔

# 可乐丢失转身后：第二次低姿态前必须特别正对限高框
SECOND_GANZI_DEAD_ZONE = 14        # 二次低姿态前限高框中心误差，越小越正
SECOND_GANZI_CONFIRM_FRAMES = 5    # 连续满足几次才切低姿态

# 二次低姿态通过后的手动收尾动作，速度仿照 cyberdog_control.py
MANUAL_VX = 0.4
MANUAL_VY = 0.3
MANUAL_VYAW = 0.6
MANUAL_SEND_GAP = 0.1
POST_LOW_FORWARD_STEPS = 9
POST_LOW_FORWARD_SLEEP = 15.0
POST_LOW_RIGHT_TURN_STEPS = 12
POST_LOW_RIGHT_TURN_SLEEP = 15.0
POST_LOW_BACKWARD_SLEEP = 5.0
POST_LOW_GANZI_FINAL_DEAD_ZONE = 35
POST_LOW_RIGHT_SHIFT_STEPS = 6
POST_LOW_RIGHT_SHIFT_SLEEP = 7.0


robot_cmd_template = {
    "mode": 0,
    "gait_id": 0,
    "contact": 0,
    "life_count": 0,
    "vel_des": [0.0, 0.0, 0.0],
    "rpy_des": [0.0, 0.0, 0.0],
    "pos_des": [0.0, 0.0, 0.0],
    "acc_des": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "ctrl_point": [0.0, 0.0, 0.0],
    "foot_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "step_height": [0.0, 0.0],
    "value": 0,
    "duration": 0,
}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class LcmController:
    def __init__(self):
        self.lc_cmd = lcm.LCM(LCM_URL)
        self.lc_file = lcm.LCM(LCM_URL)
        self.msg = robot_control_cmd_lcmt()
        self.life_count = 0
        self.lock = threading.Lock()
        self.user_gait_duration_ms = LOWPOSTURE_DURATION_MS
        self.in_lowposture = False
        self.keepalive_stop = threading.Event()
        self.keepalive_thread = threading.Thread(
            target=self._keepalive_loop,
            daemon=True,
        )
        self.keepalive_thread.start()

    def _keepalive_loop(self):
        while not self.keepalive_stop.is_set():
            with self.lock:
                self.lc_cmd.publish("robot_control_cmd", self.msg.encode())
            time.sleep(0.1)

    def send(self, mode, gait_id=0, vx=0.0, vy=0.0, vyaw=0.0,
             duration=500, contact=0, value=0, step_height=0.08):
        with self.lock:
            self.life_count = (self.life_count + 1) % 127
            self.msg.mode = mode
            self.msg.gait_id = gait_id
            self.msg.life_count = self.life_count
            self.msg.duration = duration
            self.msg.contact = contact
            self.msg.value = value
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

    def upload_user_gait(self, gait_name):
        gait_def_path = GAIT_DIR / f"Gait_Def_{gait_name}.toml"
        gait_params_path = GAIT_DIR / f"Gait_Params_{gait_name}.toml"
        gait_params_full_path = GAIT_DIR / f"Gait_Params_{gait_name}_full.toml"

        steps = toml.load(gait_params_path)
        full_steps = {"step": []}
        total_duration = 0
        for item in steps["step"]:
            cmd = copy.deepcopy(robot_cmd_template)
            cmd["duration"] = item["duration"]
            total_duration += item["duration"]
            if item["type"] == "usergait":
                cmd["mode"] = MODE_LOCOMOTION
                cmd["gait_id"] = GAIT_USER
                cmd["vel_des"] = item["body_vel_des"]
                cmd["rpy_des"] = item["body_pos_des"][0:3]
                cmd["pos_des"] = item["body_pos_des"][3:6]
                cmd["foot_pose"][0:2] = item["landing_pos_des"][0:2]
                cmd["foot_pose"][2:4] = item["landing_pos_des"][3:5]
                cmd["foot_pose"][4:6] = item["landing_pos_des"][6:8]
                cmd["ctrl_point"][0:2] = item["landing_pos_des"][9:11]
                cmd["step_height"][0] = (
                    math.ceil(item["step_height"][0] * 1e3)
                    + math.ceil(item["step_height"][1] * 1e3) * 1e3
                )
                cmd["step_height"][1] = (
                    math.ceil(item["step_height"][2] * 1e3)
                    + math.ceil(item["step_height"][3] * 1e3) * 1e3
                )
                cmd["acc_des"] = item["weight"]
                cmd["value"] = item["use_mpc_traj"]
                cmd["contact"] = math.floor(item["landing_gain"] * 1e1)
                cmd["ctrl_point"][2] = item["mu"]
            full_steps["step"].append(cmd)

        gait_params_full_path.write_text("# Gait Params\n" + toml.dumps(full_steps))
        self.user_gait_duration_ms = total_duration

        msg = file_send_lcmt()
        msg.data = gait_def_path.read_text()
        self.lc_file.publish("user_gait_file", msg.encode())
        time.sleep(0.5)
        msg.data = gait_params_full_path.read_text()
        self.lc_file.publish("user_gait_file", msg.encode())
        time.sleep(0.2)
        print(f"[控制] 已上传自定义步态: {gait_name}, "
              f"duration={self.user_gait_duration_ms}ms")

    def run_lowposture(self):
        self.in_lowposture = True
        print("[控制] 执行低姿态通过...")
        try:
            self.stop(1.5)
            print("[控制] 低姿态前先恢复站立，稳定身体...")
            self.send(MODE_RECOVERY, duration=2000)
            self.hold_current(5.0)
            self.send(MODE_STAND, duration=1000)
            self.hold_current(2.0)
            print(f"[控制] 开始低姿态行走，预计 {self.user_gait_duration_ms * 3.5 / 1000.0:.2f}s")
            self.send(MODE_USER_GAIT, GAIT_USER,
                      duration=self.user_gait_duration_ms,
                      contact=15)
            lowposture_hold_s = max(
                0.0,
                self.user_gait_duration_ms * 3.5 / 1000.0,
            )
            self.hold_current(lowposture_hold_s)
            print("[控制] 低姿态即将结束，直接强制恢复站立...")
            self.send(MODE_RECOVERY, duration=3000)
            self.hold_current(10.0)
            self.send(MODE_STAND, duration=1500)
            self.hold_current(3.0)
            self.stand(1.0)
            self.enter_locomotion()
            print("[控制] 低姿态通过完成，恢复正常前进")
        finally:
            self.in_lowposture = False

    def run_coke_lost_recovery(self):
        print("[控制] 连续丢失可乐，停止追踪并执行恢复动作...")
        self.stop(0.8)

        print("[控制] 原地转身约180度...")
        self.motion_for(TURN_AROUND_DURATION, vyaw=TURN_AROUND_VYAW)
        self.stop(10)
        print(f"[控制] 转身结束，等待 {TURN_TO_BACKWARD_WAIT:.1f} 秒后再后退...")
        self.stop(TURN_TO_BACKWARD_WAIT)

        print(f"[控制] 后退 {BACKWARD_STEP_COUNT} 步...")
        for index in range(BACKWARD_STEP_COUNT):
            print(f"[控制] 后退第 {index + 1}/{BACKWARD_STEP_COUNT} 步")
            self.motion_for(BACKWARD_STEP_DURATION, vx=BACKWARD_VX)
            self.stop(BACKWARD_STEP_SETTLE)

        self.stand(1.0)
        print("[控制] 可乐丢失恢复动作完成")


class ImageDetector(Node):
    def __init__(self):
        super().__init__("coke_ganzi_detector")
        self.bridge = CvBridge()
        self.detector = VisionDetector(
            enable_coke=True,
            enable_football=False,
            enable_cube=False,
            enable_ganzi=True,
        )
        self.lock = threading.Lock()
        self.coke = None
        self.last_coke = None
        self.last_coke_time = 0.0
        self.ganzi = None
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

        coke = self._detect_coke_robust(frame)
        ganzi = self.detector.get_closest_target(
            frame, target_type="ganzi", area_threshold=GANZI_AREA_THRESHOLD)

        with self.lock:
            self.image_width = frame.shape[1]
            self.coke = coke
            self.ganzi = ganzi

    def _detect_coke_robust(self, frame):
        dets = []
        if self.detector.coke_model is not None:
            results = self.detector.coke_model(
                frame,
                verbose=False,
                conf=COKE_CONF_THRESHOLD,
                iou=0.45,
                imgsz=COKE_IMGSZ,
            )
            if results and results[0].boxes is not None:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    dets.append([x1, y1, x2, y2, conf])

        best = self._largest_detection(dets, COKE_AREA_THRESHOLD)
        now = time.time()
        if best is not None:
            self.last_coke = best
            self.last_coke_time = now
            return best

        if self.last_coke is not None and now - self.last_coke_time <= COKE_MEMORY_SECONDS:
            return self.last_coke
        return None

    @staticmethod
    def _largest_detection(detections, area_threshold):
        if not detections:
            return None
        best = max(detections, key=lambda d: (d[2] - d[0]) * (d[3] - d[1]))
        x1, y1, x2, y2, conf = best
        area = (x2 - x1) * (y2 - y1)
        if area < area_threshold:
            return None
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        return (cx, cy, area, conf)

    def snapshot(self):
        with self.lock:
            return self.coke, self.ganzi, self.image_width

    def second_lowposture_snapshot(self):
        with self.lock:
            return self.ganzi, self.image_width


def start_detector():
    rclpy.init()
    node = ImageDetector()
    thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    thread.start()
    return node


def follow_coke_step(ctrl, coke, image_width):
    cx, _, area, conf = coke
    error = cx - image_width / 2.0

    if area >= COKE_CLOSE_AREA:
        ctrl.stop(0.5)
        print(f"[任务] 已接近可乐瓶 area={area:.0f}, conf={conf:.2f}")
        return True

    if abs(error) <= COKE_DEAD_ZONE:
        print(f"[跟随] 可乐居中，前进 area={area:.0f}, conf={conf:.2f}")
        ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vx=FORWARD_VX)
        ctrl.hold_current(FORWARD_DURATION)
        ctrl.stop(0.25)
        return False

    error_ratio = clamp(abs(error) / (image_width / 2.0), 0.0, 1.0)
    vyaw_mag = TURN_VYAW_MIN + (TURN_VYAW_MAX - TURN_VYAW_MIN) * error_ratio
    duration = TURN_DURATION_MIN + (
        TURN_DURATION_MAX - TURN_DURATION_MIN) * error_ratio
    vyaw = vyaw_mag if error < 0 else -vyaw_mag
    print(f"[跟随] 对准可乐 error={error:.0f}, vyaw={vyaw:.2f}, "
          f"duration={duration:.2f}s")
    ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=vyaw)
    ctrl.hold_current(duration)
    ctrl.stop(0.25)
    return False


def face_ganzi_precisely_and_run_lowposture(ctrl, detector):
    print(f"[任务] 转身/后退结束，等待 {GANZI_REDETECT_SLEEP:.1f} 秒后识别限高框...")
    time.sleep(GANZI_REDETECT_SLEEP)

    ready_count = 0
    while True:
        if ctrl.in_lowposture:
            ctrl.hold_current(0.5)
            continue

        ganzi, width = detector.second_lowposture_snapshot()
        if ganzi is None:
            ready_count = 0
            print("[二次低姿态] 暂未识别到限高框，原地等待...")
            ctrl.stand(0.8)
            time.sleep(GANZI_CONTROL_CYCLE_TIME)
            continue

        cx, _, area, conf = ganzi
        ganzi_error = cx - width / 2.0
        if abs(ganzi_error) > SECOND_GANZI_DEAD_ZONE:
            ready_count = 0
            error_ratio = clamp(abs(ganzi_error) / (width / 2.0), 0.0, 1.0)
            vyaw_mag = GANZI_TURN_VYAW_MIN + (
                GANZI_TURN_VYAW_MAX - GANZI_TURN_VYAW_MIN) * error_ratio
            duration = GANZI_TURN_DURATION_MIN + (
                GANZI_TURN_DURATION_MAX - GANZI_TURN_DURATION_MIN) * error_ratio
            vyaw = vyaw_mag if ganzi_error < 0 else -vyaw_mag
            print(f"[二次低姿态] 限高框未足够居中 error={ganzi_error:.0f}, "
                  f"vyaw={vyaw:.2f}, duration={duration:.2f}s, area={area:.0f}")
            ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=vyaw)
            time.sleep(duration)
            ctrl.stop(0.25)
            time.sleep(GANZI_CONTROL_CYCLE_TIME)
            continue

        if area < GANZI_CLOSE_AREA:
            ready_count = 0
            print(f"[二次低姿态] 限高框已正对但距离还不够近，前进靠近 area={area:.0f}")
            ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vx=GANZI_APPROACH_VX)
            time.sleep(GANZI_APPROACH_DURATION)
            ctrl.stop(0.25)
            time.sleep(GANZI_CONTROL_CYCLE_TIME)
            continue

        ready_count += 1
        print(f"[二次低姿态] 限高框特别正对且足够近 {ready_count}/"
              f"{SECOND_GANZI_CONFIRM_FRAMES}, ganzi_error={ganzi_error:.0f}, "
              f"area={area:.0f}, conf={conf:.2f}")
        if ready_count >= SECOND_GANZI_CONFIRM_FRAMES:
            ctrl.stop(0.5)
            ctrl.run_lowposture()
            ctrl.hold_current(1.0)
            return

        ctrl.stop(0.5)
        time.sleep(GANZI_CONTROL_CYCLE_TIME)


def send_manual_burst(ctrl, label, send_count, sleep_s, vx=0.0, vy=0.0, vyaw=0.0):
    print(f"[手动收尾] {label}: 连发 {send_count} 次，然后等待 {sleep_s:.1f}s")
    for index in range(send_count):
        print(f"[手动收尾] {label} 第 {index + 1}/{send_count} 次")
        ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vx=vx, vy=vy, vyaw=vyaw)
        time.sleep(MANUAL_SEND_GAP)
    time.sleep(sleep_s)


def face_ganzi_after_second_lowposture(ctrl, detector):
    print("[手动收尾] 开始重新识别限高杆，只通过旋转保证正面限高杆...")
    while True:
        _, ganzi, width = detector.snapshot()
        if ganzi is None:
            print("[手动收尾] 暂未识别到限高杆，原地等待...")
            ctrl.stand(0.8)
            time.sleep(GANZI_CONTROL_CYCLE_TIME)
            continue

        cx, _, area, conf = ganzi
        error = cx - width / 2.0
        if abs(error) <= POST_LOW_GANZI_FINAL_DEAD_ZONE:
            print(f"[手动收尾] 已正面限高杆 error={error:.0f}, "
                  f"area={area:.0f}, conf={conf:.2f}")
            ctrl.stop(0.8)
            return

        error_ratio = clamp(abs(error) / (width / 2.0), 0.0, 1.0)
        vyaw_mag = GANZI_TURN_VYAW_MIN + (
            GANZI_TURN_VYAW_MAX - GANZI_TURN_VYAW_MIN) * error_ratio
        duration = GANZI_TURN_DURATION_MIN + (
            GANZI_TURN_DURATION_MAX - GANZI_TURN_DURATION_MIN) * error_ratio
        vyaw = vyaw_mag if error < 0 else -vyaw_mag
        print(f"[手动收尾] 旋转对准限高杆 error={error:.0f}, "
              f"vyaw={vyaw:.2f}, duration={duration:.2f}s, area={area:.0f}")
        ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=vyaw)
        time.sleep(duration)
        ctrl.stop(0.25)
        time.sleep(GANZI_CONTROL_CYCLE_TIME)


def run_after_second_lowposture_sequence(ctrl, detector):
    print("[任务] 第二次低姿态通过完成，开始执行专用收尾流程...")
    ctrl.stand(1.0)
    ctrl.enter_locomotion()

    send_manual_burst(
        ctrl,
        "前进",
        POST_LOW_FORWARD_STEPS,
        POST_LOW_FORWARD_SLEEP,
        vx=MANUAL_VX,
    )
    time.sleep(5.0)
    send_manual_burst(
        ctrl,
        "右转",
        POST_LOW_RIGHT_TURN_STEPS,
        POST_LOW_RIGHT_TURN_SLEEP,
        vyaw=-MANUAL_VYAW,
    )
    time.sleep(10.0)
    print("[手动收尾] 自转完成，后退 1 步")
    ctrl.motion_for(BACKWARD_STEP_DURATION, vx=BACKWARD_VX)
    ctrl.stop(BACKWARD_STEP_SETTLE)

    print(f"[手动收尾] 后退完成，等待 {POST_LOW_BACKWARD_SLEEP:.1f}s")
    time.sleep(POST_LOW_BACKWARD_SLEEP)
    time.sleep(5.0)
    face_ganzi_after_second_lowposture(ctrl, detector)
    ctrl.enter_locomotion()
    send_manual_burst(
        ctrl,
        "右平移",
        POST_LOW_RIGHT_SHIFT_STEPS,
        POST_LOW_RIGHT_SHIFT_SLEEP,
        vy=-MANUAL_VY,
    )
    ctrl.stop(1.0)
    ctrl.stand(1.0)
    time.sleep(8.0)
    print("[任务] 二次低姿态后的专用收尾流程完成，程序准备结束")


def main():
    ctrl = LcmController()
    detector = start_detector()

    lowposture_done = False
    ganzi_close_count = 0
    coke_lost_count = 0

    ctrl.upload_user_gait(LOWPOSTURE_NAME)
    ctrl.stand_up()
    ctrl.enter_locomotion()

    try:
        while True:
            if ctrl.in_lowposture:
                ctrl.hold_current(0.5)
                continue

            coke, ganzi, width = detector.snapshot()

            if (not lowposture_done and ganzi is not None
                    and ganzi[2] >= GANZI_CLOSE_AREA):
                ganzi_close_count += 1
                print(f"[视觉] 限高杆接近 {ganzi_close_count}/"
                      f"{GANZI_CONFIRM_FRAMES}, area={ganzi[2]:.0f}")
                if ganzi_close_count >= GANZI_CONFIRM_FRAMES:
                    ctrl.run_lowposture()
                    lowposture_done = True
                    ganzi_close_count = 0
                    ctrl.hold_current(1.0)
                continue
            else:
                ganzi_close_count = 0

            if coke is None:
                coke_lost_count += 1
                print(f"[视觉] 未识别到可乐瓶 {coke_lost_count}/{COKE_LOST_LIMIT}")
                if coke_lost_count >= COKE_LOST_LIMIT:
                    ctrl.run_coke_lost_recovery()
                    print("[任务] 可乐追踪已停止")
                    face_ganzi_precisely_and_run_lowposture(ctrl, detector)
                    print("[任务] 二次限高杆低姿态通过完成")
                    run_after_second_lowposture_sequence(ctrl, detector)
                    break
                ctrl.stand(0.8)
                ctrl.hold_current(FOLLOW_CYCLE_TIME)
                continue

            coke_lost_count = 0

            finished = follow_coke_step(ctrl, coke, width)
            if finished:
                print("[任务] 到达可乐瓶前，任务完成")
                break

            ctrl.hold_current(FOLLOW_CYCLE_TIME)

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
