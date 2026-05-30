#!/usr/bin/env python3
"""
白色小球追踪 + 半程低姿态通过

流程：
1. 追踪白色小球/足球，控制方式参考 test.py 的三秒步进控制。
2. 当小球检测框面积足够大，认为已经非常靠近。
3. 停止追踪，执行 lowposture_forward 自定义步态，行走时间为原来的一半。
4. 第一次低姿态结束后，按手动控制步进左转，再直接执行第二次低姿态。
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
GAIT_DIR = BASE_DIR / "loco_hl_example" / "customized_gait"
LCM_TYPE_DIR = BASE_DIR / "src" / "cyberdog_locomotion" / "common" / "lcm_type" / "lcm"

VISION_DIR_CANDIDATES = [
    BASE_DIR / "vision_module",
    BASE_DIR / "vision2",
]
VISION_DIR = next((path for path in VISION_DIR_CANDIDATES if (path / "vision_module.py").exists()), None)
if VISION_DIR is None:
    raise FileNotFoundError("找不到视觉模块，请确认 vision_module/vision_module.py 已复制到程序同级目录")

sys.path.insert(0, str(VISION_DIR))
sys.path.insert(0, str(GAIT_DIR))
sys.path.insert(0, str(LCM_TYPE_DIR))
sys.path.insert(0, "/usr/local/lib/python3.8/site-packages")

from file_send_lcmt import file_send_lcmt
from robot_control_cmd_lcmt import robot_control_cmd_lcmt


def load_vision_detector_class():
    module_path = VISION_DIR / "vision_module.py"
    spec = importlib.util.spec_from_file_location("cyberdog_ball_vision", str(module_path))
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
LOWPOSTURE_TIME_SCALE = 0.3  # 低姿态前进时间：原来的一半

# 白球追踪参数
BALL_AREA_THRESHOLD = 80       # 过滤太小的白球检测框
BALL_CLOSE_AREA = 2500        # 小球足够近的面积阈值，可按画面调
BALL_DEAD_ZONE = 25            # 白球追踪前进死区，越小越居中
BALL_FINAL_DEAD_ZONE = 12      # 切低姿态前的最终居中死区，必须非常靠近中心
BALL_CONFIRM_FRAMES = 2        # 连续几帧接近且居中后触发低姿态

FOLLOW_CYCLE_TIME = 5.0        # 追踪阶段每条控制指令后的等待时间
STOP_SETTLE = 0.5
FORWARD_VX = 0.55              # 直线前进速度，可调
FORWARD_DURATION = 1.0         # 单次直线前进时间，可调
TURN_VYAW_MIN = 0.10           # 小误差转向速度，可调
TURN_VYAW_MAX = 0.32           # 大误差转向速度，可调
TURN_DURATION_MIN = 0.25       # 单次转向最短时间，可调
TURN_DURATION_MAX = 0.75       # 单次转向最长时间，可调
LOST_SEARCH_AFTER = 3
SEARCH_VYAW = 0.18
SEARCH_DURATION = 0.6
YAW_DIR = 1                    # 转向方向不对时改成 -1

# 第一次低姿态结束后的转身参数
MANUAL_FORWARD_VX = 0.4        # 手动控制基准前进速度
MANUAL_LATERAL_VY = 0.3        # 手动控制基准左右平移速度
MANUAL_TURN_VYAW = 0.6         # 手动控制基准左转角速度
MANUAL_SEND_GAP = 0.1          # 手动步进连发间隔
MANUAL_LEFT_TURN_STEPS = 12    # 第一次低姿态后，左转次数
MANUAL_FINE_TURN_SCALE = 0.6   # 最后一次左转使用一半速度
MANUAL_TURN_SETTLE = 1.0       # 转身完成后停稳时间
POST_TURN_TO_LOWPOSTURE_SLEEP = 1.0 # 转身完成后等待多久再切第二次低姿态
POST_SECOND_LOWPOSTURE_SLEEP = 10.0
FINAL_FORWARD_STEPS = 11
FINAL_FORWARD_SLEEP = 20.0
FINAL_LEFT_SHIFT_STEPS = 6
FINAL_LEFT_SHIFT_SLEEP = 10.0
FINAL_RIGHT_TURN_STEPS = 12
FINAL_RIGHT_TURN_FINE_SCALE = 0.5
FINAL_RIGHT_TURN_SLEEP = 20.0


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
        self.keepalive_stop = threading.Event()
        self.keepalive_thread = threading.Thread(target=self._keepalive_loop, daemon=True)
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
        print(f"[控制] 已上传自定义步态: {gait_name}, duration={self.user_gait_duration_ms}ms")

    def run_lowposture_half(self):
        print("[控制] 执行半程低姿态前进...")
        self.stop(1.5)
        print("[控制] 低姿态前先恢复站立，稳定身体...")
        self.send(MODE_RECOVERY, duration=2000)
        self.hold_current(5.0)
        self.send(MODE_STAND, duration=1000)
        self.hold_current(2.0)

        half_duration_ms = max(1, int(self.user_gait_duration_ms * LOWPOSTURE_TIME_SCALE))
        hold_s = max(1.0, self.user_gait_duration_ms * 3.5 * LOWPOSTURE_TIME_SCALE / 1000.0)
        print(f"[控制] 开始半程低姿态行走，预计 {hold_s:.2f}s")
        self.send(MODE_USER_GAIT, GAIT_USER, duration=half_duration_ms, contact=15)
        self.hold_current(hold_s)
        print("[控制] 保留低姿态后的固定等待 40s...")
        time.sleep(40)

        print("[控制] 半程低姿态结束，恢复站立...")
        self.send(MODE_RECOVERY, duration=3000)
        self.hold_current(10.0)
        self.send(MODE_STAND, duration=1500)
        self.hold_current(3.0)
        self.stand(1.0)
        print("[控制] 半程低姿态完成")

    def turn_around_180(self):
        time.sleep(20)
        print("[控制] 第一次低姿态完成，按手动控制步进左转...")
        self.stop(0.8)
        self.enter_locomotion()
        for index in range(MANUAL_LEFT_TURN_STEPS):
            print(f"[控制] 手动左转第 {index + 1}/{MANUAL_LEFT_TURN_STEPS} 次")
            self.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=MANUAL_TURN_VYAW)
            time.sleep(MANUAL_SEND_GAP)
        
        fine_vyaw = MANUAL_TURN_VYAW * MANUAL_FINE_TURN_SCALE
        print(f"[控制] 半速左转 1 次，vyaw={fine_vyaw:.2f}")
        self.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=fine_vyaw)
        time.sleep(MANUAL_SEND_GAP)
        self.stop(MANUAL_TURN_SETTLE)
        time.sleep(60)
        print("[控制] 手动步进转身完成")

    def manual_burst(self, label, count, vx=0.0, vy=0.0, vyaw=0.0):
        print(f"[手动收尾] {label}: 连发 {count} 次")
        for index in range(count):
            print(f"[手动收尾] {label} 第 {index + 1}/{count} 次")
            self.send(MODE_LOCOMOTION, GAIT_TROT, vx=vx, vy=vy, vyaw=vyaw)
            time.sleep(MANUAL_SEND_GAP)

    def run_final_finish_sequence(self):
        print("[任务] 第二次低姿态后，执行最终收尾动作序列...")
        self.enter_locomotion()

        self.manual_burst("前进", FINAL_FORWARD_STEPS, vx=MANUAL_FORWARD_VX)
        print(f"[手动收尾] 前进完成，等待 {FINAL_FORWARD_SLEEP:.1f}s")
        time.sleep(FINAL_FORWARD_SLEEP)

        self.manual_burst("左平移", FINAL_LEFT_SHIFT_STEPS, vy=MANUAL_LATERAL_VY)
        print(f"[手动收尾] 左平移完成，等待 {FINAL_LEFT_SHIFT_SLEEP:.1f}s")
        time.sleep(FINAL_LEFT_SHIFT_SLEEP)

        self.manual_burst("右转", FINAL_RIGHT_TURN_STEPS, vyaw=-MANUAL_TURN_VYAW)
        fine_vyaw = -MANUAL_TURN_VYAW * FINAL_RIGHT_TURN_FINE_SCALE
        print(f"[手动收尾] 半速右转 1 次，vyaw={fine_vyaw:.2f}")
        self.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=fine_vyaw)
        time.sleep(MANUAL_SEND_GAP)
        print(f"[手动收尾] 右转完成，等待 {FINAL_RIGHT_TURN_SLEEP:.1f}s")
        time.sleep(FINAL_RIGHT_TURN_SLEEP)

        self.stop(1.0)
        self.stand(1.0)
        print("[任务] 最终收尾动作完成")

    def run_second_lowposture_after_turn(self):
        print("[任务] 手动转身完成，准备直接进入第二次低姿态...")
        self.stop(POST_TURN_TO_LOWPOSTURE_SLEEP)
        self.run_lowposture_half()
        time.sleep(20)
        print(f"[任务] 第二次低姿态后追加等待 {POST_SECOND_LOWPOSTURE_SLEEP:.1f}s")
        time.sleep(POST_SECOND_LOWPOSTURE_SLEEP)
        self.run_final_finish_sequence()
        print("[任务] 第二次低姿态执行完成")


class BallDetector(Node):
    def __init__(self):
        super().__init__("ball_lowposture_detector")
        self.bridge = CvBridge()
        self.detector = VisionDetector(
            enable_coke=False,
            enable_football=True,
            enable_cube=False,
            enable_ganzi=False,
        )
        self.lock = threading.Lock()
        self.ball = None
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

        target = self.detector.get_closest_target(
            frame,
            target_type="football",
            area_threshold=BALL_AREA_THRESHOLD,
        )
        with self.lock:
            self.image_width = frame.shape[1]
            self.ball = target

    def snapshot(self):
        with self.lock:
            return self.ball, self.image_width


def start_detector():
    rclpy.init()
    node = BallDetector()
    thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    thread.start()
    return node


def follow_ball_step(ctrl, ball, image_width):
    cx, _, area, conf = ball
    error = cx - image_width / 2.0

    if area >= BALL_CLOSE_AREA:
        print(f"[跟随] 白球已达到靠近阈值，禁止继续前进 area={area:.0f}, conf={conf:.2f}")
        ctrl.stop(STOP_SETTLE)
        return

    if abs(error) <= BALL_DEAD_ZONE:
        print(f"[跟随] 白球居中，前进 area={area:.0f}, conf={conf:.2f}")
        ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vx=FORWARD_VX)
        time.sleep(FORWARD_DURATION)
        ctrl.stop(STOP_SETTLE)
        return

    error_ratio = clamp(abs(error) / (image_width / 2.0), 0.0, 1.0)
    vyaw_mag = TURN_VYAW_MIN + (TURN_VYAW_MAX - TURN_VYAW_MIN) * error_ratio
    duration = TURN_DURATION_MIN + (TURN_DURATION_MAX - TURN_DURATION_MIN) * error_ratio
    vyaw = YAW_DIR * vyaw_mag if error < 0 else YAW_DIR * (-vyaw_mag)
    print(f"[跟随] 对准白球 error={error:.0f}, vyaw={vyaw:.2f}, duration={duration:.2f}s")
    ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=vyaw)
    time.sleep(duration)
    ctrl.stop(STOP_SETTLE)


def align_close_ball_turn_only(ctrl, ball, image_width):
    cx, _, area, conf = ball
    error = cx - image_width / 2.0

    if abs(error) <= BALL_FINAL_DEAD_ZONE:
        print(f"[近距离对准] 白球已在最终中心范围，停住确认 "
              f"error={error:.0f}, area={area:.0f}, conf={conf:.2f}")
        ctrl.stop(0.8)
        return

    error_ratio = clamp(abs(error) / (image_width / 2.0), 0.0, 1.0)
    vyaw_mag = TURN_VYAW_MIN + (TURN_VYAW_MAX - TURN_VYAW_MIN) * error_ratio
    duration = TURN_DURATION_MIN + (TURN_DURATION_MAX - TURN_DURATION_MIN) * error_ratio
    vyaw = YAW_DIR * vyaw_mag if error < 0 else YAW_DIR * (-vyaw_mag)
    print(f"[近距离对准] 白球已足够近，只原地调整 error={error:.0f}, "
          f"vyaw={vyaw:.2f}, duration={duration:.2f}s, area={area:.0f}, conf={conf:.2f}")
    ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=vyaw)
    time.sleep(duration)
    ctrl.stop(STOP_SETTLE)


def main():
    ctrl = LcmController()
    detector = start_detector()
    close_count = 0
    lost_count = 0
    last_error = 0.0

    ctrl.upload_user_gait(LOWPOSTURE_NAME)
    ctrl.stand_up()
    ctrl.enter_locomotion()

    try:
        while True:
            ball, width = detector.snapshot()

            if ball is None:
                close_count = 0
                lost_count += 1
                if lost_count >= LOST_SEARCH_AFTER:
                    search_dir = 1 if last_error < 0 else -1
                    vyaw = YAW_DIR * search_dir * SEARCH_VYAW
                    print(f"[视觉] 未识别到白球，慢速搜索 vyaw={vyaw:.2f}")
                    ctrl.send(MODE_LOCOMOTION, GAIT_TROT, vyaw=vyaw)
                    time.sleep(SEARCH_DURATION)
                    ctrl.stop(STOP_SETTLE)
                else:
                    print("[视觉] 未识别到白球，停住等待...")
                    ctrl.stop(STOP_SETTLE)
                time.sleep(FOLLOW_CYCLE_TIME)
                continue

            cx, _, area, conf = ball
            error = cx - width / 2.0
            last_error = error
            lost_count = 0

            if area >= BALL_CLOSE_AREA:
                if abs(error) <= BALL_FINAL_DEAD_ZONE:
                    close_count += 1
                    print(f"[视觉] 白球非常近且在正中心 "
                          f"{close_count}/{BALL_CONFIRM_FRAMES}, "
                          f"error={error:.0f}, area={area:.0f}, conf={conf:.2f}")
                    ctrl.stop(0.8)
                    if close_count >= BALL_CONFIRM_FRAMES:
                        ctrl.run_lowposture_half()
                        ctrl.turn_around_180()
                        ctrl.run_second_lowposture_after_turn()
                        print("[任务] 白球赛道二次低姿态完成")
                        break
                    time.sleep(FOLLOW_CYCLE_TIME)
                    continue

                close_count = 0
                print(f"[视觉] 白球已经足够近，禁止前进，只原地对准 error={error:.0f}")
                align_close_ball_turn_only(ctrl, ball, width)
                time.sleep(FOLLOW_CYCLE_TIME)
                continue

            close_count = 0
            follow_ball_step(ctrl, ball, width)
            time.sleep(FOLLOW_CYCLE_TIME)

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
