"""
赛段一：石径探路
策略：用TF精确对齐到-180度，然后倒走通过石板路
"""

import time
import math
import sys

sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')

import lcm
import robot_control_cmd_lcmt

# 速度参数
VX_ROCKROAD = -0.20
VX_SCURVE   = 0.25
STEP_ROCKROAD = 0.06
STEP_NORMAL   = 0.03
PITCH       = 0.30

# LCM
LCM_URL = "udpm://239.255.76.67:7671?ttl=255"
_lc = lcm.LCM(LCM_URL)
_count = 0

def _raw_cmd(mode, gait=0, vx=0.0, vy=0.0, vyaw=0.0, step_height=0.06):
    global _count
    _count = (_count + 1) % 127
    m = robot_control_cmd_lcmt.robot_control_cmd_lcmt()
    m.mode = mode
    m.gait_id = gait
    m.life_count = _count
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
    _lc.publish("robot_control_cmd", m.encode())


class StagePath:
    """
    mode='rockroad' : 赛段一，石板路（对齐-180度+倒走）
    mode='scurve'   : 赛段三，S形弯道（正走）
    """

    def __init__(self, ctrl, mode='rockroad'):
        self.ctrl = ctrl
        self.mode = mode
        self.vx = VX_ROCKROAD if mode == 'rockroad' else VX_SCURVE
        self.step_h = STEP_ROCKROAD if mode == 'rockroad' else STEP_NORMAL
        self._prep_done = mode != 'rockroad'
        self._started = False

    def _align_yaw(self, target_yaw, tracker, tolerance=0.05):
        """用tracker的yaw精确对齐"""
        print(f"  对齐方向到 {math.degrees(target_yaw):.1f}°...")
        # 预热：发运动指令让tf开始更新
        for _ in range(60):
            _raw_cmd(11, 9, vx=0.05, step_height=self.step_h)
            time.sleep(0.05)
        for _ in range(10):
            _raw_cmd(11, 9, vx=0.0)
            time.sleep(0.1)

        stable_start = None
        while True:
            yaw = tracker.get_yaw()
            err = target_yaw - yaw
            while err > math.pi: err -= 2*math.pi
            while err < -math.pi: err += 2*math.pi

            if abs(err) < tolerance:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start > 1.0:
                    _raw_cmd(11, 9, vx=0.0)
                    break
                _raw_cmd(11, 9, vx=0.0, vyaw=0.0, step_height=self.step_h)
            else:
                stable_start = None
                vyaw = max(-0.4, min(0.4, 0.8 * err))
                _raw_cmd(11, 9, vx=0.0, vyaw=vyaw, step_height=self.step_h)

            print(f'\r  yaw={math.degrees(yaw):.1f}° 差={math.degrees(err):.1f}°',
                  end='', flush=True)
            time.sleep(0.05)
        print(f"\n  对齐完成: {math.degrees(tracker.get_yaw()):.1f}°")

    def step(self, current_y, tracker=None):
        """每帧调用一次，返回True表示完成"""
        if self.mode == 'rockroad' and not self._prep_done:
            if not self._started:
                self._started = True
                self._align_yaw(-math.pi, tracker, tolerance=0.05)
                self._prep_done = True
                print("[石板路] 开始倒走")
            return False

        # 倒走/正走（用队友步态 gait=3）
        _raw_cmd(11, 3, vx=self.vx, vyaw=0.0, step_height=self.step_h)
        time.sleep(0.05)
        return False

    def cleanup(self):
        _raw_cmd(11, 9, vx=0.0)
        print(f"[{self.mode}] 赛段结束，停止")
