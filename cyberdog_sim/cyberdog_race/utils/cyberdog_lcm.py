"""
CyberDog LCM 控制封装
统一管理所有运动指令的发送，包含 Wait_finish 机制
"""

import lcm
import sys
import time
import threading

sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
import robot_control_cmd_lcmt
import robot_control_response_lcmt

LCM_URL_SEND = "udpm://239.255.76.67:7671?ttl=255"
LCM_URL_RECV = "udpm://239.255.76.67:7670?ttl=255"

# 运动模式（来自官方 basic_motion 示例）
MODE_PASSIVE      = 0   # 趴下/被动
MODE_STAND        = 3   # 站立
MODE_PUREDAMPER   = 7   # 安全收尾（比趴下更稳，官方推荐结束动作）
MODE_LOCOMOTION   = 11  # 行走
MODE_JUMP         = 16  # 跳跃
MODE_RECOVERY     = 12  # 恢复站立
MODE_POSITION     = 21  # 姿态插值控制
MODE_PRESET       = 62  # 预制动作（握手/坐下/芭蕾等）
MODE_TWOLEG       = 64  # 两腿站立

# 站立
GAIT_STAND        = 0   # 站立

# 步态
GAIT_FAST_WALK    = 3   # 快速行走
GAIT_SLOW_WALK    = 27  # 慢速行走
GAIT_BOUND        = 7   # 跳跃行走
GAIT_TROT         = 10  # 小跑
GAIT_TORT_24_16   = 26  # 变频
GAIT_SLOPE_BALANCE    = 110  # 斜坡适应

# 跳跃
GAIT_JUMP         = 1   # 原地跳远

# 预制动作 gait_id（mode=62时使用）
GAIT_SHAKE_HAND   = 2   # 握手
GAIT_SIT_DOWN     = 3   # 坐下
GAIT_HIP_SWING    = 4   # 扭屁股
GAIT_HEAD_TWIST   = 5   # 扭头
GAIT_STRETCH      = 6   # 伸懒腰
GAIT_BALLET       = 11  # 芭蕾舞
GAIT_MOONWALK     = 12  # 太空步
GAIT_PUSH_UP      = 34  # 俯卧撑


class CyberdogController:
    def __init__(self):
        self.lc_s = lcm.LCM(LCM_URL_SEND)
        self.lc_r = lcm.LCM(LCM_URL_RECV)
        self.msg = robot_control_cmd_lcmt.robot_control_cmd_lcmt()
        self._life_count = 0
        self._lock = threading.Lock()
        # [LIFE-MOD-1] 按 life_count 跟踪等待中的命令
        self._pending_lock = threading.Lock()
        self._pending_events = {}

        # Wait_finish 状态
        self._mode_ok = 0
        self._gait_ok = 0
        self._running = True

        # 启动接收线程（订阅 robot_control_response）
        self.lc_r.subscribe("robot_control_response", self._response_handler)
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        # 启动保活线程
        self._stop_keepalive = threading.Event()
        self._keepalive_thread = threading.Thread(
            target=self._keepalive, daemon=True
        )
        self._keepalive_thread.start()

    def _response_handler(self, channel, data):
        """接收控制响应，更新当前完成状态"""
        rec = robot_control_response_lcmt.robot_control_response_lcmt().decode(data)
        if rec.order_process_bar >= 95:
            self._mode_ok = rec.mode
            self._gait_ok = rec.gait_id
            # [LIFE-MOD-2] 优先按 life_count 精确唤醒对应等待者
            rec_life = getattr(rec, 'life_count', None)
            if rec_life is not None:
                with self._pending_lock:
                    evt = self._pending_events.get(rec_life)
                if evt is not None:
                    evt.set()
        else:
            self._mode_ok = 0

    def _recv_loop(self):
        while self._running:
            self.lc_r.handle()
            time.sleep(0.002)

    def _next_life(self):
        self._life_count = (self._life_count + 1) % 127
        return self._life_count

    def _keepalive(self):
        """每100ms重发当前指令，防止控制程序超时"""
        while not self._stop_keepalive.is_set():
            with self._lock:
                try:
                    self.lc_s.publish("robot_control_cmd", self.msg.encode())
                except Exception:
                    pass
            time.sleep(0.1)

    # [WAIT-MOD-1] 最小改动：send 增加可选 wait/timeout，默认仍为非阻塞
    # [LIFE-MOD-3] wait=True 时优先使用 life_count 等待，超时后回退到 mode/gait
    def send(self, mode, gait_id=0, vx=0.0, vy=0.0, vyaw=0.0,
             body_roll=0.0, pitch=0.0, body_height=0.225,
             duration=500, step_height=0.08, 
             wait=True, timeout=10.0):
        life_count = None
        life_event = None

        with self._lock:
            # [WAIT-MOD-2] 避免复用上一次完成状态导致 wait 误判为已完成
            self._mode_ok = 0
            self._gait_ok = 0

            self.msg.mode        = mode
            self.msg.gait_id     = gait_id
            life_count = self._next_life()

            self.msg.life_count  = life_count
            self.msg.duration    = duration
            self.msg.contact     = 0
            self.msg.value       = 0
            self.msg.vel_des     = [vx, vy, vyaw]
            self.msg.rpy_des     = [body_roll, pitch, 0.0]
            self.msg.pos_des     = [0.0, 0.0, body_height]
            self.msg.acc_des     = [0.0] * 6
            self.msg.ctrl_point  = [0.0] * 3
            self.msg.foot_pose   = [0.0] * 6
            self.msg.step_height = [step_height, step_height]

            if wait:
                life_event = threading.Event()
                with self._pending_lock:
                    self._pending_events[life_count] = life_event

            self.lc_s.publish("robot_control_cmd", self.msg.encode())

        if wait:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if life_event is not None and life_event.wait(timeout=max(0.0, remaining)):
                    with self._pending_lock:
                        self._pending_events.pop(life_count, None)
                    return True

            with self._pending_lock:
                self._pending_events.pop(life_count, None)

            # [LIFE-MOD-4] 回退兼容：若响应无 life_count，则沿用 mode/gait 判定
            return self.wait_finish(mode, gait_id=gait_id, timeout=0.2)

        return True

    def wait_finish(self, mode, gait_id=0, timeout=10.0):
        """
        等待控制程序完成指定动作（order_process_bar >= 90）
        timeout: 最长等待秒数，超时后继续执行
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._mode_ok == mode and self._gait_ok == gait_id:
                return True
            time.sleep(0.005)
        print(f"[警告] wait_finish 超时: mode={mode}, gait_id={gait_id}")
        return False

    def stand_up(self):
        """完整站立流程：recovery → 等待完成"""
        print("[控制] 恢复站立...")
        self.send(MODE_RECOVERY, duration=2000, wait=False)
        self.wait_finish(MODE_RECOVERY, timeout=3.0)
        print("[控制] 站立完成")

    def lie_down(self):
        """安全收尾"""
        self.send(MODE_PUREDAMPER, duration=0, wait=True)
        self.wait_finish(MODE_PUREDAMPER, timeout=3.0)
        print("[控制] 安全收尾完成")

    # [WAIT-MOD-3] 最小改动：move 透传 wait/timeout，默认行为不变
    def move(self, vx=0.0, vy=0.0, vyaw=0.0,
             step_height=0.06, body_height=0.0,
             wait=True, timeout=0.5):
        """行走指令（默认不等待；可选 wait=True 等待完成）"""
        return self.send(MODE_LOCOMOTION, GAIT_FAST_WALK,
                         vx=vx, vy=vy, vyaw=vyaw,
                         step_height=step_height,
                         body_height=body_height,
                         wait=wait, timeout=timeout)
    
    def move_lowhead(self, vx=0.0, vy=0.0, vyaw=0.0, 
                     step_height=0.06, pitch=0.20,
                     wait=True, timeout=0.5):
        """
        低头行走：机身前倾约15度，方便相机看到近处黄线
        用于赛段一/三黄线跟踪
        """
        return self.send(MODE_LOCOMOTION, GAIT_FAST_WALK,
                         vx=vx, vy=vy, vyaw=vyaw,
                        #  body_roll=0.0, body_height=0.225,
                         step_height=step_height, pitch=pitch,
                         wait=wait, timeout=timeout)

    def slow_move(self, vx=0.0, vy=0.0, vyaw=0.0,
                    step_height=0.06, body_height=0.0,
                    wait=True, timeout=0.5):
        """慢速行走指令（默认不等待；可选 wait=True 等待完成）"""
        return self.send(MODE_LOCOMOTION, GAIT_SLOW_WALK,
                         vx=vx, vy=vy, vyaw=vyaw,
                         step_height=step_height,
                         body_height=body_height,
                         wait=wait, timeout=timeout)
    
    def slope_balance(self, vx=0.03, vy=0.0, vyaw=0.0,
                      step_height=0.02, body_height=-0.050,
                      wait=True, timeout=0.5):
        """斜坡适应行走指令（默认不等待；可选 wait=True 等待完成）"""
        return self.send(MODE_LOCOMOTION, GAIT_SLOPE_BALANCE,
                         vx=vx, vy=vy, vyaw=vyaw,
                         step_height=step_height,
                         body_height=body_height,
                         wait=wait, timeout=timeout)
    
    def move_uphead(self, vx=0.0, vy=0.0, vyaw=0.0, 
                        step_height=0.02, pitch=-0.20,
                        wait=True, timeout=0.5):
        return self.send(MODE_LOCOMOTION, GAIT_FAST_WALK,
                         vx=vx, vy=vy, vyaw=vyaw,
                        #  body_roll=0.0, body_height=0.225,
                         step_height=step_height, pitch=pitch,
                         wait=wait, timeout=timeout)
        
        
    # [WAIT-MOD-4] 最小改动：jump 透传 wait/timeout
    def jump(self, duration=1000, wait=True, timeout=10.0):
        """跳跃指令（默认不等待；可选 wait=True）"""
        return self.send(MODE_JUMP, GAIT_JUMP,
                         duration=duration,
                         wait=wait, timeout=timeout)

    # [WAIT-MOD-6] 最小改动：stop 透传 wait/timeout
    def stop(self, wait=True, body_roll=0.0, timeout=5.0):
        return self.send(MODE_LOCOMOTION, GAIT_SLOW_WALK,
                         body_roll=body_roll,
                         wait=wait, timeout=timeout)

    def preset_action(self, gait_id, wait=True, timeout=8.0):
        """执行预制动作（mode=62）"""
        self.send(MODE_PRESET, gait_id=gait_id, duration=0)
        if wait:
            self.wait_finish(MODE_PRESET, gait_id, timeout=timeout)

    def shutdown(self):
        self._running = False
        self._stop_keepalive.set()
        self.lie_down()
