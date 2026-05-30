# """
# CyberDog LCM 控制封装
# 统一管理所有运动指令的发送，包含 Wait_finish 机制
# """

# import lcm
# import sys
# import time
# import threading

# sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')
# sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
# import robot_control_cmd_lcmt
# import robot_control_response_lcmt

# LCM_URL_SEND = "udpm://239.255.76.67:7671?ttl=255"
# LCM_URL_RECV = "udpm://239.255.76.67:7670?ttl=255"

# # 运动模式（来自官方 basic_motion 示例）
# MODE_PASSIVE      = 0   # 趴下/被动
# MODE_STAND        = 6   # 站立
# MODE_PUREDAMPER   = 7   # 安全收尾（比趴下更稳，官方推荐结束动作）
# MODE_LOCOMOTION   = 11  # 行走
# MODE_JUMP         = 16  # 跳跃
# MODE_RECOVERY     = 12  # 恢复站立
# MODE_POSITION     = 21  # 姿态插值控制
# MODE_PRESET       = 62  # 预制动作（握手/坐下/芭蕾等）
# MODE_TWOLEG       = 64  # 两腿站立

# # 步态
# GAIT_FAST_WALK    = 3   # 快速行走
# GAIT_SLOW_WALK    = 27  # 慢速行走
# GAIT_BOUND        = 7   # 跳跃行走
# GAIT_TROT         = 10  # 小跑

# # 跳跃
# GAIT_JUMP         = 1   # 原地跳远

# # 预制动作 gait_id（mode=62时使用）
# GAIT_SHAKE_HAND   = 2   # 握手
# GAIT_SIT_DOWN     = 3   # 坐下
# GAIT_HIP_SWING    = 4   # 扭屁股
# GAIT_HEAD_TWIST   = 5   # 扭头
# GAIT_STRETCH      = 6   # 伸懒腰
# GAIT_BALLET       = 11  # 芭蕾舞
# GAIT_MOONWALK     = 12  # 太空步
# GAIT_PUSH_UP      = 34  # 俯卧撑


# class CyberdogController:
#     def __init__(self):
#         self.lc_s = lcm.LCM(LCM_URL_SEND)
#         self.lc_r = lcm.LCM(LCM_URL_RECV)
#         self.msg = robot_control_cmd_lcmt.robot_control_cmd_lcmt()
#         self._life_count = 0
#         self._lock = threading.Lock()

#         # Wait_finish 状态
#         self._mode_ok = 0
#         self._gait_ok = 0
#         self._running = True

#         # 启动接收线程（订阅 robot_control_response）
#         self.lc_r.subscribe("robot_control_response", self._response_handler)
#         self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
#         self._recv_thread.start()

#         # 启动保活线程
#         self._stop_keepalive = threading.Event()
#         self._keepalive_thread = threading.Thread(
#             target=self._keepalive, daemon=True
#         )
#         self._keepalive_thread.start()

#     def _response_handler(self, channel, data):
#         """接收控制响应，更新当前完成状态"""
#         rec = robot_control_response_lcmt.robot_control_response_lcmt().decode(data)
#         if rec.order_process_bar >= 95:
#             self._mode_ok = rec.mode
#             self._gait_ok = rec.gait_id
#         else:
#             self._mode_ok = 0

#     def _recv_loop(self):
#         while self._running:
#             self.lc_r.handle()
#             time.sleep(0.002)

#     def _next_life(self):
#         self._life_count = (self._life_count + 1) % 127
#         return self._life_count

#     def _keepalive(self):
#         """每100ms重发当前指令，防止控制程序超时"""
#         while not self._stop_keepalive.is_set():
#             with self._lock:
#                 try:
#                     self.lc_s.publish("robot_control_cmd", self.msg.encode())
#                 except Exception:
#                     pass
#             time.sleep(0.1)

#     def send(self, mode, gait_id=0, vx=0.0, vy=0.0, vyaw=0.0,
#              duration=500, step_height=0.08, body_height=0.0):
#         with self._lock:
#             self.msg.mode        = mode
#             self.msg.gait_id     = gait_id
#             self.msg.life_count  = self._next_life()
#             self.msg.duration    = duration
#             self.msg.contact     = 0
#             self.msg.value       = 0
#             self.msg.vel_des     = [vx, vy, vyaw]
#             self.msg.rpy_des     = [0.0, 0.0, 0.0]
#             self.msg.pos_des     = [0.0, body_height, 0.0]
#             self.msg.acc_des     = [0.0] * 6
#             self.msg.ctrl_point  = [0.0] * 3
#             self.msg.foot_pose   = [0.0] * 6
#             self.msg.step_height = [step_height, step_height]
#             self.lc_s.publish("robot_control_cmd", self.msg.encode())

#     def wait_finish(self, mode, gait_id=0, timeout=10.0):
#         """
#         等待控制程序完成指定动作（order_process_bar >= 95）
#         timeout: 最长等待秒数，超时后继续执行
#         """
#         deadline = time.monotonic() + timeout
#         while time.monotonic() < deadline:
#             if self._mode_ok == mode and self._gait_ok == gait_id:
#                 return True
#             time.sleep(0.005)
#         print(f"[警告] wait_finish 超时: mode={mode}, gait_id={gait_id}")
#         return False

#     def stand_up(self):
#         """完整站立流程：recovery → 等待完成"""
#         print("[控制] 恢复站立...")
#         self.send(MODE_RECOVERY, duration=2000)
#         self.wait_finish(MODE_RECOVERY, timeout=5.0)
#         print("[控制] 站立完成")

#     def lie_down(self):
#         """安全收尾"""
#         self.send(MODE_PUREDAMPER, duration=0)
#         self.wait_finish(MODE_PUREDAMPER, timeout=3.0)
#         print("[控制] 安全收尾完成")

#     def move(self, vx=0.0, vy=0.0, vyaw=0.0,
#              step_height=0.08, body_height=0.0):
#         """行走指令（不等待完成，持续发送）"""
#         self.send(MODE_LOCOMOTION, GAIT_FAST_WALK,
#                   vx=vx, vy=vy, vyaw=vyaw,
#                   step_height=step_height,
#                   body_height=body_height)
        
#     def jump(self, duration=1000):
#         """跳跃指令（不等待完成，持续发送）"""
#         self.send(MODE_JUMP, GAIT_JUMP,
#                   duration=1000)
        
#     def dance(self, gait_id):
#         """预制动作指令（不等待完成，持续发送）"""
#         self.send(MODE_PRESET, gait_id=gait_id)

#     def stop(self):
#         self.send(MODE_LOCOMOTION, GAIT_TROT,
#                   vx=0.0, vy=0.0, vyaw=0.0)

#     def preset_action(self, gait_id, wait=True, timeout=8.0):
#         """执行预制动作（mode=62）"""
#         self.send(MODE_PRESET, gait_id=gait_id, duration=0)
#         if wait:
#             self.wait_finish(MODE_PRESET, gait_id, timeout=timeout)

#     def shutdown(self):
#         self._running = False
#         self._stop_keepalive.set()
#         self.lie_down()
