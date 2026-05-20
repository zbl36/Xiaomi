"""
赛段五：孤梁稳渡
独木桥行走：IMU roll角修正居中，检测到末端后跳下
"""

import time
from utils.cyberdog_lcm import MODE_LOCOMOTION, GAIT_TROT

VX_BRIDGE    = 0.15   # 独木桥行走速度（慢）
Kp_roll      = 0.3    # roll角修正增益
JUMP_VX      = 0.8    # 跳下速度
JUMP_DURATION = 0.4   # 跳下持续时间


class StageBridge:
    S_WALK  = 'WALK'
    S_JUMP  = 'JUMP'
    S_DONE  = 'DONE'

    def __init__(self, ctrl):
        self.ctrl = ctrl
        self._sub_state = self.S_WALK
        self._timer = 0.0
        self._walk_start_y = None

        # IMU数据（通过ROS2订阅）
        self._roll = 0.0
        self._imu_sub = None
        self._start_imu()

    def _start_imu(self):
        """订阅IMU数据获取roll角"""
        try:
            import rclpy
            from sensor_msgs.msg import Imu
            import threading

            if not rclpy.ok():
                rclpy.init()

            from rclpy.node import Node
            import math

            class ImuNode(Node):
                def __init__(self, bridge):
                    super().__init__('bridge_imu')
                    self.bridge = bridge
                    self.create_subscription(Imu, '/imu', self._cb, 10)

                def _cb(self, msg):
                    # 从四元数提取roll角
                    q = msg.orientation
                    sinr = 2.0 * (q.w * q.x + q.y * q.z)
                    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
                    self.bridge._roll = math.atan2(sinr, cosr)

            self._imu_node = ImuNode(self)
            t = threading.Thread(
                target=lambda: rclpy.spin(self._imu_node), daemon=True)
            t.start()
        except Exception as e:
            print(f"[赛段五] IMU订阅失败: {e}，使用默认值")

    def step(self, current_y):
        if self._sub_state == self.S_WALK:
            if self._walk_start_y is None:
                self._walk_start_y = current_y
                print("[赛段五] 开始走独木桥")

            # 用roll角修正横向偏移
            vy_correction = -Kp_roll * self._roll
            vy_correction = max(-0.15, min(0.15, vy_correction))

            self.ctrl.move(
                vx=VX_BRIDGE,
                vy=vy_correction,
                step_height=0.06   # 独木桥上小步幅
            )

            # 走了约1.5m后准备跳下（独木桥长约1.5m）
            walked = current_y - self._walk_start_y
            if walked >= 1.3:
                print("[赛段五] 接近末端，准备跳下")
                self._sub_state = self.S_JUMP
                self._timer = time.time()

        elif self._sub_state == self.S_JUMP:
            # 加速冲出独木桥
            self.ctrl.move(vx=JUMP_VX)
            if time.time() - self._timer > JUMP_DURATION:
                self.ctrl.stop()
                print("[赛段五] 跳下独木桥")
                self._sub_state = self.S_DONE
                time.sleep(1.0)  # 等待落地稳定

        elif self._sub_state == self.S_DONE:
            return True

        return False

    def cleanup(self):
        self.ctrl.stop()
        print("[赛段五] 结束")
