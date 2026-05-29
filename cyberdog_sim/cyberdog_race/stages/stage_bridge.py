"""
赛段五：孤梁稳渡
正方形赛道绕行：上坡 → 左转 → 绕行四条边（roll修正） → 跳下
"""

import time
from utils.cyberdog_lcm import MODE_LOCOMOTION, GAIT_TROT, MODE_STAND

# ===== 阶段参数 =====
# 上坡阶段
VX_CLIMB        = 0.18
STEP_CLIMB      = 0.06
CLIMB_DISTANCE  = 4.53

# 转向阶段
VX_TURN_IN       = 0.00
STEP_TURN        = 0.03
VYAW_TURN_RIGHT  = 0.40  # 右转角速度
TURN_IN_TIME     = 7.04   # 转向持续时间
STOP_BEFORE_TURN = 1.20   # 转向前短暂停顿
STOP_AFTER_TURN  = 0.50   # 转向后短暂停顿

# 绕行阶段：每条边单独设置长度
VX_LOOP         = 0.05
VY_LOOP         = 0.15
VY_TURN         = 0.05
STEP_LOOP       = 0.02
Kp_roll         = 0.20  # roll角修正增益（正向：左高右低时向左修正）
EDGE_1_DISTANCE = 3.24
EDGE_2_DISTANCE = 2.50
EDGE_3_DISTANCE = 3.20
EDGE_4_DISTANCE = 1.28
EDGE_LENGTHS    = [EDGE_1_DISTANCE, EDGE_2_DISTANCE, EDGE_3_DISTANCE, EDGE_4_DISTANCE]
VY_LIMIT        = 0.04  # 横向修正限幅

# 跳下阶段
JUMP_DURATION   = 1000


class StageBridge:
    # 子状态定义
    S_CLIMB     = 'CLIMB'
    S_TURN_IN   = 'TURN_IN'
    S_TURN_RIGHT = 'TURN_RIGHT'
    S_LOOP      = 'LOOP'
    S_JUMP      = 'JUMP'
    S_DONE      = 'DONE'

    def __init__(self, ctrl):
        self.ctrl = ctrl
        self._sub_state = self.S_CLIMB
        self._timer = 0.0
        self._start_x = None
        self._start_y = None
        self._climb_end_y = None
        self._edge_index = 0  # 当前正在行驶的边序号
        self._last_edge_x = None
        self._last_edge_y = None
        self._turn_start = None  # 左/右转开始时间

        # IMU数据（通过ROS2订阅）
        self._roll = 0.0
        self._yaw = 0.0
        self._imu_sub = None
        self._imu_started = False

    def _start_imu(self):
        """订阅IMU数据获取roll角和yaw角"""
        try:
            import rclpy
            from sensor_msgs.msg import Imu
            import threading

            if not rclpy.ok():
                rclpy.init()

            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            import math

            class ImuNode(Node):
                def __init__(self, bridge):
                    super().__init__('bridge_imu_stage5')
                    self.bridge = bridge
                    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
                    self.create_subscription(Imu, '/imu', self._cb, qos)

                def _cb(self, msg):
                    q = msg.orientation
                    sinr = 2.0 * (q.w * q.x + q.y * q.z)
                    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
                    self.bridge._roll = math.atan2(sinr, cosr)

                    siny = 2.0 * (q.w * q.z + q.x * q.y)
                    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    self.bridge._yaw = math.atan2(siny, cosy)

            self._imu_node = ImuNode(self)

            def _spin_node(node):
                from rclpy.executors import SingleThreadedExecutor
                executor = SingleThreadedExecutor()
                executor.add_node(node)
                try:
                    executor.spin()
                finally:
                    executor.shutdown()

            t = threading.Thread(target=_spin_node, args=(self._imu_node,), daemon=True)
            t.start()
        except Exception as e:
            print(f"[赛段五] IMU订阅失败: {e}，使用默认值")

    def step(self, current_x, current_y):
        # 初始化起始绝对坐标与上坡目标点
        if self._start_y is None:
            self._start_x = current_x
            self._start_y = current_y
            self._climb_end_y = current_y + CLIMB_DISTANCE
            self._timer = time.monotonic()
            print("[赛段五] 开始上坡")

        # ===== S_CLIMB：上坡阶段 =====
        if self._sub_state == self.S_CLIMB:
            if current_y < self._climb_end_y:
                print(current_y)
                self.ctrl.move_lowhead(
                    vx=-VX_CLIMB,
                    vy=0.0,
                    step_height=STEP_CLIMB
                )
            else:
                print("[赛段五] 上坡完成，进入正方形赛道")
                # print("[赛段五] 停顿")
                # self.ctrl.stop()
                # time.sleep(STOP_BEFORE_TURN)
                self._sub_state = self.S_LOOP
                self._edge_index = 0
                self._last_edge_x = current_x
                self._last_edge_y = current_y
                self._turn_start = time.monotonic()

        # # ===== S_TURN_IN：左转进入阶段 =====
        # elif self._sub_state == self.S_TURN_IN:
        #     if self._turn_start is None:
        #         self._turn_start = time.monotonic()

        #     if not self._imu_started:
        #         self._start_imu()
        #         self._imu_started = True

        #     vy_correction = Kp_roll * self._roll
        #     if vy_correction < 0:
        #         vy_correction = 0
        #     vy_correction = max(-VY_LIMIT, min(VY_LIMIT, vy_correction))

        #     print(vy_correction)

        #     elapsed = time.monotonic() - self._turn_start
        #     if elapsed < TURN_IN_TIME:
        #         self.ctrl.slow_move(
        #             vx=VX_TURN_IN,
        #             vyaw=VYAW_TURN_IN,
        #             vy=vy_correction,
        #             step_height=STEP_CLIMB
        #         )
        #     else:
        #         print("[赛段五] 左转后停顿")
        #         self.ctrl.stop(
        #             body_roll=vy_correction,
        #         )
        #         time.sleep(STOP_AFTER_TURN)
        #         print("[赛段五] 左转完成，开始绕行赛道")
        #         self._sub_state = self.S_LOOP
        #         self._edge_index = 0
        #         self._last_edge_x = current_x
        #         self._last_edge_y = current_y
        #         self._turn_start = None

        # ===== S_TURN_RIGHT：右转进入下一边 =====
        elif self._sub_state == self.S_TURN_RIGHT:
            if self._turn_start is None:
                self._turn_start = time.monotonic()

            if not self._imu_started:
                self._start_imu()
                self._imu_started = True

            vy_correction = Kp_roll * self._roll
            if vy_correction < 0:
                vy_correction = 0
            vy_correction = max(-VY_LIMIT, min(VY_LIMIT, vy_correction))

            # 根据当前是哪一条边的转向，选择原地转向或带前进量的转向
            # 当从第3条边(索引2)转向第4条边时，要求原地转向且多转一倍角度
            # 当第4条边完成后（索引3），也要求原地转向一次
            elapsed = time.monotonic() - self._turn_start
            # 默认转向时长
            turn_time = TURN_IN_TIME
            # 如果是从第3条边转向第4条边，直接跳过转向倒走
            if self._edge_index == 2:
                turn_time = 0.0
            # 如果是第4条边完成后的右转，也使用原地转向
            elif self._edge_index == 3:
                use_vy = 0.00
                use_vyaw = -VYAW_TURN_RIGHT
            else:
                use_vy = VY_TURN
                use_vyaw = VYAW_TURN_RIGHT

            if elapsed < turn_time:
                self.ctrl.move(
                    vx=0.01,
                    vy=-use_vy,
                    vyaw=-use_vyaw,
                    step_height=STEP_CLIMB
                )
            else:
                print("[赛段五] 右转后停顿")
                # self.ctrl.stop()
                # time.sleep(STOP_AFTER_TURN)
                self._edge_index += 1
                self._turn_start = None
                self._last_edge_x = current_x
                self._last_edge_y = current_y

                if self._edge_index >= len(EDGE_LENGTHS):
                    print("[赛段五] 右转完成，准备跳下台阶")
                    self._sub_state = self.S_JUMP
                    self._timer = time.monotonic()
                else:
                    print(f"[赛段五] 右转完成，开始第 {self._edge_index + 1} 条 边")
                    self._sub_state = self.S_LOOP

        # ===== S_LOOP：绕行每一条边 =====
        elif self._sub_state == self.S_LOOP:
            if not self._imu_started:
                self._start_imu()
                self._imu_started = True

            vy_correction = Kp_roll * self._roll
            vy_correction = max(-VY_LIMIT, min(VY_LIMIT, vy_correction))

            # 第4条边（索引3）改为向前行走，不再侧向移动；其他边保持原有横向修正移动
            if self._edge_index == 3:
                # 向前行走
                self.ctrl.move_lowhead(
                    vx=-0.20,
                    vy=0.0,
                    pitch=0.18
                )
            else:
                self.ctrl.move_lowhead(
                    vx=0.03,
                    vy=-VY_LOOP,
                    pitch=0.18
                )

            # 第1、3条边沿地图x方向，第2、4条边沿地图y方向
            if self._edge_index % 2 == 0:
                edge_walked = abs(current_x - self._last_edge_x)
            else:
                edge_walked = abs(current_y - self._last_edge_y)

            target_distance = EDGE_LENGTHS[self._edge_index]

            if edge_walked >= target_distance:
                # self.ctrl.stop()
                # time.sleep(STOP_BEFORE_TURN)
                print(f"[赛段五] 完成第 {self._edge_index + 1} 条边，准备右转")
                self._sub_state = self.S_TURN_RIGHT
                self._turn_start = time.monotonic()

        # ===== S_JUMP：跳下阶段 =====
        elif self._sub_state == self.S_JUMP:
            self.ctrl.jump(duration=JUMP_DURATION)
            self.ctrl.stop()
            print("[赛段五] 跳下完成")
            self._sub_state = self.S_DONE
            time.sleep(1.0)

        # ===== S_DONE：完成阶段 =====
        elif self._sub_state == self.S_DONE:
            return True

        return False

    def cleanup(self):
        self.ctrl.stop()
        print("[赛段五] 结束")
