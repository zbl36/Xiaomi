#!/usr/bin/env python3
"""
独立测试 stage_bridge 的跳下阶段（S_JUMP）
用法：
  source /opt/ros/galactic/setup.bash
  source /home/cyberdog_sim/install/setup.bash
  export PYTHONPATH=/usr/local/lib/python3.8/site-packages:$PYTHONPATH
  python3.8 test_bridge_jump_stage.py
"""

import time
import sys
import os

# 与 test_bridge_stage.py 保持一致的路径处理
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')

from utils.cyberdog_lcm import CyberdogController
from utils.pose_tracker import start_pose_tracker
from stages.stage_bridge import StageBridge


class TestStage:
    INIT = 0
    STAGE5_JUMP = 1
    FINISH = 2


STAGE_NAMES = {
    TestStage.INIT: '初始化',
    TestStage.STAGE5_JUMP: '赛段五·跳下阶段(单测)',
    TestStage.FINISH: '测试结束',
}


class BridgeJumpStageTestController:
    def __init__(self):
        self.ctrl = CyberdogController()
        self.tracker = start_pose_tracker()
        self.stage = TestStage.INIT
        self.stage5 = StageBridge(self.ctrl)

    def _transition(self, new_stage):
        print(f"\n{'=' * 40}")
        print(f"[状态切换] {STAGE_NAMES[self.stage]} → {STAGE_NAMES[new_stage]}")
        print(f"{'=' * 40}\n")
        self.stage = new_stage

    def _get_pos(self):
        return self.tracker.get_pos()

    def _enter_jump_sub_state(self):
        # 强制进入 StageBridge 的跳下子状态，复用原始跳下逻辑参数
        self.stage5._sub_state = self.stage5.S_JUMP
        self.stage5._timer = time.monotonic()
        print('[跳下单测] 已切换到 S_JUMP，开始执行跳下动作')

    def run(self):
        print('=' * 40)
        print('  CyberDog 赛段五跳下阶段单测启动')
        print('=' * 40)

        # 与完整测试保持一致：先站立再开始动作
        self.ctrl.stand_up()
        time.sleep(0.3)

        self._transition(TestStage.STAGE5_JUMP)
        self._enter_jump_sub_state()

        try:
            while self.stage != TestStage.FINISH:
                x, y, _ = self._get_pos()

                if self.stage == TestStage.STAGE5_JUMP:
                    done = self.stage5.step(x, y)
                    if done:
                        self.stage5.cleanup()
                        self._transition(TestStage.FINISH)

                time.sleep(0.05)  # 20Hz 主循环

        except KeyboardInterrupt:
            print('\n[中断] 手动停止测试')
        finally:
            self.ctrl.shutdown()
            print('[结束] 跳下阶段单测退出')


if __name__ == '__main__':
    controller = BridgeJumpStageTestController()
    controller.run()
