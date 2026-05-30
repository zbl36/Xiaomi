#!/usr/bin/env python3
"""
独立测试 stage_bridge 赛段（控制逻辑尽量对齐 race_controller）
用法：
  source /opt/ros/galactic/setup.bash
  source /home/cyberdog_sim/install/setup.bash
  export PYTHONPATH=/usr/local/lib/python3.8/site-packages:$PYTHONPATH
  python3.8 test_bridge_stage.py [--sim] [--duration 120] [--y-end 14.5]
"""

import time
import sys
import os

# 确保路径正确
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')

sys.path.insert(0, os.path.dirname(__file__))
from utils.cyberdog_lcm import CyberdogController
from utils.pose_tracker import start_pose_tracker
from stages.stage_bridge import StageBridge


class TestStage:
    INIT = 0
    STAGE5 = 5
    FINISH = 7


STAGE_NAMES = {
    0: '初始化',
    5: '赛段五·孤梁稳渡(测试)',
    7: '测试结束',
}


class BridgeStageTestController:
    def __init__(self):
        self.ctrl = CyberdogController()
        self.tracker = start_pose_tracker()
        self.stage = TestStage.INIT

        self.stage = TestStage.INIT
        self.stage5 = StageBridge(self.ctrl)

    def _transition(self, new_stage):
        print(f"\n{'='*40}")
        print(f"[状态切换] {STAGE_NAMES[self.stage]} → {STAGE_NAMES[new_stage]}")
        print(f"{'='*40}\n")
        self.stage = new_stage

    def _get_pos(self):
        return self.tracker.get_pos()


    def run(self):
        print('=' * 40)
        print('  CyberDog 赛段五测试程序启动')
        print('=' * 40)

        # 与 race_controller 一致：先站立，再切入首个 stage
        self.ctrl.stand_up()
        self._transition(TestStage.STAGE5)

        try:
            while self.stage != TestStage.FINISH:
                x, y, _ = self._get_pos()
                
                if self.stage == TestStage.STAGE5:
                    done = self.stage5.step(x, y)
                    if done or y >= 100:
                        self.stage5.cleanup()
                        self._transition(TestStage.FINISH)

                time.sleep(0.05)  # 20Hz 主循环

        except KeyboardInterrupt:
            print('\n[中断] 手动停止测试')
        finally:
            # 与 race_controller 一致：统一 shutdown
            self.ctrl.shutdown()
            print('[结束] 测试程序退出')


if __name__ == '__main__':
    controller = BridgeStageTestController()
    controller.run()
