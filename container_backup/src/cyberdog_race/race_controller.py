"""
比赛主状态机
基于 /tf 世界坐标自动切换赛段状态

赛道坐标系说明（来自 race.world）：
  race模型全局pose: x=1.5, y=5.87, z=1.18, yaw=-1.57rad
  机器人沿世界坐标 +Y 方向前进
  起点 y≈0，终点 y≈16

赛段Y坐标边界（世界坐标）：
  赛段一：y = 0  ~ 1.0   石径探路（石板+弯道）
  赛段二：y = 1.0 ~ 4.5  荒野寻珠（球阵 y=1.34~3.86）
  赛段三：y = 4.5 ~ 8.0  曲道冲锋（S形弯道）
  赛段四：y = 8.0 ~ 13.0 深隧寻珍（coke y=11.1, football2 y=10.8）
  赛段五：y = 13.0~ 14.5 孤梁稳渡（独木桥）
  赛段六：y = 14.5~ 16.0 撷金建功（football3 y=14.7）
"""

import time
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
from utils.cyberdog_lcm import CyberdogController
from utils.pose_tracker import start_pose_tracker
from stages.stage_path import StagePath
from stages.stage_ball import StageBall
from stages.stage_hunt import StageHunt
from stages.stage_bridge import StageBridge
from stages.stage_kick import StageKick

# 赛段切换Y坐标阈值
Y_STAGE1_END   = 1.0
Y_STAGE2_END   = 4.5
Y_STAGE3_END   = 8.0
Y_STAGE4_END   = 13.0
Y_STAGE5_END   = 14.5
Y_FINISH       = 15.8


class RaceStage:
    INIT     = 0
    STAGE1   = 1
    STAGE2   = 2
    STAGE3   = 3
    STAGE4   = 4
    STAGE5   = 5
    STAGE6   = 6
    FINISH   = 7

STAGE_NAMES = {
    0: "初始化",
    1: "赛段一·石径探路",
    2: "赛段二·荒野寻珠",
    3: "赛段三·曲道冲锋",
    4: "赛段四·深隧寻珍",
    5: "赛段五·孤梁稳渡",
    6: "赛段六·撷金建功",
    7: "比赛结束",
}


class RaceController:
    def __init__(self):
        self.ctrl = CyberdogController()
        self.tracker = start_pose_tracker()
        self.stage = RaceStage.INIT

        # 初始化各赛段处理器
        self.stage1 = StagePath(self.ctrl, mode='rockroad')
        self.stage2 = StageBall(self.ctrl)
        self.stage3 = StagePath(self.ctrl, mode='scurve')
        self.stage4 = StageHunt(self.ctrl)
        self.stage5 = StageBridge(self.ctrl)
        self.stage6 = StageKick(self.ctrl)

    def _transition(self, new_stage):
        print(f"\n{'='*40}")
        print(f"[状态切换] {STAGE_NAMES[self.stage]} → {STAGE_NAMES[new_stage]}")
        print(f"{'='*40}\n")
        self.stage = new_stage

    def _get_pos(self):
        return self.tracker.get_pos()

    def run(self):
        print("=" * 40)
        print("  CyberDog 比赛程序启动")
        print("=" * 40)

        # 站立
        self.ctrl.stand_up()
        self._transition(RaceStage.STAGE1)

        try:
            while self.stage != RaceStage.FINISH:
                x, y, _ = self._get_pos()

                if self.stage == RaceStage.STAGE1:
                    done = self.stage1.step(y, self.tracker)
                    if done or y >= Y_STAGE1_END:
                        self.stage1.cleanup()
                        self._transition(RaceStage.STAGE2)

                elif self.stage == RaceStage.STAGE2:
                    done = self.stage2.step(y)
                    if done or y >= Y_STAGE2_END:
                        self.stage2.cleanup()
                        self._transition(RaceStage.STAGE3)

                elif self.stage == RaceStage.STAGE3:
                    done = self.stage3.step(y)
                    if done or y >= Y_STAGE3_END:
                        self.stage3.cleanup()
                        self._transition(RaceStage.STAGE4)

                elif self.stage == RaceStage.STAGE4:
                    done = self.stage4.step(y)
                    if done or y >= Y_STAGE4_END:
                        self.stage4.cleanup()
                        self._transition(RaceStage.STAGE5)

                elif self.stage == RaceStage.STAGE5:
                    done = self.stage5.step(x, y)
                    if done or y >= Y_STAGE5_END:
                        self.stage5.cleanup()
                        self._transition(RaceStage.STAGE6)

                elif self.stage == RaceStage.STAGE6:
                    done = self.stage6.step(y)
                    if done or y >= Y_FINISH:
                        self.stage6.cleanup()
                        self._transition(RaceStage.FINISH)

                time.sleep(0.05)  # 20Hz 主循环

        except KeyboardInterrupt:
            print("\n[中断] 手动停止比赛")
        finally:
            self.ctrl.shutdown()
            print("[结束] 程序退出")


if __name__ == '__main__':
    controller = RaceController()
    controller.run()
