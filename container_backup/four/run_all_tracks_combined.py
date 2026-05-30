#!/usr/bin/env python3
"""
同一程序内顺序执行三条赛道。

每条赛道自己的 finally 会在结束前让机器狗站立 10 秒，
然后才返回这里继续执行下一条赛道。
"""

import ball_lowposture_half
import coke_ganzi_lowposture
import obstacle_center_align


TRACKS = [
    ("可乐瓶赛道", coke_ganzi_lowposture.main),
    ("障碍物赛道", obstacle_center_align.main),
    ("白球赛道", ball_lowposture_half.main),
]


def main():
    for name, entry in TRACKS:
        print(f"\n[总控] 开始执行{name}")
        entry()
        print(f"[总控] {name}结束，机器狗已在该程序末尾站立 10 秒")
    print("[总控] 三条赛道全部执行完成")


if __name__ == "__main__":
    main()
