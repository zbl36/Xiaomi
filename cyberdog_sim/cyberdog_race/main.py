#!/usr/bin/env python3
"""
比赛启动入口
用法：
  容器内运行：
    source /opt/ros/galactic/setup.bash
    source /home/cyberdog_sim/install/setup.bash
    export PYTHONPATH=/usr/local/lib/python3.8/site-packages:$PYTHONPATH
    python3.8 main.py
"""

import sys
import os

# 确保路径正确
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, '/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')

from race_controller import RaceController

if __name__ == '__main__':
    controller = RaceController()
    controller.run()
