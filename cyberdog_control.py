#!/usr/bin/env python3
"""
CyberDog 实时键盘控制脚本
- 按住方向键持续移动，松开停止
- 支持转向
用法：python3.8 cyberdog_control.py
"""

import lcm
import sys
import time
import threading
import tty
import termios

sys.path.append('/home/cyberdog_sim/src/cyberdog_locomotion/common/lcm_type/lcm')
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
import robot_control_cmd_lcmt

LCM_URL = "udpm://239.255.76.67:7671?ttl=255"

MODE_PASSIVE    = 0
MODE_STAND      = 6
MODE_LOCOMOTION = 11
MODE_RECOVERY   = 12
GAIT_TROT       = 9

lc = lcm.LCM(LCM_URL)
msg = robot_control_cmd_lcmt.robot_control_cmd_lcmt()
life_count = 0
lock = threading.Lock()

def send_cmd(mode, gait_id=0, vx=0.0, vy=0.0, vyaw=0.0, duration=500):
    global life_count
    with lock:
        life_count = (life_count + 1) % 127  # int8_t 范围限制
        msg.mode        = mode
        msg.gait_id     = gait_id
        msg.life_count  = life_count
        msg.duration    = duration
        msg.contact     = 0
        msg.value       = 0
        msg.vel_des     = [vx, vy, vyaw]
        msg.rpy_des     = [0.0, 0.0, 0.0]
        msg.pos_des     = [0.0, 0.0, 0.0]
        msg.acc_des     = [0.0] * 6
        msg.ctrl_point  = [0.0] * 3
        msg.foot_pose   = [0.0] * 6
        msg.step_height = [0.08, 0.08]
        lc.publish("robot_control_cmd", msg.encode())

def keep_alive(stop_event):
    while not stop_event.is_set():
        with lock:
            lc.publish("robot_control_cmd", msg.encode())
        time.sleep(0.1)

def get_key():
    """读取单个按键，不需要回车"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # 处理方向键（ESC序列）
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return ch + ch2 + ch3
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def print_help():
    print("""
╔════════════════════════════════════════╗
║       CyberDog 实时键盘控制            ║
╠════════════════════════════════════════╣
║  状态切换：                            ║
║    r  - 恢复站立 (recovery)           ║
║    t  - 站立 (stand)                  ║
║    y  - 行走模式                       ║
║    0  - 趴下 (passive)                ║
║                                        ║
║  移动控制（实时，无需回车）：           ║
║    w / ↑  - 前进                      ║
║    s / ↓  - 后退                      ║
║    a / ←  - 左转                      ║
║    d / →  - 右转                      ║
║    z      - 左平移                     ║
║    c      - 右平移                     ║
║    空格   - 停止                       ║
║                                        ║
║  推荐顺序：r → t → y → 移动           ║
║    h  - 显示帮助    q  - 退出          ║
╚════════════════════════════════════════╝
""")

def main():
    print_help()

    stop_event = threading.Event()
    t = threading.Thread(target=keep_alive, args=(stop_event,), daemon=True)
    t.start()

    VX   = 0.4
    VY   = 0.3
    VYAW = 0.6

    current_mode = MODE_PASSIVE
    send_cmd(MODE_PASSIVE)
    print("当前：passive（趴下）  →  请按 r 恢复站立\n")

    try:
        while True:
            key = get_key()

            # 退出
            if key in ('q', '\x03'):  # q 或 Ctrl+C
                break

            # 状态切换
            elif key == 'r':
                send_cmd(MODE_RECOVERY, duration=2000)
                current_mode = MODE_RECOVERY
                print("\r>> 恢复站立中...等待2秒          ")
                time.sleep(2.0)
                print("\r>> 完成，按 t 站立               ")

            elif key == 't':
                send_cmd(MODE_STAND, duration=1000)
                current_mode = MODE_STAND
                print("\r>> 站立                          ")

            elif key == 'y':
                send_cmd(MODE_LOCOMOTION, gait_id=GAIT_TROT, duration=500)
                current_mode = MODE_LOCOMOTION
                print("\r>> 行走模式（小跑）               ")

            elif key == '0':
                send_cmd(MODE_PASSIVE)
                current_mode = MODE_PASSIVE
                print("\r>> 趴下                          ")

            # 移动控制
            elif key in ('w', '\x1b[A'):  # w 或 ↑
                send_cmd(MODE_LOCOMOTION, GAIT_TROT, vx=VX)
                print(f"\r>> 前进 {VX} m/s                 ", end='', flush=True)

            elif key in ('s', '\x1b[B'):  # s 或 ↓
                send_cmd(MODE_LOCOMOTION, GAIT_TROT, vx=-VX)
                print(f"\r>> 后退 {VX} m/s                 ", end='', flush=True)

            elif key in ('a', '\x1b[D'):  # a 或 ←  → 左转
                send_cmd(MODE_LOCOMOTION, GAIT_TROT, vyaw=VYAW)
                print(f"\r>> 左转 {VYAW} rad/s             ", end='', flush=True)

            elif key in ('d', '\x1b[C'):  # d 或 →  → 右转
                send_cmd(MODE_LOCOMOTION, GAIT_TROT, vyaw=-VYAW)
                print(f"\r>> 右转 {VYAW} rad/s             ", end='', flush=True)

            elif key == 'z':  # 左平移
                send_cmd(MODE_LOCOMOTION, GAIT_TROT, vy=VY)
                print(f"\r>> 左平移 {VY} m/s               ", end='', flush=True)

            elif key == 'c':  # 右平移
                send_cmd(MODE_LOCOMOTION, GAIT_TROT, vy=-VY)
                print(f"\r>> 右平移 {VY} m/s               ", end='', flush=True)

            elif key == ' ':  # 停止
                send_cmd(MODE_LOCOMOTION, GAIT_TROT, vx=0.0, vy=0.0, vyaw=0.0)
                print("\r>> 停止                          ", end='', flush=True)

            elif key == 'h':
                print_help()

    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        print("\n退出，发送趴下指令...")
        send_cmd(MODE_PASSIVE)
        time.sleep(0.5)
        stop_event.set()

if __name__ == '__main__':
    main()
