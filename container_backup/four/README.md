# four - CyberDog 三赛道整合程序

这个文件夹是最终可复制版本。把整个 `four/` 文件夹复制到另一台电脑后，保持当前目录结构即可运行。

## 目录结构

```text
four/
├── run_all_tracks_combined.py
├── coke_ganzi_lowposture.py
├── obstacle_center_align.py
├── ball_lowposture_half.py
├── requirements.txt
├── vision_module/
│   ├── vision_module.py
│   ├── coke_best.pt
│   ├── ganzi_best.pt
│   ├── cube_best.pt
│   ├── blue_ball_best.pt
│   ├── football_best.pt
│   └── orange_ball_best.pt
└── loco_hl_example/
    └── customized_gait/
        ├── main.py
        ├── robot_control_cmd_lcmt.py
        ├── file_send_lcmt.py
        ├── Gait_Def_lowposture_forward.toml
        ├── Gait_Params_lowposture_forward.toml
        └── Usergait_List_lowposture_forward.toml
```

## 一键运行

进入 `four/` 目录：

```bash
cd four
python3 run_all_tracks_combined.py
```

执行顺序：

```text
1. 可乐瓶赛道
2. 站立 10 秒
3. 障碍物赛道
4. 站立 10 秒
5. 白球赛道
6. 站立 10 秒
7. 全部结束
```

## 单独运行

```bash
python3 coke_ganzi_lowposture.py
python3 obstacle_center_align.py
python3 ball_lowposture_half.py
```

## 三个程序说明

### coke_ganzi_lowposture.py

可乐瓶 + 限高杆赛道：

```text
追踪可乐瓶
-> 识别限高杆
-> 低姿态通过
-> 可乐丢失恢复
-> 第二次限高杆低姿态
-> 收尾动作
```

### obstacle_center_align.py

障碍物 + 蓝球赛道：

```text
识别蓝色障碍物
-> 居中前进或左右平移
-> 连续三次识别不到障碍物后切动作序列
-> 追踪蓝色球
-> 蓝球丢失后转身
-> 重新靠近障碍物
-> 最终收尾
```

### ball_lowposture_half.py

白球赛道：

```text
追踪白球
-> 白球足够近后禁止前进，只能原地调整
-> 第一次低姿态
-> 手动左转 12 次 + 半速左转 1 次
-> 第二次低姿态
-> 最终收尾动作
```

## 依赖安装

```bash
pip3 install -r requirements.txt
```

如果没有网络，需要提前离线安装：

```text
ultralytics
opencv-python
numpy
toml
lcm
```

`ultralytics` 会依赖 PyTorch，目标电脑需要根据自己的 CUDA/CPU 环境安装合适版本。

## 运行前环境

运行前请确认：

```text
1. Gazebo 仿真已启动
2. CyberDog locomotion 控制程序已启动
3. 相机 topic 存在：/rgb_camera/image_raw
4. LCM 控制链路可用：udpm://239.255.76.67:7671?ttl=255
5. 当前目录就是 four/
```

## 低姿态步态

低姿态步态名：

```text
lowposture_forward
```

程序会自动读取：

```text
loco_hl_example/customized_gait/Gait_Def_lowposture_forward.toml
loco_hl_example/customized_gait/Gait_Params_lowposture_forward.toml
```

运行时会自动生成：

```text
Gait_Params_lowposture_forward_full.toml
```

这个是运行产物，不需要提前准备。

## 常用调参

可乐瓶赛道：

```python
COKE_CLOSE_AREA
COKE_LOST_LIMIT
GANZI_CLOSE_AREA
SECOND_GANZI_DEAD_ZONE
POST_LOW_FORWARD_STEPS
POST_LOW_RIGHT_TURN_STEPS
POST_LOW_RIGHT_SHIFT_STEPS
```

障碍物赛道：

```python
OBSTACLE_DEAD_ZONE
LOST_STOP_LIMIT
MANUAL_VX
MANUAL_VY
MANUAL_VYAW
BLUE_FORWARD_VX
BLUE_LOST_TURN_DURATION
RECHECK_OBSTACLE_LOST_LIMIT
```

白球赛道：

```python
BALL_CLOSE_AREA
BALL_FINAL_DEAD_ZONE
FOLLOW_CYCLE_TIME
MANUAL_LEFT_TURN_STEPS
MANUAL_FINE_TURN_SCALE
FINAL_FORWARD_STEPS
FINAL_LEFT_SHIFT_STEPS
FINAL_RIGHT_TURN_STEPS
```

## 结束姿态

三个程序结束前都会执行：

```python
ctrl.stop(0.3)
ctrl.stand(10.0)
ctrl.shutdown()
```

所以每条赛道结束后会让机器狗保持站立 10 秒，再进入下一条赛道。
