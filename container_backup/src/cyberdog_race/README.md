# CyberDog 比赛程序

2026年全国大学生计算机系统能力大赛·小米杯

## 目录结构

```
cyberdog_race/
├── main.py                  # 启动入口
├── race_controller.py       # 主状态机（基于/tf坐标切换赛段）
├── utils/
│   ├── cyberdog_lcm.py      # LCM运动控制封装
│   └── pose_tracker.py      # ROS2 /tf 位置订阅
└── stages/
    ├── stage_path.py        # 赛段一/三：黄线跟踪
    ├── stage_ball.py        # 赛段二：橙色球检测撞击
    ├── stage_hunt.py        # 赛段四：深隧寻珍
    ├── stage_bridge.py      # 赛段五：独木桥
    └── stage_kick.py        # 赛段六：踢球+终点
```

## 运行方法

### 1. 复制到容器

```bash
sudo docker cp ~/Xiaomi/cyberdog_race clever_chatelet:/home/cyberdog_sim/cyberdog_race
```

### 2. 启动仿真

```bash
docker exec -it clever_chatelet bash
cd /home/cyberdog_sim
python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py
```

### 3. 新开终端运行比赛程序

```bash
docker exec -it clever_chatelet bash
source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash
export PYTHONPATH=/usr/local/lib/python3.8/site-packages:$PYTHONPATH
cd /home/cyberdog_sim/cyberdog_race
python3.8 main.py
```

## 状态切换逻辑

基于 `/tf` 中机器人世界坐标Y值自动切换赛段：

| 赛段 | Y坐标范围 | 说明 |
|------|-----------|------|
| 赛段一 | 0 ~ 1.0 | 石径探路 |
| 赛段二 | 1.0 ~ 4.5 | 荒野寻珠（球阵） |
| 赛段三 | 4.5 ~ 8.0 | 曲道冲锋 |
| 赛段四 | 8.0 ~ 13.0 | 深隧寻珍 |
| 赛段五 | 13.0 ~ 14.5 | 孤梁稳渡 |
| 赛段六 | 14.5 ~ 16.0 | 撷金建功 |

## 调参说明

### 黄线跟踪（stage_path.py）
- `Kp / Ki / Kd`：PID参数，Kp越大转向越灵敏
- `VX_ROCKROAD`：石板路速度，建议0.2~0.3
- `STEP_ROCKROAD`：石板路抬腿高度，建议0.10~0.15

### 橙色球检测（stage_ball.py）
- `ORANGE_LOW/HIGH`：HSV颜色范围，在仿真中用取色工具校准
- `HIT_AREA_THRESHOLD`：撞击触发面积阈值，越大越近才撞

### 深隧寻珍（stage_hunt.py）
- 语音播报依赖 `espeak`，容器内安装：`apt-get install -y espeak`
- 各检测函数的 `min_area` 参数根据实际摄像头距离调整

### 独木桥（stage_bridge.py）
- `Kp_roll`：roll角修正增益，越大横向修正越强
- `VX_BRIDGE`：独木桥速度，建议0.1~0.2

## 注意事项

1. 坐标阈值基于仿真 `race.world` 文件，实机比赛需重新标定
2. 摄像头索引默认为0，如有多个摄像头需修改 `cv2.VideoCapture(0)`
3. 比赛前在仿真中完整跑一遍，确认各赛段Y坐标阈值正确
4. `espeak` 语音播报需提前安装：`apt-get install -y espeak`
