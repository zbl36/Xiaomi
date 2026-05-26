# 更新日志

---

## 2026-05-26

### 新增
- `test_lane_follow.py`：黄线跟踪独立测试脚本
  - 容器路径：`/home/cyberdog_sim/test_lane_follow.py`
  - 自动完成站立流程，按 `s` 开始跟踪，按 `x` 停止，按 `q` 退出
  - 实时显示黄线检测结果（赛道中心线、误差值、mask）
  - PID 参数可在脚本顶部调整：`Kp/Ki/Kd`

- `record_video.py`：相机录制工具
  - 容器路径：`/home/cyberdog_sim/record_video.py`
  - 支持录制 mp4 视频并拆帧为 jpg 图片，用于 YOLOv8 训练数据采集

- `cyberdog2-ctrl-user-parameters.yaml`：运控参数配置
  - 容器路径：`/home/cyberdog_sim/src/cyberdog_locomotion/common/config/cyberdog2-ctrl-user-parameters.yaml`
  - 修改：`rpy_min` pitch 限制从 `-0.25` 扩大到 `-0.52`，`step_height_max` 从 `0.06` 改为 `0.12`

- `cyberdog_race/gaits/lane_follow_gait.toml`：黄线跟踪步态参数文件

- `cyberdog_locomotion/`：运控源码（含修改）
  - `control/src/convex_mpc/convex_mpc_loco_gaits.cpp`：
    - `SetDefaultParams()` 中 `rpy_cmd_max_` 从 `0.1` 扩大到 `0.35`（扩大前倾范围）
    - 注释掉速度对 pitch 的压缩逻辑（4处），行走时 pitch 不再被速度缩减

### 修改
- `cyberdog_control.py`
  - 新增 `p` 键：前倾低头（pitch=0.20），移动时保持前倾
  - 新增 `u` 键：恢复正常姿态
  - 退出时不再发送趴下指令，保持当前站立状态
  - keepalive 线程：前倾模式下自动发送小速度保持行走模式

- `cyberdog_race/utils/cyberdog_lcm.py`
  - 新增 `move_lowhead()` 方法：前倾行走（pitch=0.20）
  - 完善 mode 定义注释

- `cyberdog_race/stages/stage_path.py`
  - 行走时使用 `move_lowhead()` 替代普通 `move()`

## 2026-05-21

### 新增
- `gazebo.xacro`：在仿真中为机器狗添加 RGB 相机和 AI 相机插件
  - 替换路径：`/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro/gazebo.xacro`
  - 新增 topic：`/rgb_camera/image_raw`、`/ai_camera/image_raw`

- `camera_relay.py`：相机 QoS 转发节点，解决 rqt_image_view 收不到画面的问题
  - 容器路径：`/home/cyberdog_sim/camera_relay.py`
  - 输入：`/rgb_camera/image_raw`（best_effort）→ 输出：`/camera/image_rgb`（reliable）
  - 输入：`/ai_camera/image_raw`（best_effort）→ 输出：`/camera/image_ai`（reliable）

- `view_camera.py`：相机查看一体化工具，自动启动 relay 并打开 rqt_image_view
  - 容器路径：`/home/cyberdog_sim/view_camera.py`
  - 用法：`python3.8 view_camera.py`，在下拉菜单选 `/camera/image_rgb`

- `cyberdog_race/utils/camera_subscriber.py`：公共相机订阅封装
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/utils/camera_subscriber.py`
  - 订阅 `/rgb_camera/image_raw`（best_effort QoS），返回 numpy BGR 数组

### 修改
- `cyberdog_race/stages/stage_path.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/stages/stage_path.py`
  - 相机输入从 `cv2.VideoCapture(0)` 改为订阅 `/rgb_camera/image_raw`

- `cyberdog_race/stages/stage_ball.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/stages/stage_ball.py`
  - 相机输入从 `cv2.VideoCapture(0)` 改为订阅 `/rgb_camera/image_raw`

- `cyberdog_race/stages/stage_hunt.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/stages/stage_hunt.py`
  - 相机输入从 `cv2.VideoCapture(0)` 改为订阅 `/rgb_camera/image_raw`

- `cyberdog_race/stages/stage_kick.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/stages/stage_kick.py`
  - 相机输入从 `cv2.VideoCapture(0)` 改为订阅 `/rgb_camera/image_raw`

- `README.md`：补充日常使用命令（查看相机、新开终端、完整启动流程）

---

## 2026-05-19

### 新增
- `cyberdog_race/main.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/main.py`
  - 比赛程序启动入口

- `cyberdog_race/race_controller.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/race_controller.py`
  - 主状态机，基于 `/tf` 世界坐标 Y 值自动切换六个赛段

- `cyberdog_race/utils/cyberdog_lcm.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/utils/cyberdog_lcm.py`
  - LCM 运动控制封装，含 Wait_finish 机制和完整 mode 定义（mode 0/7/11/12/21/62/64）

- `cyberdog_race/utils/pose_tracker.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/utils/pose_tracker.py`
  - 订阅 ROS2 `/tf`，持续更新机器人世界坐标

- `cyberdog_race/stages/stage_path.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/stages/stage_path.py`
  - 赛段一（石径探路）/ 赛段三（曲道冲锋）黄线跟踪 PID 控制

- `cyberdog_race/stages/stage_ball.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/stages/stage_ball.py`
  - 赛段二（荒野寻珠）橙色球 HSV 检测 + 撞击逻辑

- `cyberdog_race/stages/stage_hunt.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/stages/stage_hunt.py`
  - 赛段四（深隧寻珍）目标识别、语音播报、避障逻辑

- `cyberdog_race/stages/stage_bridge.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/stages/stage_bridge.py`
  - 赛段五（孤梁稳渡）独木桥行走，IMU roll 角修正居中

- `cyberdog_race/stages/stage_kick.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_race/stages/stage_kick.py`
  - 赛段六（撷金建功）足球检测、踢球、终点趴下

- `cyberdog_control.py`
  - 容器路径：`/home/cyberdog_sim/cyberdog_control.py`
  - 实时键盘控制脚本，支持方向键/wasd，含转向和平移

- `race.world`
  - 容器路径：`/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_gazebo/world/race.world`
  - 2026 比赛赛道仿真场景文件（通过 `-v` 挂载同步）

- `docs/比赛方案.md`
  - 本地路径：`~/Xiaomi/docs/比赛方案.md`
  - 完整比赛方案，含六赛段技术方案、人员分工、时间规划、代码结构说明
