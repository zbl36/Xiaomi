# XiaomiCup CyberDog 仿真与官方例程使用说明

## 1. 启动基础环境

### 1.1 导入 Docker 镜像

```bash
sudo docker load -i cyberdog_race2026.tar
```

### 1.2 授权 X Server

```bash
xhost +
```

### 1.3 启动容器

```bash
sudo docker run -it --privileged=true \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  cyberdog_sim:v2026
```

### 1.4 安装 Python 例程依赖

```bash
sudo apt update
sudo apt install -y python3-pip
pip3 install lcm toml
```

### 1.5 启动仿真

容器内执行：

```bash
cd /home/cyberdog_sim
python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py
```

说明：

- `/home/cyberdog_ws/install/setup.bash` 对应运动控制工作空间
- `/home/cyberdog_sim/install/setup.bash` 对应仿真工作空间
- 如果 `install/setup.bash` 不存在，先进入对应工作空间执行 `colcon build`

### 1.6 启动运动管理服务

另开一个容器终端执行：

```bash
cd /home/cyberdog_ws
source /opt/ros/galactic/setup.bash
source /home/cyberdog_ws/install/setup.bash
ros2 run motion_manager motion_manager
```

## 2. 获取并使用官方例程仓库 `loco_hl_example`

官方仓库地址：

```bash
https://github.com/MiRoboticsLab/loco_hl_example
```

如果当前目录下已经包含 `loco_hl_example`，可以直接使用，无需重复克隆；如果没有，则执行：

```bash
git clone https://github.com/MiRoboticsLab/loco_hl_example.git
```

### 2.1 仓库结构与运行前提

仓库主要包含 3 个高层接口示例：

- `loco_hl_example/basic_motion`：基础动作示例
- `loco_hl_example/sequential_motion`：按 `.toml` 顺序执行动作
- `loco_hl_example/customized_gait`：自定义步态与用户步态示例

这些例程都通过 LCM 与运控通信，核心话题如下：

- `robot_control_cmd`：发送控制命令
- `robot_control_response`：接收动作执行反馈
- `user_gait_file`：发送用户自定义步态文件

运行官方例程前，建议先确认：

1. Gazebo 仿真已经启动；
2. `motion_manager` 已经在另一个终端正常运行；
3. Python 依赖已经安装完成：`lcm`、`toml`；
4. 执行脚本时位于对应示例目录下，因为脚本内部大量使用相对路径加载本地 `.toml` 文件。

### 2.2 基础动作例程 `basic_motion`

入口文件：

```bash
loco_hl_example/basic_motion/main.py
```

脚本会按顺序发送以下动作：

- `mode = 12`：恢复站立（Recovery stand）
- `mode = 62, gait_id = 2`：握手
- `mode = 64`：双足站立
- `mode = 21`：位置插值控制，分别执行抬头、低头、调整机身高度
- `mode = 11, gait_id = 26`：进入机动步态并原地转向
- `mode = 7`：最后进入 PureDamper / 阻尼趴下

从源码看，这个示例还有两个很重要的特点：

- 每次发送新命令前都会让 `life_count += 1`，这是命令生效的关键；
- 脚本会订阅 `robot_control_response`，并通过 `order_process_bar >= 95` 判断动作是否基本执行完成，再进入下一步。

运行方法：

```bash
cd loco_hl_example/basic_motion
python3 main.py
```

如果这个脚本能顺利跑完，通常说明基础控制链路已经正常。

### 2.3 顺序动作例程 `sequential_motion`

入口文件：

```bash
loco_hl_example/sequential_motion/main.py
```

默认动作序列文件：

```bash
loco_hl_example/sequential_motion/cyberdog2_ctrl.toml
```

这个例程的核心思路是：读取一个 `.toml` 文件中的 `[[step]]` 数组，然后把每一段动作依次发布到 `robot_control_cmd`。相比 `basic_motion`，它更适合做“可编辑、可复用”的动作编排。

`cyberdog2_ctrl.toml` 中一个典型动作块包含这些字段：

- `mode`：控制模式，例如恢复站立、插值控制、机动步态、PureDamper
- `gait_id`：步态编号
- `contact`：触地状态位掩码
- `duration`：该动作的期望执行时长
- `vel_des`：机身速度或转向速度
- `rpy_des`：机身姿态期望（roll / pitch / yaw）
- `pos_des`：机身位置 / 质心高度期望
- `acc_des`、`ctrl_point`、`foot_pose`、`step_height`、`value`：更细粒度的控制参数

其中，官方示例对 `contact` 给出了注释示例：

- `15`：四足接触地面
- `14`：抬右前足
- `13`：抬左前足
- `11`：抬右后足
- `7`：抬左后足

示例 `cyberdog2_ctrl.toml` 本身就包含了一套完整流程，包括：

- 恢复站立
- 调节机身高度
- 调节姿态和旋转中心
- 单腿抬腿动作
- 机动步态转向
- 最后进入 PureDamper

运行方法：

```bash
cd loco_hl_example/sequential_motion
python3 main.py
```

运行后程序会列出当前目录中的文件，并提示输入编号；通常选择 `cyberdog2_ctrl.toml` 对应编号即可。

说明：

- 程序会把每一步动作直接发布出去，不像 `basic_motion` 那样等待反馈完成后再继续；
- 全部动作发送完毕后，还会继续发送一段时间心跳，保持当前命令有效；
- 如果执行过程中按 `Ctrl + C`，脚本会补发 `mode = 7` 的 PureDamper 命令。

### 2.4 自定义步态例程 `customized_gait`

入口文件：

```bash
loco_hl_example/customized_gait/main.py
```

关键文件：

```bash
loco_hl_example/customized_gait/Gait_Def_moonwalk.toml
loco_hl_example/customized_gait/Gait_Params_moonwalk.toml
loco_hl_example/customized_gait/Usergait_List.toml
```

这个例程展示的是“先定义用户步态，再触发执行”的完整流程。以仓库内置的 `moonwalk`（太空步）为例，脚本大致会做 3 件事：

1. 读取 `Gait_Params_moonwalk.toml`，并生成展开后的 `Gait_Params_moonwalk_full.toml`；
2. 先后把 `Gait_Def_moonwalk.toml` 和 `Gait_Params_moonwalk_full.toml` 通过 `user_gait_file` 话题发送给运控；
3. 读取 `Usergait_List.toml`，再通过 `robot_control_cmd` 触发用户步态执行。

从源码和配置文件可以看出，这套机制里几个文件的职责分别是：

- `Gait_Def_moonwalk.toml`：定义步态相位序列，每个 `[[section]]` 描述一个接触相和持续时间；
- `Gait_Params_moonwalk.toml`：定义每一步的参数，如 `body_vel_des`、`body_pos_des`、`landing_pos_des`、`step_height`、`weight`、`mu`、`landing_gain` 等；
- `Usergait_List.toml`：定义正式执行流程，例如先恢复站立，再以 `gait_id = 110` 执行用户步态，最后进入 PureDamper。

这里有一个容易忽略但很关键的点：

- 在参数展开阶段，脚本把自定义步态封装成 `mode = 11`、`gait_id = 110` 的 locomotion 用户步态；
- 在最终执行列表 `Usergait_List.toml` 中，则通过 `mode = 62`、`gait_id = 110` 去触发用户定义步态动作。

运行方法：

```bash
cd loco_hl_example/customized_gait
python3 main.py
```

### 2.5 新增动作程序库 `motion_programs`

```bash
loco_hl_example/motion_programs
```

当前一共补充了 10 个程序，其中至少 8 个使用了不同的 `motion id / gait_id`：

- `01_left_hand_wave.toml`：`mode = 62, gait_id = 1`，握左手
- `02_sit_down.toml`：`mode = 62, gait_id = 3`，坐下
- `03_hip_swing.toml`：`mode = 62, gait_id = 4`，扭屁股
- `04_head_twist.toml`：`mode = 62, gait_id = 5`，扭头
- `05_stretch_body.toml`：`mode = 62, gait_id = 6`，伸懒腰
- `06_ballet_dance.toml`：`mode = 62, gait_id = 11`，芭蕾舞
- `07_built_in_moonwalk.toml`：`mode = 62, gait_id = 12`，内置太空步
- `08_push_up.toml`：`mode = 62, gait_id = 34`，俯卧撑
- `09_trot_slow_circle.toml`：`mode = 11, gait_id = 27`，慢跑转圈
- `10_bound_turn.toml`：`mode = 11, gait_id = 7`，四足跳跑转向

新增入口脚本：

```bash
loco_hl_example/motion_programs/main.py
```

脚本会读取：

```bash
loco_hl_example/motion_programs/catalog.toml
```

并列出所有动作程序供选择，然后再从：

```bash
loco_hl_example/motion_programs/programs/*.toml
```

加载对应序列并发布到 `robot_control_cmd`。

运行方法：

```bash
cd loco_hl_example/motion_programs
python3 main.py
```

也可以直接传入编号或关键字：

```bash
python3 main.py 7
python3 main.py built_in_moonwalk
```

- 所有程序默认先执行 `mode = 12` 恢复站立；
- locomotion 类程序额外补充了 `gait_id = 27`（慢跑）和 `gait_id = 7`（四足跳跑）两组步态；
- 所有程序最后统一切换到 `mode = 7`，便于结束后继续调试或重新启动下一个动作。

## 3. 参考资料

- 运控接口文档：<https://miroboticslab.github.io/blogs/#/cn/cyberdog_loco_cn>
- 运动管理文档：<https://miroboticslab.github.io/blogs/#/cn/motion_manager_cn>
- 仿真平台文档：<https://miroboticslab.github.io/blogs/#/cn/cyberdog_gazebo_cn>
- `loco_hl_example`：<https://github.com/MiRoboticsLab/loco_hl_example>
