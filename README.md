# 2026小米杯 · CyberDog 比赛代码

2026年全国大学生计算机系统能力大赛 · 智能系统创新设计赛（小米杯）

---

## 环境搭建

### 第一步：安装 Docker

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://mirrors.tuna.tsinghua.edu.cn/docker-ce/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://mirrors.tuna.tsinghua.edu.cn/docker-ce/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io
sudo usermod -aG docker $USER
```

### 第二步：获取 Docker 镜像

从百度网盘下载 `cyberdog_race2026.tar`，导入镜像：

```bash
sudo docker load -i cyberdog_race2026.tar
# 导入完成后显示：Loaded image: cyberdog_sim:v2026
```

### 第三步：克隆本仓库

```bash
git clone https://github.com/zbl36/Xiaomi.git
cd Xiaomi
```

### 第四步：启动容器

```bash
xhost +
sudo docker run -it --name cyberdog --shm-size="1g" --privileged=true \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/Xiaomi/race.world:/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_gazebo/world/race.world \
  cyberdog_sim:v2026
```

### 第五步：应用所有修改

在宿主机新开终端，把仓库里的修改文件推进容器：

```bash
cd ~/Xiaomi

# 应用相机配置（添加了RGB相机和AI相机插件）
sudo docker cp gazebo.xacro cyberdog:/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_robot/cyberdog_description/xacro/gazebo.xacro

# 应用比赛代码
sudo docker cp cyberdog_race cyberdog:/home/cyberdog_sim/cyberdog_race
sudo docker cp cyberdog_control.py cyberdog:/home/cyberdog_sim/cyberdog_control.py
```

### 第六步：容器内安装依赖并重新编译

```bash
sudo docker exec -it cyberdog bash

# 安装依赖
apt-get install -y espeak wget
pip3 install lcm toml

# 设置环境变量（永久生效）
echo 'export PYTHONPATH=/usr/local/lib/python3.8/site-packages:$PYTHONPATH' >> ~/.bashrc
echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc
echo 'source /home/cyberdog_sim/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

# 重新编译（应用相机配置）
cd /home/cyberdog_sim
colcon build --packages-select cyberdog_gazebo --merge-install
source install/setup.bash
```

### 第七步：保存镜像状态

```bash
# 退出容器后在宿主机执行
sudo docker commit cyberdog cyberdog_sim:v2026_save
```

---

## 日常使用

### 启动仿真

```bash
# 1. 授权GUI
xhost +

# 2. 启动容器（如提示名称冲突先执行 sudo docker rm cyberdog）
sudo docker run -it --name cyberdog --shm-size="1g" --privileged=true \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/Xiaomi/race.world:/home/cyberdog_sim/src/cyberdog_simulator/cyberdog_gazebo/world/race.world \
  cyberdog_sim:v2026_save

# 3. 容器内启动仿真
cd /home/cyberdog_sim
python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py
```

### 手动键盘控制机器狗

新开终端进入容器：

```bash
sudo docker exec -it cyberdog bash
cd /home/cyberdog_sim
python3.8 cyberdog_control.py
```

| 按键 | 动作 |
|------|------|
| `r` | 恢复站立 |
| `t` | 站立 |
| `y` | 行走模式 |
| `w/↑` | 前进 |
| `s/↓` | 后退 |
| `a/←` | 左转 |
| `d/→` | 右转 |
| `z/c` | 左/右平移 |
| 空格 | 停止 |
| `0` | 趴下 |
| `q` | 退出 |

### 运行比赛自动程序

```bash
sudo docker exec -it cyberdog bash
cd /home/cyberdog_sim/cyberdog_race
python3.8 main.py
```

### 保存进度

```bash
sudo docker commit cyberdog cyberdog_sim:v2026_save
```

---

## 仓库文件说明

| 文件/目录 | 说明 |
|-----------|------|
| `cyberdog_race/` | 比赛自动控制代码（状态机+各赛段逻辑） |
| `cyberdog_control.py` | 手动键盘控制脚本 |
| `gazebo.xacro` | 添加了RGB/AI相机插件的仿真配置 |
| `race.world` | 2026比赛赛道仿真场景 |
| `docs/` | 比赛规则、赛题、技术文档、比赛方案 |
| `code and readme/` | 官方示例代码（basic_motion等） |
