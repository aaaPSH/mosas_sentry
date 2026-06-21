# MOSAS Sentry - RM 哨兵导航系统

基于 FAST_LIO2 的 RoboMaster 哨兵机器人导航系统，运行在 ROS2 Humble + Docker 开发环境中。

## 环境要求

- Ubuntu 22.04 (WSL2 或原生)
- Docker + Docker Compose
- NVIDIA GPU + nvidia-container-toolkit（Gazebo GPU 加速）
- X11 显示服务（rviz2 / Gazebo GUI）

## 快速开始

### 1. 构建 Docker 镜像

```bash
cd docker
./build.sh
```

### 2. 启动开发容器

```bash
./run.sh
```

### 3. 克隆依赖（容器内）

```bash
cd /root/mosas_sentry

# FAST_LIO2 核心
git clone https://github.com/hku-mars/FAST_LIO.git src/FAST_LIO

# Livox 驱动（FAST_LIO 依赖）
git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
```

### 4. 编译工作空间

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 5. 运行

```bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py
```

## 项目结构

```
mosas_sentry/
├── docker/
│   ├── Dockerfile            # 开发镜像定义
│   ├── docker-compose.yml    # 容器服务配置
│   ├── build.sh              # 构建脚本
│   └── run.sh                # 启动脚本
├── .devcontainer/
│   └── devcontainer.json     # VS Code Remote Container 配置
├── src/                      # ROS2 包（自行克隆/创建）
│   ├── FAST_LIO/             # FAST_LIO2 算法
│   ├── livox_ros_driver2/    # Livox 雷达驱动
│   ├── mosas_sentry_description/  # URDF 模型
│   ├── mosas_sentry_bringup/      # Launch 文件
│   ├── mosas_sentry_nav/          # Nav2 参数配置
│   └── mosas_sentry_sim/          # Gazebo 仿真
└── README.md
```

## Docker 镜像包含

| 组件 | 说明 |
|------|------|
| ROS2 Humble Desktop Full | 核心 + Gazebo Fortress + rviz2 |
| Nav2 | 导航栈（规划器/控制器/行为树） |
| SLAM Toolbox | 在线 SLAM 建图 |
| robot_localization | EKF/UKF 传感器融合 |
| ros2_control + controllers | 硬件抽象 + 差速驱动 |
| Gazebo ROS2 Control | 仿真插件 |
| PCL + Eigen3 | FAST_LIO2 依赖 |
| Livox SDK2 | Livox 雷达 SDK |
| colcon / rosdep / gdb | 开发调试工具 |

## VS Code 开发

在 VS Code 中安装 [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) 扩展，打开项目后选择 **"Reopen in Container"** 即可获得完整的 ROS2 开发环境（IntelliSense、调试、终端）。

## 常见问题

**Gazebo/rviz2 无法打开显示**
- 确保运行了 `xhost +local:docker`
- 检查 `DISPLAY` 环境变量是否正确

**DDS 节点无法互相发现**
- 确认使用 `network_mode: host`
- 检查 `ROS_DOMAIN_ID` 是否一致

**GPU 未生效**
- 确认安装了 `nvidia-container-toolkit`
- 运行 `docker info | grep -i runtime` 检查 nvidia runtime
