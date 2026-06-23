# MOSAS Sentry - RM 哨兵导航系统

基于 FAST_LIO2 的 RoboMaster 哨兵机器人导航系统，运行在 ROS2 Humble + Docker 开发环境中。

## 环境要求

- Ubuntu 22.04 (WSL2 或原生)
- ROS2 Humble
- Livox SDK2（`livox_ros_driver2` 依赖，需安装到 `/usr/local`）
- Gazebo / rviz2 图形环境（仿真和可视化时需要）

## 快速开始

### 1. 初始化子模块

```bash
git submodule update --init --recursive
```

### 2. 安装依赖

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

`livox_ros_driver2` 还需要先安装 Livox SDK2，并确保系统中存在：

```bash
/usr/local/lib/liblivox_lidar_sdk_shared.so
```

### 3. 编译工作空间

```bash
./scripts/build_humble.sh
```

或手动执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

### 4. 运行

```bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py
```

## 项目结构

```
mosas_sentry/
├── scripts/                  # 工作区脚本
│   └── build_humble.sh       # ROS2 Humble 统一构建入口
├── src/                      # ROS2 源码区，只放源码包和子模块
│   ├── driver/
│   │   └── livox_ros_driver2/ # Livox 雷达驱动
│   ├── localization/
│   │   ├── FAST_LIO/         # FAST_LIO2 定位/建图
│   │   └── Point-LIO/        # Point-LIO，可按需启用
│   └── rm_simulation/
│       ├── pb_rm_simulation/ # RoboMaster 仿真场景
│       └── livox_laser_simulation_RO2/
│                             # Livox 雷达仿真插件
├── build/                    # colcon 生成目录，不提交
├── install/                  # colcon 安装目录，不提交
├── log/                      # colcon 日志目录，不提交
└── README.md
```

## 架构约定

- 根目录是唯一 colcon 工作区入口，不要在 `src/` 内执行 `colcon build`。
- `src/driver` 放硬件驱动，`src/localization` 放定位建图算法，`src/rm_simulation` 放仿真相关包。
- 第三方包以 Git submodule 管理，更新依赖时优先更新子模块指针。
- `Point-LIO` 当前带有 `COLCON_IGNORE`，需要参与编译时删除该文件。

## 常见问题

**Livox 驱动提示找不到 `package.xml`**

`livox_ros_driver2` 上游同时支持 ROS1/ROS2，仓库中原始文件名是 `package_ROS2.xml`。根目录构建脚本会在编译前自动准备 ROS2 用的 `package.xml`。

**Livox 驱动提示找不到 `liblivox_lidar_sdk_shared.so`**

请先安装 Livox SDK2，并确认 `/usr/local/lib` 在动态库路径中：

```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
```

**FAST_LIO 提示找不到 ikd-Tree**

初始化嵌套子模块：

```bash
git submodule update --init --recursive
```
