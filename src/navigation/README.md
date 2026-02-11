# Unitree Go2 SLAM 建图与导航功能包

基于 **ROS 2 Humble** 的 Unitree Go2 机器人 SLAM 建图和自主导航系统。

## 📦 功能包说明

### 1. **go2_sensors** - 传感器处理
- 将 UTlidar 3D 点云 (`/utlidar/cloud_deskewed`) 转换为 2D 激光扫描 (`/scan`)
- 为 SLAM 和导航提供标准的激光扫描数据

### 2. **go2_slam** - SLAM 建图
- 使用 **SLAM Toolbox** 进行实时建图
- 支持闭环检测和地图优化
- 生成可用于导航的栅格地图

### 3. **go2_navigation** - 自主导航
- 基于 **Nav2** 导航栈
- 支持路径规划、避障、定位
- 提供完整的导航功能

---

## 🔧 依赖安装

### 1. 安装 Nav2 和 SLAM Toolbox

```bash
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan
```

### 2. 编译工作空间

```bash
cd ~/unitree_go2_ws
colcon build --packages-select go2_sensors go2_slam go2_navigation
source install/setup.bash
```

---

## 🚀 使用指南

### 📍 **步骤 1：SLAM 建图**

#### 启动完整建图系统（推荐）

```bash
# 启动驱动 + 传感器处理 + SLAM Toolbox
ros2 launch go2_slam mapping.launch.py
```

**包含的节点：**
- Go2 驱动（里程计、TF、关节状态）
- 点云转激光扫描
- SLAM Toolbox 建图
- RViz 可视化

#### 或者分步启动

```bash
# 终端1：启动 Go2 驱动
ros2 launch go2_driver_py driver.launch.py

# 终端2：启动点云转激光
ros2 launch go2_sensors pointcloud_to_laserscan.launch.py

# 终端3：启动 SLAM
ros2 launch go2_slam slam_toolbox.launch.py
```

#### 控制机器人建图

```bash
# 使用键盘控制（如果有）
ros2 run go2_teleop_ctrl_keyboard keyboard_teleop

# 或使用手柄/其他控制方式
```

#### 保存地图

在 RViz 中：
1. 点击 **Panels** → **Add New Panel** → **SlamToolboxPlugin**
2. 点击 **"Serialize Map"** 保存实时地图
3. 或在终端运行：

```bash
# 保存地图到指定位置
ros2 run nav2_map_server map_saver_cli -f ~/unitree_go2_ws/src/navigation/go2_navigation/maps/my_map
```

保存后会生成两个文件：
- `my_map.yaml` - 地图配置文件
- `my_map.pgm` - 地图图像文件

---

### 🧭 **步骤 2：自主导航**

#### 启动完整导航系统（推荐）

```bash
# 使用保存的地图进行导航
ros2 launch go2_navigation navigation.launch.py map:=/path/to/your/map.yaml
```

例如：
```bash
ros2 launch go2_navigation navigation.launch.py \
    map:=$HOME/unitree_go2_ws/src/navigation/go2_navigation/maps/my_map.yaml
```

**包含的节点：**
- Go2 驱动
- 点云转激光扫描
- AMCL 定位
- Nav2 导航栈（路径规划、避障、控制器等）

#### 在 RViz 中设置导航目标

1. **设置初始位姿**（机器人当前位置）
   - 点击 RViz 顶部的 **"2D Pose Estimate"**
   - 在地图上点击并拖动箭头，指示机器人的位置和朝向

2. **发送导航目标**
   - 点击 **"Nav2 Goal"** 或 **"2D Goal Pose"**
   - 在地图上点击目标位置
   - 机器人将自动规划路径并导航到目标

#### 或使用命令行发送目标

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

---

## 🔍 话题和服务

### 输入话题（需要）
| 话题名称 | 消息类型 | 说明 | 提供者 |
|---------|---------|------|--------|
| `/utlidar/cloud_deskewed` | `sensor_msgs/PointCloud2` | UTlidar 点云数据 | Go2 系统 |
| `/odom` | `nav_msgs/Odometry` | 里程计数据 | go2_driver |
| `/lf/sportmodestate` | `unitree_go::msg::SportModeState` | 机器人状态 | Go2 SDK |

### 输出话题
| 话题名称 | 消息类型 | 说明 | 发布者 |
|---------|---------|------|--------|
| `/scan` | `sensor_msgs/LaserScan` | 2D 激光扫描 | go2_sensors |
| `/map` | `nav_msgs/OccupancyGrid` | 栅格地图 | SLAM Toolbox / Map Server |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度命令 | Nav2 |

### TF 树
```
map
 └─ odom (由 SLAM Toolbox 或 AMCL 发布)
     └─ base_link (由 go2_driver 发布)
         └─ radar
             └─ utlidar_lidar
```

---

## ⚙️ 参数调优

### 传感器参数
编辑 `go2_sensors/config/pointcloud_to_laserscan_params.yaml`：
```yaml
min_height: -0.5    # 提取点云的最小高度
max_height: 1.0     # 提取点云的最大高度
range_max: 20.0     # 最大检测距离
```

### SLAM 参数
编辑 `go2_slam/config/mapper_params_online_async.yaml`：
```yaml
resolution: 0.05              # 地图分辨率（米/像素）
minimum_travel_distance: 0.2  # 触发扫描匹配的最小移动距离
do_loop_closing: true         # 是否启用闭环检测
```

### 导航参数
编辑 `go2_navigation/config/nav2_params.yaml`：
```yaml
robot_radius: 0.3       # 机器人半径（米）
max_vel_x: 0.5          # 最大线速度
max_vel_theta: 1.0      # 最大角速度
inflation_radius: 0.55  # 障碍物膨胀半径
```

---

## 📝 常见问题

### 1. 没有激光扫描数据？
检查点云话题：
```bash
ros2 topic echo /utlidar/cloud_deskewed --no-arr
```
如果没有数据，确保 Go2 的 UTlidar 服务已启动。

### 2. 地图质量差？
- 降低机器人移动速度
- 增加 `minimum_travel_distance` 参数
- 确保环境有足够的特征点

### 3. 导航时机器人不动？
- 检查 `/cmd_vel` 话题是否有数据：
  ```bash
  ros2 topic echo /cmd_vel
  ```
- 确认 `go2_twist_bridge` 正在运行
- 检查机器人状态是否允许运动

### 4. TF 树错误？
查看 TF 树：
```bash
ros2 run tf2_tools view_frames
```
确保 `map -> odom -> base_link -> radar -> utlidar_lidar` 链完整。

---

## 🎯 下一步扩展

- [ ] 添加多点巡航功能
- [ ] 集成语音控制
- [ ] 添加动态障碍物避障
- [ ] 实现多机器人协同建图

---

## 📚 参考资料

- [Nav2 官方文档](https://navigation.ros.org/)
- [SLAM Toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [Unitree Go2 开发者文档](https://support.unitree.com/)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)

---

**作者**: ztl
**许可证**: Apache-2.0
**ROS 版本**: ROS 2 Humble
