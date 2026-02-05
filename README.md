# unitree_go2_ws

基于 **ROS 2 Humble** 的 Unitree Go2 机器人控制与导航工作空间  
包含 Go2 运动控制、Action 通信（导航客户端 / 服务端）、里程计反馈等示例。

---

## 一、环境要求

### 1. 操作系统
- Ubuntu 22.04 (推荐)

### 2. ROS 版本
- ROS 2 Humble

请确保已正确安装 ROS 2 Humble，并已 source：

```bash
source /opt/ros/humble/setup.bash
二、在其他设备上下载（克隆仓库）
在目标设备的终端中执行：

cd ~
git clone https://github.com/ztl3106742440-hub/unitree_go2_ws.git
进入工作空间：

cd unitree_go2_ws
三、安装依赖
1. ROS 依赖（推荐方式）
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
⚠️ 如果某些 Unitree 相关包无法通过 rosdep 安装，请确保：

已正确安装 Unitree SDK

已 source Unitree 环境（如有）

四、编译工作空间
colcon build
编译完成后，source 本地环境：

source install/setup.bash
👉 建议加入 ~/.bashrc：

echo "source ~/unitree_go2_ws/install/setup.bash" >> ~/.bashrc
