# ROS-2-workspace
# ROS2 水下仿真 (D题)

## 项目简介

基于 ROS2 Humble 的水下机器人运动仿真，实现：
- 物理引擎（质量、阻力、转动惯量）模拟水下运动
- DVL / IMU 传感器数据发布（50Hz）
- 三种控制模式：定速巡航、画圆轨迹、定点移动

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- colcon 构建工具

## 安装与运行

### 1. 创建工作空间并克隆代码
```bash
mkdir -p ~/ros2_ws_d/src
cd ~/ros2_ws_d/src
git clone https://github.com/你的用户名/underwater_sim_d.git
