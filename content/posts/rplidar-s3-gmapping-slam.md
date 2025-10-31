---
title: "RPLidar S3 + GMapping SLAM建图系统实践与问题解决"
date: 2025-10-31
draft: false
weight: 1
categories: ["ROS2", "SLAM", "机器人"]
tags: ["ROS2", "GMapping", "RPLidar", "SLAM", "激光雷达", "建图"]
---

# RPLidar S3 + GMapping SLAM建图系统实践与问题解决

## 项目概述

本项目实现了在 ROS 2 Humble 环境下，使用思岚科技 RPLidar S3 激光雷达配合 GMapping 算法进行 2D SLAM 建图的完整系统。在实施过程中遇到并解决了多个技术问题，形成了一套完整的解决方案。

### 系统环境

- **操作系统**: Ubuntu 22.04
- **ROS版本**: ROS 2 Humble
- **激光雷达**: 思岚 RPLidar S3（DenseBoost 模式）
- **SLAM算法**: GMapping

### 系统架构

```
RPLidar S3 (DenseBoost模式)
    ↓ scan_raw (原始高密度数据)
    ↓
laser_scan_decimator (降采样节点)
    ↓ scan (360个点)
    ↓
simple_odom_publisher (里程计模拟)
    ↓ odom
    ↓
GMapping SLAM
    ↓ map + tf
    ↓
RViz2 可视化
```

---

## 核心技术问题与解决方案

### 问题1: ROS 2 参数文件格式错误

#### 问题描述

启动 `slam_gmapping` 节点时崩溃，错误信息：
```
Cannot have a value before ros__parameters at line 2
```

#### 根本原因

ROS 2 的参数文件格式与 ROS 1 有重大差异。ROS 2 要求在参数定义前必须包含节点名和 `ros__parameters` 键。

#### 解决方案

修改 `slam_params.yaml` 文件格式：

**错误格式：**
```yaml
slam_gmapping:
  base_frame: "base_link"
  odom_frame: "odom"
  ...
```

**正确格式：**
```yaml
slam_gmapping:
  ros__parameters:
    base_frame: "base_link"
    odom_frame: "odom"
    map_frame: "map"
    maxUrange: 10.0
    maxRange: 40.0
    particles: 30
    max_beams: 60
```

#### 技术要点

- ROS 2 参数文件必须遵循 `node_name -> ros__parameters -> actual_params` 的三层结构
- 对于多节点配置，可以使用 `/**:` 通配符应用于所有节点
- 参数类型需要正确匹配（int、float、string、bool）

---

### 问题2: 缺少里程计数据导致建图失败

#### 问题描述

- RViz 显示 "No map received"
- TF 警告：找不到从 `base_footprint`/`base_link`/`laser` 到 `map` 的变换
- 消息队列溢出，RViz 不断丢弃消息

#### 根本原因

GMapping 需要完整的 TF 树结构：`map -> odom -> base_footprint -> base_link -> laser`。系统中缺少 `odom` 坐标系和里程计数据发布。

#### 解决方案

实现了三种方案：

##### 方案1: 静态里程计（测试用）

在 launch 文件中添加静态 TF 变换：

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='odom_to_base_footprint',
    arguments=['--x', '0', '--y', '0', '--z', '0',
               '--roll', '0', '--pitch', '0', '--yaw', '0',
               '--frame-id', 'odom', 
               '--child-frame-id', 'base_footprint']
)
```

**特点**：
- ✅ 简单快速，适合系统测试
- ❌ 无法真正建图（位置固定）

##### 方案2: 简单里程计发布器（推荐）

创建 `simple_odom_publisher.py` 节点，根据速度命令模拟里程计：

```python
class SimpleOdomPublisher(Node):
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        # 订阅速度命令
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 发布里程计
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # 发布TF变换
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 位置和速度状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        
        # 定时器发布
        self.timer = self.create_timer(0.02, self.timer_callback)
    
    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z
    
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = 0.02
        
        # 更新位置
        delta_x = self.vx * cos(self.theta) * dt
        delta_y = self.vx * sin(self.theta) * dt
        delta_theta = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # 发布 odom -> base_footprint TF
        # 发布 Odometry 消息
        ...
```

配合键盘遥控使用：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**特点**：
- ✅ 可以真正进行建图
- ✅ 适合无底盘时测试
- ⚠️ 精度有限，仅供测试

##### 方案3: 真实机器人底盘（生产环境）

要求：
- 底盘发布 `/odom` 话题（`nav_msgs/Odometry`）
- 底盘发布 `odom -> base_footprint` TF 变换

---

### 问题3: max_beams 参数配置错误

#### 问题描述

启动后 `slam_gmapping` 节点崩溃：
```
slam_gmapping: Assertion `beams<LASER_MAXBEAMS' failed.
[ERROR] slam_gmapping: process has died [exit code -6]
```

#### 根本原因

配置文件中 `max_beams: 1024` 过大，超过了 GMapping 源代码中定义的 `LASER_MAXBEAMS` 常量限制（通常为 512）。

#### 技术背景

`max_beams` 参数的含义：
- 不是控制输入数据的点数
- 而是 GMapping 用于扫描匹配时的采样数量
- GMapping 会从输入的激光点中均匀采样 `max_beams` 个点用于处理

#### 解决方案

修改 `slam_params.yaml`：
```yaml
max_beams: 60  # 从 1024 改为 60
```

#### 参数推荐值

| max_beams值 | 效果 | 适用场景 |
|------------|------|---------|
| 30 | 速度快，精度低 | 快速测试 |
| 60 | **平衡推荐** ⭐ | 一般应用 |
| 90 | 精度高，速度慢 | 精确建图 |
| >100 | 可能崩溃 | 不推荐 |

相关参数说明：
```yaml
maxUrange: 10.0   # 建图最大距离（米），超过此距离的点不用于建图
maxRange: 40.0    # 雷达最大测量距离（米），应设置为雷达量程
particles: 30     # 粒子滤波器粒子数，越多越精确但越慢
```

---

### 问题4: 激光数据点数超过GMapping限制

#### 问题描述

思岚 S3 在 DenseBoost 模式下产生大量激光点（可达数千个），远超 GMapping 内部固定数组大小限制，导致断言失败。

#### 根本原因

- 思岚 S3 的 DenseBoost 模式：32000 Hz 采样率，产生高密度点云
- GMapping 内部使用固定大小数组，编译时定义 `LASER_MAXBEAMS`（通常 512）
- 输入数据点数超过此限制会触发断言

之前误以为 `max_beams` 参数可以控制输入数据，实际上它只控制 GMapping 内部采样，不影响输入。

#### 解决方案：自定义降采样节点

创建 `laser_scan_decimator.py` 节点进行数据预处理：

```python
class LaserScanDecimator(Node):
    def __init__(self):
        super().__init__('laser_scan_decimator')
        
        # 目标点数（默认360）
        self.declare_parameter('target_beams', 360)
        self.target_beams = self.get_parameter('target_beams').value
        
        # 订阅原始激光数据
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_raw', 
            self.scan_callback, 
            qos_profile=qos_profile_sensor_data)
        
        # 发布降采样后的数据
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', 10)
    
    def scan_callback(self, scan_msg):
        original_num = len(scan_msg.ranges)
        
        if original_num <= self.target_beams:
            self.scan_pub.publish(scan_msg)
            return
        
        # 计算降采样步长
        step = int(np.ceil(original_num / self.target_beams))
        
        # 均匀采样
        decimated_ranges = scan_msg.ranges[::step]
        decimated_intensities = scan_msg.intensities[::step]
        
        # 创建新消息
        new_scan = LaserScan()
        new_scan.header = scan_msg.header
        new_scan.angle_min = scan_msg.angle_min
        new_scan.angle_max = scan_msg.angle_max
        new_scan.angle_increment = scan_msg.angle_increment * step
        new_scan.time_increment = scan_msg.time_increment * step
        new_scan.scan_time = scan_msg.scan_time
        new_scan.range_min = scan_msg.range_min
        new_scan.range_max = scan_scan.range_max
        new_scan.ranges = decimated_ranges
        new_scan.intensities = decimated_intensities
        
        self.scan_pub.publish(new_scan)
```

#### 降采样算法原理

1. **输入**：原始激光数据（N 个点，如 8192）
2. **计算步长**：`step = ceil(N / target_beams)`，例如 `ceil(8192 / 360) = 23`
3. **均匀采样**：`decimated = original[::step]`，每隔 23 个点取一个
4. **输出**：降采样数据（约 360 个点）

#### 数据流程

```
RPLidar S3 → scan_raw (8192点) → laser_scan_decimator → scan (360点) → GMapping
```

#### 优点

- ✅ 保留激光扫描的完整角度范围
- ✅ 均匀采样，不丢失重要特征
- ✅ 大幅减少数据量，满足 GMapping 要求
- ✅ 几乎不影响建图质量（360 点已足够）

---

### 问题5: laser_filters 配置格式问题

#### 问题描述

尝试使用 ROS 官方 `laser_filters` 包时出错：
```
[ERROR] Sequences can only be values and not keys in params
Error at line 2
```

#### 根本原因

ROS 2 的 `laser_filters` 参数格式与 ROS 1 不同，必须遵循特定的节点配置格式。

#### 解决方案

创建正确的 `laser_filter_params.yaml`：

```yaml
scan_to_scan_filter_chain:
  ros__parameters:
    filter1:
      name: range_filter
      type: laser_filters/LaserScanRangeFilter
      params:
        use_message_range_limits: false
        lower_threshold: 0.2
        upper_threshold: 40.0
    
    filter2:
      name: speckle_filter
      type: laser_filters/LaserScanSpeckleFilter
      params:
        filter_type: 0
        max_range: 40.0
        max_range_difference: 0.2
        filter_window: 2
```

#### laser_filters 主要滤波器

| 滤波器 | 功能 | 典型应用 |
|--------|------|---------|
| **LaserScanRangeFilter** | 过滤指定距离范围外的点 | 移除过近/过远噪声 |
| **LaserScanSpeckleFilter** | 移除离群点和噪声 | 清理扫描数据噪声 |
| **LaserScanAngularBoundsFilter** | 限制角度范围 | 只保留特定方向数据 |
| **LaserScanBoxFilter** | 移除3D盒子内的点 | 过滤机器人本体 |
| **ScanShadowsFilter** | 移除阴影效应点 | 提高边缘质量 |

#### 室内建图推荐配置

```yaml
scan_filter_chain:
  - name: range
    type: laser_filters/LaserScanRangeFilter
    params:
      lower_threshold: 0.2
      upper_threshold: 10.0
  
  - name: speckle
    type: laser_filters/LaserScanSpeckleFilter
    params:
      filter_type: 0
      max_range: 10.0
      max_range_difference: 0.1
      filter_window: 2
```

#### QoS 兼容性问题

`laser_filters` 默认使用 Reliable QoS，而 `slam_gmapping` 使用 SensorDataQoS（Best Effort）。需要添加 QoS 桥接节点：

```python
# 在 laser_scan_decimator 中同时处理 QoS 转换
self.scan_sub = self.create_subscription(
    LaserScan, '/scan_filtered',
    self.scan_callback,
    qos_profile=10)  # Reliable

self.scan_pub = self.create_publisher(
    LaserScan, '/scan',
    qos_profile=qos_profile_sensor_data)  # Best Effort
```

---

### 问题6: static_transform_publisher 参数格式警告

#### 问题描述

启动时出现警告：
```
[WARN] Old-style arguments are deprecated
```

#### 解决方案

更新为新式命名参数格式：

**旧式（已弃用）：**
```python
arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
```

**新式（推荐）：**
```python
arguments=[
    '--x', '0', '--y', '0', '--z', '0',
    '--roll', '0', '--pitch', '0', '--yaw', '0',
    '--frame-id', 'base_link',
    '--child-frame-id', 'laser'
]
```

---

## 完整系统实现

### Launch 文件

项目中有两个主要的 launch 文件：

#### Launch 文件 A：使用官方 laser_filters（推荐）

文件名：`bringup_rplidar_s3_gmapping_with_laser_filters.launch.py`

这个版本使用 ROS 官方的 `laser_filters` 包进行数据预处理。

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_params_file = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'config', 'slam_params.yaml')
    
    filter_params_file = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'config', 'laser_filter_params.yaml')
    
    return LaunchDescription([
        # 1. RPLidar S3 节点（输出到 /scan_raw）
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 1000000,
                'frame_id': 'laser',
                'scan_mode': 'DenseBoost'
            }],
            remappings=[('/scan', '/scan_raw')],
            output='screen'),
        
        # 2. 官方 laser_filters 节点
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[filter_params_file],
            remappings=[
                ('/scan', '/scan_raw'),
                ('/scan_filtered', '/scan')
            ],
            output='screen'),
        
        # 3-7. 其他节点（里程计、TF、GMapping、RViz）同下
        # ...
    ])
```

对应的滤波器配置文件 `laser_filter_params.yaml`：

```yaml
scan_to_scan_filter_chain:
  ros__parameters:
    filter1:
      name: range_filter
      type: laser_filters/LaserScanRangeFilter
      params:
        use_message_range_limits: false
        lower_threshold: 0.2
        upper_threshold: 40.0
    
    filter2:
      name: speckle_filter
      type: laser_filters/LaserScanSpeckleFilter
      params:
        filter_type: 0
        max_range: 40.0
        max_range_difference: 0.2
        filter_window: 2
```

#### Launch 文件 B：使用自定义降采样

文件名：`bringup_rplidar_s3_gmapping_with_teleop.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 参数
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    
    # 配置文件路径
    slam_params_file = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'config', 'slam_params.yaml')
    
    rviz_config_file = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'rviz', 'gmapping.rviz')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='RPLidar serial port'),
        
        # 1. RPLidar S3 节点
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': 1000000,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'DenseBoost'
            }],
            remappings=[('/scan', '/scan_raw')],
            output='screen'),
        
        # 2. 激光降采样节点
        Node(
            package='slam_gmapping',
            executable='laser_scan_decimator.py',
            name='laser_scan_decimator',
            parameters=[{'target_beams': 360}],
            output='screen'),
        
        # 3. 简单里程计发布器
        Node(
            package='slam_gmapping',
            executable='simple_odom_publisher.py',
            name='simple_odom_publisher',
            output='screen'),
        
        # 4. 静态 TF: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link']),
        
        # 5. 静态 TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser']),
        
        # 6. GMapping SLAM 节点
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            parameters=[slam_params_file],
            output='screen'),
        
        # 7. RViz2 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'),
    ])
```

### GMapping 参数配置

优化后的 `slam_params.yaml`：

```yaml
slam_gmapping:
  ros__parameters:
    # 坐标系
    base_frame: "base_link"
    odom_frame: "odom"
    map_frame: "map"
    
    # 激光参数
    maxUrange: 10.0      # 建图最大距离（米）
    maxRange: 40.0       # 雷达最大量程（米）
    max_beams: 60        # GMapping采样数
    
    # 粒子滤波
    particles: 30        # 粒子数量
    
    # 分辨率
    delta: 0.05          # 地图分辨率（米/像素）
    
    # 更新参数
    linearUpdate: 0.2    # 线性移动阈值（米）
    angularUpdate: 0.1   # 角度旋转阈值（弧度）
    temporalUpdate: 3.0  # 时间更新间隔（秒）
    
    # 里程计模型
    srr: 0.1            # 平移误差（相对平移）
    srt: 0.2            # 平移误差（相对旋转）
    str: 0.1            # 旋转误差（相对平移）
    stt: 0.2            # 旋转误差（相对旋转）
```

### 便捷启动脚本

创建 `start_gmapping_with_teleop.sh`：

```bash
#!/bin/bash

echo "=========================================="
echo "  启动 RPLidar S3 + GMapping 建图系统"
echo "=========================================="
echo ""

# 检查雷达连接
if [ ! -e "/dev/ttyUSB0" ]; then
    echo "⚠️  警告: 未检测到 /dev/ttyUSB0"
    echo "   请检查雷达是否连接"
    ls -l /dev/ttyUSB* 2>/dev/null || echo "   未找到任何 USB 串口设备"
    echo ""
fi

# 进入工作空间
cd /home/uav/rplidar_gmapping

# Source 环境
echo "📦 加载 ROS 2 环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动
echo ""
echo "🚀 启动建图系统..."
echo ""
echo "启动后："
echo "  1. 在新终端运行键盘控制："
echo "     source install/setup.bash"
echo "     ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "  2. 使用 i,j,k,l 控制移动进行建图"
echo ""
echo "  3. 建图完成后保存地图："
echo "     ros2 run nav2_map_server map_saver_cli -f my_map"
echo ""
echo "=========================================="
echo ""

# 启动 launch 文件
ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_teleop.launch.py
```

---

## 使用指南

### 快速开始

项目提供了两种启动方式，根据您的需求选择：

### 方式1：使用官方 laser_filters（推荐）

```bash
# 1. 启动建图系统（使用官方 laser_filters 进行数据滤波）
cd /path/to/workspace
source install/setup.bash
ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_laser_filters.launch.py

# 2. 新终端：启动键盘控制
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 3. 键盘控制建图
# i - 前进, , - 后退
# j - 左转, l - 右转, k - 停止

# 4. 保存地图
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 方式2：使用自定义降采样（简单）

```bash
# 1. 启动建图系统（使用自定义降采样节点）
cd /path/to/workspace
source install/setup.bash
ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_teleop.launch.py

# 2-4. 其他步骤同上
```

**两种方式的主要区别**：
- **方式1**：使用 ROS 官方的 `laser_filters` 包，提供更专业的滤波功能（范围过滤、离群点过滤等）
- **方式2**：使用自定义的简单降采样节点，配置更简单，适合快速测试

### 验证系统

```bash
# 检查所有节点
ros2 node list

# 检查话题
ros2 topic list

# 查看激光数据频率
ros2 topic hz /scan

# 查看里程计频率
ros2 topic hz /odom

# 查看地图更新（移动后）
ros2 topic hz /map

# 检查 TF 树
ros2 run tf2_tools view_frames
```

### 参数调优

根据实际场景调整参数：

**室内小场景：**
```yaml
maxUrange: 5.0
max_beams: 30
particles: 20
delta: 0.025
```

**室内大场景：**
```yaml
maxUrange: 10.0
max_beams: 60
particles: 30
delta: 0.05
```

**室外场景：**
```yaml
maxUrange: 15.0
max_beams: 90
particles: 50
delta: 0.05
```

---

## 方案对比与选择

### 降采样方案选择

| 方案 | 优点 | 缺点 | 推荐场景 |
|------|------|------|---------|
| **自定义降采样节点** | 简单直接，易于理解和修改 | 功能单一，无滤波 | 快速测试，学习使用 |
| **laser_filters官方包** | 专业稳定，功能丰富，C++高性能 | 配置复杂，学习曲线陡 | 生产环境，长期项目 |
| **两者结合** | 先滤波后降采样，效果最佳 | 系统复杂度增加 | 高质量建图需求 |

### 里程计方案选择

| 方案 | 适用场景 |
|------|---------|
| **静态里程计** | 系统测试、验证配置 |
| **简单里程计+键盘控制** | 学习SLAM、无底盘测试 |
| **真实机器人底盘** | 生产环境、实际应用 |

---

## 常见问题解决

### Q1: slam_gmapping 节点崩溃

**可能原因：**
1. `max_beams` 参数过大
2. 激光数据点数过多
3. 参数文件格式错误

**解决方法：**
1. 检查并降低 `max_beams` 到 60
2. 确保使用降采样节点
3. 验证 YAML 格式包含 `ros__parameters`

### Q2: 地图不更新

**可能原因：**
1. 缺少里程计数据
2. 机器人未移动
3. TF 树不完整

**解决方法：**
```bash
# 检查里程计
ros2 topic hz /odom  # 应该有数据

# 检查激光
ros2 topic hz /scan  # 应该有数据

# 检查速度命令（使用键盘控制时）
ros2 topic hz /cmd_vel  # 移动时应该有数据

# 检查 TF 树
ros2 run tf2_tools view_frames
```

### Q3: RViz 显示异常

**解决方法：**
1. 确认 Fixed Frame 设置为 `map`
2. 添加 LaserScan 显示，话题选择 `/scan`
3. 添加 Map 显示，话题选择 `/map`
4. 添加 TF 显示，查看坐标系关系

### Q4: 键盘控制无效

**解决方法：**
```bash
# 安装 teleop 工具
sudo apt install ros-humble-teleop-twist-keyboard

# 确保终端有焦点
# 检查是否有速度命令输出
ros2 topic echo /cmd_vel
```

---

## 技术总结

### 核心经验

1. **ROS 2 参数格式**：必须包含 `ros__parameters` 键，这是与 ROS 1 的重要区别

2. **激光数据处理**：高密度雷达需要降采样，不能仅依赖算法内部参数

3. **TF 树完整性**：SLAM 需要完整的 TF 树结构，缺少任何一环都会导致失败

4. **QoS 兼容性**：ROS 2 中不同 QoS 策略可能导致通信失败，需要注意兼容

5. **参数理解**：`max_beams` 等参数的真实含义需要深入理解，不能想当然

### 最佳实践

1. **分步验证**：先测试雷达，再测试 SLAM，再集成
2. **日志分析**：仔细阅读错误信息，理解问题根源
3. **参数调优**：从默认值开始，逐步调整优化
4. **文档完善**：记录问题和解决方案，便于后续参考
5. **模块化设计**：各功能节点独立，便于调试和替换

---

## 参考资源

### 官方文档

- [GMapping 论文](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf)
- [ROS 2 参数文档](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html)
- [laser_filters Wiki](https://wiki.ros.org/laser_filters)
- [laser_filters GitHub](https://github.com/ros-perception/laser_filters)

### 相关工具

- **RViz2**: 3D 可视化工具
- **teleop_twist_keyboard**: 键盘遥控
- **tf2_tools**: TF 调试工具
- **nav2_map_server**: 地图保存和加载

---

## 项目文件结构

```
rplidar_gmapping/
├── src/
│   └── slam_gmapping/
│       └── slam_gmapping/
│           ├── launch/
│           │   ├── slam_gmapping.launch.py
│           │   ├── bringup_rplidar_s3_gmapping.launch.py
│           │   └── bringup_rplidar_s3_gmapping_with_teleop.launch.py
│           ├── config/
│           │   ├── slam_params.yaml
│           │   └── laser_filter_params.yaml
│           ├── scripts/
│           │   ├── laser_scan_decimator.py
│           │   └── simple_odom_publisher.py
│           ├── rviz/
│           │   └── gmapping.rviz
│           ├── CMakeLists.txt
│           └── package.xml
├── start_gmapping_with_teleop.sh
└── README.md
```

---

## 结语

通过本项目的实践，我们成功构建了一个完整的 SLAM 建图系统，并解决了多个技术难题。这些经验对于 ROS 2 下的机器人开发具有重要参考价值。

主要收获：
- 深入理解了 ROS 2 的参数系统和 TF 树结构
- 掌握了激光雷达数据处理和降采样技术
- 学会了 GMapping 参数调优方法
- 积累了系统集成和问题排查经验

希望本文能够帮助到正在进行类似项目开发的同学们。

**项目状态**：✅ 已完成，系统稳定运行

**最后更新**：2025-10-31

