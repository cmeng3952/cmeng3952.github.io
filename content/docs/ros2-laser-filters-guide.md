---
title: "ROS 2 Laser Filters 使用指南"
weight: 10
---

# ROS 2 Laser Filters 使用指南

## 简介

`laser_filters` 是 ROS 官方维护的激光雷达数据滤波包，提供了多种专业的滤波器来清理和处理激光扫描数据。本文详细介绍如何在 ROS 2 Humble 环境下正确配置和使用 laser_filters。

## 安装

```bash
sudo apt install ros-humble-laser-filters
```

## 配置文件格式

### ROS 2 的特殊要求

ROS 2 的参数文件格式与 ROS 1 有重大差异。必须遵循以下结构：

```yaml
节点名称:
  ros__parameters:
    实际参数...
```

### 正确的 laser_filters 配置示例

文件：`laser_filter_params.yaml`

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

### 常见配置错误

❌ **错误示例 1：缺少 ros__parameters**
```yaml
scan_to_scan_filter_chain:
  filter1:  # 错误：缺少 ros__parameters
    name: range_filter
```

❌ **错误示例 2：使用列表格式**
```yaml
scan_filter_chain:  # 错误：不能直接用列表
  - name: range_filter
    type: laser_filters/LaserScanRangeFilter
```

## 主要滤波器详解

### 1. LaserScanRangeFilter - 距离过滤器

**功能**：过滤指定距离范围外的激光点

**参数说明**：
```yaml
filter_name:
  name: range_filter
  type: laser_filters/LaserScanRangeFilter
  params:
    use_message_range_limits: false  # 是否使用消息自带的范围限制
    lower_threshold: 0.2             # 最小距离（米）
    upper_threshold: 40.0            # 最大距离（米）
```

**应用场景**：
- 移除雷达盲区内的噪声点（过近）
- 移除超出有效范围的点（过远）
- 限制建图或导航的有效范围

**推荐配置**：
```yaml
# 室内环境
lower_threshold: 0.15
upper_threshold: 10.0

# 室外环境
lower_threshold: 0.5
upper_threshold: 40.0
```

### 2. LaserScanSpeckleFilter - 离群点过滤器

**功能**：移除孤立的噪声点和离群点

**参数说明**：
```yaml
filter_name:
  name: speckle_filter
  type: laser_filters/LaserScanSpeckleFilter
  params:
    filter_type: 0              # 0: 距离过滤, 1: 半径过滤
    max_range: 40.0             # 最大有效范围
    max_range_difference: 0.2   # 邻居点最大距离差
    filter_window: 2            # 检查的邻居数量
```

**filter_type 说明**：
- **0 (Distance)**: 检查相邻点之间的距离差异
- **1 (Radius)**: 检查点周围一定半径内的邻居数量

**应用场景**：
- 清理扫描数据中的随机噪声
- 移除由于反射产生的虚假点
- 提高边缘检测质量

**推荐配置**：
```yaml
# 一般场景（推荐）
filter_type: 0
max_range_difference: 0.1
filter_window: 2

# 噪声较多的环境
filter_type: 1
max_range_difference: 0.2
filter_window: 3
```

### 3. LaserScanAngularBoundsFilter - 角度范围过滤器

**功能**：只保留特定角度范围内的激光点

**参数说明**：
```yaml
filter_name:
  name: angular_bounds
  type: laser_filters/LaserScanAngularBoundsFilter
  params:
    lower_angle: -1.57   # 最小角度（弧度）
    upper_angle: 1.57    # 最大角度（弧度）
```

**应用场景**：
- 限制传感器视野（如只用前180度）
- 屏蔽特定方向的干扰
- 减少计算量

**常用角度**：
```yaml
# 前180度
lower_angle: -1.5708  # -90度
upper_angle: 1.5708   # +90度

# 前270度
lower_angle: -2.3562  # -135度
upper_angle: 2.3562   # +135度

# 全360度
lower_angle: -3.14159  # -180度
upper_angle: 3.14159   # +180度
```

### 4. LaserScanBoxFilter - 盒子过滤器

**功能**：移除指定3D盒子区域内的激光点

**参数说明**：
```yaml
filter_name:
  name: box_filter
  type: laser_filters/LaserScanBoxFilter
  params:
    box_frame: "base_link"  # 盒子坐标系
    min_x: -0.3             # 盒子最小 x
    max_x: 0.3              # 盒子最大 x
    min_y: -0.2             # 盒子最小 y
    max_y: 0.2              # 盒子最大 y
    min_z: 0.0              # 盒子最小 z
    max_z: 0.3              # 盒子最大 z
    invert: false           # false: 移除盒内点, true: 移除盒外点
```

**应用场景**：
- 过滤机器人本体产生的激光点
- 移除固定障碍物（如支撑柱）
- 定义感兴趣区域

**示例：过滤机器人本体**
```yaml
# 机器人尺寸 60cm x 40cm x 30cm
box_frame: "base_link"
min_x: -0.30
max_x: 0.30
min_y: -0.20
max_y: 0.20
min_z: -0.05
max_z: 0.30
invert: false  # 移除盒内的点
```

### 5. ScanShadowsFilter - 阴影过滤器

**功能**：移除由边缘效应产生的"阴影"噪声点

**参数说明**：
```yaml
filter_name:
  name: shadows
  type: laser_filters/ScanShadowsFilter
  params:
    min_angle: 5    # 最小角度（度）
    max_angle: 175  # 最大角度（度）
    neighbors: 20   # 检查的邻居数量
    window: 1       # 窗口大小
```

**应用场景**：
- 清理物体边缘的虚假点
- 提高障碍物检测精度
- 改善地图质量

### 6. LaserScanIntensityFilter - 强度过滤器

**功能**：基于激光强度值过滤点

**参数说明**：
```yaml
filter_name:
  name: intensity_filter
  type: laser_filters/LaserScanIntensityFilter
  params:
    lower_threshold: 100   # 最小强度值
    upper_threshold: 1000  # 最大强度值
    disp_histogram: 0      # 0: 不显示, 1: 显示强度直方图
```

**应用场景**：
- 过滤低反射率表面的噪声
- 移除过曝光的点
- 特定材质检测

## 实际应用配置

### 场景1：室内建图（推荐）

适合办公室、家庭等室内环境。

```yaml
scan_to_scan_filter_chain:
  ros__parameters:
    # 1. 距离过滤：保留 0.2-10 米范围
    filter1:
      name: range
      type: laser_filters/LaserScanRangeFilter
      params:
        use_message_range_limits: false
        lower_threshold: 0.2
        upper_threshold: 10.0
    
    # 2. 离群点过滤：清理噪声
    filter2:
      name: speckle
      type: laser_filters/LaserScanSpeckleFilter
      params:
        filter_type: 0
        max_range: 10.0
        max_range_difference: 0.1
        filter_window: 2
    
    # 3. 机器人本体过滤（可选）
    filter3:
      name: box
      type: laser_filters/LaserScanBoxFilter
      params:
        box_frame: "base_link"
        min_x: -0.25
        max_x: 0.25
        min_y: -0.15
        max_y: 0.15
        min_z: 0.0
        max_z: 0.2
        invert: false
```

### 场景2：室外导航

适合停车场、广场等开阔环境。

```yaml
scan_to_scan_filter_chain:
  ros__parameters:
    # 1. 距离过滤：保留 0.5-40 米
    filter1:
      name: range
      type: laser_filters/LaserScanRangeFilter
      params:
        use_message_range_limits: false
        lower_threshold: 0.5
        upper_threshold: 40.0
    
    # 2. 离群点过滤：更宽松的参数
    filter2:
      name: speckle
      type: laser_filters/LaserScanSpeckleFilter
      params:
        filter_type: 1
        max_range: 40.0
        max_range_difference: 0.5
        filter_window: 3
```

### 场景3：高精度建图

需要最佳地图质量的场景。

```yaml
scan_to_scan_filter_chain:
  ros__parameters:
    filter1:
      name: range
      type: laser_filters/LaserScanRangeFilter
      params:
        use_message_range_limits: false
        lower_threshold: 0.15
        upper_threshold: 15.0
    
    filter2:
      name: speckle
      type: laser_filters/LaserScanSpeckleFilter
      params:
        filter_type: 0
        max_range: 15.0
        max_range_difference: 0.05
        filter_window: 3
    
    filter3:
      name: shadows
      type: laser_filters/ScanShadowsFilter
      params:
        min_angle: 10
        max_angle: 170
        neighbors: 20
        window: 1
    
    filter4:
      name: box
      type: laser_filters/LaserScanBoxFilter
      params:
        box_frame: "base_link"
        min_x: -0.30
        max_x: 0.30
        min_y: -0.20
        max_y: 0.20
        min_z: 0.0
        max_z: 0.25
        invert: false
```

## Launch 文件集成

### 基本启动

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 激光雷达节点
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            remappings=[('/scan', '/scan_raw')],  # 输出到 scan_raw
            output='screen'),
        
        # laser_filters 节点
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[{
                'config_file': 'path/to/laser_filter_params.yaml'
            }],
            remappings=[
                ('/scan', '/scan_raw'),      # 输入：原始数据
                ('/scan_filtered', '/scan')   # 输出：过滤后数据
            ],
            output='screen'),
    ])
```

### 完整示例（带 GMapping）

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 配置文件路径
    filter_params = os.path.join(
        get_package_share_directory('your_package'),
        'config', 'laser_filter_params.yaml')
    
    slam_params = os.path.join(
        get_package_share_directory('your_package'),
        'config', 'slam_params.yaml')
    
    return LaunchDescription([
        # 1. 雷达节点
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser'
            }],
            remappings=[('/scan', '/scan_raw')],
            output='screen'),
        
        # 2. laser_filters 节点
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[filter_params],
            remappings=[
                ('/scan', '/scan_raw'),
                ('/scan_filtered', '/scan')
            ],
            output='screen'),
        
        # 3. GMapping 节点
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            parameters=[slam_params],
            output='screen'),
    ])
```

## QoS 兼容性问题

### 问题说明

- `laser_filters` 默认使用 **Reliable** QoS
- 很多 SLAM 算法（如 GMapping）使用 **Best Effort** QoS（SensorDataQoS）
- QoS 不匹配会导致数据无法接收

### 解决方案

添加 QoS 桥接节点：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class QoSBridge(Node):
    def __init__(self):
        super().__init__('laser_scan_qos_bridge')
        
        # 订阅：Reliable QoS
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        
        self.sub = self.create_subscription(
            LaserScan, '/scan_filtered',
            self.callback, reliable_qos)
        
        # 发布：Best Effort QoS
        from rclpy.qos import qos_profile_sensor_data
        self.pub = self.create_publisher(
            LaserScan, '/scan',
            qos_profile_sensor_data)
    
    def callback(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = QoSBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 验证和调试

### 检查滤波效果

```bash
# 查看原始数据点数
ros2 topic echo /scan_raw --once | grep -c "ranges:"

# 查看过滤后数据点数
ros2 topic echo /scan --once | grep -c "ranges:"

# 比较频率
ros2 topic hz /scan_raw
ros2 topic hz /scan

# 查看节点信息
ros2 node info /laser_filter
```

### RViz 可视化对比

```python
# 在 RViz 中同时添加：
# 1. LaserScan - /scan_raw (红色)
# 2. LaserScan - /scan (白色)
# 
# 对比查看滤波效果
```

### 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|---------|---------|
| 节点启动后崩溃 | YAML 格式错误 | 检查 `ros__parameters` 键 |
| `/scan` 无数据 | QoS 不匹配 | 添加 QoS 桥接节点 |
| 过滤过度，点太少 | 阈值设置过严 | 放宽 `max_range_difference` |
| 噪声仍然很多 | 滤波器未生效 | 检查参数和滤波器链 |

## 性能优化

### 滤波器顺序优化

推荐的滤波器应用顺序：

1. **Range Filter** - 首先过滤距离，减少后续处理量
2. **Box Filter** - 移除机器人本体
3. **Speckle Filter** - 清理离群点
4. **Shadows Filter** - 处理边缘效应
5. **Intensity Filter** - 最后基于强度过滤

### 计算量考虑

```yaml
# 轻量级配置（低算力设备）
filter_window: 1
neighbors: 10

# 标准配置
filter_window: 2
neighbors: 20

# 高精度配置（高算力）
filter_window: 3
neighbors: 30
```

## 总结

### 关键要点

1. ✅ ROS 2 配置必须包含 `ros__parameters`
2. ✅ 使用 `filter1`, `filter2` 等键组织多个滤波器
3. ✅ 注意 QoS 兼容性问题
4. ✅ 根据实际场景选择合适的滤波器组合
5. ✅ 使用 RViz 验证滤波效果

### 推荐实践

- **新手**：使用 Range + Speckle 两个基本滤波器
- **进阶**：添加 Box Filter 过滤机器人本体
- **专家**：根据具体需求组合多个滤波器

### 参考资源

- [laser_filters ROS Wiki](https://wiki.ros.org/laser_filters)
- [laser_filters GitHub](https://github.com/ros-perception/laser_filters)
- [ROS 2 QoS 文档](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

---

**文档版本**: 1.0  
**适用 ROS 版本**: ROS 2 Humble  
**最后更新**: 2025-10-31

