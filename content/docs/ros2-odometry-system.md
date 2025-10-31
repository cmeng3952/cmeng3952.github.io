---
title: "ROS 2 里程计系统详解"
weight: 11
---

# ROS 2 里程计系统详解

## 概述

里程计（Odometry）是移动机器人定位的基础，提供机器人相对于起始位置的位姿估计。本文详细介绍 ROS 2 中里程计系统的工作原理、实现方法和最佳实践。

## 里程计在 SLAM 中的作用

### 为什么 SLAM 需要里程计？

```
激光雷达 ──→ 环境观测（Where am I relative to landmarks）
                 ↓
里程计 ──────→ 运动估计（How did I move）
                 ↓
          ┌─────────────┐
          │ SLAM 融合   │
          │ 位姿估计    │
          └─────────────┘
                 ↓
            准确的地图
```

### TF 树结构

完整的机器人 TF 树：

```
map (全局地图坐标系)
  └── odom (里程计坐标系) ← SLAM算法发布
       └── base_footprint (机器人底盘) ← 里程计发布
            └── base_link (机器人基座)
                 ├── laser (激光雷达)
                 ├── camera (相机)
                 └── imu (惯性传感器)
```

**各坐标系说明**：

- **map**: 全局坐标系，固定不变
- **odom**: 里程计坐标系，连续平滑但可能漂移
- **base_footprint**: 机器人在地面的投影
- **base_link**: 机器人中心
- **laser/camera/imu**: 各传感器坐标系

**谁发布什么**：

- `里程计节点` 发布：`odom → base_footprint`
- `SLAM节点` 发布：`map → odom`
- `static_transform_publisher` 发布：其他静态变换

## 里程计消息格式

### nav_msgs/Odometry 消息结构

```python
# Header
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id  # "odom"

# 子坐标系
string child_frame_id  # "base_footprint" 或 "base_link"

# 位姿（位置 + 姿态）
geometry_msgs/PoseWithCovariance pose
  Pose pose
    Point position    # x, y, z (米)
    Quaternion orientation  # 四元数表示姿态
  float64[36] covariance  # 6x6 协方差矩阵

# 速度（线速度 + 角速度）
geometry_msgs/TwistWithCovariance twist
  Twist twist
    Vector3 linear    # vx, vy, vz (米/秒)
    Vector3 angular   # wx, wy, wz (弧度/秒)
  float64[36] covariance  # 6x6 协方差矩阵
```

### 示例消息

```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: "odom"
child_frame_id: "base_footprint"

pose:
  pose:
    position:
      x: 1.5
      y: 0.8
      z: 0.0
    orientation:  # 旋转45度
      x: 0.0
      y: 0.0
      z: 0.38268343  # sin(45°/2)
      w: 0.92387953  # cos(45°/2)
  covariance: [0.01, 0, 0, 0, 0, 0,
               0, 0.01, 0, 0, 0, 0,
               0, 0, 0.01, 0, 0, 0,
               0, 0, 0, 0.01, 0, 0,
               0, 0, 0, 0, 0.01, 0,
               0, 0, 0, 0, 0, 0.01]

twist:
  twist:
    linear:
      x: 0.2   # 前进 0.2 m/s
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.1   # 左转 0.1 rad/s
  covariance: [...]  # 同上
```

## 里程计实现方法

### 方法1: 轮式里程计（最常见）

#### 原理

基于车轮编码器测量：

```
距离 = 轮子转数 × 轮子周长
速度 = 轮子转速 × 轮子周长
```

#### 差分驱动模型

```python
# 差分驱动机器人（两轮独立驱动）
class DifferentialDriveOdometry:
    def __init__(self, wheel_base, wheel_radius):
        self.wheel_base = wheel_base      # 两轮间距（米）
        self.wheel_radius = wheel_radius  # 车轮半径（米）
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # 航向角
    
    def update(self, left_ticks, right_ticks, dt):
        """
        left_ticks: 左轮编码器增量
        right_ticks: 右轮编码器增量
        dt: 时间间隔（秒）
        """
        # 计算车轮移动距离
        left_distance = (left_ticks / ENCODER_TICKS_PER_REV) * \
                       (2 * pi * self.wheel_radius)
        right_distance = (right_ticks / ENCODER_TICKS_PER_REV) * \
                        (2 * pi * self.wheel_radius)
        
        # 计算机器人运动
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        # 更新位姿（使用中点积分）
        if abs(delta_theta) < 1e-6:
            # 直线运动
            self.x += distance * cos(self.theta)
            self.y += distance * sin(self.theta)
        else:
            # 弧线运动
            radius = distance / delta_theta
            self.x += radius * (sin(self.theta + delta_theta) - sin(self.theta))
            self.y += radius * (cos(self.theta) - cos(self.theta + delta_theta))
        
        self.theta += delta_theta
        
        # 计算速度
        v = distance / dt  # 线速度
        w = delta_theta / dt  # 角速度
        
        return self.x, self.y, self.theta, v, w
```

#### 完整的轮式里程计节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster
import math

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        
        # 参数
        self.declare_parameter('wheel_base', 0.3)  # 米
        self.declare_parameter('wheel_radius', 0.05)  # 米
        self.declare_parameter('encoder_resolution', 1024)  # ticks/rev
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        
        # 状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = self.get_clock().now()
        
        # 发布者
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅者（假设编码器发布 [left_ticks, right_ticks]）
        self.encoder_sub = self.create_subscription(
            Int32MultiArray, '/encoders',
            self.encoder_callback, 10)
        
        self.get_logger().info('Wheel Odometry node started')
    
    def encoder_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt < 1e-6:
            return
        
        # 获取编码器数据
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]
        
        # 计算增量
        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks
        
        # 计算距离
        left_distance = (delta_left / self.encoder_resolution) * \
                       (2 * math.pi * self.wheel_radius)
        right_distance = (delta_right / self.encoder_resolution) * \
                        (2 * math.pi * self.wheel_radius)
        
        # 计算运动
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base
        
        # 更新位姿
        self.x += distance * math.cos(self.theta + delta_theta / 2.0)
        self.y += distance * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        
        # 计算速度
        vx = distance / dt
        vy = 0.0
        vth = delta_theta / dt
        
        # 发布里程计
        self.publish_odometry(current_time, vx, vy, vth)
        
        # 更新状态
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time
    
    def publish_odometry(self, current_time, vx, vy, vth):
        # 创建四元数
        q = self.quaternion_from_euler(0, 0, self.theta)
        
        # 发布 TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        # 设置协方差（简化版）
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.01  # theta
        
        self.odom_pub.publish(odom)
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0, 0, 0, 0]
        q[3] = cr * cp * cy + sr * sp * sy  # w
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        
        return q

def main():
    rclpy.init()
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 方法2: IMU 里程计

#### 原理

使用惯性测量单元（IMU）：
- **加速度计**：测量线性加速度，二次积分得位置
- **陀螺仪**：测量角速度，积分得姿态

#### 实现

```python
class IMUOdometry(Node):
    def __init__(self):
        super().__init__('imu_odometry')
        
        # 状态
        self.vx = 0.0
        self.vy = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 订阅 IMU
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data',
            self.imu_callback, 10)
        
        # 发布里程计
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.last_time = self.get_clock().now()
    
    def imu_callback(self, imu_msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # 获取加速度（需要去除重力）
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        
        # 获取角速度
        wz = imu_msg.angular_velocity.z
        
        # 积分计算速度
        self.vx += ax * dt
        self.vy += ay * dt
        self.theta += wz * dt
        
        # 积分计算位置
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        
        # 发布
        self.publish_odometry(current_time)
        self.last_time = current_time
```

**注意**：IMU 里程计漂移严重，通常需要与其他传感器融合。

### 方法3: 简单里程计（测试用）

用于没有真实传感器时的测试：

```python
class SimpleOdometry(Node):
    """基于速度命令的简单里程计模拟"""
    
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        # 状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        
        # 订阅速度命令
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel',
            self.cmd_vel_callback, 10)
        
        # 发布里程计
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 定时器（50 Hz）
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.last_time = self.get_clock().now()
    
    def cmd_vel_callback(self, msg):
        """接收速度命令"""
        self.vx = msg.linear.x
        self.vth = msg.angular.z
    
    def timer_callback(self):
        """定期更新位姿"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # 更新位置（简单积分）
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # 发布
        self.publish_odometry(current_time)
        self.last_time = current_time
```

**特点**：
- ✅ 简单易用，适合快速测试
- ❌ 不反映真实物理，精度低
- 💡 适合模拟、演示、学习

## 里程计误差与校准

### 常见误差源

1. **系统误差**（可校准）：
   - 车轮直径测量不准
   - 轮距测量不准
   - 编码器分辨率偏差
   - 地面打滑

2. **随机误差**（难以消除）：
   - 地面不平
   - 车轮磨损
   - 负载变化

### 校准方法

#### 直线校准

```python
# 1. 让机器人直线行驶固定距离（如5米）
# 2. 测量实际距离
# 3. 计算校准系数

actual_distance = 5.0  # 米
measured_distance = 4.85  # 里程计测量值

wheel_radius_corrected = wheel_radius * (actual_distance / measured_distance)
```

#### 旋转校准

```python
# 1. 让机器人原地旋转 360 度
# 2. 测量实际角度
# 3. 计算校准系数

actual_angle = 2 * pi
measured_angle = 2.1  # 里程计测量值

wheel_base_corrected = wheel_base * (measured_angle / actual_angle)
```

## 多传感器融合

### 扩展卡尔曼滤波（EKF）

使用 `robot_localization` 包融合多个传感器：

```yaml
# ekf_params.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,  # x, y, z
                   false, false, true,   # roll, pitch, yaw
                   true,  true,  false,  # vx, vy, vz
                   false, false, true,   # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az
    
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  true]
```

### Launch 文件

```python
Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[ekf_params_file],
    remappings=[('odometry/filtered', '/odom')]
)
```

## 最佳实践

### 1. 协方差设置

准确设置协方差矩阵：

```python
# 位置协方差（示例）
odom.pose.covariance[0] = 0.001  # x 方差 (m²)
odom.pose.covariance[7] = 0.001  # y 方差
odom.pose.covariance[35] = 0.01  # yaw 方差 (rad²)

# 速度协方差
odom.twist.covariance[0] = 0.01  # vx 方差 ((m/s)²)
odom.twist.covariance[35] = 0.01  # vyaw 方差 ((rad/s)²)
```

### 2. 时间戳同步

确保时间戳准确：

```python
# 使用当前时间
odom.header.stamp = self.get_clock().now().to_msg()

# 或使用传感器时间戳
odom.header.stamp = encoder_msg.header.stamp
```

### 3. 坐标系规范

遵循 REP-105 规范：

- `odom` 是父坐标系
- `base_footprint` 或 `base_link` 是子坐标系
- 使用右手坐标系（x前，y左，z上）

### 4. 发布频率

```python
# 推荐频率
里程计消息: 50-100 Hz
TF 变换: 50-100 Hz (与消息同步)
```

### 5. 重置功能

提供重置服务：

```python
from std_srvs.srv import Empty

class OdometryNode(Node):
    def __init__(self):
        # ...
        self.reset_srv = self.create_service(
            Empty, '/reset_odometry',
            self.reset_callback)
    
    def reset_callback(self, request, response):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info('Odometry reset')
        return response
```

## 调试工具

### 查看里程计数据

```bash
# 查看话题
ros2 topic echo /odom

# 查看频率
ros2 topic hz /odom

# 查看 TF 树
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_footprint

# RViz 可视化
ros2 run rviz2 rviz2
# 添加 Odometry 和 TF 显示
```

### 绘制轨迹

```python
# 在 RViz 中添加 Path 显示
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class OdometryWithPath(OdometryNode):
    def __init__(self):
        super().__init__()
        self.path_pub = self.create_publisher(Path, '/odom_path', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'
    
    def publish_odometry(self, current_time, vx, vy, vth):
        super().publish_odometry(current_time, vx, vy, vth)
        
        # 添加到路径
        pose = PoseStamped()
        pose.header = self.path.header
        pose.header.stamp = current_time.to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation = self.quaternion_from_euler(0, 0, self.theta)
        
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
```

## 总结

### 关键要点

1. ✅ 里程计是 SLAM 的必需输入
2. ✅ 需要同时发布 `/odom` 话题和 TF 变换
3. ✅ 轮式里程计最常用，但需要校准
4. ✅ 多传感器融合可提高精度
5. ✅ 协方差矩阵影响融合效果

### 方案选择

| 场景 | 推荐方案 |
|------|---------|
| **真实机器人** | 轮式里程计 + IMU 融合 |
| **学习测试** | 简单里程计 + 键盘控制 |
| **高精度需求** | 轮式 + IMU + 视觉里程计 |
| **快速验证** | 静态 TF（仅测试系统） |

### 参考资源

- [REP-105: 坐标系规范](https://www.ros.org/reps/rep-0105.html)
- [robot_localization 文档](http://docs.ros.org/en/humble/p/robot_localization/)
- [nav_msgs/Odometry 消息](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)

---

**文档版本**: 1.0  
**适用 ROS 版本**: ROS 2 Humble  
**最后更新**: 2025-10-31

