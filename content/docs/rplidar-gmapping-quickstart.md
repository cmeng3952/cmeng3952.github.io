---
title: "RPLidar + GMapping 快速开始"
weight: 5
---

# RPLidar + GMapping 快速开始

## 30秒快速开始 🚀

已经配置好环境？直接开始建图：

### 方式1：使用官方 laser_filters（推荐）

```bash
# 一键启动（使用官方 laser_filters 滤波）
cd /path/to/workspace
source install/setup.bash
ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_laser_filters.launch.py

# 新终端：键盘控制
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 建图完成后保存
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 方式2：使用自定义降采样（简单快速）

```bash
# 一键启动（使用自定义降采样节点）
cd /path/to/workspace
source install/setup.bash
ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_teleop.launch.py

# 新终端：键盘控制
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**两种方式的区别**：
- **方式1**：使用 ROS 官方的 `laser_filters` 包进行专业数据滤波（需要安装 `ros-humble-laser-filters`）
- **方式2**：使用自定义的简单降采样节点，配置更简单

## 详细步骤

### 步骤1: 环境准备 ✅

#### 1.1 系统要求

- **操作系统**: Ubuntu 22.04
- **ROS版本**: ROS 2 Humble
- **硬件**: 思岚 RPLidar S3（或其他型号）

#### 1.2 安装依赖

```bash
# 更新系统
sudo apt update

# 安装 ROS 2 Humble（如未安装）
# 参考：https://docs.ros.org/en/humble/Installation.html

# 安装必要的包
sudo apt install -y \
    ros-humble-slam-gmapping \
    ros-humble-teleop-twist-keyboard \
    ros-humble-nav2-map-server \
    ros-humble-rviz2 \
    ros-humble-laser-filters
```

**注意**：`ros-humble-laser-filters` 是使用官方滤波功能所必需的。如果您选择使用自定义降采样方式，这个包不是必需的。

#### 1.3 克隆和编译项目

```bash
# 创建工作空间
mkdir -p ~/rplidar_ws/src
cd ~/rplidar_ws/src

# 克隆 rplidar_ros
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# 克隆 slam_gmapping
git clone -b ros2 https://github.com/Project-MANAS/slam_gmapping.git

# 返回工作空间根目录
cd ~/rplidar_ws

# 编译
colcon build

# Source 环境
source install/setup.bash
echo "source ~/rplidar_ws/install/setup.bash" >> ~/.bashrc
```

---

### 步骤2: 硬件连接 🔌

#### 2.1 连接雷达

1. 通过 USB 线连接思岚雷达到计算机
2. 等待系统识别设备

#### 2.2 检查设备

```bash
# 查看串口设备
ls -l /dev/ttyUSB*

# 通常显示：
# /dev/ttyUSB0
```

#### 2.3 设置权限

```bash
# 临时方法（立即生效）
sudo chmod 666 /dev/ttyUSB0

# 永久方法（推荐）
sudo usermod -a -G dialout $USER
# 然后注销重新登录
```

#### 2.4 测试雷达

```bash
# 单独测试雷达
ros2 launch rplidar_ros rplidar_s3_launch.py

# 应该看到：
# [INFO] RPLidar health status: OK
# [INFO] current scan mode: DenseBoost

# 查看雷达数据
ros2 topic echo /scan
```

按 `Ctrl+C` 停止测试。

---

### 步骤3: 配置文件 📝

#### 3.1 创建配置目录

```bash
cd ~/rplidar_ws/src/slam_gmapping/slam_gmapping
mkdir -p config launch rviz scripts
```

#### 3.2 创建 GMapping 参数文件

文件：`config/slam_params.yaml`

```yaml
slam_gmapping:
  ros__parameters:
    # 坐标系
    base_frame: "base_link"
    odom_frame: "odom"
    map_frame: "map"
    
    # 激光参数
    maxUrange: 10.0
    maxRange: 40.0
    max_beams: 60
    
    # 粒子滤波
    particles: 30
    
    # 地图参数
    delta: 0.05
    xmin: -50.0
    ymin: -50.0
    xmax: 50.0
    ymax: 50.0
    
    # 运动模型
    srr: 0.1
    srt: 0.2
    str: 0.1
    stt: 0.2
    
    # 更新参数
    linearUpdate: 0.2
    angularUpdate: 0.1
    temporalUpdate: 3.0
```

#### 3.3 创建降采样节点

文件：`scripts/laser_scan_decimator.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class LaserScanDecimator(Node):
    def __init__(self):
        super().__init__('laser_scan_decimator')
        
        self.declare_parameter('target_beams', 360)
        self.target_beams = self.get_parameter('target_beams').value
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_raw',
            self.scan_callback,
            qos_profile_sensor_data)
        
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan',
            qos_profile_sensor_data)
        
        self.get_logger().info(
            f'Laser Scan Decimator started, target: {self.target_beams} beams')
    
    def scan_callback(self, scan_msg):
        original_num = len(scan_msg.ranges)
        
        if original_num <= self.target_beams:
            self.scan_pub.publish(scan_msg)
            return
        
        step = int(np.ceil(original_num / self.target_beams))
        
        new_scan = LaserScan()
        new_scan.header = scan_msg.header
        new_scan.angle_min = scan_msg.angle_min
        new_scan.angle_max = scan_msg.angle_max
        new_scan.angle_increment = scan_msg.angle_increment * step
        new_scan.time_increment = scan_msg.time_increment * step
        new_scan.scan_time = scan_msg.scan_time
        new_scan.range_min = scan_msg.range_min
        new_scan.range_max = scan_msg.range_max
        new_scan.ranges = list(scan_msg.ranges[::step])
        new_scan.intensities = list(scan_msg.intensities[::step])
        
        self.scan_pub.publish(new_scan)

def main():
    rclpy.init()
    node = LaserScanDecimator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
# 添加执行权限
chmod +x scripts/laser_scan_decimator.py
```

#### 3.4 创建简单里程计节点

文件：`scripts/simple_odom_publisher.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class SimpleOdomPublisher(Node):
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vth = 0.0
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Simple Odometry Publisher started')
    
    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z
    
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_theta = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = self.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)
        self.last_time = current_time
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        return [
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy,  # z
            cr * cp * cy + sr * sp * sy   # w
        ]

def main():
    rclpy.init()
    node = SimpleOdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
chmod +x scripts/simple_odom_publisher.py
```

#### 3.5 创建 Launch 文件

**重要说明**：项目中有多个 launch 文件，选择适合您的版本。

##### 选项A：使用官方 laser_filters（推荐，功能强大）

文件：`launch/bringup_rplidar_s3_gmapping_with_laser_filters.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    
    slam_params_file = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'config', 'slam_params.yaml')
    
    filter_params_file = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'config', 'laser_filter_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='RPLidar serial port'),
        
        # RPLidar node
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
        
        # Official laser_filters
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
        
        # Simple odometry
        Node(
            package='slam_gmapping',
            executable='simple_odom_publisher.py',
            name='simple_odom_publisher',
            output='screen'),
        
        # Static TF: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link']),
        
        # Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser']),
        
        # GMapping SLAM
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            parameters=[slam_params_file],
            output='screen'),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])
```

**还需要创建**：`config/laser_filter_params.yaml`

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

##### 选项B：使用自定义降采样（简单快速）

文件：`launch/bringup_rplidar_s3_gmapping_with_teleop.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    
    slam_params_file = os.path.join(
        get_package_share_directory('slam_gmapping'),
        'config', 'slam_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='RPLidar serial port'),
        
        # RPLidar node
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
        
        # Laser decimator
        Node(
            package='slam_gmapping',
            executable='laser_scan_decimator.py',
            name='laser_scan_decimator',
            parameters=[{'target_beams': 360}],
            output='screen'),
        
        # Simple odometry
        Node(
            package='slam_gmapping',
            executable='simple_odom_publisher.py',
            name='simple_odom_publisher',
            output='screen'),
        
        # Static TF: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link']),
        
        # Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser']),
        
        # GMapping SLAM
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            parameters=[slam_params_file],
            output='screen'),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])
```

#### 3.6 更新 CMakeLists.txt

在 `CMakeLists.txt` 中添加：

```cmake
# 安装 Python 脚本
install(PROGRAMS
  scripts/laser_scan_decimator.py
  scripts/simple_odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安装 rviz 配置
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
```

#### 3.7 重新编译

```bash
cd ~/rplidar_ws
colcon build --packages-select slam_gmapping
source install/setup.bash
```

---

### 步骤4: 启动建图 🗺️

#### 4.1 启动系统

```bash
# 终端1：启动建图系统
cd ~/rplidar_ws
source install/setup.bash
ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_teleop.launch.py
```

应该看到：
- ✅ RPLidar 正常连接
- ✅ 所有节点启动
- ✅ RViz 窗口打开

#### 4.2 配置 RViz

在 RViz 中：

1. **设置 Fixed Frame**: `map`

2. **添加显示**：
   - 点击 "Add" → "By topic"
   - 添加 `/scan` (LaserScan) - 激光点云
   - 添加 `/map` (Map) - 地图
   - 添加 "TF" - 坐标系

3. **调整显示**：
   - LaserScan: Size = 0.05, Color = White
   - Map: Alpha = 0.7

#### 4.3 启动键盘控制

```bash
# 终端2：键盘控制
cd ~/rplidar_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键盘控制说明：
```
移动：
  i - 前进
  , - 后退
  j - 左转
  l - 右转
  k - 停止

速度调整：
  q/z - 增加/减少最大速度
  w/x - 增加/减少当前线速度
  e/c - 增加/减少当前角速度
```

#### 4.4 开始建图

1. **缓慢移动**：使用键盘控制缓慢移动
2. **观察地图**：在 RViz 中实时查看地图生成
3. **覆盖区域**：移动到所有需要建图的区域
4. **避免快速转向**：保持平稳运动

**建图技巧**：
- ✅ 速度不要太快（< 0.3 m/s）
- ✅ 多次经过同一区域（闭环）
- ✅ 确保环境有足够特征
- ❌ 避免空旷无特征区域
- ❌ 避免快速旋转

---

### 步骤5: 保存地图 💾

#### 5.1 保存地图文件

```bash
# 终端3：保存地图
cd ~/rplidar_ws
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

生成的文件：
- `my_map.pgm` - 地图图像（灰度图）
- `my_map.yaml` - 地图配置文件

#### 5.2 查看地图

```bash
# 使用图像查看器
eog my_map.pgm

# 或使用其他工具
gimp my_map.pgm
```

地图颜色含义：
- **白色** (255): 自由空间（可通行）
- **黑色** (0): 障碍物
- **灰色** (128): 未知区域

#### 5.3 地图参数

`my_map.yaml` 文件内容：
```yaml
image: my_map.pgm
resolution: 0.05        # 5cm/像素
origin: [-10.0, -10.0, 0.0]  # 地图原点
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

---

## 验证和调试 🔍

### 检查系统状态

```bash
# 查看所有节点
ros2 node list

# 应该看到：
# /rplidar_node
# /laser_scan_decimator
# /simple_odom_publisher
# /slam_gmapping
# /rviz2

# 查看所有话题
ros2 topic list

# 应该包括：
# /scan_raw
# /scan
# /odom
# /map
# /tf

# 查看话题频率
ros2 topic hz /scan      # 应该 ~10 Hz
ros2 topic hz /odom      # 应该 ~50 Hz
ros2 topic hz /map       # 移动后 >0 Hz

# 查看 TF 树
ros2 run tf2_tools view_frames
# 生成 frames.pdf，应该显示完整的 TF 树
```

### 常见问题排查

#### 问题1：雷达无法连接

```bash
# 检查设备
ls -l /dev/ttyUSB*

# 检查权限
sudo chmod 666 /dev/ttyUSB0

# 检查雷达健康状态
ros2 topic echo /rplidar_node/health
```

#### 问题2：没有地图更新

```bash
# 检查激光数据
ros2 topic echo /scan --once

# 检查里程计数据
ros2 topic echo /odom --once

# 检查速度命令（移动时）
ros2 topic echo /cmd_vel
```

**解决方法**：
1. 确保键盘控制终端有焦点
2. 确保实际移动了机器人
3. 等待几秒让系统初始化

#### 问题3：RViz 显示异常

- 检查 Fixed Frame 是否设置为 `map`
- 重新添加显示组件
- 重启 RViz

---

## 下一步 🎓

### 学习进阶主题

1. **参数调优** → [GMapping 参数调优指南](/docs/gmapping-parameters-tuning/)
2. **激光滤波** → [Laser Filters 使用指南](/docs/ros2-laser-filters-guide/)
3. **里程计系统** → [里程计系统详解](/docs/ros2-odometry-system/)
4. **完整项目** → [RPLidar S3 + GMapping 项目](/posts/rplidar-s3-gmapping-slam/)

### 实际应用

- 使用真实机器人底盘替换简单里程计
- 集成 Nav2 进行自主导航
- 结合 IMU 提高精度
- 多层地图建图

---

## 快速参考 📋

### 常用命令

```bash
# 启动建图
ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_teleop.launch.py

# 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 保存地图
ros2 run nav2_map_server map_saver_cli -f map_name

# 查看话题
ros2 topic list
ros2 topic hz /topic_name
ros2 topic echo /topic_name

# 查看节点
ros2 node list
ros2 node info /node_name

# 查看 TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo parent_frame child_frame
```

### 文件位置

```
~/rplidar_ws/
└── src/slam_gmapping/slam_gmapping/
    ├── config/
    │   └── slam_params.yaml
    ├── launch/
    │   └── bringup_rplidar_s3_gmapping_with_teleop.launch.py
    └── scripts/
        ├── laser_scan_decimator.py
        └── simple_odom_publisher.py
```

---

**祝你建图成功！** 🎉

有问题？查看 [故障排查指南](/docs/troubleshooting-guide/) 或 [完整项目文档](/posts/rplidar-s3-gmapping-slam/)。

---

**文档版本**: 1.0  
**最后更新**: 2025-10-31

