---
title: "ROS 2 QoS 服务质量详解"
weight: 13
---

# ROS 2 QoS (Quality of Service) 服务质量详解

## 概述

QoS (Quality of Service，服务质量) 是 ROS 2 相对于 ROS 1 的重要改进之一。它允许开发者根据网络条件和应用需求配置节点间的通信特性。

本文基于实际项目中遇到的 QoS 兼容性问题，详细介绍 ROS 2 QoS 的概念、配置和最佳实践。

---

## 为什么需要 QoS？

### ROS 1 的局限性

ROS 1 使用 TCP 作为默认传输协议，所有通信都是"可靠"的：
- ✅ 保证消息送达
- ✅ 保证顺序
- ❌ 网络波动时延迟增加
- ❌ 实时性差
- ❌ 不适合无线网络

### ROS 2 的改进

ROS 2 基于 DDS (Data Distribution Service)，提供灵活的 QoS 配置：
- ✅ 可根据需求选择可靠性
- ✅ 支持实时通信
- ✅ 适应各种网络环境
- ✅ 优化资源使用

---

## QoS 核心概念

### 1. Reliability（可靠性）

**定义**：消息传输的可靠程度

#### RELIABLE（可靠）
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.RELIABLE
```

**特点**：
- ✅ 保证消息送达
- ✅ 有重传机制
- ❌ 延迟可能增加
- ❌ 网络负载较高

**适用场景**：
- 导航指令
- 服务调用
- 重要的状态更新
- 配置参数

#### BEST_EFFORT（尽力而为）
```python
qos.reliability = ReliabilityPolicy.BEST_EFFORT
```

**特点**：
- ✅ 低延迟
- ✅ 低网络负载
- ❌ 可能丢包
- ❌ 不保证送达

**适用场景**：
- 传感器数据（激光雷达、相机）
- 里程计数据
- 高频率数据流
- 可以容忍偶尔丢失的数据

---

### 2. Durability（持久性）

**定义**：订阅者在发布者之后加入时，是否接收之前的消息

#### VOLATILE（易失）
```python
from rclpy.qos import DurabilityPolicy

qos.durability = DurabilityPolicy.VOLATILE
```

**特点**：
- 只接收订阅后的消息
- 不保存历史消息
- 默认行为

#### TRANSIENT_LOCAL（本地持久）
```python
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
```

**特点**：
- 保存最近的历史消息
- 新订阅者可以接收历史消息
- 适用于状态信息

**适用场景**：
- 地图数据
- 机器人状态
- 配置信息

---

### 3. History（历史策略）

**定义**：保留多少历史消息

#### KEEP_LAST
```python
from rclpy.qos import HistoryPolicy

qos.history = HistoryPolicy.KEEP_LAST
qos.depth = 10  # 保留最近 10 条消息
```

**特点**：
- 只保留最近的 N 条消息
- 节省内存
- 默认行为

#### KEEP_ALL
```python
qos.history = HistoryPolicy.KEEP_ALL
```

**特点**：
- 保留所有未发送的消息
- 可能占用大量内存
- 需要谨慎使用

---

### 4. Depth（队列深度）

**定义**：消息队列的大小

```python
qos.depth = 10  # 队列大小为 10
```

**选择建议**：
- **高频数据**（> 10 Hz）：depth = 1-5
- **中频数据**（1-10 Hz）：depth = 5-10
- **低频数据**（< 1 Hz）：depth = 10-50

---

### 5. Liveliness（活跃度）

**定义**：检测发布者是否存活

```python
from rclpy.qos import LivelinessPolicy
import rclpy.duration

qos.liveliness = LivelinessPolicy.AUTOMATIC
qos.liveliness_lease_duration = rclpy.duration.Duration(seconds=5)
```

**策略**：
- `AUTOMATIC`：自动检测
- `MANUAL_BY_TOPIC`：手动标记（按话题）
- `MANUAL_BY_NODE`：手动标记（按节点）

---

### 6. Deadline（截止时间）

**定义**：期望消息到达的最大间隔

```python
qos.deadline = rclpy.duration.Duration(seconds=1)
```

**用途**：
- 检测发布者是否按时发送
- 触发警告或错误处理

---

## 预定义的 QoS 配置

ROS 2 提供了几种常用的 QoS 预设：

### 1. Sensor Data QoS（传感器数据）

```python
from rclpy.qos import qos_profile_sensor_data

# 等价于：
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)
```

**用途**：
- 激光雷达数据 (`/scan`)
- 相机图像
- IMU 数据
- 里程计数据

### 2. System Default QoS（系统默认）

```python
from rclpy.qos import qos_profile_system_default

# 等价于：
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**用途**：
- 一般的话题通信
- 不确定时的默认选择

### 3. Services Default QoS（服务默认）

```python
from rclpy.qos import qos_profile_services_default

# 等价于：
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**用途**：
- ROS 服务调用

### 4. Parameters QoS（参数）

```python
from rclpy.qos import qos_profile_parameters

# 等价于：
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1000
)
```

**用途**：
- 参数服务器

---

## 实际问题：QoS 兼容性

### 问题场景

在 RPLidar + GMapping 项目中遇到的真实问题：

```
雷达节点 (rplidar_ros)
    ↓ /scan (Sensor Data QoS - Best Effort)
    ↓
laser_filters 节点
    ↓ /scan_filtered (System Default QoS - Reliable)
    ↓
GMapping 节点 (期望 Sensor Data QoS - Best Effort)
    ↓
❌ 无法接收数据！
```

### 问题原因

**QoS 兼容性规则**：

| 发布者 | 订阅者 | 结果 |
|--------|--------|------|
| RELIABLE | RELIABLE | ✅ 兼容 |
| RELIABLE | BEST_EFFORT | ✅ 兼容 |
| BEST_EFFORT | RELIABLE | ❌ **不兼容** |
| BEST_EFFORT | BEST_EFFORT | ✅ 兼容 |

**规则总结**：
- 订阅者的可靠性要求 ≤ 发布者的可靠性保证
- RELIABLE 发布者可以被任何订阅者接收
- BEST_EFFORT 发布者只能被 BEST_EFFORT 订阅者接收

### 解决方案

#### 方案 1：QoS 桥接节点

创建一个中间节点进行 QoS 转换：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

class QoSBridge(Node):
    def __init__(self):
        super().__init__('laser_scan_qos_bridge')
        
        # 订阅：使用 Reliable QoS（来自 laser_filters）
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.callback,
            reliable_qos)
        
        # 发布：使用 Best Effort QoS（给 GMapping）
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile_sensor_data)
        
        self.get_logger().info('QoS Bridge: Reliable → Best Effort')
    
    def callback(self, msg):
        # 直接转发消息
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = QoSBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 方案 2：修改订阅者 QoS

如果可以修改订阅者代码：

```python
# 原来（不兼容）
self.scan_sub = self.create_subscription(
    LaserScan, '/scan',
    self.callback,
    qos_profile_sensor_data)  # Best Effort

# 修改后（兼容）
from rclpy.qos import QoSProfile, ReliabilityPolicy

reliable_qos = QoSProfile(depth=10)
reliable_qos.reliability = ReliabilityPolicy.RELIABLE

self.scan_sub = self.create_subscription(
    LaserScan, '/scan',
    self.callback,
    reliable_qos)  # Reliable
```

#### 方案 3：修改发布者 QoS

如果可以修改发布者（如 laser_filters）：

```python
# 修改 laser_filters 输出为 Best Effort
self.scan_pub = self.create_publisher(
    LaserScan, '/scan_filtered',
    qos_profile_sensor_data)  # Best Effort
```

---

## 完整示例：传感器数据发布订阅

### 发布者（激光雷达）

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class LaserPublisher(Node):
    def __init__(self):
        super().__init__('laser_publisher')
        
        # 使用 Sensor Data QoS
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile_sensor_data)
        
        # 10 Hz 发布频率
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Laser publisher started (Best Effort)')
    
    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        # ... 填充激光数据
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = LaserPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 订阅者（SLAM 算法）

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        
        # 使用相同的 Sensor Data QoS
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            qos_profile_sensor_data)
        
        self.get_logger().info('Laser subscriber started (Best Effort)')
    
    def callback(self, msg):
        self.get_logger().info(f'Received scan with {len(msg.ranges)} points')

def main():
    rclpy.init()
    node = LaserSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## QoS 调试技巧

### 1. 查看话题的 QoS

```bash
# 查看话题信息（包括 QoS）
ros2 topic info /scan -v

# 输出示例：
# Type: sensor_msgs/msg/LaserScan
# Publisher count: 1
#   QoS profile:
#     Reliability: BEST_EFFORT
#     Durability: VOLATILE
#     History: KEEP_LAST (depth: 5)
```

### 2. 检查 QoS 兼容性

使用 `ros2 doctor` 检查系统问题：

```bash
ros2 doctor --report
```

### 3. 日志输出 QoS 信息

在节点中输出 QoS 配置：

```python
def print_qos_profile(qos, name=""):
    print(f"=== QoS Profile: {name} ===")
    print(f"Reliability: {qos.reliability}")
    print(f"Durability: {qos.durability}")
    print(f"History: {qos.history}")
    print(f"Depth: {qos.depth}")
    print(f"========================")

# 使用
print_qos_profile(qos_profile_sensor_data, "Sensor Data")
```

---

## 最佳实践

### 1. 选择合适的 QoS

| 数据类型 | 推荐 QoS | 理由 |
|---------|---------|------|
| **传感器数据** | Sensor Data | 高频、可容忍丢失 |
| **控制命令** | System Default | 需要可靠送达 |
| **地图数据** | Transient Local | 新节点需要历史数据 |
| **诊断信息** | System Default | 重要但不高频 |
| **参数更新** | Parameters | 需要可靠且深队列 |

### 2. 发布者和订阅者匹配

**规则**：订阅者的 QoS 应该 ≥ 发布者的 QoS（在可靠性上）

```python
# ✅ 正确：兼容
publisher_qos = qos_profile_sensor_data  # Best Effort
subscriber_qos = qos_profile_sensor_data  # Best Effort

# ✅ 正确：兼容
publisher_qos = qos_profile_system_default  # Reliable
subscriber_qos = qos_profile_sensor_data    # Best Effort

# ❌ 错误：不兼容
publisher_qos = qos_profile_sensor_data     # Best Effort
subscriber_qos = qos_profile_system_default  # Reliable
```

### 3. 深度（Depth）设置

```python
# 高频数据（> 10 Hz）
qos.depth = 1  # 只保留最新

# 中频数据（1-10 Hz）
qos.depth = 10  # 保留最近 10 条

# 低频数据（< 1 Hz）
qos.depth = 50  # 保留更多历史
```

### 4. 实时性 vs 可靠性权衡

| 场景 | 选择 | 原因 |
|------|------|------|
| **机器人避障** | Best Effort | 实时性优先 |
| **导航目标** | Reliable | 准确性优先 |
| **状态监控** | Reliable | 不能丢失 |
| **视频流** | Best Effort | 可容忍丢帧 |

---

## 常见问题

### Q1: 为什么订阅者收不到消息？

**检查清单**：
1. ✅ 话题名称是否正确？
2. ✅ QoS 是否兼容？
3. ✅ 网络连接是否正常？
4. ✅ 节点是否正常运行？

**诊断命令**：
```bash
# 检查发布者
ros2 topic info /scan -v

# 检查话题数据
ros2 topic echo /scan

# 检查 QoS 兼容性
ros2 topic info /scan -v  # 查看发布者和订阅者的 QoS
```

### Q2: 如何选择 RELIABLE 还是 BEST_EFFORT？

**决策树**：
```
数据丢失是否可接受？
├─ 是 → BEST_EFFORT
│   └─ 例如：传感器数据、视频流
└─ 否 → RELIABLE
    └─ 例如：控制命令、配置参数
```

### Q3: Depth 设置多大合适？

**计算公式**：
```
建议 depth = 发布频率(Hz) × 最大延迟(秒)

例如：
- 10 Hz 的激光数据，容忍 1 秒延迟
  depth = 10 × 1 = 10

- 50 Hz 的里程计，容忍 0.2 秒延迟
  depth = 50 × 0.2 = 10
```

### Q4: 如何处理 QoS 不兼容？

**方案对比**：

| 方案 | 优点 | 缺点 | 适用场景 |
|------|------|------|---------|
| **QoS 桥接** | 不修改原代码 | 增加节点 | 无法修改源码 |
| **修改订阅者** | 简单直接 | 需要修改代码 | 可以修改订阅者 |
| **修改发布者** | 统一 QoS | 可能影响其他订阅者 | 可以修改发布者 |

---

## C++ 示例

### 发布者

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserPublisher : public rclcpp::Node
{
public:
    LaserPublisher() : Node("laser_publisher")
    {
        // 使用 Sensor Data QoS
        auto qos = rclcpp::SensorDataQoS();
        
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", qos);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LaserPublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Laser publisher started");
    }

private:
    void timer_callback()
    {
        auto msg = sensor_msgs::msg::LaserScan();
        msg.header.stamp = this->now();
        msg.header.frame_id = "laser";
        // ... 填充数据
        publisher_->publish(msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

### 订阅者

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserSubscriber : public rclcpp::Node
{
public:
    LaserSubscriber() : Node("laser_subscriber")
    {
        // 使用 Sensor Data QoS
        auto qos = rclcpp::SensorDataQoS();
        
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos,
            std::bind(&LaserSubscriber::callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Laser subscriber started");
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received scan with %zu points", msg->ranges.size());
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 总结

### 关键要点

1. **理解 QoS 参数**
   - Reliability：可靠性（RELIABLE vs BEST_EFFORT）
   - Durability：持久性（VOLATILE vs TRANSIENT_LOCAL）
   - History：历史策略（KEEP_LAST vs KEEP_ALL）
   - Depth：队列深度

2. **兼容性规则**
   - 订阅者的可靠性要求 ≤ 发布者的可靠性保证
   - BEST_EFFORT 发布者只能被 BEST_EFFORT 订阅者接收

3. **选择原则**
   - 传感器数据 → Sensor Data QoS (Best Effort)
   - 控制命令 → System Default QoS (Reliable)
   - 需要历史数据 → Transient Local

4. **调试方法**
   - 使用 `ros2 topic info -v` 查看 QoS
   - 使用 `ros2 doctor` 检查问题
   - 创建 QoS 桥接节点解决不兼容

### 快速参考

| 应用场景 | Reliability | Durability | Depth | 预定义 QoS |
|---------|------------|-----------|-------|-----------|
| 激光雷达 | Best Effort | Volatile | 5 | Sensor Data |
| 相机图像 | Best Effort | Volatile | 5 | Sensor Data |
| 里程计 | Best Effort | Volatile | 5 | Sensor Data |
| 控制命令 | Reliable | Volatile | 10 | System Default |
| 地图数据 | Reliable | Transient Local | 1 | 自定义 |
| 参数 | Reliable | Volatile | 1000 | Parameters |

---

## 参考资源

### 官方文档
- [ROS 2 QoS 文档](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [DDS QoS 策略](https://www.omg.org/spec/DDS/1.4/PDF)

### 相关文档
- [RPLidar + GMapping 项目](/posts/rplidar-s3-gmapping-slam/) - 实际 QoS 问题案例
- [Laser Filters 使用指南](/docs/ros2-laser-filters-guide/) - QoS 兼容性解决
- [故障排查指南](/docs/troubleshooting-guide/) - QoS 问题诊断

---

**文档版本**: 1.0  
**适用 ROS 版本**: ROS 2 Humble  
**最后更新**: 2025-10-31

