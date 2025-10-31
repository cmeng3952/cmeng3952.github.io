---
title: "Launch 文件版本对比"
weight: 6
---

# Launch 文件版本对比说明

## 概述

本项目提供了**两个主要的 launch 文件**来启动 SLAM 建图系统。它们的主要区别在于**激光数据预处理方式**。

---

## 两个版本对比

### 版本 A：使用官方 laser_filters ⭐ 推荐

**文件名**：`bringup_rplidar_s3_gmapping_with_laser_filters.launch.py`

**启动命令**：
```bash
ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_laser_filters.launch.py
```

**数据流程**：
```
RPLidar S3 → /scan_raw → laser_filters(官方) → /scan → GMapping
                             ↓
                    range filter + speckle filter
```

**特点**：
- ✅ 使用 ROS 官方的 `laser_filters` 包
- ✅ 提供专业的滤波功能
- ✅ 可配置多种滤波器
- ✅ C++ 实现，性能好
- ⚠️ 需要安装额外包：`ros-humble-laser-filters`
- ⚠️ 配置稍复杂

**适用场景**：
- 生产环境
- 需要高质量建图
- 长期项目
- 需要过滤噪声数据

---

### 版本 B：使用自定义降采样

**文件名**：`bringup_rplidar_s3_gmapping_with_teleop.launch.py`

**启动命令**：
```bash
ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_teleop.launch.py
```

**数据流程**：
```
RPLidar S3 → /scan_raw → laser_scan_decimator(自定义) → /scan → GMapping
                             ↓
                         均匀降采样
```

**特点**：
- ✅ 配置简单，易于理解
- ✅ 不需要额外安装包
- ✅ Python 实现，易于修改
- ✅ 可调整降采样参数
- ⚠️ 功能单一（仅降采样）
- ⚠️ 性能略低（Python）

**适用场景**：
- 快速测试
- 学习和实验
- 不需要复杂滤波
- 快速验证系统

---

## 详细对比表

| 特性 | laser_filters 版本 | 自定义降采样版本 |
|------|-------------------|----------------|
| **launch 文件** | `bringup_...with_laser_filters.launch.py` | `bringup_...with_teleop.launch.py` |
| **依赖包** | 需要 `ros-humble-laser-filters` | 无额外依赖 |
| **滤波功能** | ✅ 范围过滤<br>✅ 离群点过滤<br>✅ 角度过滤<br>✅ 阴影过滤 | ❌ 仅降采样 |
| **性能** | ⭐⭐⭐⭐⭐ C++ | ⭐⭐⭐ Python |
| **配置复杂度** | ⭐⭐⭐ 中等 | ⭐ 简单 |
| **可扩展性** | ⭐⭐⭐⭐⭐ 丰富 | ⭐⭐ 有限 |
| **学习曲线** | ⭐⭐⭐ 陡峭 | ⭐ 平缓 |
| **推荐度** | ⭐⭐⭐⭐⭐ 生产 | ⭐⭐⭐ 测试 |

---

## 配置文件对比

### laser_filters 版本的配置

需要创建 `config/laser_filter_params.yaml`：

```yaml
scan_to_scan_filter_chain:
  ros__parameters:
    filter1:
      name: range_filter
      type: laser_filters/LaserScanRangeFilter
      params:
        use_message_range_limits: false
        lower_threshold: 0.2      # 过滤 < 0.2m 的点
        upper_threshold: 40.0     # 过滤 > 40m 的点
    
    filter2:
      name: speckle_filter
      type: laser_filters/LaserScanSpeckleFilter
      params:
        filter_type: 0            # 基于距离的过滤
        max_range: 40.0
        max_range_difference: 0.2 # 邻居点距离差 > 0.2m 视为噪声
        filter_window: 2          # 检查前后各2个点
```

**可以添加更多滤波器**：
```yaml
    filter3:
      name: angular_bounds
      type: laser_filters/LaserScanAngularBoundsFilter
      params:
        lower_angle: -1.57  # -90度
        upper_angle: 1.57   # +90度
```

### 自定义降采样版本的配置

在 launch 文件中直接设置：

```python
Node(
    package='slam_gmapping',
    executable='laser_scan_decimator.py',
    parameters=[{'target_beams': 360}],  # 降采样到 360 个点
    output='screen')
```

---

## 话题数据流对比

### laser_filters 版本

```
话题流程：
/scan_raw (8000+ 点) 
    ↓
laser_filters 节点
    ↓ 范围过滤
    ↓ 离群点过滤
    ↓
/scan_filtered (~7000 点)
    ↓
/scan (最终输出)
    ↓
GMapping
```

### 自定义降采样版本

```
话题流程：
/scan_raw (8000+ 点)
    ↓
laser_scan_decimator 节点
    ↓ 均匀采样（每 N 个点取 1 个）
    ↓
/scan (360 点)
    ↓
GMapping
```

---

## 如何选择？

### 选择 laser_filters 版本（推荐）如果：

- ✅ 需要在生产环境中使用
- ✅ 环境噪声较多
- ✅ 需要高质量的地图
- ✅ 愿意学习 laser_filters 配置
- ✅ 有足够的计算资源

**典型场景**：
- 商用服务机器人
- 工业 AGV
- 研究项目
- 长期部署的系统

### 选择自定义降采样版本如果：

- ✅ 快速测试和验证
- ✅ 学习 SLAM 原理
- ✅ 环境相对干净
- ✅ 希望简单配置
- ✅ 计算资源有限

**典型场景**：
- 学习和教学
- 原型开发
- 概念验证
- 演示系统

---

## 切换方法

### 从自定义降采样切换到 laser_filters

1. **安装依赖**：
   ```bash
   sudo apt install ros-humble-laser-filters
   ```

2. **创建配置文件**：
   ```bash
   cd ~/rplidar_ws/src/slam_gmapping/slam_gmapping
   # 创建 config/laser_filter_params.yaml
   ```

3. **更新 CMakeLists.txt**：
   ```cmake
   install(DIRECTORY config
     DESTINATION share/${PROJECT_NAME}
   )
   ```

4. **重新编译**：
   ```bash
   cd ~/rplidar_ws
   colcon build --packages-select slam_gmapping
   source install/setup.bash
   ```

5. **使用新的 launch 文件**：
   ```bash
   ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_laser_filters.launch.py
   ```

### 从 laser_filters 切换到自定义降采样

1. **移除 laser_filters**（可选）：
   ```bash
   sudo apt remove ros-humble-laser-filters
   ```

2. **使用原来的 launch 文件**：
   ```bash
   ros2 launch slam_gmapping bringup_rplidar_s3_gmapping_with_teleop.launch.py
   ```

---

## 常见问题

### Q1: 两个版本可以同时安装吗？

**A**: 可以！它们是独立的 launch 文件，可以根据需要选择使用。

### Q2: 性能差异大吗？

**A**: 
- **laser_filters**（C++）：处理 8000 点约 1-2ms
- **自定义降采样**（Python）：处理 8000 点约 5-10ms
- 对于 10Hz 的激光数据，两者都能满足实时性要求

### Q3: 建图质量有差别吗？

**A**: 
- **laser_filters**：过滤噪声后，建图质量更高，墙壁更清晰
- **自定义降采样**：如果环境干净，质量差异不大；如果有噪声，可能影响质量

### Q4: 可以混合使用吗？

**A**: 理论上可以，但不推荐。选择一种方式并优化其参数即可。

### Q5: 官方 launch 文件在哪里？

**A**: 
- laser_filters 版本：实际项目中最常用
- 自定义降采样版本：快速开始时使用
- 两者功能都完整，根据需求选择

---

## 推荐配置

### 生产环境配置（laser_filters）

```yaml
# laser_filter_params.yaml
scan_to_scan_filter_chain:
  ros__parameters:
    filter1:
      name: range_filter
      type: laser_filters/LaserScanRangeFilter
      params:
        lower_threshold: 0.2
        upper_threshold: 10.0    # 室内场景
    
    filter2:
      name: speckle_filter
      type: laser_filters/LaserScanSpeckleFilter
      params:
        filter_type: 0
        max_range_difference: 0.1  # 更严格
        filter_window: 2
    
    filter3:
      name: box_filter
      type: laser_filters/LaserScanBoxFilter
      params:
        box_frame: "base_link"
        min_x: -0.3
        max_x: 0.3
        min_y: -0.2
        max_y: 0.2
        min_z: 0.0
        max_z: 0.3
        invert: false
```

### 测试环境配置（自定义降采样）

```python
# 在 launch 文件中
Node(
    package='slam_gmapping',
    executable='laser_scan_decimator.py',
    parameters=[{'target_beams': 360}],
    output='screen')
```

---

## 总结

| 使用场景 | 推荐版本 | 理由 |
|---------|---------|------|
| **生产环境** | laser_filters | 专业、稳定、功能丰富 |
| **快速测试** | 自定义降采样 | 简单、快速、易配置 |
| **学习研究** | 两者都可 | 比较两种方案的差异 |
| **长期项目** | laser_filters | 可扩展性和维护性好 |

---

**建议**：
1. **初学者**：先用自定义降采样版本快速上手
2. **进阶用户**：切换到 laser_filters 版本，学习专业配置
3. **生产部署**：使用 laser_filters 版本

---

**相关文档**：
- [快速开始指南](/docs/rplidar-gmapping-quickstart/) - 两个版本的详细安装步骤
- [Laser Filters 使用指南](/docs/ros2-laser-filters-guide/) - laser_filters 详细配置
- [故障排查指南](/docs/troubleshooting-guide/) - 常见问题解决

---

**文档版本**: 1.0  
**最后更新**: 2025-10-31

