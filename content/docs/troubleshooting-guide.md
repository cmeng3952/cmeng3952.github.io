---
title: "故障排查指南"
weight: 15
---

# ROS 2 GMapping SLAM 故障排查指南

## 快速诊断 🔍

遇到问题？按以下顺序检查：

1. ✅ 雷达连接正常？ → [雷达问题](#雷达连接问题)
2. ✅ 节点都在运行？ → [节点问题](#节点启动问题)
3. ✅ 有激光数据？ → [激光数据问题](#激光数据问题)
4. ✅ 有里程计数据？ → [里程计问题](#里程计问题)
5. ✅ TF 树完整？ → [TF问题](#tf-坐标系问题)
6. ✅ 地图在更新？ → [建图问题](#建图问题)

---

## 雷达连接问题

### 问题1: 找不到 /dev/ttyUSB0

**症状**：
```
[ERROR] Error, cannot bind to the specified serial port /dev/ttyUSB0
```

**检查方法**：
```bash
ls -l /dev/ttyUSB*
```

**解决方案**：

#### 方案A: 设备不存在
```bash
# 检查 USB 连接
lsusb

# 应该看到类似：
# Bus 001 Device 005: ID 10c4:ea60 Silicon Labs CP210x UART Bridge

# 如果没有，检查：
# 1. USB 线是否连接牢固
# 2. 雷达是否通电（转动）
# 3. 更换 USB 端口
```

#### 方案B: 设备名称不同
```bash
# 查看实际设备名
ls -l /dev/ttyUSB*

# 输出示例：
# /dev/ttyUSB1  # ← 实际设备是 ttyUSB1

# 启动时指定正确端口
ros2 launch slam_gmapping ... serial_port:=/dev/ttyUSB1
```

---

### 问题2: 权限不足

**症状**：
```
[ERROR] Permission denied: '/dev/ttyUSB0'
```

**解决方案**：

#### 临时方案（立即生效）
```bash
sudo chmod 666 /dev/ttyUSB0
```

#### 永久方案（推荐）
```bash
# 添加用户到 dialout 组
sudo usermod -a -G dialout $USER

# 注销并重新登录
# 或重启系统

# 验证
groups | grep dialout
```

---

### 问题3: 雷达健康状态异常

**症状**：
```
[WARN] RPLidar health status: WARNING
[ERROR] RPLidar health status: ERROR
```

**检查方法**：
```bash
ros2 topic echo /rplidar_node/health
```

**解决方案**：

1. **断电重启雷达**
   ```bash
   # 拔掉 USB
   # 等待 5 秒
   # 重新插入
   ```

2. **检查雷达状态**
   ```bash
   # 观察雷达是否旋转
   # 检查是否有异常声音
   # 清洁雷达镜头
   ```

3. **固件问题**
   - 联系思岚科技获取最新固件
   - 使用 RoboStudio 更新固件

---

## 节点启动问题

### 问题4: slam_gmapping 节点崩溃

**症状**：
```
[ERROR] slam_gmapping: process has died [exit code -6]
Assertion `beams<LASER_MAXBEAMS' failed
```

**原因**：激光点数超过 GMapping 内部限制

**解决方案**：

#### 方案A: 使用降采样节点（推荐）
确保 launch 文件包含：
```python
# 雷达输出重映射到 scan_raw
Node(..., remappings=[('/scan', '/scan_raw')])

# 添加降采样节点
Node(
    package='slam_gmapping',
    executable='laser_scan_decimator.py',
    ...)
```

#### 方案B: 降低 max_beams 参数
```yaml
# slam_params.yaml
max_beams: 60  # 从 1024 改为 60
```

---

### 问题5: 参数文件格式错误

**症状**：
```
[ERROR] Cannot have a value before ros__parameters
[ERROR] Sequences can only be values and not keys
```

**原因**：ROS 2 参数文件格式不正确

**解决方案**：

确保 YAML 格式正确：
```yaml
# ✅ 正确格式
slam_gmapping:
  ros__parameters:
    base_frame: "base_link"
    maxUrange: 10.0

# ❌ 错误格式（缺少 ros__parameters）
slam_gmapping:
  base_frame: "base_link"
```

---

### 问题6: 找不到 Python 脚本

**症状**：
```
[ERROR] Package 'slam_gmapping' not found
[ERROR] Executable 'laser_scan_decimator.py' not found
```

**解决方案**：

1. **检查文件存在**
   ```bash
   ls ~/rplidar_ws/src/slam_gmapping/slam_gmapping/scripts/
   ```

2. **添加执行权限**
   ```bash
   chmod +x scripts/*.py
   ```

3. **更新 CMakeLists.txt**
   ```cmake
   install(PROGRAMS
     scripts/laser_scan_decimator.py
     scripts/simple_odom_publisher.py
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

4. **重新编译**
   ```bash
   cd ~/rplidar_ws
   colcon build --packages-select slam_gmapping
   source install/setup.bash
   ```

---

## 激光数据问题

### 问题7: /scan 话题无数据

**检查方法**：
```bash
# 查看话题列表
ros2 topic list | grep scan

# 查看话题频率
ros2 topic hz /scan

# 查看数据内容
ros2 topic echo /scan --once
```

**解决方案**：

#### 情况A: scan_raw 有数据，scan 无数据
```bash
# 检查 scan_raw
ros2 topic hz /scan_raw  # 应该有数据

# 检查 scan
ros2 topic hz /scan      # 无数据

# 原因：降采样节点未运行
ros2 node list | grep decimator

# 解决：重新启动系统
```

#### 情况B: scan_raw 也无数据
```bash
# 检查雷达节点
ros2 node list | grep rplidar

# 检查雷达健康
ros2 topic echo /rplidar_node/health

# 重启雷达节点
```

---

### 问题8: 激光数据异常

**症状**：
- 所有点距离为 `inf`
- 点数过少或过多
- 角度范围不对

**检查方法**：
```bash
ros2 topic echo /scan --once | grep -E "(range_min|range_max|angle|ranges)"
```

**解决方案**：

1. **检查雷达环境**
   - 确保周围有物体（不要对着空旷区域）
   - 清洁雷达镜头
   - 移除玻璃等反射物体

2. **检查雷达参数**
   ```python
   # rplidar launch 文件
   parameters=[{
       'scan_mode': 'DenseBoost',  # 或 'Standard'
       'frame_id': 'laser'
   }]
   ```

---

## 里程计问题

### 问题9: 没有 /odom 话题

**症状**：
```
[WARN] Waiting for /odom
```

**检查方法**：
```bash
ros2 topic list | grep odom
ros2 topic hz /odom
```

**解决方案**：

#### 方案A: 启动里程计节点
```bash
# 检查节点是否运行
ros2 node list | grep odom

# 如果没有，手动启动
ros2 run slam_gmapping simple_odom_publisher.py
```

#### 方案B: 使用真实底盘
如果有真实机器人底盘：
```bash
# 启动底盘驱动（替代简单里程计）
ros2 launch your_robot robot.launch.py
```

---

### 问题10: 键盘控制无效

**症状**：
- 按键没有反应
- `/cmd_vel` 话题无数据

**检查方法**：
```bash
# 查看 cmd_vel 话题
ros2 topic hz /cmd_vel

# 按键时应该有数据
```

**解决方案**：

1. **确保终端有焦点**
   - 点击 teleop 终端窗口
   - 不要点击其他窗口

2. **检查 teleop 节点**
   ```bash
   # 查看节点
   ros2 node list | grep teleop
   
   # 如果没有，启动
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. **安装 teleop 包**
   ```bash
   sudo apt install ros-humble-teleop-twist-keyboard
   ```

---

## TF 坐标系问题

### 问题11: TF 树不完整

**症状**：
```
[WARN] Lookup would require extrapolation into the past
[ERROR] Could not transform from laser to map
```

**检查方法**：
```bash
# 生成 TF 树图
ros2 run tf2_tools view_frames

# 查看 frames.pdf
# 应该有完整的链：map -> odom -> base_footprint -> base_link -> laser
```

**解决方案**：

#### 缺少的变换及对应节点：

| 缺少变换 | 负责节点 | 解决方法 |
|---------|---------|---------|
| `odom → base_footprint` | 里程计节点 | 启动里程计发布器 |
| `base_footprint → base_link` | static_tf | 检查 launch 文件 |
| `base_link → laser` | static_tf | 检查 launch 文件 |
| `map → odom` | slam_gmapping | 检查建图节点 |

#### 修复 static_transform_publisher
```python
# 确保 launch 文件包含：
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'base_footprint',
        '--child-frame-id', 'base_link']),

Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'laser']),
```

---

### 问题12: TF 时间戳不同步

**症状**：
```
[WARN] TF_OLD_DATA ignoring data from the past
```

**原因**：不同节点的时间戳不一致

**解决方案**：

1. **使用统一时间源**
   ```bash
   # 检查系统时间同步
   timedatectl status
   
   # 如果需要，启用 NTP
   sudo timedatectl set-ntp true
   ```

2. **确保使用当前时间戳**
   ```python
   # 在节点中
   msg.header.stamp = self.get_clock().now().to_msg()
   ```

---

## 建图问题

### 问题13: 地图不更新

**症状**：
- RViz 显示 "No map received"
- `/map` 话题频率为 0 Hz

**检查清单**：

```bash
# 1. 检查 GMapping 节点
ros2 node list | grep slam_gmapping

# 2. 检查激光数据
ros2 topic hz /scan  # 应该 >0

# 3. 检查里程计数据
ros2 topic hz /odom  # 应该 >0

# 4. 检查速度命令（移动时）
ros2 topic hz /cmd_vel  # 移动时应该 >0

# 5. 检查地图话题
ros2 topic list | grep map

# 6. 检查 TF 树
ros2 run tf2_tools view_frames
```

**解决方案**：

#### 情况A: 机器人未移动
- 使用键盘控制移动
- 确保 `/cmd_vel` 有数据
- 确保里程计在更新

#### 情况B: 移动但不更新
```yaml
# 检查更新阈值是否过大
# slam_params.yaml
linearUpdate: 0.2  # 如果过大，改为 0.1
angularUpdate: 0.1
```

#### 情况C: GMapping 崩溃
```bash
# 查看日志
ros2 node info /slam_gmapping

# 查看进程
ps aux | grep slam_gmapping
```

---

### 问题14: 地图质量差

**症状**：
- 墙壁模糊或有多个影子
- 地图错位
- 闭环时无法对齐

**原因分析**：

| 症状 | 可能原因 | 解决方案 |
|------|---------|---------|
| 多个重叠影子 | 粒子数不足 | 增加 `particles` |
| 墙壁模糊 | 采样点太少 | 增加 `max_beams` |
| 快速运动时失败 | 更新频率不够 | 减小 `linearUpdate` |
| 对称走廊迷失 | 歧义性高 | 增加 `particles`<br>提高 `minimumScore` |
| 地图抖动 | 里程计误差大 | 调大运动模型参数 |

**调优建议**：

参考 [GMapping 参数调优指南](/docs/gmapping-parameters-tuning/)

```yaml
# 提高精度配置
particles: 50
max_beams: 90
delta: 0.025
linearUpdate: 0.1
```

---

### 问题15: 建图时系统卡顿

**症状**：
- RViz 显示延迟
- CPU 使用率 100%
- 内存占用过高

**解决方案**：

#### 降低计算负载
```yaml
# slam_params.yaml
particles: 20      # 从 30 降到 20
max_beams: 30      # 从 60 降到 30
delta: 0.1         # 从 0.05 提高到 0.1

# 减少更新频率
linearUpdate: 0.3
angularUpdate: 0.15
```

#### 优化系统
```bash
# 关闭不必要的程序
# 增加交换分区
# 使用更高性能的硬件
```

---

## RViz 显示问题

### 问题16: RViz 中看不到激光点云

**检查**：

1. **Fixed Frame 设置**
   - 应设置为 `map` 或 `odom`

2. **LaserScan 话题**
   - Topic 选择 `/scan`
   - Size 设置 0.05-0.1

3. **数据存在**
   ```bash
   ros2 topic hz /scan
   ```

**解决方案**：

```bash
# 删除并重新添加 LaserScan 显示
# 1. 点击 Displays 面板中的 LaserScan
# 2. 点击 Remove
# 3. 点击 Add → By topic → /scan → LaserScan
```

---

### 问题17: RViz 中看不到地图

**检查**：

1. **Map 话题**
   ```bash
   ros2 topic hz /map
   # 如果为 0，说明地图没有更新
   ```

2. **Fixed Frame**
   - 必须设置为 `map`

3. **Map 显示设置**
   - Topic: `/map`
   - Alpha: 0.7
   - Color Scheme: map

**解决方案**：

```bash
# 确保机器人移动了
# 等待几秒让地图初始化
# 检查 GMapping 节点状态
ros2 node info /slam_gmapping
```

---

## 系统诊断脚本

### 完整检查脚本

保存为 `check_system.sh`：

```bash
#!/bin/bash

echo "=== ROS 2 GMapping 系统诊断 ==="
echo ""

echo "1. 检查雷达设备..."
ls -l /dev/ttyUSB* 2>/dev/null || echo "  ❌ 未找到串口设备"
echo ""

echo "2. 检查运行的节点..."
ros2 node list
echo ""

echo "3. 检查话题..."
echo "  /scan:" && ros2 topic hz /scan --window 10 --max-waiting-time 3 2>/dev/null || echo "    ❌ 无数据"
echo "  /odom:" && ros2 topic hz /odom --window 10 --max-waiting-time 3 2>/dev/null || echo "    ❌ 无数据"
echo "  /map:" && ros2 topic hz /map --window 10 --max-waiting-time 3 2>/dev/null || echo "    ⚠️  无数据（需要移动）"
echo ""

echo "4. 检查 TF 树..."
timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null && echo "  ✅ TF 树完整" || echo "  ❌ TF 树不完整"
echo ""

echo "5. 系统资源..."
echo "  CPU:" && top -bn1 | grep "Cpu(s)" | awk '{print $2}' || echo "    无法获取"
echo "  内存:" && free -h | grep "Mem:" || echo "    无法获取"
echo ""

echo "诊断完成"
```

```bash
chmod +x check_system.sh
./check_system.sh
```

---

## 获取帮助 🆘

### 日志信息

收集诊断信息：
```bash
# 1. 保存节点列表
ros2 node list > nodes.txt

# 2. 保存话题列表
ros2 topic list > topics.txt

# 3. 保存 TF 树
ros2 run tf2_tools view_frames

# 4. 保存系统信息
uname -a > system_info.txt
ros2 doctor > ros2_doctor.txt
```

### 常用资源

- [ROS 2 官方文档](https://docs.ros.org/en/humble/)
- [GMapping 论文](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti07tro.pdf)
- [思岚雷达文档](https://www.slamtec.com/en/support)
- [ROS Answers](https://answers.ros.org/)

### 社区支持

- **ROS Discourse**: https://discourse.ros.org/
- **Stack Overflow**: 使用标签 `ros2`, `gmapping`
- **GitHub Issues**: 相关项目的 issue 页面

---

## 总结

### 问题排查流程

```
1. 硬件 → 雷达连接
    ↓
2. 软件 → 节点启动
    ↓
3. 数据 → 激光+里程计
    ↓
4. 系统 → TF 树
    ↓
5. 功能 → 建图更新
```

### 预防措施

- ✅ 定期清洁雷达
- ✅ 使用稳定的 USB 连接
- ✅ 保持参数文件备份
- ✅ 记录成功的配置
- ✅ 定期更新软件包

---

**文档版本**: 1.0  
**最后更新**: 2025-10-31

**还有问题？** 参考 [完整项目文档](/posts/rplidar-s3-gmapping-slam/) 或 [快速开始指南](/docs/rplidar-gmapping-quickstart/)

