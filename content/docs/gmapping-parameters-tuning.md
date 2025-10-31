---
title: "GMapping 参数调优指南"
weight: 12
---

# GMapping 参数调优指南

## 概述

GMapping 是一个基于 Rao-Blackwellized 粒子滤波器的 SLAM 算法。正确的参数配置对建图质量至关重要。本文详细介绍各参数的含义、作用和调优方法。

## 核心参数详解

### 1. 激光参数

#### maxUrange

**含义**：用于建图的激光最大有效距离（米）

**作用**：
- 只有距离小于 `maxUrange` 的激光点才用于更新地图
- 超过此距离的点仅用于定位，不用于建图

**推荐值**：
```yaml
# 小房间（5-10平米）
maxUrange: 5.0

# 办公室、实验室（10-50平米）
maxUrange: 10.0

# 大型室内空间（>50平米）
maxUrange: 15.0

# 室外环境
maxUrange: 20.0
```

**调优建议**：
- 设置为实际环境大小的 70-80%
- 过大：墙面不清晰，计算量增加
- 过小：远处物体无法建图

#### maxRange

**含义**：激光雷达的最大测量距离（米）

**作用**：
- 超过此距离的测量值被认为无效
- 应设置为雷达的实际量程

**推荐值**：
```yaml
# 思岚 A1/A2 系列
maxRange: 12.0

# 思岚 S3 系列
maxRange: 40.0

# Hokuyo URG-04LX
maxRange: 5.6

# Sick TiM series
maxRange: 25.0
```

**关系**：`maxRange >= maxUrange`

#### max_beams

**含义**：GMapping 内部用于扫描匹配的激光束采样数量

**作用**：
- 从输入的激光点中均匀采样 `max_beams` 个点
- 越多越精确，但计算量越大

**推荐值**：
```yaml
# 快速测试
max_beams: 30

# 一般应用（推荐）
max_beams: 60

# 高精度建图
max_beams: 90

# 最大值（不建议超过）
max_beams: 100
```

**重要限制**：
- GMapping 源码中有硬编码限制 `LASER_MAXBEAMS`（通常 512）
- 如果输入激光点数超过此限制会崩溃
- 解决方法：使用降采样节点预处理

**性能影响**：
- 计算复杂度 ∝ `max_beams * particles`
- 30 → 60: 计算量增加约 2 倍
- 60 → 90: 计算量增加约 1.5 倍

#### minimumScore

**含义**：扫描匹配的最小得分阈值

**作用**：
- 低于此得分的匹配被拒绝
- 防止错误的扫描匹配

**默认值**：0.0

**调优**：
```yaml
# 一般环境
minimumScore: 0.0

# 高动态环境（机器人快速运动）
minimumScore: 50.0

# 重复性结构（走廊、仓库）
minimumScore: 100.0
```

---

### 2. 粒子滤波器参数

#### particles

**含义**：粒子滤波器的粒子数量

**作用**：
- 每个粒子代表一个可能的机器人位姿假设
- 越多越准确，但内存和计算量越大

**推荐值**：
```yaml
# 小环境，路径简单
particles: 10

# 中等环境（推荐）
particles: 30

# 大环境，路径复杂
particles: 50

# 对称性高的环境
particles: 80

# 极限值（谨慎使用）
particles: 100
```

**内存占用**：
- 每个粒子维护一个完整的地图
- 内存占用 ≈ `particles × map_size`
- 30个粒子，100×100米地图，分辨率0.05：约 180MB

**经验法则**：
- 环境越大 → 粒子越多
- 对称性越高 → 粒子越多
- 传感器噪声越大 → 粒子越多

#### resampleThreshold

**含义**：触发重采样的有效粒子数阈值

**作用**：
- 当有效粒子数 < `particles × resampleThreshold` 时触发重采样
- 防止粒子退化

**默认值**：0.5

**调优**：
```yaml
# 保守策略（较少重采样）
resampleThreshold: 0.3

# 标准策略（推荐）
resampleThreshold: 0.5

# 激进策略（频繁重采样）
resampleThreshold: 0.7
```

---

### 3. 地图参数

#### delta

**含义**：地图分辨率（米/像素）

**作用**：
- 每个栅格的物理尺寸
- 影响地图精度和内存占用

**推荐值**：
```yaml
# 粗糙地图（快速测试）
delta: 0.1        # 10cm/像素

# 标准地图（推荐）
delta: 0.05       # 5cm/像素

# 高精度地图
delta: 0.025      # 2.5cm/像素

# 超高精度（谨慎使用）
delta: 0.01       # 1cm/像素
```

**内存影响**：
```python
# 100×100米环境
delta=0.1:  1000×1000 像素 = 1MB
delta=0.05: 2000×2000 像素 = 4MB
delta=0.025: 4000×4000 像素 = 16MB
delta=0.01: 10000×10000 像素 = 100MB

# 每个粒子都有一个地图副本
总内存 = 单个地图大小 × particles
```

#### xmin, ymin, xmax, ymax

**含义**：地图的初始边界（米）

**作用**：
- 定义初始地图范围
- 实际建图时会自动扩展

**推荐设置**：
```yaml
# 对称设置（推荐）
xmin: -50.0
ymin: -50.0
xmax: 50.0
ymax: 50.0

# 小环境
xmin: -10.0
ymin: -10.0
xmax: 10.0
ymax: 10.0

# 大型仓库
xmin: -100.0
ymin: -100.0
xmax: 100.0
ymax: 100.0
```

---

### 4. 运动模型参数

运动模型描述里程计误差：

#### srr (sigma range range)

**含义**：平移运动引起的平移误差（相对值）

**物理意义**：机器人前进1米，位置误差的标准差

**默认值**：0.1

**调优**：
```yaml
# 高质量里程计（轮式编码器 + IMU）
srr: 0.05

# 标准里程计（推荐）
srr: 0.1

# 低质量里程计（仅估算）
srr: 0.2
```

#### srt (sigma range theta)

**含义**：旋转运动引起的平移误差

**物理意义**：机器人旋转1弧度，位置误差的标准差

**默认值**：0.2

**调优**：
```yaml
# 高质量里程计
srt: 0.1

# 标准里程计（推荐）
srt: 0.2

# 低质量里程计
srt: 0.4
```

#### str (sigma theta range)

**含义**：平移运动引起的旋转误差

**物理意义**：机器人前进1米，角度误差的标准差

**默认值**：0.1

**调优**：
```yaml
# 高质量里程计
str: 0.05

# 标准里程计（推荐）
str: 0.1

# 低质量里程计
str: 0.2
```

#### stt (sigma theta theta)

**含义**：旋转运动引起的旋转误差

**物理意义**：机器人旋转1弧度，角度误差的标准差

**默认值**：0.2

**调优**：
```yaml
# 高质量里程计
stt: 0.1

# 标准里程计（推荐）
stt: 0.2

# 低质量里程计
stt: 0.4
```

**里程计误差规则**：
- 里程计越差 → 参数越大
- 环境越复杂 → 参数略增大
- 四个参数通常成对调整：(srr, srt) 和 (str, stt)

---

### 5. 更新参数

#### linearUpdate

**含义**：触发地图更新的最小线性移动距离（米）

**作用**：
- 移动距离 < linearUpdate 时不更新地图
- 减少计算量，避免原地更新

**推荐值**：
```yaml
# 高频更新（计算量大）
linearUpdate: 0.1

# 标准更新（推荐）
linearUpdate: 0.2

# 低频更新（快速运动）
linearUpdate: 0.5

# 测试/演示（频繁更新）
linearUpdate: 0.05
```

#### angularUpdate

**含义**：触发地图更新的最小旋转角度（弧度）

**作用**：
- 旋转角度 < angularUpdate 时不更新地图

**推荐值**：
```yaml
# 高频更新
angularUpdate: 0.05  # ~3度

# 标准更新（推荐）
angularUpdate: 0.1   # ~6度

# 低频更新
angularUpdate: 0.2   # ~11度
```

#### temporalUpdate

**含义**：触发地图更新的最大时间间隔（秒）

**作用**：
- 即使位置没变，超过此时间也强制更新
- 防止长时间不更新

**推荐值**：
```yaml
# 频繁更新
temporalUpdate: 1.0

# 标准更新（推荐）
temporalUpdate: 3.0

# 稀疏更新
temporalUpdate: 5.0

# 禁用时间更新
temporalUpdate: -1.0
```

**平衡考虑**：
- 更新越频繁 → 地图越精细，计算量越大
- 更新越稀疏 → 计算量小，可能错过重要特征

---

### 6. 扫描匹配器参数

#### iterations

**含义**：扫描匹配器的迭代次数

**作用**：
- 迭代优化扫描配准
- 越多越精确但越慢

**默认值**：5

**调优**：
```yaml
# 快速测试
iterations: 3

# 标准配置（推荐）
iterations: 5

# 高精度配准
iterations: 10
```

#### lsigma

**含义**：似然函数的标准差

**默认值**：0.075

**调优**：
```yaml
# 精确匹配
lsigma: 0.05

# 标准配置（推荐）
lsigma: 0.075

# 宽松匹配
lsigma: 0.1
```

#### lstep

**含义**：线性搜索步长（米）

**默认值**：0.05

#### astep

**含义**：角度搜索步长（弧度）

**默认值**：0.05

---

## 场景化配置模板

### 场景1：小房间（< 20㎡）

```yaml
slam_gmapping:
  ros__parameters:
    # 激光参数
    maxUrange: 5.0
    maxRange: 12.0
    max_beams: 30
    
    # 粒子滤波
    particles: 10
    
    # 地图参数
    delta: 0.05
    xmin: -10.0
    ymin: -10.0
    xmax: 10.0
    ymax: 10.0
    
    # 运动模型（标准）
    srr: 0.1
    srt: 0.2
    str: 0.1
    stt: 0.2
    
    # 更新参数
    linearUpdate: 0.1
    angularUpdate: 0.1
    temporalUpdate: 1.0
```

### 场景2：办公室/实验室（50-200㎡）

```yaml
slam_gmapping:
  ros__parameters:
    # 激光参数
    maxUrange: 10.0
    maxRange: 40.0
    max_beams: 60
    
    # 粒子滤波
    particles: 30
    
    # 地图参数
    delta: 0.05
    xmin: -20.0
    ymin: -20.0
    xmax: 20.0
    ymax: 20.0
    
    # 运动模型
    srr: 0.1
    srt: 0.2
    str: 0.1
    stt: 0.2
    
    # 更新参数
    linearUpdate: 0.2
    angularUpdate: 0.1
    temporalUpdate: 3.0
    
    # 扫描匹配
    iterations: 5
```

### 场景3：大型室内空间（> 500㎡）

```yaml
slam_gmapping:
  ros__parameters:
    # 激光参数
    maxUrange: 15.0
    maxRange: 40.0
    max_beams: 90
    
    # 粒子滤波（增加以应对复杂环境）
    particles: 50
    
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
    linearUpdate: 0.3
    angularUpdate: 0.1
    temporalUpdate: 5.0
```

### 场景4：对称性强的环境（走廊、仓库）

```yaml
slam_gmapping:
  ros__parameters:
    # 激光参数
    maxUrange: 12.0
    maxRange: 40.0
    max_beams: 90
    
    # 粒子滤波（增加以处理歧义性）
    particles: 80
    resampleThreshold: 0.5
    
    # 地图参数
    delta: 0.05
    
    # 运动模型（信任里程计）
    srr: 0.05
    srt: 0.1
    str: 0.05
    stt: 0.1
    
    # 更新参数（频繁更新）
    linearUpdate: 0.1
    angularUpdate: 0.05
    temporalUpdate: 2.0
    
    # 扫描匹配（提高精度）
    minimumScore: 100.0
    iterations: 10
```

### 场景5：室外环境

```yaml
slam_gmapping:
  ros__parameters:
    # 激光参数（利用远距离）
    maxUrange: 20.0
    maxRange: 40.0
    max_beams: 90
    
    # 粒子滤波
    particles: 50
    
    # 地图参数（大范围）
    delta: 0.1  # 粗分辨率
    xmin: -100.0
    ymin: -100.0
    xmax: 100.0
    ymax: 100.0
    
    # 运动模型（GPS辅助可降低）
    srr: 0.15
    srt: 0.3
    str: 0.15
    stt: 0.3
    
    # 更新参数
    linearUpdate: 0.5
    angularUpdate: 0.2
    temporalUpdate: 5.0
```

---

## 调优流程

### 第一步：确定基础参数

1. **测量环境大小** → 设置 `xmin/ymin/xmax/ymax`
2. **查看雷达规格** → 设置 `maxRange`
3. **评估实际需求** → 设置 `maxUrange` (环境大小的70-80%)
4. **选择分辨率** → 设置 `delta` (通常 0.05)

### 第二步：设置合理起点

使用推荐的标准配置：
```yaml
particles: 30
max_beams: 60
linearUpdate: 0.2
angularUpdate: 0.1
```

### 第三步：识别问题并调整

| 现象 | 可能原因 | 调整方法 |
|------|---------|---------|
| 地图重影、有鬼影 | 粒子数不足 | 增加 `particles` |
| 闭环时错位 | 扫描匹配不准 | 增加 `max_beams`<br>增加 `iterations` |
| 走廊中迷失 | 对称性歧义 | 增加 `particles`<br>提高 `minimumScore` |
| 快速运动时失败 | 更新不及时 | 减小 `linearUpdate`<br>减小 `angularUpdate` |
| 地图抖动 | 里程计误差大 | 增大 `srr/srt/str/stt` |
| 计算太慢 | 参数过大 | 减少 `particles`<br>减少 `max_beams` |
| 远处墙不清晰 | 有效距离不足 | 增加 `maxUrange` |

### 第四步：微调优化

1. **观察建图质量**
   - 墙壁是否清晰？
   - 是否有重影？
   - 闭环是否准确？

2. **检查系统负载**
   ```bash
   # 查看CPU使用率
   htop
   
   # 查看内存使用
   free -h
   
   # 查看节点负载
   ros2 run tf2_tools view_frames
   ```

3. **记录测试结果**
   - 不同参数组合的效果
   - 建图所用时间
   - 地图精度评估

---

## 性能优化建议

### CPU 优化

```yaml
# 减少计算量
particles: 20        # 从 30 降到 20
max_beams: 45        # 从 60 降到 45
iterations: 3        # 从 5 降到 3

# 减少更新频率
linearUpdate: 0.3    # 从 0.2 提高到 0.3
angularUpdate: 0.15  # 从 0.1 提高到 0.15
```

### 内存优化

```yaml
# 降低地图分辨率
delta: 0.1           # 从 0.05 提高到 0.1

# 减少粒子数
particles: 20        # 从 30 降到 20

# 限制地图范围
xmin: -30.0
xmax: 30.0
ymin: -30.0
ymax: 30.0
```

### 精度优化

```yaml
# 提高采样
max_beams: 90        # 从 60 提高到 90
particles: 50        # 从 30 提高到 50

# 提高地图分辨率
delta: 0.025         # 从 0.05 降低到 0.025

# 优化扫描匹配
iterations: 10       # 从 5 提高到 10
minimumScore: 100.0
```

---

## 调试技巧

### 1. 可视化粒子分布

在 RViz 中添加 PoseArray 显示：
```yaml
# 订阅话题
/particlecloud
```

观察：
- 粒子是否收敛到一个位置？
- 是否有多个聚类？（对称性问题）
- 粒子是否发散？（里程计误差大）

### 2. 监控日志输出

```bash
ros2 launch slam_gmapping ... | grep -E "(score|neff|particles)"
```

关注：
- `score`: 扫描匹配得分，应该稳定
- `neff`: 有效粒子数，不应太低
- `update`: 更新频率，是否合理

### 3. 录制数据包

```bash
# 录制
ros2 bag record /scan /odom /tf

# 回放测试不同参数
ros2 bag play xxx.bag
```

### 4. 定量评估

```bash
# 使用 map_server 保存地图
ros2 run nav2_map_server map_saver_cli -f test_map

# 对比不同参数的地图质量
# 评估指标：
# - 墙壁清晰度
# - 重影程度
# - 闭环一致性
```

---

## 常见问题与解决

### Q1: 地图有多个重叠的影子

**原因**：粒子数不足，多个假设共存

**解决**：
```yaml
particles: 50  # 从 30 增加到 50
```

### Q2: 建图时机器人"跳跃"

**原因**：扫描匹配失败

**解决**：
```yaml
max_beams: 90           # 提高采样
minimumScore: 50.0      # 设置最小得分
iterations: 10          # 增加迭代
```

### Q3: 闭环时无法正确对齐

**原因**：局部匹配陷入局部最优

**解决**：
```yaml
particles: 80           # 增加粒子
resampleThreshold: 0.3  # 减少重采样
lsigma: 0.05           # 提高匹配精度
```

### Q4: 系统太慢，无法实时建图

**原因**：计算量过大

**解决**：
```yaml
particles: 20      # 减少粒子
max_beams: 30      # 减少采样
delta: 0.1         # 降低分辨率
linearUpdate: 0.5  # 减少更新
```

---

## 总结

### 核心原则

1. **从标准配置开始**
2. **根据实际问题逐步调整**
3. **平衡精度和性能**
4. **记录测试结果**

### 关键参数优先级

**高优先级**（影响最大）：
- `particles` - 决定鲁棒性
- `max_beams` - 决定匹配精度
- `maxUrange` - 决定建图范围

**中优先级**：
- `delta` - 地图精度
- `linearUpdate/angularUpdate` - 更新频率
- 运动模型参数 - 里程计信任度

**低优先级**（微调）：
- `iterations` - 匹配迭代
- `minimumScore` - 匹配阈值
- `temporalUpdate` - 时间更新

### 快速参考

| 环境类型 | particles | max_beams | maxUrange | delta |
|---------|-----------|-----------|-----------|-------|
| 小房间 | 10-20 | 30 | 5.0 | 0.05 |
| 办公室 | 30 | 60 | 10.0 | 0.05 |
| 大空间 | 50 | 90 | 15.0 | 0.05 |
| 对称环境 | 80 | 90 | 12.0 | 0.025 |

---

**文档版本**: 1.0  
**适用 GMapping 版本**: openslam_gmapping  
**最后更新**: 2025-10-31

