---
title: ROS 机器人系统命令行完整指南
weight: 7
bookToc: true
---

# ROS 机器人系统命令行完整指南

本指南详细介绍 ROS (Robot Operating System) 和 ROS 2 的命令行工具和操作，适用于机器人开发、仿真和调试。

## 1. ROS 版本说明

### 1.1 ROS 1 vs ROS 2

**ROS 1（主要版本）**:
- Melodic Morenia (Ubuntu 18.04) - 支持到 2023 年
- Noetic Ninjemys (Ubuntu 20.04) - 支持到 2025 年

**ROS 2（推荐新项目使用）**:
- Foxy Fitzroy (Ubuntu 20.04) - LTS
- Humble Hawksbill (Ubuntu 22.04) - LTS
- Iron Irwini (Ubuntu 22.04)
- Jazzy Jalisco (Ubuntu 24.04) - 最新版

### 1.2 选择建议

```bash
# 检查 Ubuntu 版本
lsb_release -a

# Ubuntu 20.04 → ROS Noetic 或 ROS 2 Foxy/Humble
# Ubuntu 22.04 → ROS 2 Humble (推荐)
# 新项目 → ROS 2
# 维护旧项目 → ROS 1
```

## 2. ROS 安装

### 2.1 ROS 1 Noetic 安装（Ubuntu 20.04）

```bash
# 1. 设置软件源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2. 添加密钥
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 3. 更新并安装
sudo apt update
sudo apt install ros-noetic-desktop-full

# 4. 环境设置
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. 安装依赖工具
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# 6. 初始化 rosdep
sudo rosdep init
rosdep update
```

### 2.2 ROS 2 Humble 安装（Ubuntu 22.04）

```bash
# 1. 设置 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 添加 ROS 2 仓库
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装 ROS 2
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# 4. 环境设置
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. 安装开发工具
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep2
```

### 2.3 验证安装

```bash
# ROS 1
rosversion -d        # 显示版本
roscore              # 启动 ROS 核心（另开终端测试）

# ROS 2
ros2 --version       # 显示版本
ros2 run demo_nodes_cpp talker    # 测试节点
```

## 3. ROS 1 核心命令

### 3.1 roscore - ROS 主节点

```bash
# 启动 ROS 主节点（必须先启动）
roscore

# 在后台启动
roscore &

# 指定端口
roscore -p 11312
```

### 3.2 rosrun - 运行节点

```bash
# 基本语法
rosrun <package_name> <node_name>

# 示例
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key

# 重命名节点
rosrun turtlesim turtlesim_node __name:=my_turtle

# 设置参数
rosrun turtlesim turtlesim_node _background_r:=255
```

### 3.3 roslaunch - 启动文件

```bash
# 启动 launch 文件
roslaunch <package_name> <launch_file>

# 示例
roslaunch turtlesim multisim.launch

# 设置参数
roslaunch turtlesim multisim.launch use_sim_time:=true

# 查看 launch 文件参数
roslaunch <package_name> <launch_file> --ros-args
```

### 3.4 rosnode - 节点管理

```bash
# 列出所有运行的节点
rosnode list

# 查看节点信息
rosnode info /node_name

# 测试节点连接
rosnode ping /node_name

# 杀死节点
rosnode kill /node_name

# 清理已死节点
rosnode cleanup
```

### 3.5 rostopic - 话题操作

```bash
# 列出所有话题
rostopic list

# 查看话题信息
rostopic info /topic_name

# 订阅话题（显示消息）
rostopic echo /topic_name

# 发布消息
rostopic pub /topic_name std_msgs/String "data: 'Hello ROS'"
rostopic pub -r 10 /topic_name std_msgs/Int32 "data: 1"  # 以 10Hz 频率发布

# 查看话题发布频率
rostopic hz /topic_name

# 查看话题带宽
rostopic bw /topic_name

# 查看消息类型
rostopic type /topic_name

# 查找使用特定消息类型的话题
rostopic find std_msgs/String
```

### 3.6 rosservice - 服务操作

```bash
# 列出所有服务
rosservice list

# 调用服务
rosservice call /service_name [args]

# 示例：清除 turtlesim 轨迹
rosservice call /clear

# 示例：生成新乌龟
rosservice call /spawn 2 2 0.2 "new_turtle"

# 查看服务类型
rosservice type /service_name

# 查找特定类型的服务
rosservice find std_srvs/Empty

# 查看服务参数
rosservice args /service_name

# 查看服务详细信息
rosservice info /service_name
```

### 3.7 rosmsg - 消息操作

```bash
# 列出所有消息类型
rosmsg list

# 显示消息定义
rosmsg show std_msgs/String
rosmsg show geometry_msgs/Twist

# 查找包含特定字段的消息
rosmsg grep "float64"

# 查看消息所在的包
rosmsg package std_msgs

# 搜索消息
rosmsg search String
```

### 3.8 rossrv - 服务消息操作

```bash
# 列出所有服务类型
rossrv list

# 显示服务定义
rossrv show std_srvs/Empty
rossrv show turtlesim/Spawn

# 查看服务所在的包
rossrv package std_srvs
```

### 3.9 rosparam - 参数服务器

```bash
# 列出所有参数
rosparam list

# 获取参数值
rosparam get /parameter_name
rosparam get /   # 获取所有参数

# 设置参数
rosparam set /parameter_name value

# 删除参数
rosparam delete /parameter_name

# 从文件加载参数
rosparam load file.yaml

# 导出参数到文件
rosparam dump file.yaml

# 导出特定命名空间的参数
rosparam dump file.yaml /namespace
```

### 3.10 rospack - 包管理

```bash
# 查找包路径
rospack find <package_name>

# 列出所有包
rospack list

# 列出包依赖
rospack depends <package_name>

# 列出依赖树
rospack depends-on <package_name>

# 查看包的清单文件
rospack export --lang=cpp --attrib=cflags <package_name>
```

### 3.11 roscd - 切换到包目录

```bash
# 切换到包目录
roscd <package_name>

# 切换到子目录
roscd turtlesim/msg

# 切换到日志目录
roscd log
```

### 3.12 rosbag - 数据记录与回放

```bash
# 记录所有话题
rosbag record -a

# 记录特定话题
rosbag record /topic1 /topic2

# 记录到指定文件
rosbag record -O output.bag /topic_name

# 回放 bag 文件
rosbag play file.bag

# 以不同速度回放
rosbag play -r 2 file.bag    # 2倍速
rosbag play -r 0.5 file.bag  # 0.5倍速

# 查看 bag 文件信息
rosbag info file.bag

# 过滤话题回放
rosbag play file.bag --topics /topic1 /topic2

# 从特定时间开始回放
rosbag play file.bag -s 10   # 从第10秒开始
```

### 3.13 catkin - 工作空间管理

```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# 编译工作空间
cd ~/catkin_ws
catkin_make

# 编译特定包
catkin_make --pkg package_name

# 清理编译
catkin_make clean

# 设置环境
source ~/catkin_ws/devel/setup.bash

# 创建包
cd ~/catkin_ws/src
catkin_create_pkg my_package rospy roscpp std_msgs

# 使用 catkin_tools（推荐）
sudo apt install python3-catkin-tools

# 初始化工作空间
catkin init

# 编译
catkin build

# 编译特定包
catkin build package_name

# 清理
catkin clean
```

## 4. ROS 2 核心命令

### 4.1 ros2 run - 运行节点

```bash
# 基本语法
ros2 run <package_name> <executable_name>

# 示例
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

# 重命名节点
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

# 设置参数
ros2 run turtlesim turtlesim_node --ros-args -p background_r:=255
```

### 4.2 ros2 launch - 启动文件

```bash
# 启动 launch 文件
ros2 launch <package_name> <launch_file>

# 示例
ros2 launch turtlesim multisim.launch.py

# 设置参数
ros2 launch turtlesim multisim.launch.py use_sim_time:=true

# 显示启动文件参数
ros2 launch <package_name> <launch_file> --show-args
```

### 4.3 ros2 node - 节点管理

```bash
# 列出所有节点
ros2 node list

# 查看节点信息
ros2 node info /node_name

# 查看节点输出
ros2 node output /node_name
```

### 4.4 ros2 topic - 话题操作

```bash
# 列出所有话题
ros2 topic list

# 列出话题及其类型
ros2 topic list -t

# 查看话题信息
ros2 topic info /topic_name

# 订阅话题
ros2 topic echo /topic_name

# 发布消息
ros2 topic pub /topic_name std_msgs/msg/String "{data: 'Hello ROS 2'}"
ros2 topic pub --rate 10 /topic_name std_msgs/msg/Int32 "{data: 1}"

# 查看话题频率
ros2 topic hz /topic_name

# 查看话题带宽
ros2 topic bw /topic_name

# 查看话题类型
ros2 topic type /topic_name

# 查找话题
ros2 topic find std_msgs/msg/String
```

### 4.5 ros2 service - 服务操作

```bash
# 列出所有服务
ros2 service list

# 列出服务及其类型
ros2 service list -t

# 调用服务
ros2 service call /service_name service_type "{request}"

# 示例
ros2 service call /clear std_srvs/srv/Empty
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'new_turtle'}"

# 查看服务类型
ros2 service type /service_name

# 查找服务
ros2 service find std_srvs/srv/Empty
```

### 4.6 ros2 interface - 接口操作

```bash
# 列出所有接口
ros2 interface list

# 显示接口定义
ros2 interface show std_msgs/msg/String
ros2 interface show geometry_msgs/msg/Twist

# 查看包中的接口
ros2 interface package std_msgs

# 查找接口
ros2 interface proto std_msgs/msg/String
```

### 4.7 ros2 param - 参数操作

```bash
# 列出所有参数
ros2 param list

# 获取参数值
ros2 param get /node_name parameter_name

# 设置参数
ros2 param set /node_name parameter_name value

# 删除参数
ros2 param delete /node_name parameter_name

# 导出参数到文件
ros2 param dump /node_name --output-dir ./

# 从文件加载参数
ros2 param load /node_name file.yaml
```

### 4.8 ros2 bag - 数据记录与回放

```bash
# 记录所有话题
ros2 bag record -a

# 记录特定话题
ros2 bag record /topic1 /topic2

# 记录到指定文件
ros2 bag record -o output_bag /topic_name

# 回放 bag 文件
ros2 bag play file_name

# 查看 bag 文件信息
ros2 bag info file_name

# 以不同速度回放
ros2 bag play file_name --rate 2.0

# 循环回放
ros2 bag play file_name --loop
```

### 4.9 ros2 pkg - 包管理

```bash
# 创建包
ros2 pkg create my_package --build-type ament_python
ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp std_msgs

# 列出所有包
ros2 pkg list

# 查找包路径
ros2 pkg prefix <package_name>

# 查看包的可执行文件
ros2 pkg executables <package_name>

# 查看包的 XML 信息
ros2 pkg xml <package_name>
```

### 4.10 colcon - 编译工具

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 编译所有包
colcon build

# 编译特定包
colcon build --packages-select package_name

# 并行编译
colcon build --parallel-workers 4

# 符号链接安装（Python 包推荐）
colcon build --symlink-install

# 测试
colcon test

# 查看测试结果
colcon test-result --all

# 清理
rm -rf build install log
```

## 5. 可视化工具

### 5.1 rqt - ROS 图形界面

```bash
# 启动 rqt
rqt

# 常用插件
rqt_graph          # 节点关系图
rqt_plot           # 数据绘图
rqt_console        # 日志查看器
rqt_bag            # bag 文件查看器
rqt_image_view     # 图像查看器
rqt_reconfigure    # 动态参数配置

# ROS 2
ros2 run rqt_graph rqt_graph
ros2 run rqt_plot rqt_plot
```

### 5.2 rviz - 3D 可视化

```bash
# ROS 1
rosrun rviz rviz
rviz

# 使用配置文件
rviz -d config_file.rviz

# ROS 2
ros2 run rviz2 rviz2
rviz2
```

### 5.3 Gazebo - 机器人仿真

```bash
# 启动 Gazebo
gazebo

# 启动空世界
gazebo --verbose

# 加载世界文件
gazebo worlds/willowgarage.world

# ROS 1 集成
roslaunch gazebo_ros empty_world.launch

# ROS 2 集成
ros2 launch gazebo_ros gazebo.launch.py
```

## 6. 调试工具

### 6.1 rostopic 调试（ROS 1）

```bash
# 实时监控话题
rostopic echo /topic_name | grep "value"

# 输出到文件
rostopic echo -b output.bag -p /topic_name > data.csv

# 延迟测试
rostopic delay /topic_name
```

### 6.2 roswtf - 诊断工具

```bash
# 检查 ROS 系统
roswtf

# 检查特定包
roswtf <package_name>
```

### 6.3 日志系统

```bash
# ROS 1
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level

# ROS 2
ros2 run rqt_console rqt_console

# 命令行查看日志级别
ros2 node list
ros2 param get /node_name use_sim_time
```

## 7. TF（坐标变换）

### 7.1 tf 工具（ROS 1）

```bash
# 查看 TF 树
rosrun tf view_frames
evince frames.pdf

# 实时查看 TF
rosrun tf tf_echo source_frame target_frame

# TF 监控
rosrun tf tf_monitor

# 静态 TF 发布
rosrun tf static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period
```

### 7.2 tf2 工具（ROS 2）

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看特定变换
ros2 run tf2_ros tf2_echo source_frame target_frame

# 静态 TF 发布
ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
```

## 8. 实用技巧

### 8.1 环境管理

```bash
# 创建别名
echo "alias ros1='source /opt/ros/noetic/setup.bash'" >> ~/.bashrc
echo "alias ros2='source /opt/ros/humble/setup.bash'" >> ~/.bashrc
echo "alias ws='source ~/catkin_ws/devel/setup.bash'" >> ~/.bashrc

# 查看当前 ROS 环境
printenv | grep ROS

# 切换工作空间
source ~/catkin_ws/devel/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 8.2 多机通信

```bash
# ROS 1 - 主机设置
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.100

# ROS 1 - 从机设置
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.101

# ROS 2 - 使用 DDS
export ROS_DOMAIN_ID=0  # 0-101，同一网络使用相同 ID
```

### 8.3 性能分析

```bash
# 话题带宽监控
rostopic bw /topic_name

# 节点 CPU 使用率
top -p $(pgrep -d',' -f node_name)

# 使用 perf 工具
sudo apt install linux-tools-common
perf record ros2 run package_name node_name
perf report
```

### 8.4 实用脚本

```bash
# 创建启动所有节点的脚本
cat << 'EOF' > ~/start_robot.sh
#!/bin/bash
source ~/catkin_ws/devel/setup.bash
roslaunch my_robot robot.launch &
sleep 5
roslaunch my_navigation navigation.launch &
EOF

chmod +x ~/start_robot.sh
```

## 9. 常见问题排查

### 9.1 ROS 1 问题

```bash
# roscore 无法启动
# 检查端口占用
netstat -tulpn | grep 11311
# 杀死进程
killall -9 roscore rosmaster

# 找不到包
rospack find package_name
source ~/catkin_ws/devel/setup.bash

# 话题无数据
rostopic list
rostopic info /topic_name
rostopic hz /topic_name
```

### 9.2 ROS 2 问题

```bash
# 节点无法发现
# 检查 DDS
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

# 多播问题
export ROS_LOCALHOST_ONLY=1  # 仅本地通信

# 编译问题
colcon build --symlink-install --cmake-clean-cache
```

### 9.3 网络问题

```bash
# 测试连接
ping 192.168.1.100

# 检查防火墙
sudo ufw status
sudo ufw allow from 192.168.1.0/24

# 检查 DNS
nslookup hostname
```

## 10. 快速参考

### ROS 1 常用命令

```bash
roscore                                    # 启动主节点
rosrun package_name node_name              # 运行节点
roslaunch package_name launch_file         # 启动 launch 文件
rostopic list/echo/pub/hz                  # 话题操作
rosnode list/info/kill                     # 节点管理
rosservice list/call                       # 服务操作
rosparam list/get/set                      # 参数操作
rosbag record/play/info                    # 数据记录
catkin_make                                # 编译工作空间
```

### ROS 2 常用命令

```bash
ros2 run package_name executable           # 运行节点
ros2 launch package_name launch_file       # 启动 launch 文件
ros2 topic list/echo/pub/hz                # 话题操作
ros2 node list/info                        # 节点管理
ros2 service list/call                     # 服务操作
ros2 param list/get/set                    # 参数操作
ros2 bag record/play/info                  # 数据记录
colcon build                               # 编译工作空间
```

## 11. 学习资源

### 官方文档
- [ROS Wiki](http://wiki.ros.org/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

### 书籍推荐
- Programming Robots with ROS
- A Gentle Introduction to ROS
- ROS Robotics Projects

### 在线课程
- [ROS for Beginners](https://www.udemy.com/course/ros-essentials/)
- [The Construct](https://www.theconstructsim.com/)
- [ROS Industrial Training](https://industrial-training-master.readthedocs.io/)

### 社区资源
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)
- [GitHub ROS](https://github.com/ros)
- [ROS 中文社区](https://www.guyuehome.com/)

## 总结

本指南涵盖了 ROS 机器人系统的核心命令和工具：

1. ✅ **安装配置** - ROS 1 和 ROS 2 安装
2. ✅ **核心命令** - 节点、话题、服务、参数操作
3. ✅ **可视化工具** - rqt、rviz、Gazebo
4. ✅ **调试工具** - 日志、诊断、性能分析
5. ✅ **工作空间** - catkin 和 colcon 编译
6. ✅ **数据记录** - rosbag 使用
7. ✅ **TF 变换** - 坐标系管理
8. ✅ **故障排查** - 常见问题解决

### 学习路线建议

1. 📚 **基础阶段**：安装 ROS → 运行示例 → 理解话题和节点
2. 🔧 **开发阶段**：创建包 → 编写节点 → 使用 launch 文件
3. 🤖 **进阶阶段**：TF 变换 → 导航 → 机械臂控制
4. 🚀 **实战阶段**：仿真测试 → 实体机器人 → 项目开发

---

*祝你在 ROS 机器人开发之路上一切顺利！*
