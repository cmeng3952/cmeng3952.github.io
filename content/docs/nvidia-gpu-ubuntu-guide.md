---
title: Ubuntu 系统 NVIDIA 显卡完整指南
weight: 6
bookToc: true
---

# Ubuntu 系统 NVIDIA 显卡完整指南

本指南详细介绍在 Ubuntu Linux 系统下 NVIDIA 显卡的安装、配置、使用和故障排除，适用于游戏、深度学习、科学计算等场景。

## 1. 显卡检测与信息查看

### 1.1 检测 NVIDIA 显卡

```bash
# 查看 PCI 设备中的 NVIDIA 显卡
lspci | grep -i nvidia

# 查看 VGA 设备
lspci | grep -i vga

# 详细信息
lspci -v | grep -i nvidia

# 查看显卡型号
ubuntu-drivers devices
```

### 1.2 查看当前驱动状态

```bash
# 检查是否已安装 NVIDIA 驱动
nvidia-smi

# 查看驱动版本
cat /proc/driver/nvidia/version

# 查看加载的内核模块
lsmod | grep nvidia

# 检查 NVIDIA 服务状态
systemctl status nvidia-persistenced
```

### 1.3 显卡信息详解

```bash
# nvidia-smi 详细输出
nvidia-smi -L                    # 列出所有 GPU
nvidia-smi -q                    # 查询详细信息
nvidia-smi -q -d MEMORY          # 只显示内存信息
nvidia-smi -q -d TEMPERATURE     # 只显示温度信息
nvidia-smi -q -d POWER           # 只显示功耗信息
nvidia-smi -q -d CLOCK           # 只显示时钟频率

# 持续监控
nvidia-smi -l 1                  # 每秒刷新一次
watch -n 1 nvidia-smi            # 使用 watch 命令
```

## 2. NVIDIA 驱动安装

### 2.1 方法一：使用 ubuntu-drivers（推荐）

这是最简单和最安全的方法。

```bash
# 更新系统
sudo apt update
sudo apt upgrade

# 检测推荐的驱动
ubuntu-drivers devices

# 自动安装推荐的驱动
sudo ubuntu-drivers autoinstall

# 或手动安装特定版本（例如 nvidia-driver-535）
sudo apt install nvidia-driver-535

# 重启系统
sudo reboot
```

### 2.2 方法二：使用 Graphics Drivers PPA

适用于需要最新驱动的情况。

```bash
# 添加 PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# 查看可用驱动
ubuntu-drivers devices

# 安装驱动
sudo apt install nvidia-driver-545  # 替换为你需要的版本

# 重启
sudo reboot
```

### 2.3 方法三：从 NVIDIA 官网安装

> [!WARNING]
> 此方法可能导致系统更新时的冲突，不推荐新手使用。

```bash
# 1. 禁用 nouveau 驱动
sudo bash -c "echo blacklist nouveau > /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo bash -c "echo options nouveau modeset=0 >> /etc/modprobe.d/blacklist-nvidia-nouveau.conf"

# 更新内核初始化镜像
sudo update-initramfs -u

# 2. 安装必要的依赖
sudo apt install build-essential gcc-multilib dkms

# 3. 下载驱动（从 NVIDIA 官网）
# 访问 https://www.nvidia.com/Download/index.aspx
wget https://us.download.nvidia.com/XFree86/Linux-x86_64/535.129.03/NVIDIA-Linux-x86_64-535.129.03.run

# 4. 关闭图形界面
sudo systemctl isolate multi-user.target

# 5. 安装驱动
sudo chmod +x NVIDIA-Linux-x86_64-535.129.03.run
sudo ./NVIDIA-Linux-x86_64-535.129.03.run

# 6. 重启到图形界面
sudo systemctl start graphical.target

# 或直接重启
sudo reboot
```

### 2.4 验证安装

```bash
# 检查驱动版本
nvidia-smi

# 查看 NVIDIA 设置
nvidia-settings

# 检查 OpenGL
glxinfo | grep -i nvidia
glxinfo | grep "OpenGL version"

# 运行 CUDA 示例（如果已安装 CUDA）
/usr/local/cuda/extras/demo_suite/deviceQuery
```

## 3. CUDA 工具包安装

CUDA 是 NVIDIA 的并行计算平台，用于深度学习和科学计算。

### 3.1 CUDA 版本选择

访问 [CUDA Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive) 选择合适的版本。

**版本对应关系**：
- CUDA 12.x: 驱动 ≥ 525.60.13
- CUDA 11.8: 驱动 ≥ 520.61.05
- CUDA 11.7: 驱动 ≥ 515.43.04
- CUDA 11.6: 驱动 ≥ 510.39.01

### 3.2 安装 CUDA（deb 方式，推荐）

以 CUDA 12.3 为例：

```bash
# 1. 下载 CUDA 仓库包
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600

# 2. 添加 CUDA 仓库
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda-repo-ubuntu2204-12-3-local_12.3.0-545.23.06-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-3-local_12.3.0-545.23.06-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-3-local/cuda-*-keyring.gpg /usr/share/keyrings/

# 3. 更新并安装
sudo apt update
sudo apt install cuda-toolkit-12-3

# 或安装完整 CUDA（包括驱动）
sudo apt install cuda
```

### 3.3 安装 CUDA（runfile 方式）

```bash
# 下载 runfile
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda_12.3.0_545.23.06_linux.run

# 运行安装器
sudo sh cuda_12.3.0_545.23.06_linux.run

# 按照提示操作：
# - 如果已安装驱动，取消勾选 Driver
# - 勾选 CUDA Toolkit
# - 选择安装路径（默认 /usr/local/cuda-12.3）
```

### 3.4 配置环境变量

```bash
# 编辑 ~/.bashrc 或 ~/.zshrc
nano ~/.bashrc

# 添加以下内容
export PATH=/usr/local/cuda-12.3/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-12.3

# 重新加载配置
source ~/.bashrc

# 或创建符号链接（方便管理多版本）
sudo ln -sf /usr/local/cuda-12.3 /usr/local/cuda
```

### 3.5 验证 CUDA 安装

```bash
# 检查 CUDA 版本
nvcc --version

# 编译并运行示例
cd /usr/local/cuda/samples/1_Utilities/deviceQuery
sudo make
./deviceQuery

# 查看 CUDA 库
ls /usr/local/cuda/lib64/

# 测试 CUDA 示例
cd /usr/local/cuda/samples/1_Utilities/bandwidthTest
sudo make
./bandwidthTest
```

## 4. cuDNN 安装

cuDNN 是深度神经网络加速库，用于深度学习框架。

### 4.1 下载 cuDNN

1. 访问 [NVIDIA cuDNN](https://developer.nvidia.com/cudnn)
2. 注册/登录 NVIDIA Developer 账号
3. 下载对应 CUDA 版本的 cuDNN

### 4.2 安装 cuDNN（tar 包方式）

```bash
# 解压下载的文件
tar -xvf cudnn-linux-x86_64-8.x.x.x_cudaX.Y-archive.tar.xz

# 复制文件到 CUDA 目录
sudo cp cudnn-*-archive/include/cudnn*.h /usr/local/cuda/include
sudo cp -P cudnn-*-archive/lib/libcudnn* /usr/local/cuda/lib64
sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*
```

### 4.3 安装 cuDNN（deb 包方式）

```bash
# 安装运行时库
sudo dpkg -i cudnn-local-repo-ubuntu2204-8.x.x.x_1.0-1_amd64.deb

# 更新仓库
sudo cp /var/cudnn-local-repo-*/cudnn-local-*-keyring.gpg /usr/share/keyrings/
sudo apt update

# 安装 cuDNN
sudo apt install libcudnn8
sudo apt install libcudnn8-dev
sudo apt install libcudnn8-samples
```

### 4.4 验证 cuDNN

```bash
# 检查 cuDNN 文件
ls -l /usr/local/cuda/include/cudnn*.h
ls -l /usr/local/cuda/lib64/libcudnn*

# 编译并运行 cuDNN 示例
cp -r /usr/src/cudnn_samples_v8/ $HOME
cd $HOME/cudnn_samples_v8/mnistCUDNN
make clean && make
./mnistCUDNN
```

## 5. 性能监控与管理

### 5.1 nvidia-smi 高级用法

```bash
# 实时监控
nvidia-smi dmon -s pucvmet     # 监控功耗、利用率、时钟、显存、温度

# 查询特定 GPU
nvidia-smi -i 0                # 查询 GPU 0

# 查看进程
nvidia-smi pmon                # 进程监控
nvidia-smi pmon -s u           # 只显示利用率

# 查看拓扑结构
nvidia-smi topo -m

# 导出为 CSV
nvidia-smi --query-gpu=timestamp,name,utilization.gpu,utilization.memory,memory.total,memory.used,temperature.gpu --format=csv -l 1
```

### 5.2 设置 GPU 性能模式

```bash
# 设置持久模式（推荐服务器使用）
sudo nvidia-smi -pm 1

# 设置功耗限制（例如限制为 250W）
sudo nvidia-smi -pl 250

# 设置应用时钟
sudo nvidia-smi -ac 5001,1410  # 内存时钟,GPU时钟

# 重置设置
sudo nvidia-smi -rac
sudo nvidia-smi -pm 0
```

### 5.3 GPU 温度和风扇控制

```bash
# 查看温度
nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader

# 设置风扇速度（需要 nvidia-settings）
nvidia-settings -a "[gpu:0]/GPUFanControlState=1"
nvidia-settings -a "[fan:0]/GPUTargetFanSpeed=75"

# 使用 coolbits（允许超频和风扇控制）
sudo nvidia-xconfig --cool-bits=4
# 重启 X server
sudo systemctl restart gdm  # 或 lightdm
```

### 5.4 第三方监控工具

```bash
# nvtop - GPU 进程监控（类似 htop）
sudo apt install nvtop
nvtop

# gpustat - 简洁的 GPU 状态显示
pip install gpustat
gpustat -cp

# 实时监控
watch -n 1 gpustat -cp
```

## 6. 多 GPU 管理

### 6.1 查看多 GPU 配置

```bash
# 列出所有 GPU
nvidia-smi -L

# 查看 GPU 拓扑
nvidia-smi topo -m

# 查看 NVLink 状态
nvidia-smi nvlink --status

# 检查 GPU 之间的带宽
/usr/local/cuda/samples/1_Utilities/p2pBandwidthLatencyTest/p2pBandwidthLatencyTest
```

### 6.2 指定 GPU 运行程序

```bash
# 使用环境变量指定 GPU
CUDA_VISIBLE_DEVICES=0 python script.py        # 使用 GPU 0
CUDA_VISIBLE_DEVICES=1,2 python script.py      # 使用 GPU 1 和 2
CUDA_VISIBLE_DEVICES=-1 python script.py       # 不使用 GPU（CPU 模式）

# 在 Python 代码中设置
import os
os.environ["CUDA_VISIBLE_DEVICES"] = "0,1"
```

### 6.3 多 GPU 负载均衡

```bash
# 查看各 GPU 使用情况
nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv

# 使用脚本自动选择空闲 GPU
# 创建 select_gpu.py
cat << 'EOF' > select_gpu.py
import subprocess
import numpy as np

def get_free_gpu():
    gpu_stats = subprocess.check_output(
        ["nvidia-smi", "--format=csv,noheader,nounits",
         "--query-gpu=index,memory.used,memory.total"]
    ).decode('utf-8')
    
    gpu_stats = [line.split(',') for line in gpu_stats.strip().split('\n')]
    gpu_stats = [[int(x) for x in line] for line in gpu_stats]
    
    # 计算内存使用率
    gpu_usage = [(i, used/total) for i, used, total in gpu_stats]
    gpu_usage.sort(key=lambda x: x[1])
    
    return gpu_usage[0][0]  # 返回最空闲的 GPU

if __name__ == "__main__":
    print(get_free_gpu())
EOF

# 使用
GPU_ID=$(python select_gpu.py)
CUDA_VISIBLE_DEVICES=$GPU_ID python script.py
```

## 7. 深度学习环境配置

### 7.1 PyTorch 安装

```bash
# 访问 https://pytorch.org/ 获取安装命令

# CUDA 12.1
pip3 install torch torchvision torchaudio

# CUDA 11.8
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# 验证 PyTorch 可以使用 GPU
python3 -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

### 7.2 TensorFlow 安装

```bash
# TensorFlow 2.x（自动支持 GPU）
pip install tensorflow

# 验证 GPU 支持
python3 -c "import tensorflow as tf; print(tf.config.list_physical_devices('GPU'))"

# 测试简单计算
python3 << EOF
import tensorflow as tf
with tf.device('/GPU:0'):
    a = tf.constant([[1.0, 2.0], [3.0, 4.0]])
    b = tf.constant([[1.0, 1.0], [0.0, 1.0]])
    c = tf.matmul(a, b)
    print(c)
EOF
```

### 7.3 Conda 虚拟环境

```bash
# 安装 Miniconda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# 创建环境
conda create -n ml python=3.10
conda activate ml

# 安装 CUDA 工具包（conda 版本）
conda install cudatoolkit=11.8 -c nvidia

# 安装深度学习框架
conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia
```

### 7.4 Docker GPU 支持

```bash
# 安装 NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker

# 测试 GPU Docker
docker run --rm --gpus all nvidia/cuda:12.3.0-base-ubuntu22.04 nvidia-smi

# 运行支持 GPU 的容器
docker run --gpus all -it tensorflow/tensorflow:latest-gpu bash
```

## 8. 常见问题排查

### 8.1 驱动安装失败

```bash
# 检查 nouveau 是否被禁用
lsmod | grep nouveau

# 如果仍然加载，手动禁用
sudo bash -c "echo blacklist nouveau > /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo bash -c "echo options nouveau modeset=0 >> /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo update-initramfs -u
sudo reboot

# 检查 Secure Boot 状态
mokutil --sb-state

# 如果启用了 Secure Boot，需要禁用或注册 NVIDIA 模块
```

### 8.2 nvidia-smi 不工作

```bash
# 检查驱动是否加载
lsmod | grep nvidia

# 尝试重新加载模块
sudo modprobe nvidia

# 检查驱动版本冲突
dpkg -l | grep nvidia

# 完全卸载并重装驱动
sudo apt purge nvidia-*
sudo apt autoremove
sudo ubuntu-drivers autoinstall
sudo reboot
```

### 8.3 CUDA 程序运行失败

```bash
# 检查 CUDA 版本兼容性
nvcc --version
nvidia-smi  # 查看驱动支持的 CUDA 版本

# 检查环境变量
echo $PATH
echo $LD_LIBRARY_PATH
echo $CUDA_HOME

# 重新设置环境变量
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# 检查库文件
ldconfig -p | grep cuda
```

### 8.4 显存不足

```bash
# 查看显存使用
nvidia-smi

# 杀死占用显存的进程
nvidia-smi --query-compute-apps=pid --format=csv,noheader | xargs -n1 sudo kill -9

# 清理 GPU 缓存（PyTorch）
python3 -c "import torch; torch.cuda.empty_cache()"

# 减少 batch size 或使用混合精度训练
```

### 8.5 驱动更新后问题

```bash
# 重建 DKMS 模块
sudo dkms autoinstall

# 检查内核模块
sudo dkms status

# 如果仍有问题，重装驱动
sudo apt install --reinstall nvidia-driver-535
```

### 8.6 黑屏或无法进入图形界面

```bash
# 进入恢复模式
# 启动时按住 Shift，选择 Recovery Mode

# 或进入 TTY（Ctrl+Alt+F3）

# 卸载 NVIDIA 驱动
sudo apt purge nvidia-*

# 安装 nouveau 驱动
sudo apt install xserver-xorg-video-nouveau

# 重建 initramfs
sudo update-initramfs -u

# 重启
sudo reboot
```

## 9. 性能优化

### 9.1 GPU 性能调优

```bash
# 启用持久模式（减少初始化时间）
sudo nvidia-smi -pm 1

# 设置自动启动
sudo nvidia-persistenced --user foo

# 锁定时钟频率（避免降频）
sudo nvidia-smi -lgc 1410,1410  # GPU 时钟
sudo nvidia-smi -lmc 5001       # 内存时钟

# 查看支持的时钟频率
nvidia-smi -q -d SUPPORTED_CLOCKS
```

### 9.2 电源管理

```bash
# 设置电源模式
sudo nvidia-smi -pm 1           # 持久模式

# 最大性能模式
sudo nvidia-settings -a "[gpu:0]/GPUPowerMizerMode=1"

# 自适应模式
sudo nvidia-settings -a "[gpu:0]/GPUPowerMizerMode=0"
```

### 9.3 系统优化

```bash
# 禁用 CPU 降频
sudo cpupower frequency-set -g performance

# 检查 PCIe 速度
sudo lspci -vv | grep -i "lnksta:"

# 确保使用 PCIe 3.0 x16 或 4.0 x16
```

## 10. 驱动卸载与清理

### 10.1 完全卸载 NVIDIA 驱动

```bash
# 卸载所有 NVIDIA 软件包
sudo apt purge nvidia-*
sudo apt purge libnvidia-*

# 卸载 CUDA
sudo apt purge cuda-*
sudo rm -rf /usr/local/cuda*

# 卸载 cuDNN
sudo apt purge libcudnn*

# 清理残留
sudo apt autoremove
sudo apt autoclean

# 删除配置文件
sudo rm -rf /etc/modprobe.d/blacklist-nvidia*
sudo rm -rf ~/.nv/

# 更新 initramfs
sudo update-initramfs -u
```

### 10.2 切换回开源驱动

```bash
# 安装 nouveau 驱动
sudo apt install xserver-xorg-video-nouveau

# 移除 NVIDIA 驱动黑名单
sudo rm /etc/modprobe.d/blacklist-nvidia-nouveau.conf

# 更新 initramfs
sudo update-initramfs -u

# 重启
sudo reboot
```

## 11. 实用脚本

### 11.1 GPU 监控脚本

```bash
# 创建 gpu_monitor.sh
cat << 'EOF' > ~/gpu_monitor.sh
#!/bin/bash
while true; do
    clear
    echo "=== GPU Status at $(date) ==="
    nvidia-smi --query-gpu=index,name,temperature.gpu,utilization.gpu,utilization.memory,memory.used,memory.total,power.draw \
        --format=csv,noheader,nounits | \
        awk -F', ' '{
            printf "GPU %s: %s\n", $1, $2
            printf "  Temperature: %s°C\n", $3
            printf "  GPU Util: %s%%\n", $4
            printf "  Mem Util: %s%%\n", $5
            printf "  Memory: %s/%s MB\n", $6, $7
            printf "  Power: %s W\n\n", $8
        }'
    sleep 2
done
EOF

chmod +x ~/gpu_monitor.sh
~/gpu_monitor.sh
```

### 11.2 自动选择空闲 GPU

```bash
# 创建 auto_gpu.sh
cat << 'EOF' > ~/auto_gpu.sh
#!/bin/bash
# 找到显存使用最少的 GPU
export CUDA_VISIBLE_DEVICES=$(nvidia-smi --query-gpu=index,memory.used --format=csv,noheader,nounits | \
    sort -t',' -k2 -n | head -1 | cut -d',' -f1)
echo "Using GPU: $CUDA_VISIBLE_DEVICES"
exec "$@"
EOF

chmod +x ~/auto_gpu.sh

# 使用
~/auto_gpu.sh python train.py
```

### 11.3 GPU 温度报警

```bash
# 创建 temp_monitor.sh
cat << 'EOF' > ~/temp_monitor.sh
#!/bin/bash
THRESHOLD=80
while true; do
    TEMP=$(nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader,nounits)
    for t in $TEMP; do
        if [ $t -gt $THRESHOLD ]; then
            notify-send "GPU Temperature Warning" "GPU temperature: ${t}°C"
        fi
    done
    sleep 60
done
EOF

chmod +x ~/temp_monitor.sh
```

## 12. 参考资源

### 官方文档
- [NVIDIA Driver Downloads](https://www.nvidia.com/Download/index.aspx)
- [CUDA Toolkit Documentation](https://docs.nvidia.com/cuda/)
- [cuDNN Documentation](https://docs.nvidia.com/deeplearning/cudnn/)
- [NVIDIA Linux Driver README](https://download.nvidia.com/XFree86/Linux-x86_64/535.129.03/README/)

### 社区资源
- [Ubuntu NVIDIA Wiki](https://help.ubuntu.com/community/BinaryDriverHowto/Nvidia)
- [NVIDIA Developer Forums](https://forums.developer.nvidia.com/)
- [GitHub - NVIDIA Docker](https://github.com/NVIDIA/nvidia-docker)

### 版本兼容性
- [CUDA Compatibility Guide](https://docs.nvidia.com/deploy/cuda-compatibility/)
- [PyTorch Get Started](https://pytorch.org/get-started/locally/)
- [TensorFlow GPU Support](https://www.tensorflow.org/install/gpu)

## 总结

本指南涵盖了 Ubuntu 系统下 NVIDIA 显卡的完整使用流程：

1. ✅ **驱动安装** - 多种方式安装和配置
2. ✅ **CUDA/cuDNN** - 深度学习必备工具
3. ✅ **性能监控** - 实时监控和优化
4. ✅ **多 GPU 管理** - 充分利用多卡资源
5. ✅ **深度学习** - PyTorch、TensorFlow 配置
6. ✅ **故障排查** - 常见问题解决方案
7. ✅ **实用脚本** - 提高工作效率

### 最佳实践

- 🔧 使用 `ubuntu-drivers autoinstall` 安装驱动（最稳定）
- 📦 使用 conda 管理不同的 CUDA 版本
- 🐳 使用 Docker 隔离深度学习环境
- 📊 定期监控 GPU 温度和使用率
- 💾 训练前检查显存是否足够
- 🔄 保持驱动和 CUDA 版本匹配

---

*本指南会持续更新，欢迎反馈和补充。祝你的 GPU 高效运转！*
