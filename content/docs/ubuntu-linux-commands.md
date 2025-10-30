---
title: Ubuntu Linux 系统命令手册
weight: 5
bookToc: true
---

# Ubuntu Linux 系统命令手册

本手册详细介绍 Ubuntu/Linux 系统下的常用命令和操作，适用于日常系统管理、开发和运维工作。

## 1. 系统信息查看

### 1.1 系统版本信息

```bash
# 查看 Ubuntu 版本
lsb_release -a

# 查看内核版本
uname -r

# 查看所有系统信息
uname -a

# 查看发行版信息
cat /etc/os-release

# 查看主机名
hostname

# 查看系统架构
arch
# 或
uname -m
```

### 1.2 硬件信息

```bash
# 查看 CPU 信息
lscpu
cat /proc/cpuinfo

# 查看内存信息
free -h
cat /proc/meminfo

# 查看磁盘信息
lsblk
fdisk -l

# 查看 PCI 设备
lspci

# 查看 USB 设备
lsusb

# 查看所有硬件信息
sudo lshw
sudo lshw -short

# 查看 BIOS 信息
sudo dmidecode

# 查看显卡信息
lspci | grep VGA
```

### 1.3 系统运行信息

```bash
# 查看系统运行时间
uptime

# 查看当前登录用户
who
w

# 查看最近登录记录
last

# 查看系统负载
top
htop  # 需要安装

# 查看系统日志
journalctl
dmesg
```

## 2. 文件和目录操作

### 2.1 基础操作

```bash
# 列出文件和目录
ls
ls -l      # 详细列表
ls -lh     # 人类可读的文件大小
ls -la     # 包括隐藏文件
ls -ltr    # 按时间排序
ls -lS     # 按大小排序

# 改变目录
cd /path/to/directory
cd ~       # 进入主目录
cd -       # 返回上一个目录
cd ..      # 上级目录

# 显示当前目录
pwd

# 创建目录
mkdir directory
mkdir -p /path/to/nested/directory  # 递归创建

# 删除目录
rmdir directory        # 只能删除空目录
rm -r directory        # 递归删除
rm -rf directory       # 强制递归删除（谨慎使用）

# 创建文件
touch file.txt
> file.txt             # 创建空文件或清空文件

# 复制文件
cp source.txt destination.txt
cp -r source_dir/ dest_dir/    # 复制目录
cp -p source dest              # 保留属性
cp -i source dest              # 交互式（覆盖前询问）

# 移动/重命名
mv old_name new_name
mv file.txt /path/to/destination/
mv -i source dest      # 交互式

# 删除文件
rm file.txt
rm -i file.txt         # 交互式删除
rm -f file.txt         # 强制删除
```

### 2.2 文件查看

```bash
# 查看文件内容
cat file.txt           # 显示全部内容
less file.txt          # 分页查看（可上下翻页）
more file.txt          # 分页查看
head file.txt          # 查看前 10 行
head -n 20 file.txt    # 查看前 20 行
tail file.txt          # 查看后 10 行
tail -n 20 file.txt    # 查看后 20 行
tail -f file.txt       # 实时查看文件更新（常用于日志）

# 按页查看
cat file.txt | less
```

### 2.3 文件搜索

```bash
# 按名称搜索文件
find /path -name "filename"
find . -name "*.txt"
find /home -iname "file*"  # 忽略大小写

# 按类型搜索
find . -type f             # 文件
find . -type d             # 目录
find . -type l             # 符号链接

# 按大小搜索
find . -size +100M         # 大于 100MB
find . -size -1M           # 小于 1MB

# 按时间搜索
find . -mtime -7           # 最近 7 天修改
find . -atime +30          # 30 天前访问

# 按权限搜索
find . -perm 644

# 执行操作
find . -name "*.tmp" -delete
find . -name "*.log" -exec rm {} \;

# 快速搜索（需要更新数据库）
sudo updatedb
locate filename
```

### 2.4 文件权限

```bash
# 查看权限
ls -l

# 修改权限
chmod 755 file.txt         # 数字方式
chmod u+x file.txt         # 给所有者添加执行权限
chmod g-w file.txt         # 移除组的写权限
chmod o+r file.txt         # 给其他人添加读权限
chmod -R 755 directory/    # 递归修改

# 修改所有者
chown user file.txt
chown user:group file.txt
chown -R user:group directory/

# 只修改组
chgrp group file.txt

# 权限说明：
# r(read)=4, w(write)=2, x(execute)=1
# 755 = rwxr-xr-x (所有者：读写执行，组：读执行，其他：读执行)
# 644 = rw-r--r-- (所有者：读写，组：读，其他：读)
```

### 2.5 链接文件

```bash
# 创建硬链接
ln source.txt link.txt

# 创建软链接（符号链接）
ln -s /path/to/source /path/to/link

# 查看链接
ls -l
readlink link_name
```

## 3. 文本处理

### 3.1 文本搜索

```bash
# grep 搜索
grep "pattern" file.txt
grep -i "pattern" file.txt     # 忽略大小写
grep -r "pattern" /path        # 递归搜索
grep -n "pattern" file.txt     # 显示行号
grep -v "pattern" file.txt     # 反向搜索（不包含）
grep -c "pattern" file.txt     # 计数
grep -l "pattern" *.txt        # 只显示文件名
grep -A 3 "pattern" file.txt   # 显示匹配行及后 3 行
grep -B 3 "pattern" file.txt   # 显示匹配行及前 3 行
grep -C 3 "pattern" file.txt   # 显示匹配行及前后 3 行

# 使用正则表达式
grep -E "pattern1|pattern2" file.txt
egrep "pattern1|pattern2" file.txt
```

### 3.2 文本编辑

```bash
# sed 流编辑器
sed 's/old/new/' file.txt              # 替换每行第一个匹配
sed 's/old/new/g' file.txt             # 替换所有匹配
sed -i 's/old/new/g' file.txt          # 直接修改文件
sed '1,10s/old/new/g' file.txt         # 只替换 1-10 行
sed '/pattern/d' file.txt              # 删除匹配的行
sed -n '5,10p' file.txt                # 只打印 5-10 行

# awk 文本处理
awk '{print $1}' file.txt              # 打印第一列
awk '{print $1,$3}' file.txt           # 打印第 1 和第 3 列
awk -F':' '{print $1}' /etc/passwd     # 指定分隔符
awk 'NR==5' file.txt                   # 打印第 5 行
awk 'NR>=5 && NR<=10' file.txt         # 打印 5-10 行
awk '{sum+=$1} END {print sum}' file.txt  # 求和
```

### 3.3 文本统计

```bash
# wc 统计
wc file.txt            # 行数、单词数、字节数
wc -l file.txt         # 统计行数
wc -w file.txt         # 统计单词数
wc -c file.txt         # 统计字节数

# sort 排序
sort file.txt          # 排序
sort -r file.txt       # 反向排序
sort -n file.txt       # 数字排序
sort -u file.txt       # 去重排序
sort -k2 file.txt      # 按第二列排序

# uniq 去重
uniq file.txt          # 去除相邻重复行
sort file.txt | uniq   # 去除所有重复行
uniq -c file.txt       # 统计重复次数
uniq -d file.txt       # 只显示重复行

# cut 切割
cut -d':' -f1 /etc/passwd    # 以 : 分隔，取第 1 列
cut -c1-10 file.txt          # 取每行的 1-10 字符
```

### 3.4 文本比较

```bash
# diff 比较文件
diff file1.txt file2.txt
diff -u file1.txt file2.txt    # 统一格式
diff -y file1.txt file2.txt    # 并排显示

# comm 比较已排序的文件
comm file1.txt file2.txt
```

## 4. 软件包管理

### 4.1 APT 包管理器

```bash
# 更新包列表
sudo apt update

# 升级所有包
sudo apt upgrade
sudo apt full-upgrade      # 升级并处理依赖关系变化

# 安装软件
sudo apt install package_name
sudo apt install package1 package2 package3

# 重新安装
sudo apt reinstall package_name

# 移除软件
sudo apt remove package_name        # 保留配置文件
sudo apt purge package_name         # 删除配置文件

# 自动清理
sudo apt autoremove                 # 删除不需要的依赖
sudo apt autoclean                  # 清理旧的安装包

# 搜索软件
apt search keyword
apt-cache search keyword

# 查看包信息
apt show package_name
apt-cache show package_name

# 列出已安装的包
apt list --installed
dpkg -l

# 查看包的文件列表
dpkg -L package_name

# 查看文件属于哪个包
dpkg -S /path/to/file
```

### 4.2 Snap 包管理

```bash
# 查找软件
snap find package_name

# 安装
sudo snap install package_name

# 列出已安装
snap list

# 更新
sudo snap refresh package_name
sudo snap refresh --list

# 卸载
sudo snap remove package_name
```

### 4.3 从源码安装

```bash
# 典型的编译安装流程
./configure
make
sudo make install

# 或使用 checkinstall（生成 deb 包）
sudo apt install checkinstall
./configure
make
sudo checkinstall
```

## 5. 进程管理

### 5.1 查看进程

```bash
# 查看所有进程
ps aux
ps -ef

# 查看进程树
pstree
ps auxf

# 实时监控进程
top
htop        # 更友好的界面（需安装）

# 查找特定进程
ps aux | grep process_name
pgrep process_name
pidof process_name
```

### 5.2 管理进程

```bash
# 结束进程
kill PID
kill -9 PID            # 强制结束
killall process_name   # 按名称结束
pkill pattern          # 按模式结束

# 后台运行
command &
nohup command &        # 终端关闭后继续运行

# 查看后台任务
jobs

# 将后台任务调到前台
fg %job_number

# 将前台任务放到后台
# 按 Ctrl+Z 暂停，然后
bg %job_number

# 查看进程优先级
nice -n 10 command     # 以较低优先级运行
renice -n 5 -p PID     # 修改运行中进程的优先级
```

## 6. 用户和组管理

### 6.1 用户管理

```bash
# 添加用户
sudo adduser username
sudo useradd -m username       # -m 创建主目录

# 删除用户
sudo deluser username
sudo userdel username
sudo userdel -r username       # 同时删除主目录

# 修改用户密码
sudo passwd username
passwd                         # 修改当前用户密码

# 切换用户
su username
su -                          # 切换到 root
sudo -i                       # 以 root 登录

# 修改用户信息
sudo usermod -aG group username    # 添加用户到组
sudo usermod -l newname oldname    # 重命名用户
sudo usermod -d /new/home username # 修改主目录

# 查看用户信息
id username
finger username
w username
```

### 6.2 组管理

```bash
# 添加组
sudo groupadd groupname

# 删除组
sudo groupdel groupname

# 查看组
groups username
cat /etc/group

# 修改文件的组
chgrp groupname file
```

### 6.3 权限提升

```bash
# 使用 sudo
sudo command

# 编辑 sudoers 文件（谨慎）
sudo visudo

# 查看 sudo 权限
sudo -l
```

## 7. 磁盘和文件系统

### 7.1 磁盘使用情况

```bash
# 查看磁盘空间
df -h                  # 人类可读格式
df -i                  # inode 使用情况

# 查看目录大小
du -h directory
du -sh directory       # 只显示总计
du -h --max-depth=1    # 一级子目录大小
du -ah | sort -rh | head -10  # 最大的 10 个文件/目录

# 查看文件系统类型
df -T
```

### 7.2 磁盘分区

```bash
# 查看分区
lsblk
fdisk -l
parted -l

# 分区工具
sudo fdisk /dev/sda    # 传统分区工具
sudo parted /dev/sda   # 更现代的工具
sudo gparted           # 图形界面（需安装）
```

### 7.3 挂载文件系统

```bash
# 挂载
sudo mount /dev/sdb1 /mnt/usb
sudo mount -t ntfs /dev/sda1 /mnt/windows

# 卸载
sudo umount /mnt/usb

# 查看已挂载
mount
df -h

# 编辑自动挂载
sudo nano /etc/fstab

# 重新加载 fstab
sudo mount -a
```

### 7.4 文件系统操作

```bash
# 创建文件系统
sudo mkfs.ext4 /dev/sdb1
sudo mkfs.ntfs /dev/sdb1

# 检查和修复文件系统
sudo fsck /dev/sda1
sudo e2fsck /dev/sda1

# 调整文件系统大小
sudo resize2fs /dev/sda1

# 查看文件系统信息
sudo tune2fs -l /dev/sda1
```

## 8. 网络配置和诊断

### 8.1 网络接口

```bash
# 查看网络接口
ip addr
ip a
ifconfig              # 旧命令

# 启用/禁用网络接口
sudo ip link set eth0 up
sudo ip link set eth0 down

# 配置 IP 地址
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip addr del 192.168.1.100/24 dev eth0

# 查看路由表
ip route
route -n

# 添加路由
sudo ip route add 192.168.2.0/24 via 192.168.1.1
```

### 8.2 网络诊断

```bash
# ping 测试
ping google.com
ping -c 4 8.8.8.8      # 只发送 4 个包

# 追踪路由
traceroute google.com
tracepath google.com

# DNS 查询
nslookup google.com
dig google.com
host google.com

# 查看网络连接
netstat -tuln          # TCP/UDP 监听端口
netstat -antp          # 所有 TCP 连接
ss -tuln               # 更现代的替代命令
ss -s                  # 统计信息

# 查看开放端口
sudo lsof -i          # 所有网络连接
sudo lsof -i :80      # 特定端口
sudo netstat -tulpn | grep :80
```

### 8.3 下载工具

```bash
# wget 下载
wget https://example.com/file.zip
wget -c url            # 断点续传
wget -O filename url   # 指定文件名

# curl 下载
curl -O https://example.com/file.zip
curl -o filename url   # 指定文件名
curl -L url            # 跟随重定向

# 查看 HTTP 头
curl -I https://example.com
```

### 8.4 网络配置文件

```bash
# NetworkManager（Ubuntu 桌面版）
nmcli device status
nmcli connection show

# Netplan（Ubuntu Server）
sudo nano /etc/netplan/01-netcfg.yaml
sudo netplan apply

# 传统网络配置
sudo nano /etc/network/interfaces

# DNS 配置
sudo nano /etc/resolv.conf
```

## 9. 系统服务管理

### 9.1 Systemd 服务管理

```bash
# 启动服务
sudo systemctl start service_name

# 停止服务
sudo systemctl stop service_name

# 重启服务
sudo systemctl restart service_name

# 重新加载配置
sudo systemctl reload service_name

# 查看服务状态
sudo systemctl status service_name

# 开机自启
sudo systemctl enable service_name

# 禁用开机自启
sudo systemctl disable service_name

# 列出所有服务
systemctl list-units --type=service
systemctl list-unit-files

# 查看服务日志
sudo journalctl -u service_name
sudo journalctl -u service_name -f    # 实时查看
sudo journalctl -u service_name --since today
```

### 9.2 定时任务

```bash
# Cron 定时任务
crontab -e             # 编辑当前用户的 crontab
crontab -l             # 列出定时任务
crontab -r             # 删除所有定时任务

# Crontab 格式：
# 分 时 日 月 周 命令
# * * * * * command
# 0 2 * * * /path/to/script.sh    # 每天凌晨 2 点执行

# 系统 cron
sudo nano /etc/crontab
ls /etc/cron.daily/
ls /etc/cron.weekly/
ls /etc/cron.monthly/

# systemd 定时器
systemctl list-timers
```

## 10. 压缩和归档

### 10.1 tar 归档

```bash
# 创建 tar 归档
tar -cvf archive.tar files/
tar -czvf archive.tar.gz files/    # gzip 压缩
tar -cjvf archive.tar.bz2 files/   # bzip2 压缩
tar -cJvf archive.tar.xz files/    # xz 压缩

# 解压
tar -xvf archive.tar
tar -xzvf archive.tar.gz
tar -xjvf archive.tar.bz2

# 查看归档内容
tar -tvf archive.tar

# 解压到指定目录
tar -xzvf archive.tar.gz -C /path/to/directory
```

### 10.2 压缩文件

```bash
# gzip 压缩
gzip file.txt          # 压缩后删除原文件
gzip -k file.txt       # 保留原文件
gzip -d file.txt.gz    # 解压
gunzip file.txt.gz     # 解压

# bzip2 压缩
bzip2 file.txt
bzip2 -d file.txt.bz2

# xz 压缩
xz file.txt
xz -d file.txt.xz

# zip 压缩
zip archive.zip file1 file2
zip -r archive.zip directory/    # 递归压缩
unzip archive.zip
unzip -l archive.zip             # 查看内容

# 7z 压缩
7z a archive.7z files/
7z x archive.7z
```

## 11. 系统监控

### 11.1 性能监控

```bash
# CPU 监控
top
htop
mpstat 1               # 每秒更新

# 内存监控
free -h
vmstat 1               # 每秒更新

# 磁盘 I/O 监控
iostat
iostat -x 1            # 详细信息，每秒更新
iotop                  # 按进程显示 I/O

# 网络监控
iftop                  # 实时网络流量
nethogs                # 按进程显示网络使用
vnstat                 # 网络流量统计

# 综合监控
glances                # 需要安装
dstat                  # 需要安装
```

### 11.2 日志查看

```bash
# 系统日志
sudo journalctl
sudo journalctl -f     # 实时查看
sudo journalctl -b     # 本次启动的日志
sudo journalctl --since "2 hours ago"
sudo journalctl --since "2023-01-01" --until "2023-01-31"

# 传统日志文件
sudo tail -f /var/log/syslog
sudo tail -f /var/log/auth.log
sudo tail -f /var/log/kern.log
sudo less /var/log/messages

# 清理日志
sudo journalctl --vacuum-time=7d    # 只保留 7 天
sudo journalctl --vacuum-size=1G    # 只保留 1GB
```

## 12. 防火墙和安全

### 12.1 UFW 防火墙

```bash
# 启用/禁用防火墙
sudo ufw enable
sudo ufw disable

# 查看状态
sudo ufw status
sudo ufw status verbose

# 允许端口
sudo ufw allow 22
sudo ufw allow 80/tcp
sudo ufw allow 1000:2000/tcp       # 端口范围

# 允许服务
sudo ufw allow ssh
sudo ufw allow http
sudo ufw allow https

# 拒绝端口
sudo ufw deny 23

# 允许特定 IP
sudo ufw allow from 192.168.1.100

# 删除规则
sudo ufw delete allow 80
sudo ufw status numbered
sudo ufw delete 2

# 重置防火墙
sudo ufw reset
```

### 12.2 SSH 安全

```bash
# 生成 SSH 密钥
ssh-keygen -t ed25519

# 复制公钥到服务器
ssh-copy-id user@server

# SSH 配置文件
sudo nano /etc/ssh/sshd_config

# 重启 SSH 服务
sudo systemctl restart sshd

# 查看 SSH 登录记录
sudo lastlog
sudo last
```

### 12.3 系统更新和安全

```bash
# 检查安全更新
sudo apt update
apt list --upgradable

# 自动安全更新
sudo apt install unattended-upgrades
sudo dpkg-reconfigure -plow unattended-upgrades

# 检查可疑进程
ps aux | less
sudo netstat -tulpn
```

## 13. 实用技巧

### 13.1 命令行快捷键

```bash
# 光标移动
Ctrl + A          # 移到行首
Ctrl + E          # 移到行尾
Ctrl + U          # 删除光标前的内容
Ctrl + K          # 删除光标后的内容
Ctrl + W          # 删除光标前的单词
Ctrl + L          # 清屏（相当于 clear）
Ctrl + R          # 搜索历史命令
Ctrl + C          # 终止当前命令
Ctrl + Z          # 暂停当前命令
Ctrl + D          # 退出当前 shell

# 历史命令
history           # 显示历史命令
!n                # 执行第 n 条历史命令
!!                # 执行上一条命令
!string           # 执行最近以 string 开头的命令
```

### 13.2 管道和重定向

```bash
# 管道
command1 | command2

# 重定向
command > file.txt        # 覆盖
command >> file.txt       # 追加
command 2> error.log      # 重定向错误输出
command &> all.log        # 重定向所有输出
command < input.txt       # 输入重定向

# 示例
ls -l | grep ".txt"
cat file.txt | sort | uniq
ps aux | grep nginx | awk '{print $2}'
```

### 13.3 变量和环境

```bash
# 设置变量
VAR="value"
export VAR="value"        # 导出环境变量

# 查看变量
echo $VAR
echo $PATH
env                       # 查看所有环境变量
printenv

# 永久设置环境变量
nano ~/.bashrc
nano ~/.profile
source ~/.bashrc          # 重新加载配置
```

### 13.4 别名

```bash
# 临时别名
alias ll='ls -lah'
alias update='sudo apt update && sudo apt upgrade'

# 永久别名
nano ~/.bashrc
# 添加：alias ll='ls -lah'
source ~/.bashrc

# 查看别名
alias

# 删除别名
unalias ll
```

## 14. 常用工具

### 14.1 文本编辑器

```bash
# nano（简单易用）
nano file.txt

# vim（强大但有学习曲线）
vim file.txt
# i - 进入插入模式
# Esc - 退出插入模式
# :w - 保存
# :q - 退出
# :wq - 保存并退出
# :q! - 不保存退出

# gedit（图形界面）
gedit file.txt
```

### 14.2 屏幕管理

```bash
# screen（终端复用）
screen                    # 启动新会话
screen -S session_name    # 命名会话
Ctrl + A, D              # 分离会话
screen -ls                # 列出会话
screen -r session_name    # 恢复会话

# tmux（更现代的终端复用器）
tmux                      # 启动
tmux new -s session_name
Ctrl + B, D              # 分离
tmux ls                   # 列出会话
tmux attach -t session_name
```

### 14.3 文件传输

```bash
# scp（安全复制）
scp file.txt user@server:/path/
scp user@server:/path/file.txt ./
scp -r directory/ user@server:/path/

# rsync（同步文件）
rsync -avz source/ destination/
rsync -avz --progress source/ user@server:/path/
rsync -avz --delete source/ destination/  # 删除目标中多余的文件
```

## 15. 故障排除

### 15.1 系统问题

```bash
# 系统无响应
# Ctrl + Alt + F2-F6    # 切换到其他 TTY
# Ctrl + Alt + F1       # 返回图形界面

# 强制重启
sudo reboot -f

# 检查磁盘错误
sudo fsck /dev/sda1    # 需要先卸载

# 恢复模式
# 开机时按 Shift 进入 GRUB，选择 Recovery Mode
```

### 15.2 软件包问题

```bash
# 修复损坏的包
sudo apt --fix-broken install
sudo dpkg --configure -a

# 清理包缓存
sudo apt clean
sudo apt autoclean

# 重新安装包
sudo apt install --reinstall package_name
```

### 15.3 网络问题

```bash
# 重启网络
sudo systemctl restart NetworkManager
sudo systemctl restart networking

# 重置网络配置
sudo ip addr flush dev eth0
sudo systemctl restart networking

# DNS 问题
sudo systemctl restart systemd-resolved
```

### 15.4 权限问题

```bash
# 修复权限
sudo chown -R $USER:$USER /path/to/directory
sudo chmod -R 755 /path/to/directory

# 修复主目录权限
sudo chown -R $USER:$USER $HOME
```

## 16. 常用命令速查

### 文件操作
```bash
ls, cd, pwd, mkdir, rm, cp, mv, touch, cat, less, head, tail
```

### 系统管理
```bash
sudo, apt, systemctl, ps, top, kill, df, du, free
```

### 网络
```bash
ping, ip, netstat, ss, wget, curl, ssh, scp
```

### 文本处理
```bash
grep, sed, awk, sort, uniq, wc, cut, diff
```

### 压缩
```bash
tar, gzip, zip, unzip, 7z
```

## 总结

本手册涵盖了 Ubuntu Linux 系统的常用命令和操作。建议：

1. **循序渐进**：从基础命令开始学习
2. **多加练习**：在虚拟机或测试环境中尝试
3. **使用 man**：`man command` 查看详细文档
4. **善用 --help**：`command --help` 快速查看选项
5. **备份重要数据**：在执行危险操作前做好备份
6. **保持系统更新**：定期运行 `sudo apt update && sudo apt upgrade`

## 参考资源

- [Ubuntu 官方文档](https://help.ubuntu.com/)
- [Linux 命令大全](https://man.linuxde.net/)
- [The Linux Command Line](http://linuxcommand.org/tlcl.php)
- [Ubuntu Server Guide](https://ubuntu.com/server/docs)

---

*持续更新中，欢迎反馈和补充。*
