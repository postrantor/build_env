---
date: 2025-01-07 13:33:00
---

## 查看操作指令

```bash
history
```

## debug imu 问题

### 0. 编译(需要掌握)

```bash
# colcon build 需要在 ~/project/model_ws 下执行
ros2 launch fdilink_ahrs ahrs_driver.launch.py
rm -rf build/ install log

# 单独编译指定的package
colcon build --packages-select fdilink_ahrs

# 在编译完应用之后，可以尝试重新source一下环境变量
source ~/.bashrc
```

### 0. 运行 imu launch(需要掌握)

```bash
# 查看topic下指定的字段
ros2 topic echo /trunk_imu --field orientation
# 需要掌握，详细使用问gpt
ros2 topic --help
ros2 echo <>
# 进阶
ros2 --help
ros2 node --help
```

### 2. 依据错误日志找到源码中问题的位置(不太需要)

```bash
# 在代码中查找出错的信息，通过代码中的逻辑猜测可能得问题
# 代码提示是判断读取到的协议栈的字段不满足
# - 可以排除已经正常读取串口信息
# - 但是可能是乱码，可能是通信频率不对，需要查看硬件的手册，我之后会更新代码，把这个固定下来
grep -rsn "head_type erro"
```

### 1. 排查硬件问题(不太需要)

对于接入 linux 的设备需要确保有 w/r 权限

> (ask-gpt: 在 linux 系统中如何判断插入系统的串口设备是否具备读写权限？)

```bash
# 查看串口是否被操作系统识别到
lsusb
# 被识别到后，检查是否被正确映射到文件，这样程序才能读写这个文件
# 默认映射的文件是没有写入权限的，需要通过以下的配置，指定 MODE="0777" 表示有写入的权限
# > sudo vim /etc/udev/rules.d/unitree-legged.rules
# > ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="fdilink_ahrs", MODE="0777"

# 可以查看被 `SYMLINK+="fdilink_ahrs"` 映射的别名
ll /dev | grep ttyUSB*

# 更改rules文件后需要重新加载，有时候也需要尝试重启系统，确保配置生效了（即正确被映射为 /dev/fdilink_arhs）
# 配置一次就行，但是需要注意，还是有可能多个设备具备同样的序列号，导致映射的名称有冲突
# 类似于昨天遇到的blrtty的问题
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo service udev reload
sudo service udev restart
ll /dev | grep ttyUSB*
sudo reboot
```
