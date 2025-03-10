---
data: 2025-03-08 17:20:37
title ros-dev-tools
---

## ros2 development tools

```shell
# https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
```

对其中 `ros-dev-tools` 包含的工具列表：

- `cmake`
- `git`
- `python3`
- `python3-setuptools`
- `python3-bloom`
- `python3-colcon-common-extensions`
- `python3-rosdep`
- `python3-vcstool`
- `wget`

变体 ros-build-essential 仅包含这些依赖项中的前四个，主要针对 build-farm 基础设施。

```shell
sudo apt install ros-build-essential
```

## reference

详细信息参考一下讨论：

[ROS Developer Tools - Now in Binary Form! - General - ROS Discourse](https://discourse.ros.org/t/ros-developer-tools-now-in-binary-form/29802)
