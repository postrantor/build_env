---
data: 2025-03-08 17:26:31
title: ros-cross-compile
---

## ros 社区提供的交叉编译工具(模拟)[已弃用]

```shell
sudo apt-get install qemu-user-static
pip install ros-cross-compile
# Successfully installed docker-2.7.0 docker-pycreds-0.4.0 ros-cross-compile-0.10.0 websocket-client-1.8.0
```

python 官方地址，已弃用：

[ros-cross-compile · PyPI](https://pypi.org/project/ros-cross-compile/)

github 官方地址，已弃用：

[ros-tooling/cross_compile: A tool to build ROS and ROS2 workspaces for various targets](https://github.com/ros-tooling/cross_compile)

## help

```shell
ros_cross_compile /path/to/my/workspace --arch aarch64 --os ubuntu --rosdistro foxy
```

```shell
> ros_cross_compile -h
usage: ros_cross_compile [-h] -a {armhf,aarch64,x86_64} [-d {melodic,noetic,foxy,galactic,humble,rolling}] -o
                         OS [--sysroot-base-image SYSROOT_BASE_IMAGE] [--sysroot-nocache]
                         [--custom-rosdep-script CUSTOM_ROSDEP_SCRIPT]
                         [--custom-setup-script CUSTOM_SETUP_SCRIPT]
                         [--custom-post-build-script CUSTOM_POST_BUILD_SCRIPT]
                         [--custom-data-dir CUSTOM_DATA_DIR] [--colcon-defaults COLCON_DEFAULTS]
                         [--skip-rosdep-keys SKIP_ROSDEP_KEYS [SKIP_ROSDEP_KEYS ...]]
                         [--custom-metric-file CUSTOM_METRIC_FILE] [--print-metrics]
                         [--stages-skip {gather_rosdeps,sysroot,emulated_build,runtime} [{gather_rosdeps,sysroot,emulated_build,runtime} ...]]
                         [--runtime-tag RUNTIME_TAG]
                         ros_workspace

Sysroot creator for cross compilation workflows.

positional arguments:
  ros_workspace         Path of the colcon workspace to be cross-compiled. Contains "src" directory.

options:
  -h, --help            show this help message and exit
  -a {armhf,aarch64,x86_64}, --arch {armhf,aarch64,x86_64}
                        Target architecture
  -d {melodic,noetic,foxy,galactic,humble,rolling}, --rosdistro {melodic,noetic,foxy,galactic,humble,rolling}
                        Target ROS distribution
  -o OS, --os OS        Target OS
  --sysroot-base-image SYSROOT_BASE_IMAGE
                        Override the default base Docker image to use for building the sysroot. Ex.
                        "arm64v8/ubuntu:focal"
  --sysroot-nocache     When set to true, this disables Docker's cache when building the Docker image for the
                        workspace
  --custom-rosdep-script CUSTOM_ROSDEP_SCRIPT
                        Provide a path to a shell script that will be executed right before collecting the
                        list of dependencies for the target workspace. This allows you to install extra
                        rosdep rules/sources that are not in the standard "rosdistro" set. See the section
                        "Custom Rosdep Script" in README.md for more details.
  --custom-setup-script CUSTOM_SETUP_SCRIPT
                        Provide a path to a shell script that will be executed in the build container right
                        before running "rosdep install" for your ROS workspace. This allows for adding extra
                        apt sources that rosdep may not handle, or other arbitrary setup that is specific to
                        your application build. See the section on "Custom Setup Script" in README.md for
                        more details.
  --custom-post-build-script CUSTOM_POST_BUILD_SCRIPT
                        Provide a path to a shell script that will be executed in the build container after
                        the build successfully completes. See "Custom post-build script" in README.md for
                        more information.
  --custom-data-dir CUSTOM_DATA_DIR
                        Provide a path to a custom arbitrary directory to copy into the sysroot container.
                        You may use this data in your --custom-setup-script, it will be available as
                        "./custom_data/" in the current working directory when the script is run.
  --colcon-defaults COLCON_DEFAULTS
                        Provide a path to a configuration file that provides colcon arguments. See "Package
                        Selection and Build Customization" in README.md for more details.
  --skip-rosdep-keys SKIP_ROSDEP_KEYS [SKIP_ROSDEP_KEYS ...]
                        Skip specified rosdep keys when collecting dependencies for the workspace.
  --custom-metric-file CUSTOM_METRIC_FILE
                        Provide a filename to store the collected metrics. If no name is provided, then the
                        filename will be the current time in UNIX Epoch format.
  --print-metrics       All collected metrics will be printed to stdout via the logging framework.
  --stages-skip {gather_rosdeps,sysroot,emulated_build,runtime} [{gather_rosdeps,sysroot,emulated_build,runtime} ...]
                        Skip the given stages of the build pipeline to save time. Use at your own risk, as
                        this could cause undefined behavior if some stage has not been previously run.
  --runtime-tag RUNTIME_TAG
                        Package the resulting build as a runnable Docker image, with all runtime dependencies
                        installed. The image will use the tag provided to this argument. Example:
                        "my_registry/image_name:image_tag"
```

## reference

[Announcing first release of ROS cross-compiler tool! - General - ROS Discourse](https://discourse.ros.org/t/announcing-first-release-of-ros-cross-compiler-tool/12601)

## 进一步的讨论

### 对 ament 工具的扩展

[REP-2008 - ROS 2 Hardware Acceleration Architecture and Conventions by vmayoral · Pull Request #324 · ros-infrastructure/rep](https://github.com/ros-infrastructure/rep/pull/324/files#diff-f230b6aa06d86bf594d8e431300e453ad7343e8f4b1932252b6d36c62a8b5e0aR86-R123)
[REP 2008 -- ROS 2 Hardware Acceleration Architecture and Conventions (ROS.org)](https://www.ros.org/reps/rep-2008.html)
[ros-infrastructure/rep: ROS Enhancement Proposals](https://github.com/ros-infrastructure/rep)

### 通过 buildx 的方式构建 docker image

### ros2 官方的交叉构建方式

[ros-infrastructure/ros_buildfarm: ROS buildfarm based on Docker](https://github.com/ros-infrastructure/ros_buildfarm)

## 相关链接

rosdep 用于管理 ros2 依赖的管理器

[ros-infrastructure/rosdep: rosdep multi-package manager system dependency tool](https://github.com/ros-infrastructure/rosdep)
[ROS packages — rosdep 0.25.1 documentation](https://docs.ros.org/en/independent/api/rosdep/html/)

在 ros1 中支持 catkin 的自动化构建工具

[ros-infrastructure/bloom: A release automation tool which makes releasing catkin (http://ros.org/wiki/catkin) packages easier.](https://github.com/ros-infrastructure/bloom)
[Bloom — bloom 0.5.10 文档 --- Bloom — bloom 0.5.10 documentation](https://bloom.readthedocs.io/en/0.5.10/)
