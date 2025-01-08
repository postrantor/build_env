---
date: 2025-01-05 12:29:33
---

### 如何通过 rosdep 确定需要依赖包

在通过 rosdep 安装依赖的时候，可能会出现与 apt 安装依赖名称不同的问题，以至于不可以直接将 apt install <package_name> 作为 package.xml 中 depend 中字段的值。

如 `apt install libqt5serialport5-dev` 在 package.xml 中应该是 `<depend>libqt5-serialport-dev</depend>`，具体的名称匹配应该通过 rosdep 的 source 查找，参考：

[rosdep-source](/etc/ros/rosdep/sources.list.d/20-default.list)

具体的，对于 qt-serial package，可以通过 [rosdep/base.yaml](https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml) 找打对应的包。

### reference

> [](https://docs.ros.org/en/independent/api/rosdep/html/sources_list.html)
