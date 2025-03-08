---
date: 2025-01-11 13:24:32
---

- [Bash Functions Documentation](#bash-functions-documentation)
  - [1. `ormat`](#1-ormat)
    - [Description](#description)
    - [Parameters](#parameters)
    - [Usage](#usage)
    - [Example](#example)
  - [2. `format-packages`](#2-format-packages)
    - [Description](#description-1)
    - [Parameters](#parameters-1)
    - [Usage](#usage-1)
    - [Example](#example-1)
    - [Notes](#notes)
  - [3. `cd_ws`](#3-cd_ws)
    - [Description](#description-2)
    - [Parameters](#parameters-2)
    - [Usage](#usage-2)
    - [Example](#example-2)
    - [Notes](#notes-1)
  - [4. `colcon_ws`](#4-colcon_ws)
    - [Description](#description-3)
    - [Parameters](#parameters-3)
    - [Usage](#usage-3)
    - [Example](#example-3)
    - [Notes](#notes-2)
  - [5. `colcon_remove`](#5-colcon_remove)
    - [Description](#description-4)
    - [Parameters](#parameters-4)
    - [Usage](#usage-4)
    - [Example](#example-4)
    - [Notes](#notes-3)
  - [6. `import-quad-src`](#6-import-quad-src)
    - [Description](#description-5)
    - [Parameters](#parameters-5)
    - [Usage](#usage-5)
    - [Example](#example-5)
    - [Notes](#notes-4)
  - [7. `install-dependencies`](#7-install-dependencies)
    - [Description](#description-6)
    - [Parameters](#parameters-6)
    - [Usage](#usage-6)
    - [Example](#example-6)
    - [Notes](#notes-5)
  - [8. `ignore-directory`](#8-ignore-directory)
    - [Description](#description-7)
    - [Parameters](#parameters-7)
    - [Usage](#usage-7)
    - [Example](#example-7)
    - [Notes](#notes-6)
  - [9. `ignore-colcon-pkg`](#9-ignore-colcon-pkg)
    - [Description](#description-8)
    - [Parameters](#parameters-8)
    - [Usage](#usage-8)
    - [Example](#example-8)
    - [Notes](#notes-7)
  - [10. `cd`](#10-cd)
    - [Description](#description-9)
    - [Parameters](#parameters-9)
    - [Usage](#usage-9)
    - [Example](#example-9)
    - [Notes](#notes-8)
  - [11. `_vcs`](#11-_vcs)
    - [Description](#description-10)
    - [Parameters](#parameters-10)
    - [Usage](#usage-10)
    - [Example](#example-10)
    - [Notes](#notes-9)
  - [12. `format-target-folder-by-clang-format`](#12-format-target-folder-by-clang-format)
    - [Description](#description-11)
    - [Parameters](#parameters-11)
    - [Usage](#usage-11)
    - [Example](#example-11)
    - [Notes](#notes-10)
- [Environment Variables](#environment-variables)
  - [1. `PYTHONPATH`](#1-pythonpath)
  - [2. `RMW_IMPLEMENTATION`](#2-rmw_implementation)
  - [3. `GAZEBO_PLUGIN_PATH`](#3-gazebo_plugin_path)
  - [4. `COLCON_DEFAULTS_FILE`](#4-colcon_defaults_file)
  - [5. `COLCON_HOME`](#5-colcon_home)
  - [6. `QUAD_REPO`](#6-quad_repo)
- [Conclusion](#conclusion)

# Bash Functions Documentation

## 1. `ormat`

### Description

该函数用于对指定文件夹中的 C++ 源文件（`.h`, `.cpp`, `.hpp`）进行格式化，使用 `clang-format` 工具，并指定格式配置文件。

### Parameters

- `$1`: `folder` - 需要格式化的文件夹路径。
- `$2`: `format_file` - `.clang-format` 配置文件的路径。

### Usage

```bash
ormat /path/to/folder /path/to/.clang-format
```

### Example

```bash
ormat ./src /path/to/.clang-format
```

---

## 2. `format-packages`

### Description

在指定的搜索路径下查找所有 `package.xml` 文件，检查其内容是否包含指定的 `<name>${package_name}</name>` 标签。如果找到匹配的标签，并且目标文件夹中不存在 `COLCON_IGNORE` 文件，则对该文件夹中的 C++ 源文件进行格式化。

### Parameters

- `$@`: `package_names` - 需要查找的包名称列表（可变参数）。
- `search_path`: 搜索 `package.xml` 文件的根路径，默认为 `${QUAD_WORKDIR}/src`。
- `format_file`: `.clang-format` 配置文件的路径，默认为 `${QUAD_WORKDIR}/.clang-format`。

### Usage

```bash
format-packages "package1" "package2"
```

### Example

```bash
format-packages "quadruped" "unitree"
```

### Notes

1. 确保 `QUAD_WORKDIR` 环境变量已正确设置。
2. 确保 `.clang-format` 配置文件存在且格式正确。
3. 如果目标文件夹中存在 `COLCON_IGNORE` 文件，则跳过该文件夹的格式化操作。
4. 该函数会递归查找 `search_path` 下的所有 `package.xml` 文件。

---

## 3. `cd_ws`

### Description

切换到指定的工作空间目录，并加载相应的环境设置文件（`setup.bash` 或 `setup.zsh`）。

### Parameters

- `$1`: `workspace` - 工作空间名称，可选值为 `model`, `control`, `unitree`。

### Usage

```bash
cd_ws model
```

### Example

```bash
cd_ws control
```

### Notes

1. 如果传入的工作空间名称无效，函数会返回错误并提示正确的用法。
2. 函数会根据当前使用的 Shell（Bash 或 Zsh）加载相应的环境设置文件。

---

## 4. `colcon_ws`

### Description

切换到指定的工作空间目录，并执行 `colcon build` 命令。

### Parameters

- `$1`: `workspace` - 工作空间名称，可选值为 `model`, `control`, `unitree`。
- `$@`: 其他参数 - 传递给 `colcon build` 的额外参数。

### Usage

```bash
colcon_ws model --symlink-install
```

### Example

```bash
colcon_ws unitree --packages-select package1 package2
```

### Notes

1. 该函数会先调用 `cd_ws` 切换到指定的工作空间目录。
2. 额外的参数会直接传递给 `colcon build` 命令。

---

## 5. `colcon_remove`

### Description

删除指定包的 `install` 和 `build` 目录。

### Parameters

- `$@`: `packages` - 需要删除的包名称列表（可变参数）。

### Usage

```bash
colcon_remove package1 package2
```

### Example

```bash
colcon_remove quadruped unitree
```

### Notes

1. 如果未提供任何包名称，函数会提示正确的用法。
2. 该函数会检查 `install` 和 `build` 目录是否存在，如果存在则删除。

---

## 6. `import-quad-src`

### Description

导入 `quad` 仓库的源代码到当前工作空间的 `src` 目录中。

### Parameters

无。

### Usage

```bash
import-quad-src
```

### Example

```bash
import-quad-src
```

### Notes

1. 该函数会创建 `src` 目录（如果不存在），并使用 `vcs` 工具导入仓库。
2. 如果导入失败，函数会返回错误并提示失败原因。

---

## 7. `install-dependencies`

### Description

安装指定包的依赖项，使用 `rosdep` 工具。

### Parameters

- `$@`: `packages` - 需要安装依赖项的包路径列表（可变参数）。

### Usage

```bash
install-dependencies ./src/package1 ./src/package2
```

### Example

```bash
install-dependencies ./src/quadruped ./src/unitree
```

### Notes

1. 该函数会跳过指定的依赖项（如 `serial`, `unitree_msgs` 等）。
2. 确保 `rosdep` 已正确安装并初始化。

---

## 8. `ignore-directory`

### Description

在指定目录中创建 `COLCON_IGNORE` 文件，以忽略该目录的构建。

### Parameters

- `$1`: `ignore_file` - 忽略文件的名称。
- `$2`: `path_pattern` - 需要忽略的目录路径模式。

### Usage

```bash
ignore-directory "control" "ros2-control/ros2-control"
```

### Example

```bash
ignore-directory "simulation" "simulation"
```

### Notes

1. 如果忽略文件已存在，函数不会重复创建。
2. 函数会递归查找匹配的目录，并在其中创建 `COLCON_IGNORE` 文件。

---

## 9. `ignore-colcon-pkg`

### Description

忽略指定的 `colcon` 包，避免在构建时包含这些包。

### Parameters

无。

### Usage

```bash
ignore-colcon-pkg
```

### Example

```bash
ignore-colcon-pkg
```

### Notes

1. 该函数会遍历预定义的忽略列表，并调用 `ignore-directory` 函数创建 `COLCON_IGNORE` 文件。
2. 忽略列表包含常见的 `ros2-control` 和 `simulation` 包。

---

## 10. `cd`

### Description

扩展 `cd` 命令，支持使用多个点（`...`）快速切换到上级目录。

### Parameters

- `$1`: `path` - 目标路径或点符号（如 `...`）。

### Usage

```bash
cd ...
```

### Example

```bash
cd ....
```

### Notes

1. 该函数仅在 Bash 中有效。
2. 使用多个点（如 `...`）可以快速切换到多级上级目录。

---

## 11. `_vcs`

### Description

为 `vcs` 命令提供自动补全功能。

### Parameters

无。

### Usage

```bash
vcs <TAB>
```

### Example

```bash
vcs import <TAB>
```

### Notes

1. 该函数仅在 Bash 中有效。
2. 自动补全功能基于 `vcs` 命令的子命令。

---

## 12. `format-target-folder-by-clang-format`

### Description

对指定文件夹中的 C++ 源文件进行格式化，使用 `clang-format` 工具。

### Parameters

- `$1`: `folder` - 需要格式化的文件夹路径。
- `$2`: `format_file` - `.clang-format` 配置文件的路径。

### Usage

```bash
format-target-folder-by-clang-format ./src /path/to/.clang-format
```

### Example

```bash
format-target-folder-by-clang-format ./src/quadruped /path/to/.clang-format
```

### Notes

1. 该函数会递归查找指定文件夹中的 `.h`, `.cpp`, `.hpp` 文件，并进行格式化。
2. 确保 `clang-format` 工具已正确安装。

---

# Environment Variables

## 1. `PYTHONPATH`

- **Description**: 设置 Python 模块的搜索路径。
- **Value**: `/home/trantor/project/model_ws/install/xpp_msgs/local/lib/python3.10/dist-packages:/home/trantor/project/model_ws/install/unitree_msgs/local/lib/python3.10/dist-packages:...`

## 2. `RMW_IMPLEMENTATION`

- **Description**: 设置 ROS 2 的中间件实现。
- **Value**: `rmw_cyclonedds_cpp`

## 3. `GAZEBO_PLUGIN_PATH`

- **Description**: 设置 Gazebo 插件的搜索路径。
- **Value**: `/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/${ROS_DISTRO}/lib`

## 4. `COLCON_DEFAULTS_FILE`

- **Description**: 设置 `colcon build` 的默认配置文件路径。
- **Value**: `${QUAD_WORKDIR}/.config/build-args.yaml`

## 5. `COLCON_HOME`

- **Description**: 设置 `colcon` 的配置文件目录。
- **Value**: `${QUAD_WORKDIR}/.config/`

## 6. `QUAD_REPO`

- **Description**: 设置 `quad` 仓库的配置文件路径。
- **Value**: `${QUAD_WORKDIR}/.config/quadruped.repos`

---

# Conclusion

以上是您提供的 Bash 脚本文件中所有 `function` 的详细说明和使用方法。每个 `function` 都包含参数解释、使用示例和注意事项，帮助您更好地理解和使用这些脚本。
