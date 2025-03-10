---
title: how to use vcstool
data: 2025-03-08 13:33:57
url: [blog.csdn.net](https://blog.csdn.net/Bing_Lee/article/details/130603069)
---

- [概述](#概述)
- [下载安装](#下载安装)
- [使用](#使用)
  - [导入 import](#导入-import)
  - [导出 export](#导出-export)
  - [更新 pull](#更新-pull)
- [扩展功能](#扩展功能)

## 概述

本文用来简单记录 vcstool 如何下载，[vcs](https://so.csdn.net/so/search?q=vcs&spm=1001.2101.3001.7020) 在 ros 中用来做项目版本管理，可以用来方便保存、下载、更新依赖 repo 流程。

[官网说明文档 http://wiki.ros.org/vcstool](http://wiki.ros.org/vcstool)

[dirk-thomas/vcstool](https://github.com/dirk-thomas/vcstool)
[MaxandreOgeret/vcstool2](https://github.com/MaxandreOgeret/vcstool2)

## 下载安装

方法 1

```shell
sudo apt install python3-vcstool
```

方法 2

```shell
sudo pip install -U vcstool
```

## 使用

### 导入 import

```
# 从`.yaml`或者`.rosinstall`填充文件到工作区
vcs import deps < deps.yaml
```

文件内容类似如下：

```yaml
# deps.yaml
repositories:
	common/moveit_msgs:
		type: git
		url: https://github.com/ros-planning/moveit_msgs.git
		version: master
	common/moveit_resources:
		type: git
		url: https://github.com/ros-planning/moveit_resources.git
		version: master
	common/geometric_shapes:
	    type: git
	    url: https://github.com/ros-planning/geometric_shapes.git
	    version: noetic-devel
```

```shell
usage: vcs import [-h] [--input FILE_OR_URL] [--force] [--shallow] [--recursive] [--retry N] [--skip-existing]
                  [--debug] [-w N] [--repos]
                  [path]

Import the list of repositories

options:
  -h, --help           show this help message and exit

"import" command parameters:
  --input FILE_OR_URL  Where to read YAML from (default: -)
  --force              Delete existing directories if they don't contain the repository being imported (default:
                       False)
  --shallow            Create a shallow clone without a history (default: False)
  --recursive          Recurse into submodules (default: False)
  --retry N            Retry commands requiring network access N times on failure (default: 2)
  --skip-existing      Don't overwrite existing directories or change custom checkouts in repos using the same URL
                       (but fetch repos with same URL) (default: False)

Common parameters:
  --debug              Show debug messages (default: False)
  -w N, --workers N    Number of parallel worker threads (default: 12)
  --repos              List repositories which the command operates on (default: False)
  path                 Base path to clone repositories to (default: .)
```

### 导出 export

导出当前仓库集合

```
vcs export > deps.yaml
```

```
usage: vcs export [-h] [--exact | --exact-with-tags] [--debug] [-n] [-w N] [--repos] [path]

Export the list of repositories

options:
  -h, --help         show this help message and exit

"export" command parameters:
  --exact            Export commit hashes instead of branch names (default: False)
  --exact-with-tags  Export unique tag names or commit hashes instead of branch names (default: False)

Common parameters:
  --debug            Show debug messages (default: False)
  -n, --nested       Search for nested repositories (default: False)
  -w N, --workers N  Number of parallel worker threads (default: 12)
  --repos            List repositories which the command operates on (default: False)
  path               Base path to look for repositories (default: .)
```

### 更新 pull

```
usage: vcs pull [-h] [--debug] [-s] [-n] [-w N] [--repos] [paths ...]

Bring changes from the repository into the working copy

options:
  -h, --help            show this help message and exit

Common parameters:
  --debug               Show debug messages (default: False)
  -s, --hide-empty, --skip-empty
                        Hide repositories with empty output (default: False)
  -n, --nested          Search for nested repositories (default: False)
  -w N, --workers N     Number of parallel worker threads (default: 12)
  --repos               List repositories which the command operates on (default: False)
  paths                 Base paths to look for repositories (default: ['.'])
```

## 扩展功能

在私有仓库中扩展在 `clone` 仓库时候添加 `--bare` 选项。

```yaml
infra/vcstool:
  type: git
  url: ssh://p7xxtm1/~/repositories/ros2/ros-infrastructure/rep.git
  version: trantor/dev
```
