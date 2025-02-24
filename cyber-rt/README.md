---
date: 2025-02-24 15:14:43
title: cyber-rt 9.0.0 build env
---

## how to use

### 更新环境变量

替换 ~/.bashrc 的内容为如下：

```shell
export PATH=.:${HOME}/.local/bin:$PATH

# set compiler
# export CC=clang
# export CXX=clang++

# cyber-rt project
export WORKDIR=${HOME}/build_env/cyber-rt
export CYBER_WORKDIR=$WORKDIR

source ${WORKDIR}/.env/bashrc.bash
source ${WORKDIR}/.env/system.bash
```

### 编译

```shell
colcon build
```

## download fast-rtps

copy from apollo [dockerfile](apollo/docker/build/installers)

```bash
wget -t 10  https://apollo-system.cdn.bcebos.com/archive/6.0/fast-rtps-1.5.0-1.prebuilt.x86_64.tar.gz -P ./
wget -t 10  https://apollo-system.cdn.bcebos.com/archive/6.0/fast-rtps-1.5.0-1.prebuilt.aarch64.tar.gz -P ./
```

```
-DEPROSIMA_BUILD=ON -DCMAKE_BUILD_TYPE=Release
```
