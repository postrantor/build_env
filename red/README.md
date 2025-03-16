---
date: 2025-03-16 21:29:00
---

```bash
mkdir ~/project/model_ws/src

cd ~/project/model_ws/src

vcs import \
    --force \
    --retry 3 \
    --inport https://raw.githubusercontent.com/postrantor/build_env/main/quadruped/config/quadruped.repos \
  src

cp ~/project/model_ws/src/quadruped/model-control/.env/.bashrc ~/.bashrc
source ~/.bashrc

import-quad-src
ignore-colcon-pkg
install-despendencies

colcon build
```

## lcov

```shell
apt install python3-colcon-lcov-result
```

## plantform/af

```shell
apt install bison flex
```
