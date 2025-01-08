---
date: 2025-01-05 00:23:56
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
