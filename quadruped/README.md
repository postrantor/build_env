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

ln -s ~/project/model_ws/src/quadruped/model-control/.env ~/project/model_ws/.env
ln -s ~/project/model_ws/src/quadruped/model-control/.config ~/project/model_ws/.config
ln -s ~/project/model_ws/src/quadruped/model-control/.vscode ~/project/model_ws/.vscode

cp ~/project/model_ws/src/quadruped/model-control/.env/.bashrc ~/.bashrc

source ~/.bashrc

import-quad-src

ignore-colcon-pkg

install_despendency

colcon build
```
