---
date: 2024-08-25 21:13:06
---

# workspaces

unitree_ws -> demo_ws -> model_ws

## unitree_ws

### 电机

1. m8060 电机文档
2. 开发电机用的 sdk:
   `unitree_ws/control/unitree_sdk`

### 功能包

基本都是官方的功能包，是需要我们借鉴的。

### sdk

```bash
> tree -L 2
.
├── CMakeLists.txt
├── include
│   └── unitree_sdk
├── lib
│   ├── libUnitreeMotorSDK_M80106_Arm64.so
│   └── libUnitreeMotorSDK_M80106_Linux64.so
├── README.md
├── src
│   ├── run.cpp
│   └── stop.cpp
└── tools
    ├── bootloader_mode.gif
    ├── motor_mode.gif
    ├── README.md
    ├── v0.2.0_arm32_linux
    ├── v0.2.0_arm64_linux
    └── v0.2.0_x86_64_linux

8 directories, 9 files
```

### 编译

```bash
cmake -B build ./
cmake --build ./build -- -j2
```

### 运行

```bash
sudo ./build/motor_run 1 1
```

## demo_ws

unitree_sdk(motor) -> unitree_motor_hardware(HARDWARE) -> unitree_joint_controller(CONTROLLER) -> launch(robot_description)
