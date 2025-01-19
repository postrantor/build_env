## 代码阅读

- 功能包：功能包之间的关系
- 文件：功能包内文件之间的关系，每个文件实现的功能
- 函数：文件内，每个函数的功能，函数之间的关系

> 抓主要的功能包，有层次的看，有主次的看

## git 管理

```bash
# 下载代码
git clone git@github.com:Postrantor/quadruped-robot.git
# 查看指分支的代码
git checkout -b rl-mpc -t origin/rl-mpc_ws
# 下载子模块
git submodules init
git submodules update
```

添加到 vscode 工作空间

```bash
code --add .
```

## rl-mpc

> 参考项目

- [MetaRobotics](https://github.com/LucienJi/MetaRobotics)
- [rl-mpc-locomotion](https://github.com/silvery107/rl-mpc-locomotion)
- [LeggedRobotController](https://github.com/MJianM/LeggedRobotController)
- [usc_learning](https://github.com/yonchien/Reinforcement-Learning-for-Quadruped-Robots-Jumps)
- [rsl_rl](https://github.com/leggedrobotics/rsl_rl.git)

### 文件夹 extern/rsl_rl

## 工程目录结构

- [x] env
- [x] format_code.sh
- [x] README
- [x] README.md
- [x] thirdpart
- [x] unitree_sdk

- [-] controller
- [-] interfaces
- [-] examples
- [-] simulation
- [] quadruped

### /controller

- [x] gazebo_ros2_control
- [x] joint_state_publisher
- [x] robot_state_publisher
- [x] ros2_control

- [-] unitree_joint_controller
- [-] unitree_motor_hardware

#### /unitree_joint_controller
