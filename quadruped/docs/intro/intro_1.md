---
date: 2024-08-04 10:40:46
---

## 如何运行

```bash
ros2 launch robot_description gazebo.launch.py
ros2 run robot_sim example_gazebo
ros2 run robot_sim example_keyboard
```

## 怎么看代码, 从哪里开始看

### 工程结构

结构: 工程 -> 功能包 -> cmakelists.txt
代码: cmakelists.txt -> src/main.cpp ->src/\*.cpp

#### ROS2 编译**工程**结构

```log
> tree -L 1
.
├── build 编译中间产物
├── **install** 运行产物: so / bin
├── log debug
└── **src** 开发

4 directories, 0 files
```

#### **功能包**: cmakelists.txt 解析

```c
# 1. 包
cmake_minimum_required(VERSION 3.5)
project(gazebo_msgs)

# 2. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 3. src/
set(msg_files
  "msg/ContactState.msg"
  "msg/ContactsState.msg"
)

# 4. build/
# so
add_library(${PROJECT_NAME}
  ${source}
)
# bin
add_executable(${PROJECT_NAME}
  main.cpp
  ${source}
)

# 5. install/
install(
  FILES gazebo_msgs_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)
```

#### **src/**: quadruped **工程**结构

划分功能包依据 package.xml

##### src/

```markdown
> tree -L 1
> .
> ├── gazebo_ros_pkgs (gazebo)

├── model (**quadruped**)
├── ros2_control (**controller_manager**)

├── ros2_control_demos (demo)
└── ros2_controllers (demo)

5 directories, 0 files
```

##### src/model

```log
> tree -L 1
.
├── controller
├── env
├── examples
├── format_code.sh
├── interfaces
├── quadruped
├── README
├── README.md
├── simulation
├── thirdpart
└── unitree_sdk

9 directories, 2 files
```

##### src/model/

example：

1. **examples**(examples/example_gazebo) -> **bin**: `example` -> **so**: `quadruped`
   -> **so**: `ros2`

程序入口(**./simulation/robot_description/launch/launch.py**):

1. **launch.py**(simulation/robot_description) -> **bin**: controller_manager(ros2_control) -> **so**: `unitree_joint_controllers` -> **so**: `unitree_motor_hardware` -> **so**: `unitree_sdk` -> motor(real)
2. **launch.py**(simulation/robot_description) -> **bin**: gazebo -> **config**: `robot_description/config/*.xacro` -> ...

```log
> tree -L 2
.
├── controller/**library**/通过launch加载, controller_manager(bin)->controller(so)
│   ├── joint_state_publisher
│   ├── robot_state_publisher
│   └── unitree_joint_controller
├── env
│   ├── bashrc
│   ├── humble_path
│   └── robot
├── examples/**main.cpp**/通过launch加载, examples(bin)
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   ├── real
│   └── simulation
├── format_code.sh
├── interfaces/**library**/被其他包运行时依赖
│   ├── unitree_api
│   ├── unitree_go
│   ├── unitree_msgs
│   └── xpp_msgs
├── quadruped/**library**, examples(bin)->quadruped(so)
│   ├── CMakeLists.txt
│   ├── config
│   ├── docs
│   ├── include
│   ├── package.xml
│   ├── README
│   ├── README.md
│   └── src
├── README
│   ├── bridge_with_ros2.md
│   ├── example_a1_sim.md
│   ├── GAZEBO.md
│   ├── intro.md
│   ├── ros2与gazebo通信.md
│   ├── run_gazebo_with_controller.md
│   └── xarco.md
├── README.md
├── simulation/**launch**, launch.py, config/*.yaml
│   ├── README.md
│   ├── robot_description
│   └── xpp
├── thirdpart/**library**, quadruped(so)->thirdpart(so)
│   ├── amd
│   ├── deeprobotics_legged_sdk
│   ├── lcm
│   ├── matplotlib
│   ├── qpOASES
│   ├── quadprog
│   ├── tiny_ekf
│   ├── tinynurbs
│   └── unitree_legged_sdk/控制电机 sdk(so)
└── unitree_sdk/控制电机 sdk(so)
    ├── CMakeLists.txt
    ├── include
    ├── lib
    ├── package.xml
    ├── README.md
    └── src

37 directories, 22 files
```

## ros2_control

![ros2_control](https://wiki.ros.org/ros_control?action=AttachFile&do=get&target=gazebo_ros_control.png)

## quadruped

### 功能包之间的关系：

`quadruped` <-> `gazebo_ros_pkgs` <=> (real robot)

- 接受机器人状态：`gazebo_ros_pkgs` -> "pub state" -> "Topic_Name2" <- "sub state" <- `quadruped`
- 计算： quadruped->class
- 发送控制指令到机器人：`quadruped` -> "pub command" -> "Topic_Name1" <- "sub command" <- `gazebo_ros_pkgs`

### 编成抽象：

quadruped <-> node

```cpp
qrRobotRunner robotRunner(quadruped, home_dir_, node);
```

```
> tree -L 2
.
├── CMakeLists.txt
├── config (算法中各个类对象需要解析的配置参数，需要和真实机器人对应)
│   ├── a1
│   ├── a1_sim
│   ├── f710.yaml
│   └── user_parameters.yaml
├── docs
│   ├── demo.md
│   ├── images
│   └── README.md
├── include
│   └── quadruped
├── package.xml
├── README
│   └── qr_motor.md
├── README.md
└── src
    ├── action (算法相关)
    ├── controllers (算法相关)
    ├── dynamics (算法相关)
    ├── estimators (算法相关)
    ├── fsm (算法相关)
    ├── gait (算法相关)
    ├── planner (算法相关)
    ├── robots (map: ros2<->quadruped, qr_robot_a1_sim.cpp)
                  (create pub/sub)
                  (`step()->send_command()->pub()`)
    ├── exec (quadruped运行时调用实例)
                (sub()->update()->step())
    ├── ros (ros2 互操作的一些类，比键盘控制等)
    └── utils (通用处理函数)

20 directories, 8 files
```

## example_gazebo 结构梳理

[](~/src/model/examples/simulation/example_gazebo.cpp)

```
`class node` --------------->`control_loop(<quadruped>, <node>)`
`class quadruped`---->|
```

```cpp
/**
 * @brief 初始化并重置机器人
 * @details 初始化ROS节点和服务客户端，重置机器人姿态
 * @param node 指向ROS节点的指针
 * @return 指向Quadruped机器人实例的智能指针
 */
qrRobotA1Sim initialize_and_reset_robot(node) {

  // 重置机器人状态
  reset_robot(node, entity_state_client);

  // 创建并返回机器人实例
  return quadruped(node, config_file_path_);
}

/**
 * @brief main control loop
 * @details 执行机器人控制逻辑
 * @param quadruped 指向Quadruped机器人实例的指针
 * @param node 指向ROS节点的指针
 */
void control_loop(quadruped,  node) {
  qrRobotRunner robotRunner(quadruped, node);

  get_com_position_in_world_frame(quadruped, node, base_state_client);

  while (current_time - start_time < 1000.0f) {
    robotRunner.Update();
    robotRunner.Step();
    rclcpp::spin_some(node);
  }
}

int main(int argc, char** argv) {
  node = rclcpp::Node("robot_sim");
  robot_sim = initialize_and_reset_robot(node);

  control_loop(robot_sim, node);
}
```
