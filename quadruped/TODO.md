---
date: 2024-10-12 00:30:02
date: 2024-11-14 03:09:20
---

## quadruped

<!-- 阶段性目标 -->

- [ ] 使用现有代码调试一组电机
- [ ] 使用现有工程调试机器人结构

<!-- 下一步需要完善的 -->

- [x] 阅读 MPC 算法代码
- [x] 完善 quadruped 的 log 输出，用于调试
  - [x] 已将部分 roslog 替换为标准 spdlog，需要进一步替换
  - [x] 逐步完善 log 输出，用于定位调试
- [ ] gazebo 仿真中初始化机器人的位姿存在问题
- [ ] 支持 IMU 传感器反馈

<!-- 在开发中逐步完善的 -->

- [x] 修正所有 new 关键字，替换为 shared_ptr/unique_ptr
- [x] double -> float 的窄类型转换问题，以及其他类型问题
  - [ ] 还有部分遗留，在后续开发中修正
- [ ] quadruped 中对计算频率为 1000Hz 等数值 存在较多的硬编码

<!-- 需要较多修改的，在开发中逐步完善的 -->

- [ ] 修改 example_gazebo 中控制循环的方式，修改为 timer 定时调度
- [ ] 重构项目使用的配置文件参数传递的方式
- [ ] gazebo plugin 插件移植至 ROS2

## other

- [ ] 考虑使用 RED 框架？
- [ ] 支持 MATLAB 联合仿真？