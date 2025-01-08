---
date: 2024-12-21 11:28:30
---

## Segmentation fault

以上是在执行 example_gazebo 的时候出现的调用展，请结合 quadruped 工程整体进行分析

```log
Stack trace (most recent call last) in thread 9469:
#17   Object "", at 0xffffffffffffffff, in
#16   Source "../sysdeps/unix/sysv/linux/x86_64/clone3.S", line 81, in __clone3 [0x7a5a1172684f]
#15   Source "./nptl/pthread_create.c", line 442, in start_thread [0x7a5a11694ac2]
#14   Object "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.30", at 0x7a5a11adc252, in
#13   Object "/opt/ros/humble/lib/librclcpp.so", at 0x7a5a144ba3b9, in rclcpp::executors::MultiThreadedExecutor::run(unsigned long)
#12   Object "/opt/ros/humble/lib/librclcpp.so", at 0x7a5a144b2fd0, in rclcpp::Executor::execute_any_executable(rclcpp::AnyExecutable&)
#11   Source "/opt/ros/humble/include/rclcpp/rclcpp/timer.hpp", line 230, in execute_callback [0x60bd619039d0]
        227:   execute_callback() override
        228:   {
        229:     TRACEPOINT(callback_start, static_cast<const void *>(&callback_), false);
      > 230:     execute_callback_delegate<>();
        231:     TRACEPOINT(callback_end, static_cast<const void *>(&callback_));
        232:   }
#10   Source "/opt/ros/humble/include/rclcpp/rclcpp/timer.hpp", line 244, in execute_callback_delegate<> [0x60bd6190512d]
        241:   void
        242:   execute_callback_delegate()
        243:   {
      > 244:     callback_();
        245:   }
        246:
        247:   template<
#9    Source "/usr/include/c++/11/bits/std_function.h", line 590, in operator() [0x60bd618c64cf]
        587:       {
        588: 	if (_M_empty())
        589: 	  __throw_bad_function_call();
      > 590: 	return _M_invoker(_M_functor, std::forward<_ArgTypes>(__args)...);
        591:       }
        592:
        593: #if __cpp_rtti
#8    Source "/usr/include/c++/11/bits/std_function.h", line 290, in _M_invoke [0x60bd618bf792]
        287:       static _Res
        288:       _M_invoke(const _Any_data& __functor, _ArgTypes&&... __args)
        289:       {
      > 290: 	return std::__invoke_r<_Res>(*_Base::_M_get_pointer(__functor),
        291: 				     std::forward<_ArgTypes>(__args)...);
        292:       }
#7    Source "/usr/include/c++/11/bits/invoke.h", line 111, in __invoke_r<void, control_loop(const std::shared_ptr<Quadruped::qrRobotA1Sim>&, const std::shared_ptr<rclcpp::Node>&)::<lambda()>&> [0x60bd618bf8fb]
        108:       using __type = typename __result::type;
        109:       using __tag = typename __result::__invoke_type;
        110:       if constexpr (is_void_v<_Res>)
      > 111: 	std::__invoke_impl<__type>(__tag{}, std::forward<_Callable>(__fn),
        112: 					std::forward<_Args>(__args)...);
        113:       else
        114: 	return std::__invoke_impl<__type>(__tag{},
#6    Source "/usr/include/c++/11/bits/invoke.h", line 61, in __invoke_impl<void, control_loop(const std::shared_ptr<Quadruped::qrRobotA1Sim>&, const std::shared_ptr<rclcpp::Node>&)::<lambda()>&> [0x60bd618bfa07]
         58:   template<typename _Res, typename _Fn, typename... _Args>
         59:     constexpr _Res
         60:     __invoke_impl(__invoke_other, _Fn&& __f, _Args&&... __args)
      >  61:     { return std::forward<_Fn>(__f)(std::forward<_Args>(__args)...); }
         62:
         63:   template<typename _Res, typename _MemFun, typename _Tp, typename... _Args>
         64:     constexpr _Res
#5    Source "/home/trantor/project/model_ws/src/model/simulation/examples/simulation/example_gazebo.cpp", line 237, in operator() [0x60bd618be6a0]
        235:     // 执行控制逻辑
        236:     start_time_wall = quadruped->GetTimeSinceReset();
      > 237:     robot_runner.Update();
        238:     robot_runner.Step();
        239:
        240:     // 计算平均耗时
#4    Source "/home/trantor/project/model_ws/src/model/quadruped/src/exec/qr_robot_runner.cpp", line 229, in Update [0x7a5a13d74bc7]
        226:   // - update by "here->update()"
        227:   // - write by "here->step()"
        228:   // 运行控制FSM，执行相应的动作
      > 229:   control_fsm_->RunFSM(hybrid_action_);
        230:
        231:   return true;
        232: }
#3    Source "/home/trantor/project/model_ws/src/model/quadruped/src/fsm/qr_control_fsm.cpp", line 146, in RunFSM [0x7a5a13d85ad0]
        143:         operating_mode_ = FSM_OperatingMode::TRANSITIONING;
        144:       } else {
        145:         // 如果不需要转换状态，则继续执行当前状态的正常行为
      > 146:         current_state_->Run();
        147:       }
        148:     }
#2    Source "/home/trantor/project/model_ws/src/model/quadruped/src/fsm/qr_fsm_state_locomotion.cpp", line 207, in Run [0x7a5a13d9013d]
        205:     // 如果启用了WBC控制器并且允许在MPC之后运行，则调用WBC控制器
        206:     if (this->data->user_parameters->use_wbc && wbc_data_->allow_after_mpc) {
      > 207:       wbc_controller_->Run(wbc_data_);
        208:     }
        209:   }
        210: }
#1    Source "/home/trantor/project/model_ws/src/model/quadruped/src/controllers/wbc/qr_wbc_locomotion_controller.cpp", line 117, in Run [0x7a5a13b9f6be]
        114:   }
        115:
        116:   /* 更新腿部控制命令 */
      > 117:   UpdateLegCMD(control_fsm_data);
        118:
        119:   ++iteration;  // 迭代计数器加一
        120: }
#0    Source "/home/trantor/project/model_ws/src/model/quadruped/src/controllers/wbc/qr_wbc_locomotion_controller.cpp", line 234, in UpdateLegCMD [0x7a5a13ba0258]
        231:       int j_id = JointsOfOneLeg * id + j;
        232:
        233:       /* 只考虑支撑腿的命令，设置关节扭矩命令 */
      > 234:       if (wbc_ctrl_data->contact_states_for_foothold[id]) {
        235:         cmd[j_id].tua = cmd_joints_torque[j_id];
        236:       }
        237:     }
Segmentation fault (Address not mapped to object [0xfc])
[ros2run]: Segmentation fault
```

可能涉及到的文件

```log
src/model/quadruped/src/controllers/qr_state_dataflow.cpp
src/model/quadruped/src/fsm/qr_fsm_state_locomotion.cpp
src/model/quadruped/src/controllers/mpc/qr_mpc_stance_leg_controller.cpp
```

### 检查 qrControlFSM 的构造

过滤掉 `/home/trantor/project/model_ws/src/model/quadruped/src/fsm/qr_control_fsm.cpp` 类定义的本身，结果如下：

```bash
> find ./ -name "*.cpp" -exec grep qrControlFSM {} +

# 使用该类，不是构造
./src/controllers/wbc/qr_wbc_locomotion_controller.cpp:    std::shared_ptr<qrControlFSMData<T>> control_fsm_data)   // 控制状态机数据，包含控制算法所需的数据
./src/controllers/wbc/qr_wbc_locomotion_controller.cpp:    std::shared_ptr<qrControlFSMData<T>> control_fsm_data) {
./src/controllers/wbc/qr_wbc_locomotion_controller.cpp:qrWbcLocomotionController<T>::UpdateLegCMD(std::shared_ptr<qrControlFSMData<T>> data) {

./src/fsm/qr_fsm_state_locomotion.cpp:qrFSMStateLocomotion<T>::qrFSMStateLocomotion(std::shared_ptr<qrControlFSMData<T>> data)  //
./src/fsm/qr_fsm_state_standup.cpp:    std::shared_ptr<qrControlFSMData<T>> control_fsm_data)
./src/fsm/qr_fsm_state.cpp:    std::shared_ptr<qrControlFSMData<T>> data,
./src/fsm/qr_fsm_state_passive.cpp:qrFSMStatePassive<T>::qrFSMStatePassive(std::shared_ptr<qrControlFSMData<T>> data)  //
./src/exec/qr_robot_runner.cpp:  control_fsm_ = std::make_shared<qrControlFSM<float>>(
```

```cpp
Mat33<float>
qrRobot::ComputeJacobian(int id) {
  Vec3<float> leg_motor_angles;
  leg_motor_angles << this->GetMotorAngles().block(id * 3, 0, 3, 1);

  return this->AnalyticalLegJacobian(leg_motor_angles, id);
}
```

### 精简一下调用栈

```log
#5    Source "simulation/examples/simulation/example_gazebo.cpp", line 237, in operator() [0x60bd618be6a0]
#4    Source "quadruped/src/exec/qr_robot_runner.cpp", line 229, in Update [0x7a5a13d74bc7]

  <!-- 这里应该构造 qr_state_dateflow.cpp -->
  #3    Source "quadruped/src/fsm/qr_control_fsm.cpp", line 146, in RunFSM [0x7a5a13d85ad0]

    <!-- 这里有对 wbc_date 进行初始化，但是不全，导致nullptr -->
    #2    Source "quadruped/src/fsm/qr_fsm_state_locomotion.cpp", line 207, in Run [0x7a5a13d9013d]

      <!-- 在构造函数中有对所有的wbc_date进行初始化 -->
      #1    Source "quadruped/src/controllers/wbc/qr_wbc_locomotion_controller.cpp", line 117, in Run [0x7a5a13b9f6be]
      #0    Source "quadruped/src/controllers/wbc/qr_wbc_locomotion_controller.cpp", line 234, in UpdateLegCMD [0x7a5a13ba0258]
            > 234:       if (wbc_ctrl_data->contact_states_for_foothold[id]) {
```

### 分析

可能没有调用 "./src/controllers/qr_state_dataflow.cpp"
在键盘控制的条件下，通过 `[L]` 之后 `[J]` 才会触发一下的 `Run()`

```cpp
qrWbcLocomotionController<T>::Run(std::shared_ptr<qrWbcCtrlData> pre_compute_data) {
```

在该函数内会对以下三个属性进行初始化，该三个变量在构造函数 中并未进行初始化，同时

```cpp
this->contact_list;
this->tasks_list;
this->wbc_data
```

所以，是否需要考虑在 L 阶段可以触发`Run`

### 疑问

在能够正常运行的条件下触发，发现没有运行到出问题的函数的地方，所以接下来通过 git log 追踪解决了一下。

### solve

虽然需要在 [L] 之后指定位置执行 [J] 以触发 wbc_data 但是通过追踪 git log 可以得到一下的提示：

```log
[info] [00:26:04] [quadruped] you have pressed the stop button
[info] [00:26:04] [quadruped] joy_ctrl_state_
        - before: 5
        - is_move_: 1
[info] [00:26:04] [quadruped] [qrControlFSM::RunFSM] RC_MODE: 4
[info] [00:26:04] [quadruped] [qrFSMStateStandUp::OnExit] is_stand_up_ exit!
[info] [00:26:04] [quadruped] [qrFSMStateLocomotion<T>::OnEnter] on_enter state: 4
[info] [00:26:04] [quadruped] [qrOpenLoopGaitGenerator::Reset] set gait: stand
[info] [00:26:04] [quadruped] [MPCStanceLegController::Reset] iteration_counter: 0
[info] [00:26:04] [quadruped] [qrStateEstimatorContainer::Reset] state_estimator_container reset
[info] [00:26:04] [quadruped] [qrFSMStateLocomotion<T>::OnEnter] time step: 0.001
[info] [00:26:04] [quadruped] [qrFSMStateLocomotion<T>::OnEnter] On Enter End
[info] [00:26:04] [quadruped] [qrFSMStateLocomotion<T>::CheckTransition] reset iter_ for LOCOMOTION_STAND
[info] [00:26:05] [quadruped] change the gait
[info] [00:26:05] [quadruped] joy_ctrl_state_
        - before: 4
        - is_move_: 2
[info] [00:26:05] [quadruped] [qrControlFSM::RunFSM] RC_MODE: 1
[info] [00:26:05] [quadruped] [example_gazebo::control_loop] avg time cost: 1.5966301
[info] [00:26:06] [quadruped] [qrFSMStateLocomotion<T>::SwitchMode] transition_data done
[info] [00:26:06] [quadruped] [qrFSMStateLocomotion<T>::OnExit] locomotion exit
[info] [00:26:06] [quadruped] [qrFSMStateLocomotion<T>::OnEnter] on_enter state: 1
[info] [00:26:06] [quadruped] [qrOpenLoopGaitGenerator::Reset] set gait: trot
[info] [00:26:06] [quadruped] [qrStateEstimatorContainer::Reset] state_estimator_container reset
[info] [00:26:06] [quadruped] [qrFSMStateLocomotion<T>::OnEnter] time step: 0.001
[info] [00:26:06] [quadruped] [qrFSMStateLocomotion<T>::OnEnter] On Enter End
[info] [00:26:08] [quadruped] [example_gazebo::control_loop] avg time cost: 1.9917107
[info] [00:26:10] [quadruped] [example_gazebo::control_loop] avg time cost: 2.0119553
[info] [00:26:13] [quadruped] [example_gazebo::control_loop] avg time cost: 2.3814125
```

通过 git log 找到问题在修改了 executor，说明这里的操作不是线程安全的

```cpp
   // 创建多线程执行器
-  rclcpp::executors::SingleThreadedExecutor executor;
+  rclcpp::executors::MultiThreadedExecutor executor;
   executor.add_node(node);
   executor.spin();
 }
```
