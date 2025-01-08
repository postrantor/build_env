在发送电机的控制指令时候应该指定电机的控制模式：

```cpp
void
qrRobotA1Sim::SendCommand(const std::array<float, 60> motor_cmd) {
  for (int id = 0; id < NumMotor; id++) {
    low_cmd.motor_cmd[id].mode = 0x0A;
    low_cmd.motor_cmd[id].q = motor_cmd[id * 5 + 0];
    low_cmd.motor_cmd[id].k_q = motor_cmd[id * 5 + 1];
    low_cmd.motor_cmd[id].dq = motor_cmd[id * 5 + 2];
    low_cmd.motor_cmd[id].k_dq = motor_cmd[id * 5 + 3];
    low_cmd.motor_cmd[id].tau = motor_cmd[id * 5 + 4];

    joints_cmd_pub[id]->publish(low_cmd.motor_cmd[id]);
  }
}
```
