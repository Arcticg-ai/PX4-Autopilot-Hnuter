# Hnuter倾转舵机调试指南

## 配置总结

### 1. 机架配置 (4051_gz_hnuter)
- ✅ `SYS_CTRL_ALLOC = 1` - 启用控制分配器
- ✅ `CA_AIRFRAME = 16` - 使用自定义Hnuter控制器
- ✅ `CA_ROTOR_COUNT = 5` - 5个电机
- ✅ `CA_SV_TL_COUNT = 4` - 4个倾转舵机
- ✅ `CA_SV_TL*_CT = 0` - 禁用标准倾转控制（由自定义控制器接管）

### 2. 舵机映射
```
PX4执行器索引 → Gazebo话题 → 物理关节
Servo 0 (idx 5) → servo_0 → rj2 (右臂主倾转，偏航轴)
Servo 1 (idx 6) → servo_1 → lj2 (左臂主倾转，偏航轴)
Servo 2 (idx 7) → servo_2 → rj1 (右臂副倾转，滚转轴)
Servo 3 (idx 8) → servo_3 → lj1 (左臂副倾转，滚转轴)
```

### 3. 控制流程
```
控制器输入 (control_sp)
  ↓
ActuatorEffectivenessHnuter::updateSetpoint()
  ↓ 计算倾转角度 (alpha1, alpha2, theta1, theta2)
  ↓ 归一化到 [-1, 1]
  ↓
actuator_sp(_first_tilt_idx + 0..3) = 归一化角度
  ↓
混合输出系统 (MixingOutput)
  ↓ 使用 SIM_GZ_SV_MINA/MAXA 转换为弧度
  ↓
GZMixingInterfaceServo::updateOutputs()
  ↓
Gazebo servo_0..3 话题 (弧度值)
  ↓
JointPositionController
  ↓
物理关节运动
```

## 调试步骤

### 步骤1: 验证舵机执行器是否注册
```bash
# 启动仿真
make px4_sitl gz_hnuter

# 在PX4控制台中检查
actuator_test servo 0 0.5  # 测试Servo 0，设置为50%
actuator_test servo 1 0.5  # 测试Servo 1
actuator_test servo 2 0.5  # 测试Servo 2
actuator_test servo 3 0.5  # 测试Servo 3
```

### 步骤2: 检查参数
```bash
# 在PX4控制台中
param show CA_SV_TL_COUNT    # 应该是 4
param show CA_SV_TL0_CT      # 应该是 0
param show SIM_GZ_SV_FUNC1   # 应该是 201
param show SIM_GZ_SV_MAXA1   # 应该是 90 (deg)
```

### 步骤3: 监控Gazebo话题
```bash
# 在另一个终端
gz topic -l | grep servo
gz topic -e -t /model/hnuter/servo_0
gz topic -e -t /model/hnuter/servo_1
gz topic -e -t /model/hnuter/servo_2
gz topic -e -t /model/hnuter/servo_3
```

### 步骤4: 最小复现（仅驱动右侧两级倾转）
目标：分别只给 `servo_0 (rj2)` 或 `servo_2 (rj1)` 发送 `+0.2 rad`，观察两级是否绕不同轴运动。

- 使用 `~/px4_ros2_ws/hnuter_external_controller.py` 的舵机测试模式（`servo_test_mode=True`），它会周期性输出：
  - 阶段1：仅 `servo_0 (rj2)` 为 `+0.2 rad`
  - 阶段3：仅 `servo_2 (rj1)` 为 `+0.2 rad`
- 为避免与 PX4→gz_bridge 双通道打架，外部脚本已默认关闭直发 Gazebo 舵机（`publish_gz_servos_direct=False`），保持由 gz_bridge 作为唯一舵机控制源。

### 步骤4: 添加调试输出
在 `ActuatorEffectivenessHnuter::updateSetpoint()` 中添加：
```cpp
// 在第220行之后添加
PX4_INFO("Tilt angles: alpha1=%.3f, alpha2=%.3f, theta1=%.3f, theta2=%.3f",
         (double)alpha1_norm, (double)alpha2_norm,
         (double)theta1_norm, (double)theta2_norm);
PX4_INFO("Servo outputs: [%.3f, %.3f, %.3f, %.3f]",
         (double)actuator_sp(_first_tilt_idx + 0),
         (double)actuator_sp(_first_tilt_idx + 1),
         (double)actuator_sp(_first_tilt_idx + 2),
         (double)actuator_sp(_first_tilt_idx + 3));
```

## 可能的问题

### 问题1: num_tilts = 0
**症状**: 舵机完全不动
**原因**: `_tilts.count()` 返回0，舵机输出代码被跳过
**解决**: 确保 `CA_SV_TL_COUNT = 4` 参数正确设置

### 问题2: _first_tilt_idx 错误
**症状**: 舵机输出写入到错误的执行器索引
**原因**: 电机数量配置错误
**解决**: 确保 `CA_ROTOR_COUNT = 5`

### 问题3: 角度归一化错误
**症状**: 舵机运动范围不正确
**原因**: 归一化参数 `angle_max_rad` 不匹配
**解决**: 检查 `SIM_GZ_SV_MINA/MAXA` 参数

### 问题4: Gazebo功能码映射错误
**症状**: PX4输出舵机指令，但Gazebo没有响应
**原因**: `SIM_GZ_SV_FUNC*` 参数配置错误
**解决**: 确保 FUNC1=201, FUNC2=202, FUNC3=203, FUNC4=204

## 下一步
如果舵机仍然不工作，需要：
1. 添加调试输出确认 `num_tilts` 的值
2. 确认 `_first_tilt_idx` 的值
3. 检查 `actuator_sp` 数组是否被正确写入
4. 监控Gazebo话题是否收到数据
