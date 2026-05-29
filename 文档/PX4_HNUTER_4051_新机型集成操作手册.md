# PX4 Hnuter 4051 新机型集成操作手册

本文档记录在 PX4 中新增 `4051 hnuter` 双级倾转多旋翼机型的完整改动路径。目标是把原本位于 `hnuter/hnuter_external_controller.py` 的位置控制、姿态控制和非线性控制分配集成到 PX4 内部，并同时支持 Gazebo SITL 与实机部署。

当前 Hnuter 的特点：

- 5 个电机：4 个前部倾转旋翼电机，1 个尾部双向电机。
- 4 个倾转舵机：左右两侧各两级倾转。
- QGC 标准电机几何配置页面无法表达“两级倾转”，因此几何、分配和实物输出映射应由 PX4 代码与机型文件固定。
- 尾部电机是双向电机，控制量必须是 `[-1, 1]`，`0` 表示无尾部推力。

## 总体数据流

Hnuter 内部控制链路：

```text
trajectory_setpoint / vehicle_attitude_setpoint
        |
        v
MulticopterRateControl::runHnuterControl()
位置控制 + 姿态控制
        |
        v
vehicle_thrust_setpoint + vehicle_torque_setpoint
        |
        v
ControlAllocator
        |
        v
ActuatorEffectivenessHnuter::updateSetpoint()
双级倾转非线性控制分配
        |
        v
actuator_motors[0..4] + actuator_servos[0..3]
        |
        v
PWM_MAIN / PWM_AUX 或 Gazebo SIM_GZ_EC / SIM_GZ_SV
```

外部控制器 `hnuter/hnuter_external_controller.py` 仍可作为算法参考和对照测试工具，但集成后的主路径在 PX4 内部。

## 关键文件清单

| 功能 | 文件 |
| --- | --- |
| SITL 机型脚本 | `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter` |
| 实机机型脚本 | `ROMFS/px4fmu_common/init.d/airframes/4051_gz_hnuter` |
| 机型脚本编译注册 | `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`、`ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` |
| Gazebo 模型 | `Tools/simulation/gz/models/hnuter/model.sdf` |
| 控制分配参数声明 | `src/modules/control_allocator/module.yaml` |
| 控制分配枚举与创建 | `src/modules/control_allocator/ControlAllocator.hpp`、`src/modules/control_allocator/ControlAllocator.cpp` |
| Hnuter 专用控制分配 | `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.hpp`、`.cpp` |
| Hnuter 控制分配编译注册 | `src/modules/control_allocator/VehicleActuatorEffectiveness/CMakeLists.txt` |
| 位置和姿态控制集成 | `src/modules/mc_rate_control/MulticopterRateControl.hpp`、`.cpp` |
| Gazebo 可逆电机映射 | `src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp` |
| 外部 actuator uORB DDS 话题 | `src/modules/uxrce_dds_client/dds_topics.yaml` |

## 步骤 1：确定执行器顺序

Hnuter 的执行器顺序必须在三个地方保持一致：外部控制器、PX4 控制分配、机型输出映射。

电机顺序：

```text
Motor 1 -> xy1 -> right upper
Motor 2 -> xy2 -> right lower
Motor 3 -> xy3 -> left upper
Motor 4 -> xy4 -> left lower
Motor 5 -> xy5 -> rear bidirectional
```

舵机顺序：

```text
Servo 1 -> rj2 -> right primary tilt
Servo 2 -> lj2 -> left primary tilt
Servo 3 -> rj1 -> right secondary tilt
Servo 4 -> lj1 -> left secondary tilt
```

在 `ActuatorEffectivenessHnuter::updateSetpoint()` 中对应为：

```cpp
actuator_sp(0) = norm_right;
actuator_sp(1) = norm_right;
actuator_sp(2) = norm_left;
actuator_sp(3) = norm_left;
actuator_sp(4) = norm_tail;

servo_sp(0) = alpha2 / angle_max; // rj2
servo_sp(1) = alpha1 / angle_max; // lj2
servo_sp(2) = theta2 / angle_max; // rj1
servo_sp(3) = theta1 / angle_max; // lj1
```

## 步骤 2：创建 Gazebo 模型

模型目录：

```text
Tools/simulation/gz/models/hnuter/
```

核心文件：

```text
Tools/simulation/gz/models/hnuter/model.sdf
Tools/simulation/gz/models/hnuter/meshes/
```

Gazebo 电机插件需要和 PX4 电机函数顺序一致：

```xml
<plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
  <jointName>xyj1</jointName>
  <linkName>xy1</linkName>
  <motorNumber>0</motorNumber>
  <commandSubTopic>command/motor_speed</commandSubTopic>
</plugin>
```

5 个电机对应：

```text
motorNumber 0 -> xyj1 / xy1
motorNumber 1 -> xyj2 / xy2
motorNumber 2 -> xyj3 / xy3
motorNumber 3 -> xyj4 / xy4
motorNumber 4 -> xyj5 / xy5
```

4 个舵机对应：

```xml
<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
  <joint_name>rj2</joint_name>
  <sub_topic>servo_0</sub_topic>
</plugin>
```

```text
servo_0 -> rj2
servo_1 -> lj2
servo_2 -> rj1
servo_3 -> lj1
```

模型中曾出现的 `gz_frame_id` 和 `use_parent_model_frame` 会触发 SDFormat warning。当前已清理，验证命令：

```bash
gz sdf -k Tools/simulation/gz/models/hnuter/model.sdf
```

预期输出：

```text
Valid.
```

## 步骤 3：新增 SITL 机型脚本

新增文件：

```text
ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter
```

关键配置：

```sh
PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=hnuter}

param set-default SIM_GZ_EN 1
param set-default SIM_GZ_EN_GPS 1
param set-default SIM_GZ_EN_BARO 1
```

Gazebo 电机函数：

```sh
param set-default SIM_GZ_EC_FUNC1 101  # xy1
param set-default SIM_GZ_EC_FUNC2 102  # xy2
param set-default SIM_GZ_EC_FUNC3 103  # xy3
param set-default SIM_GZ_EC_FUNC4 104  # xy4
param set-default SIM_GZ_EC_FUNC5 105  # xy5 rear
```

Gazebo 舵机函数：

```sh
param set-default SIM_GZ_SV_FUNC1 201
param set-default SIM_GZ_SV_FUNC2 202
param set-default SIM_GZ_SV_FUNC3 203
param set-default SIM_GZ_SV_FUNC4 204
```

Hnuter 控制分配：

```sh
param set-default MAV_TYPE 2
param set-default CA_AIRFRAME 16
param set-default CA_ROTOR_COUNT 5
param set-default CA_R_REV 16
param set-default CA_SV_TL_COUNT 4
```

将该脚本加入：

```text
ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
```

示例：

```cmake
px4_add_romfs_files(
    ...
    4051_gz_hnuter
)
```

## 步骤 4：新增实机机型脚本

新增文件：

```text
ROMFS/px4fmu_common/init.d/airframes/4051_gz_hnuter
```

文件头部加入 `@output` 元信息，便于 PX4 机型元数据识别：

```sh
# @name hnuter tiltrotor
# @class VTOL
# @type tiltrotor
# @output Motor1 xy1 right upper rotor
# @output Motor2 xy2 right lower rotor
# @output Motor3 xy3 left upper rotor
# @output Motor4 xy4 left lower rotor
# @output Motor5 xy5 rear bidirectional rotor
# @output Servo1 rj2 right primary tilt
# @output Servo2 lj2 left primary tilt
# @output Servo3 rj1 right secondary tilt
# @output Servo4 lj1 left secondary tilt
```

实机输出接口固定为：

```sh
param set-default PWM_MAIN_FUNC1 101  # MAIN1 -> Motor 1 -> xy1
param set-default PWM_MAIN_FUNC2 102  # MAIN2 -> Motor 2 -> xy2
param set-default PWM_MAIN_FUNC3 103  # MAIN3 -> Motor 3 -> xy3
param set-default PWM_MAIN_FUNC4 104  # MAIN4 -> Motor 4 -> xy4
param set-default PWM_MAIN_FUNC5 105  # MAIN5 -> Motor 5 -> xy5 rear

param set-default PWM_AUX_FUNC1 201   # AUX1 -> Servo 1 -> rj2
param set-default PWM_AUX_FUNC2 202   # AUX2 -> Servo 2 -> lj2
param set-default PWM_AUX_FUNC3 203   # AUX3 -> Servo 3 -> rj1
param set-default PWM_AUX_FUNC4 204   # AUX4 -> Servo 4 -> lj1
```

尾部双向 ESC 的 PWM 语义：

```text
1000 us -> full reverse
1500 us -> neutral / stop
2000 us -> full forward
```

因此实机脚本中应固定：

```sh
param set-default PWM_MAIN_MIN5 1000
param set-default PWM_MAIN_MAX5 2000
param set-default PWM_MAIN_DIS5 1500
param set-default PWM_MAIN_FAIL5 1500
param set-default PWM_MAIN_REV 0
```

注意：`PWM_MAIN_REV` 只反转输出区间，不等同于 `CA_R_REV` 的双向电机语义。如果尾电机方向相反，优先在 ESC 或接线方向上修正，必要时再手动设置 `PWM_MAIN_REV` 的 bit 4，即数值 `16`。

将实机脚本加入：

```text
ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt
```

示例：

```cmake
px4_add_romfs_files(
    ...
    4051_gz_hnuter
)
```

## 步骤 5：注册新的控制分配类型

在：

```text
src/modules/control_allocator/module.yaml
```

给 `CA_AIRFRAME` 增加枚举：

```yaml
CA_AIRFRAME:
    values:
        16: Hnuter Tiltrotor
```

在：

```text
src/modules/control_allocator/ControlAllocator.hpp
```

包含头文件并增加枚举：

```cpp
#include <ActuatorEffectivenessHnuter.hpp>

enum class EffectivenessSource {
    ...
    HNUTER_TILTROTOR = 16,
};
```

在：

```text
src/modules/control_allocator/ControlAllocator.cpp
```

创建 Hnuter effectiveness：

```cpp
case EffectivenessSource::HNUTER_TILTROTOR:
    tmp = new ActuatorEffectivenessHnuter(this);
    break;
```

## 步骤 6：新增 Hnuter 专用控制分配

新增文件：

```text
src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.hpp
src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.cpp
```

并加入：

```text
src/modules/control_allocator/VehicleActuatorEffectiveness/CMakeLists.txt
```

示例：

```cmake
target_sources(ActuatorEffectiveness
    PRIVATE
        ActuatorEffectivenessHnuter.cpp
        ActuatorEffectivenessHnuter.hpp
)
```

Hnuter 分配器的核心原则：

- 用 PX4 的 rotor / tilt 框架注册 actuator 数量。
- 将 effectiveness matrix 置零，避免标准分配矩阵处理两级倾转几何。
- 在 `updateSetpoint()` 内直接根据 `vehicle_thrust_setpoint` 和 `vehicle_torque_setpoint` 计算 5 个电机和 4 个舵机。

注册 actuator：

```cpp
configuration.selected_matrix = 0;
_mc_rotors.enableThreeDimensionalThrust(true);

const bool mc_rotors_added_successfully = _mc_rotors.addActuators(configuration);
_first_tilt_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
const bool tilts_added_successfully = _tilts.addActuators(configuration);

configuration.effectiveness_matrices[0].setZero();
```

非线性分配中的主要变量：

```cpp
const float l1 = 0.33f;
const float l2 = 0.664f;
const float r_x = 0.105f;
const float r_z = -0.013f;
const float max_thrust_per_arm = 85.48f * 2.0f;
const float max_tail_thrust = 85.48f;
const float mass = 4.5f;
const float gravity = 9.81f;
```

控制输入还原为力和力矩：

```cpp
const float fx =  control_sp(3) * max_thrust_per_arm;
const float fy = -control_sp(4) * max_thrust_per_arm;
const float fz = -control_sp(5) * (mass * gravity * 2.0f);
const float tx =  control_sp(0) * (max_thrust_per_arm * l1);
const float ty =  control_sp(1) * (max_tail_thrust * l2);
const float tz = -control_sp(2) * (max_thrust_per_arm * l1);
```

尾电机推力允许为负：

```cpp
float F3 = Ty_comp / (r_x + l2);
F3 = math::constrain(F3, -50.0f, 50.0f);
```

双向电机归一化：

```cpp
static float thrustToNormalizedBidirectionalMotorControl(float thrust, float motor_constant, float max_velocity)
{
    if (fabsf(thrust) <= FLT_EPSILON || motor_constant <= FLT_EPSILON || max_velocity <= FLT_EPSILON) {
        return 0.f;
    }

    const float velocity = sqrtf(fabsf(thrust) / motor_constant);
    const float control = velocity / max_velocity;

    return (thrust > 0.f) ? math::constrain(control, 0.f, 1.f) : math::constrain(-control, -1.f, 0.f);
}
```

执行器输出：

```cpp
actuator_sp(0) = norm_right;
actuator_sp(1) = norm_right;
actuator_sp(2) = norm_left;
actuator_sp(3) = norm_left;
actuator_sp(4) = norm_tail;

actuator_sp(_first_tilt_idx + 0) = servo_sp(0);
actuator_sp(_first_tilt_idx + 1) = servo_sp(1);
actuator_sp(_first_tilt_idx + 2) = servo_sp(2);
actuator_sp(_first_tilt_idx + 3) = servo_sp(3);
```

## 步骤 7：处理尾部双向电机

Hnuter 的尾部电机是 Motor 5，对应 bit 4：

```sh
param set-default CA_R_REV 16
```

需要同时处理三处：

第一处，控制分配 min 值允许负数：

```cpp
const bool hnuter_reversible_tail = (_effectiveness_source_id == EffectivenessSource::HNUTER_TILTROTOR)
                                 && (actuator_type_idx == 4);

if ((_param_r_rev.get() & (1u << actuator_type_idx)) || hnuter_reversible_tail) {
    minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
}
```

第二处，发布 `actuator_motors.reversible_flags`：

```cpp
actuator_motors.reversible_flags =
    (_effectiveness_source_id == EffectivenessSource::HNUTER_TILTROTOR) ? (1u << 4) : _param_r_rev.get();
```

第三处，Gazebo 桥接要把 PX4 reversible midpoint 还原成带符号的 Gazebo 转速：

```cpp
const uint32_t reversible_outputs = _mixing_output.reversibleOutputs();

if (reversible_outputs & (1u << i)) {
    if (outputs[i] == 0) {
        output = 0.0;
    } else {
        const double midpoint = 0.5 * (min_output + max_output);
        output = ((outputs[i] - midpoint) / half_range) * max_output;
    }
}
```

这样可以避免“刚 arm 后尾部可逆电机的 0 控制量被解释成中位输出，从而猛转”的问题。

## 步骤 8：集成位置和姿态控制器

当前 Hnuter 的位置和姿态控制主路径在：

```text
src/modules/mc_rate_control/MulticopterRateControl.cpp
src/modules/mc_rate_control/MulticopterRateControl.hpp
```

在 rate controller 的主循环中，当 `CA_AIRFRAME == 16` 时进入 Hnuter 专用分支：

```cpp
if (_ca_airframe == 16 && _vehicle_control_mode.flag_control_rates_enabled
    && (_vehicle_control_mode.flag_control_position_enabled
        || _vehicle_control_mode.flag_control_offboard_enabled)) {
    if (runHnuterControl(angular_velocity, dt, rates)) {
        perf_end(_loop_perf);
        return;
    }
}
```

Hnuter 控制器订阅：

```cpp
uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
```

控制器状态：

```cpp
matrix::Vector3f _hnuter_integral_pos_error{};
matrix::Vector3f _hnuter_integral_e_R{};
matrix::Vector2f _hnuter_xy_lock_position{};
bool _hnuter_xy_lock_initialized{false};
bool _hnuter_prev_armed{false};
hrt_abstime _hnuter_armed_time{0};
```

位置控制增益与外部控制器保持一致：

```cpp
Kp(0, 0) = 2.5f;
Kp(1, 1) = 2.5f;
Kp(2, 2) = 8.f;

Dp(0, 0) = 1.8f;
Dp(1, 1) = 1.8f;
Dp(2, 2) = 4.f;

const matrix::Vector3f K_pos_I{0.f, 0.f, 3.f};
```

姿态控制增益：

```cpp
const matrix::Vector3f KR{1.5f, 1.5f, 1.5f};
const matrix::Vector3f Domega{1.2f, 1.2f, 1.2f};
const matrix::Vector3f KI{0.f, 0.f, 0.f};

matrix::Vector3f tau_c = -KR.emult(e_R) - KI.emult(_hnuter_integral_e_R) - Domega.emult(omega_error);
tau_c(2) = math::constrain(tau_c(2), -0.5f, 0.5f);
```

将控制器输出映射为 PX4 thrust / torque setpoint：

```cpp
vehicle_thrust_setpoint.xyz[0] = f_body(0) / max_thrust_per_arm;
vehicle_thrust_setpoint.xyz[1] = f_body(1) / max_thrust_per_arm;
vehicle_thrust_setpoint.xyz[2] = f_body(2) / (mass * gravity * 2.0f);

vehicle_torque_setpoint.xyz[0] = tau_c(0) / (max_thrust_per_arm * l1);
vehicle_torque_setpoint.xyz[1] = -tau_c(1) / (max_tail_thrust * l2);
vehicle_torque_setpoint.xyz[2] = tau_c(2) / (max_thrust_per_arm * l1);
```

起飞保护：

```cpp
const float takeoff_tilt_suppress_time_s = 1.f;
const float takeoff_xy_lock_time_s = 3.f;
```

作用：

- arm 后前 1 秒抑制水平推力，避免舵机和倾转机构初始瞬态太大。
- 1 到 3 秒锁定 XY 位置并限制倾转角。
- 之后恢复正常位置和姿态控制。

注意：`HnuterPositionControl.cpp/.hpp` 目前不是主控制链路。它可以作为后续拆分控制器模块的参考，但当前集成路径以 `MulticopterRateControl::runHnuterControl()` 为准。

## 步骤 9：保留外部控制器对照接口

外部控制器使用：

```text
hnuter/hnuter_external_controller.py
```

它发布：

```python
ActuatorMotors -> /fmu/in/actuator_motors
ActuatorServos -> /fmu/in/actuator_servos
```

为了让 ROS2 / uXRCE-DDS 可以访问这些 actuator 话题，需要在：

```text
src/modules/uxrce_dds_client/dds_topics.yaml
```

确认存在：

```yaml
- topic: /fmu/in/actuator_motors
- topic: /fmu/in/actuator_servos
- topic: /fmu/out/actuator_motors
- topic: /fmu/out/actuator_servos
```

外部控制器中的尾电机双向写法应与 PX4 内部一致：

```python
motor_msg.reversible_flags = 1 << 4
normalized_tail = math.copysign(velocity_tail / max_velocity, F3) if velocity_tail > 0.0 else 0.0
motor_msg.control[4] = np.clip(normalized_tail, -1.0, 1.0)
```

## 步骤 10：构建和启动

SITL 编译：

```bash
CCACHE_DISABLE=1 make px4_sitl_default -j4
```

启动 Hnuter Gazebo：

```bash
make px4_sitl gz_hnuter
```

实机固件编译根据飞控型号选择目标，例如：

```bash
make px4_fmu-v6x_default
```

如果使用其他飞控板，请替换为对应 board target。

## 步骤 11：验证清单

SITL 检查：

```bash
gz sdf -k Tools/simulation/gz/models/hnuter/model.sdf
make px4_sitl gz_hnuter
```

PX4 shell 中检查：

```sh
param show SYS_AUTOSTART
param show CA_AIRFRAME
param show CA_R_REV
param show SIM_GZ_EC_FUNC*
param show SIM_GZ_SV_FUNC*
listener actuator_motors
listener actuator_servos
control_allocator status
```

预期关键值：

```text
SYS_AUTOSTART = 4051
CA_AIRFRAME = 16
CA_R_REV = 16
actuator_motors.reversible_flags 包含 bit 4
```

实机上电前检查：

```sh
param show PWM_MAIN_FUNC*
param show PWM_AUX_FUNC*
param show PWM_MAIN_DIS5
param show PWM_MAIN_FAIL5
param show PWM_MAIN_REV
```

预期关键值：

```text
PWM_MAIN_FUNC1 = 101
PWM_MAIN_FUNC2 = 102
PWM_MAIN_FUNC3 = 103
PWM_MAIN_FUNC4 = 104
PWM_MAIN_FUNC5 = 105

PWM_AUX_FUNC1 = 201
PWM_AUX_FUNC2 = 202
PWM_AUX_FUNC3 = 203
PWM_AUX_FUNC4 = 204

PWM_MAIN_DIS5 = 1500
PWM_MAIN_FAIL5 = 1500
PWM_MAIN_REV = 0
```

安全测试顺序：

```text
1. 拆桨。
2. 选择 4051 Hnuter 机型。
3. 校准传感器和遥控器。
4. 检查 MAIN1-5 和 AUX1-4 输出功能。
5. 单独测试 4 个倾转舵机方向和行程。
6. 单独测试前 4 个电机方向。
7. 测试尾部双向电机：1500us 应停转，正负控制应分别对应正反方向。
8. 解锁观察尾部电机是否保持中位停转。
9. 系留或固定机架进行低推力联调。
10. 小油门起飞测试。
```

## 常见问题

### QGC 无法配置两级倾转

这是预期现象。QGC 标准 Actuator 页面无法描述 `rj2/lj2 + rj1/lj1` 两级倾转几何。Hnuter 不依赖 QGC 几何页面，而是：

- 用机型文件固定实物输出接口。
- 用 `CA_AIRFRAME=16` 选择 `ActuatorEffectivenessHnuter`。
- 在 `ActuatorEffectivenessHnuter::updateSetpoint()` 内直接计算 5 个电机和 4 个舵机。

### Arm 后尾电机猛转

重点检查：

```sh
param show CA_R_REV
param show PWM_MAIN_DIS5
param show PWM_MAIN_FAIL5
listener actuator_motors
```

正确状态：

```text
CA_R_REV = 16
PWM_MAIN_DIS5 = 1500
PWM_MAIN_FAIL5 = 1500
actuator_motors.reversible_flags = 16
actuator_motors.control[4] 解锁静止附近应接近 0
```

根因通常是：尾电机被标记为 reversible 后，PX4 输出层会把控制量 `0` 映射到 `MIN/MAX` 中位。如果控制分配仍按普通 `[0, 1]` 电机输出，或者 disarmed/failsafe 仍是 900us，尾电机就可能异常转动。

### Gazebo 启动时出现 SDF warning

当前 Hnuter 模型已清理 `gz_frame_id` 和 `use_parent_model_frame`。如果后续重新导出 SDF 后 warning 回来，先执行：

```bash
rg -n "<gz_frame_id>|<use_parent_model_frame>" Tools/simulation/gz/models/hnuter/model.sdf
gz sdf -k Tools/simulation/gz/models/hnuter/model.sdf
```

### `Preflight Fail: ekf2 missing data`

PX4 刚启动时 EKF2 还没收到足够传感器数据会出现该提示。如果 Gazebo 世界稳定后仍不消失，检查：

```sh
listener sensor_combined
listener vehicle_odometry
listener vehicle_gps_position
```

### 参数被旧值覆盖

PX4 的 `param set-default` 只设置默认值。如果某些参数已经被用户保存过，重新刷机或切机型后仍可能保留旧值。实机部署时至少确认：

```sh
param show PWM_MAIN_FUNC*
param show PWM_AUX_FUNC*
param show CA_AIRFRAME
param show CA_R_REV
param show PWM_MAIN_DIS5
param show PWM_MAIN_FAIL5
```

必要时手动设置：

```sh
param set CA_AIRFRAME 16
param set CA_R_REV 16
param set PWM_MAIN_FUNC1 101
param set PWM_MAIN_FUNC2 102
param set PWM_MAIN_FUNC3 103
param set PWM_MAIN_FUNC4 104
param set PWM_MAIN_FUNC5 105
param set PWM_AUX_FUNC1 201
param set PWM_AUX_FUNC2 202
param set PWM_AUX_FUNC3 203
param set PWM_AUX_FUNC4 204
param set PWM_MAIN_DIS5 1500
param set PWM_MAIN_FAIL5 1500
param save
```

## 迁移到其他 PX4 分支时的最小检查表

```text
[ ] 4051_gz_hnuter 已加入 init.d 和 init.d-posix 的 CMakeLists
[ ] CA_AIRFRAME 枚举包含 16: Hnuter Tiltrotor
[ ] ControlAllocator.hpp include 了 ActuatorEffectivenessHnuter.hpp
[ ] ControlAllocator::update_effectiveness_source() 能创建 Hnuter allocator
[ ] ControlAllocator 对 Hnuter Motor 5 设置 reversible min = -1
[ ] ControlAllocator 发布 actuator_motors.reversible_flags = 1 << 4
[ ] VehicleActuatorEffectiveness CMakeLists 编译 ActuatorEffectivenessHnuter.cpp
[ ] MulticopterRateControl 在 CA_AIRFRAME == 16 时进入 runHnuterControl()
[ ] runHnuterControl 发布 vehicle_thrust_setpoint 和 vehicle_torque_setpoint
[ ] ActuatorEffectivenessHnuter::updateSetpoint() 输出 5 motor + 4 servo
[ ] GZMixingInterfaceESC 正确处理 reversible output
[ ] 实机机型 MAIN1-5 / AUX1-4 映射正确
[ ] MAIN5 disarmed/failsafe 为 1500us
[ ] Gazebo model.sdf 电机 motorNumber 和 servo topic 顺序正确
[ ] gz sdf -k model.sdf 通过
[ ] make px4_sitl_default -j4 通过
```

## 当前验证结果

已执行：

```bash
gz sdf -k Tools/simulation/gz/models/hnuter/model.sdf
CCACHE_DISABLE=1 make px4_sitl_default -j4
```

结果：

```text
SDF: Valid.
PX4 SITL build: passed.
```

