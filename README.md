# PX4 Autopilot HNUTER 4051

这是面向 `4051 hnuter` 双级倾转多旋翼机型定制的 PX4 Autopilot 分支。项目目标是把原先通过 Offboard 外部程序完成的位置控制、姿态控制和控制分配，集成到 PX4 内部，形成一套可以在 Gazebo SITL 和实机上直接使用的专用飞控固件。

HNUTER 4051 不是标准倾转旋翼：每侧旋翼具备两级倾转机构，尾部还有一个双向电机。QGroundControl 的标准电机/舵机几何配置页面无法准确描述这种执行器结构，因此本仓库把机型几何、执行器顺序、输出接口和非线性控制分配固定在 PX4 机型文件与源码中。

## 机型设计理念

HNUTER 4051 的设计重点不是套用 PX4 现有 VTOL 转换逻辑，而是把它作为一类具有多方向力控制能力的倾转多旋翼来处理。

核心思路：

- 使用 4 个前部倾转旋翼提供主要升力、横向力和姿态力矩。
- 使用左右两侧各两级倾转机构，扩大单侧旋翼力方向的可控范围。
- 使用尾部双向电机直接提供正反向尾部推力，改善俯仰与纵向控制能力。
- 将位置控制、姿态控制和控制分配放在 PX4 内部闭环运行，减少 Offboard 链路延迟和外部程序依赖。
- 固定执行器顺序和飞控输出接口，避免 QGC 标准几何配置误覆盖两级倾转结构。

控制链路如下：

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

## 执行器布局

电机顺序：

| PX4 function | 执行器 | 位置 | 说明 |
| --- | --- | --- | --- |
| Motor 1 | `xy1` | right upper | 右侧上旋翼 |
| Motor 2 | `xy2` | right lower | 右侧下旋翼 |
| Motor 3 | `xy3` | left upper | 左侧上旋翼 |
| Motor 4 | `xy4` | left lower | 左侧下旋翼 |
| Motor 5 | `xy5` | rear | 尾部双向电机 |

舵机顺序：

| PX4 function | 执行器 | 位置 | 说明 |
| --- | --- | --- | --- |
| Servo 1 | `rj2` | right primary tilt | 右侧一级倾转 |
| Servo 2 | `lj2` | left primary tilt | 左侧一级倾转 |
| Servo 3 | `rj1` | right secondary tilt | 右侧二级倾转 |
| Servo 4 | `lj1` | left secondary tilt | 左侧二级倾转 |

`ActuatorEffectivenessHnuter` 发布的 actuator 顺序固定为：

```text
actuator_motors[0] -> Motor 1 -> xy1
actuator_motors[1] -> Motor 2 -> xy2
actuator_motors[2] -> Motor 3 -> xy3
actuator_motors[3] -> Motor 4 -> xy4
actuator_motors[4] -> Motor 5 -> xy5 rear bidirectional

actuator_servos[0] -> Servo 1 -> rj2
actuator_servos[1] -> Servo 2 -> lj2
actuator_servos[2] -> Servo 3 -> rj1
actuator_servos[3] -> Servo 4 -> lj1
```

## 快速开始

克隆仓库后初始化子模块和依赖：

```bash
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
```

运行 HNUTER Gazebo SITL：

```bash
make px4_sitl gz_hnuter
```

如果本地已经运行过旧参数配置，可以在重新启动前清理 SITL 参数文件：

```bash
rm -f build/px4_sitl_default/rootfs/parameters.bson
rm -f build/px4_sitl_default/rootfs/parameters_backup.bson
```

常用验证命令：

```bash
CCACHE_DISABLE=1 make px4_sitl_default -j4
gz sdf -k Tools/simulation/gz/models/hnuter/model.sdf
python3 -m py_compile hnuter/hnuter_external_controller.py
```

## 实机部署

实机机型脚本位于：

```text
ROMFS/px4fmu_common/init.d/airframes/4051_gz_hnuter
```

根据你的飞控板选择对应目标编译，例如：

```bash
make px4_fmu-v6x_default
```

刷写后在 QGroundControl 中选择 `hnuter tiltrotor` 机型，或确保 `SYS_AUTOSTART=4051`。本机型不依赖 QGC 的标准电机几何页面配置两级倾转结构，输出接口已经在机型文件中固定。

默认实机输出映射：

| 飞控接口 | PX4 function | 执行器 |
| --- | --- | --- |
| MAIN1 | Motor 1 | `xy1` |
| MAIN2 | Motor 2 | `xy2` |
| MAIN3 | Motor 3 | `xy3` |
| MAIN4 | Motor 4 | `xy4` |
| MAIN5 | Motor 5 | `xy5` 尾部双向电机 |
| AUX1 | Servo 1 | `rj2` |
| AUX2 | Servo 2 | `lj2` |
| AUX3 | Servo 3 | `rj1` |
| AUX4 | Servo 4 | `lj1` |

尾部电机按双向 ESC 配置：

```text
1000 us -> full reverse
1500 us -> neutral / stop
2000 us -> full forward
```

因此 MAIN5 的 disarmed 和 failsafe 默认值均为 `1500 us`。如果实机尾部方向相反，优先在 ESC 或接线侧修正；也可以手动设置 `PWM_MAIN_REV` 的 bit 4，也就是数值 `16`。

## 关键参数

HNUTER 4051 的核心参数如下：

```text
MAV_TYPE       = 2
SYS_CTRL_ALLOC = 1
CA_AIRFRAME   = 16
CA_ROTOR_COUNT = 5
CA_R_REV      = 16
CA_SV_TL_COUNT = 4
```

Gazebo 输出函数：

```text
SIM_GZ_EC_FUNC1-5 -> Motor 1-5
SIM_GZ_SV_FUNC1-4 -> Servo 1-4
```

实机输出函数：

```text
PWM_MAIN_FUNC1-5 -> Motor 1-5
PWM_AUX_FUNC1-4  -> Servo 1-4
```

## 代码结构

与 HNUTER 4051 相关的主要文件：

| 功能 | 文件 |
| --- | --- |
| SITL 机型脚本 | `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter` |
| 实机机型脚本 | `ROMFS/px4fmu_common/init.d/airframes/4051_gz_hnuter` |
| Gazebo 模型 | `Tools/simulation/gz/models/hnuter/model.sdf` |
| 控制分配参数声明 | `src/modules/control_allocator/module.yaml` |
| 控制分配注册 | `src/modules/control_allocator/ControlAllocator.hpp`、`src/modules/control_allocator/ControlAllocator.cpp` |
| HNUTER 控制分配 | `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.hpp`、`.cpp` |
| HNUTER 控制逻辑 | `src/modules/mc_rate_control/MulticopterRateControl.hpp`、`.cpp` |
| Gazebo 可逆电机映射 | `src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp` |
| 外部参考控制器 | `hnuter/hnuter_external_controller.py` |

`hnuter/hnuter_external_controller.py` 保留为算法对照、外部 actuator 控制测试和参数调试参考。正常集成路径应使用 PX4 内部控制器和 `ActuatorEffectivenessHnuter`。

## 安全检查

实机首次上电或修改参数后，建议按以下顺序检查：

1. 拆桨，确认 MAIN1-4 单向电机在 disarmed 状态下停止。
2. 确认 MAIN5 尾部双向电机在 disarmed 和 failsafe 状态下为中位停止。
3. 使用 actuator test 分别检查 Motor 1-5 和 Servo 1-4 的物理对应关系。
4. 检查尾部电机正反方向，避免 arm 后产生反向俯仰力矩。
5. 检查 4 个倾转舵机方向、零位和机械限位，必要时收窄 PWM 范围。
6. 确认 `CA_AIRFRAME=16`、`CA_R_REV=16`、`CA_SV_TL_COUNT=4` 未被 QGC 几何页面覆盖。
7. 首飞前先低油门约束测试，再进行短时离地悬停。

## 与 PX4 上游的关系

本仓库基于 PX4 Autopilot 修改，保留 PX4 原始许可证和工程结构。HNUTER 4051 相关代码是针对特定双级倾转多旋翼平台的实验性集成，不代表 PX4 上游通用机型支持。

PX4 官方项目与文档：

- PX4 官网：<https://px4.io>
- PX4 用户文档：<https://docs.px4.io>
- PX4 上游仓库：<https://github.com/PX4/PX4-Autopilot>

许可证见 [LICENSE](LICENSE)。
