# Hnuter 修改记录

最后更新：2026-07-04

本文汇总 2026-06-10 至 2026-07-04 期间 Hnuter 固件、Gazebo
模型与场景、ROS 2 外部控制器、参数和验证结果。专题实验的完整数据仍保留在
文末列出的独立文档中；当旧文档与本文冲突时，以本文记录的当前代码状态为准。

## 1. 仓库与版本状态

PX4 固件仓库：

- 路径：`~/PX4-Hnuter/PX4-Autopilot-Hnuter`
- 当前分支：`codex/hnuter-180deg-tilt-margin`
- 当前已推送提交：`526ad36e Extend Hnuter tilt range with control margin`
- `main` 当前包含狭窄空间场景提交：
  `d313c464 Add Hnuter narrow passage simulation`
- 第二级全周倾转、球面场景、数据分析和单侧故障实验目前仍属于工作区改动，
  尚未包含在上述提交中。

ROS 2 外部控制器仓库：

- 路径：`~/px4_ws_ros2`
- 当前分支：`codex/hnuter-controller-180deg-limit`
- 当前已推送提交：
  `86f11bf Stabilize Hnuter 180 degree attitude control`
- 球面控制器、球面在线参数、DDS 本机隔离和轨迹 5 当前仍属于工作区改动。

编译产生的工具链压缩包、ULog、CSV 日志和临时生成文件不属于源代码修改，
不应随代码提交。

## 2. 实机日志问题与基础固件修复

涉及文件：

- `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.cpp`
- `ROMFS/px4fmu_common/init.d/airframes/4051_gz_hnuter`
- `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`
- `src/modules/commander/esc_calibration.cpp`
- `src/lib/mixer_module/actuator_test.cpp`

根据实机日志 `log_127_2026-6-9-16-14-46.ulg` 完成以下修复：

- 修正 Hnuter 自定义控制分配的轴向和符号关系。
- Motor5 使用双向电调，`CA_R_REV=16`，PWM 中点为 `1450 us`。
- 电调校准和 actuator test 能依据 `CA_R_REV` 识别可逆电机。
- 强制清除遗留的 `PWM_MAIN_REV=1664`，避免 MAIN8、MAIN10、MAIN11
  的保存反向位破坏左右倾转机构对称性。
- 恢复非零角速度 D 增益和位置速度环 I/D 项。
- 缩短落地判定与自动上锁时间。

当前实机关键参数为：

```text
CA_R_REV=16
PWM_MAIN_REV=0
PWM_MAIN_MIN5=900
PWM_MAIN_MAX5=2000
PWM_MAIN_DIS5=1450
PWM_MAIN_FAIL5=1450
PWM_MAIN_TIM0=400
PWM_MAIN_TIM1=400

MPC_THR_HOVER=0.50
MPC_THR_MIN=0.12
MPC_USE_HTE=0
MPC_XY_VEL_I_ACC=0.5
MPC_XY_VEL_D_ACC=4.0
MPC_Z_VEL_I_ACC=0.5
MPC_Z_VEL_D_ACC=6.0

MC_ROLLRATE_D=0.001
MC_PITCHRATE_D=0.001
MC_YAWRATE_D=0.001

COM_DISARM_PRFLT=10
COM_DISARM_LAND=1
LNDMC_TRIG_TIME=0.5
LNDMC_ROT_MAX=35
```

### Motor5 最终分配状态

曾经尝试让 Motor5 只由 pitch torque 产生、完全退出总垂向力分配。该改动破坏了
Hnuter 几何模型中的力矩和合力约束，造成 Position/Offboard 起飞后上冲或失控，
因此已经回退。

当前分配同时计算：

- Motor5 的可逆尾部推力 `F3`；
- 尾部推力在质心偏置处产生的俯仰力矩；
- 前四个电机需要补偿的垂向合力 `Fz_front = W[2] - F3`；
- 主旋翼合力在 `r_x`、`r_z` 偏置处产生的寄生力矩。

因此 Motor5 不是简单跟随油门，也不是与垂向力完全解耦，而是参与满足完整六维
合力/力矩目标。其零指令仍映射到双向电调的 `1450 us` 停转点。

## 3. 解锁和起飞阶段保护

Hnuter allocator 在未解锁时将全部电机和舵机输出归零，并清除上一帧舵机状态。
解锁后的临时保护保持如下：

- 前 `1 s`：水平力目标清零，一级和二级倾转限制为 `20 deg`；
- `1-3 s`：一级和二级倾转限制为 `30 deg`；
- `3 s` 后：解除临时起飞限幅，进入完整机械工作范围。

这些限制仅用于抑制落地状态和刚解锁时的位置误差突变，不是正常飞行限幅。

## 4. 一级和二级全周倾转

涉及文件：

- `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.cpp`
- `src/modules/control_allocator/module.yaml`
- `ROMFS/px4fmu_common/init.d/airframes/4051_gz_hnuter`
- `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`
- `Tools/simulation/gz/models/hnuter/model.sdf`
- `~/px4_ws_ros2/hnuter_external_direct_controller_debug.py`
- `~/px4_ws_ros2/hnuter_direct_tuning.json`

一级倾转的机械范围改为 `-185..185 deg`，给最大 `-180..180 deg` 姿态输入
保留约 `5 deg` 控制余量：

```text
CA_SV_TL0_MINA/MAXA=-185/185
CA_SV_TL1_MINA/MAXA=-185/185
SIM_GZ_SV_MINA1/MAXA1=-185/185
SIM_GZ_SV_MINA2/MAXA2=-185/185
```

第二级倾转现已从 `-90..90 deg` 扩展为 `-180..180 deg`：

```text
CA_SV_TL2_MINA/MAXA=-180/180
CA_SV_TL3_MINA/MAXA=-180/180
SIM_GZ_SV_MINA3/MAXA3=-180/180
SIM_GZ_SV_MINA4/MAXA4=-180/180
```

Gazebo 的 `rj1`、`lj1` 关节限位和 joint position controller 命令范围同步改为
`-pi..pi`。固件中的第二级归一化也由 `theta/(pi/2)` 改为 `theta/pi`。

控制分配在两组等价万向节解之间选择距离上一帧最近的分支，并对一级
`atan2` 结果做连续展开，避免经过 `+/-180 deg` 时出现接近 `360 deg`
的舵机跳变。舵机物理角速度上限保持 `50 rad/s`。

外部 direct 控制器的手柄和轨迹输入硬限制为 `-180..180 deg`。调试阶段的
姿态安全角检查默认关闭，但 Offboard/DDS 超时、PX4 自身失效保护和水平速度
检查仍然保留。

180 度往返 SITL 验证结果：

- 目标峰值 `180.00 deg`；
- 峰值保持阶段实际平均值 `179.50 deg`；
- 俯仰 RMS 误差 `0.50 deg`；
- 一级倾转最小值 `-180.21 deg`，距机械端点仍有 `4.79 deg`；
- 水平位置误差最大值 `0.220 m`；
- 全程保持 Armed 和 Offboard，并正常返回水平悬停。

## 5. 外部控制器公共修复

涉及文件：

- `~/px4_ws_ros2/hnuter_external_controller.py`
- `~/px4_ws_ros2/hnuter_external_controller_px4_position.py`
- `~/px4_ws_ros2/hnuter_external_direct_controller_debug.py`
- `~/px4_ws_ros2/hnuter_narrow_passage_controller.py`
- `~/px4_ws_ros2/hnuter_sphere_surface_controller.py`
- `~/px4_ws_ros2/src/px4_msgs`

公共变化：

- 在导入 `rclpy` 前默认设置
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，防止局域网内另一台 PX4
  使用相同 DDS 话题时混入姿态和状态数据。
- 设置 `HNUTER_ALLOW_REMOTE_DDS=1` 可显式恢复远程 DDS；多机使用时应分配
  不同的 `ROS_DOMAIN_ID`。
- `VehicleStatus` 订阅统一改为
  `/fmu/out/vehicle_status_v1`。
- `src/px4_msgs` 子模块固定到 PX4 官方提交
  `b3f3064c6c210163522cab3fe4a29a5a000d1794`；其中的
  `VehicleStatus.msg` 与当前固件文件 SHA-256 完全一致，解决 Fast CDR
  反序列化异常。
- direct 控制器不再在空中因本地姿态检查直接 disarm；异常时优先保持输出并交由
  PX4 Offboard/failsafe 处理。
- 增加 JSON 在线参数加载、连续姿态日志、期望/实际状态和执行器输出记录。

修改 `px4_msgs` 后需要重新构建：

```bash
cd ~/px4_ws_ros2
git submodule update --init --recursive
colcon build --packages-select px4_msgs --allow-overriding px4_msgs
source install/setup.bash
```

## 6. 狭窄空间场景与控制器

新增文件：

- `Tools/simulation/gz/worlds/hnuter_narrow.sdf`
- `~/px4_ws_ros2/hnuter_narrow_passage_controller.py`

场景包含六个带上下左右四条边和碰撞体的门框。门中心沿 Gazebo `+X`
方向左右交错，高度也交错，其中第 3、5 个门具有抬高的底边。

控制器完成以下修复：

- 修正 Gazebo ENU 与 PX4 NED 的航向换算；
- 机头沿轨迹速度切线变化，不再侧着前进；
- 25 个离散航点改为三次 Hermite 连续曲线；
- 加入速度和加速度前馈，取消逐点停车；
- 飞行结束后自动着陆和上锁。

闭环验证已连续穿过全部六个门，飞行段约 `66 s`，大部分位置跟踪误差为
`0.05-0.19 m`，终点约为 ENU `(37.01, -0.02, 1.05) m`。

启动命令：

```bash
MicroXRCEAgent udp4 -p 8888
PX4_GZ_WORLD=hnuter_narrow make px4_sitl gz_hnuter

source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
python3 ~/px4_ws_ros2/hnuter_narrow_passage_controller.py
```

## 7. 巨型球面场景

新增文件：

- `Tools/simulation/gz/worlds/hnuter_sphere.sdf`
- `Tools/simulation/gz/worlds/generate_hnuter_sphere_mesh.py`
- `Tools/simulation/gz/worlds/meshes/hnuter_sphere_smooth.obj`
- `~/px4_ws_ros2/hnuter_sphere_surface_controller.py`
- `~/px4_ws_ros2/hnuter_sphere_tuning.json`

场景参数：

- 球心 ENU `[14.0, 0.0, 11.5] m`；
- 实体球半径 `10.0 m`；
- 控制轨迹半径 `9.90 m`；
- 期望机体原点位于球内 `0.10 m`，由不可达径向误差产生持续贴附压力；
- 物理碰撞使用 Gazebo 解析球体，保证碰撞法向和曲率连续；
- 可见表面使用 `1986` 顶点、`3968` 三角形的平滑 OBJ 网格；
- 球面模式禁用位置积分，避免球内不可达期望造成积分饱和。

球面控制器轨迹：

- `1`：从球顶接近并沿右侧大圆弧往返贴附滑动；
- `2`：在球顶相对角 `0/30/60/90/120 deg` 的五个位置依次接近、
  压紧、保持、释放；
- `3`：在球体下半部侧面画圆，机体法向贴合球面，默认使用机械可实现航向；
- `4`：原大姿态调试轨迹；
- `5`：单侧执行器完全故障实验。

轨迹 3 的严格切向机头要求会迫使有限转角一级关节跨越
`+180/-180 deg`。由于等价万向节解只能在第二级恰好为 `+/-90 deg`
时无扰切换，默认释放绕球面法向的航向自由度，保持完整圆周稳定。最终
`90 s` 全圆测试的位置误差 RMS 为 `0.460 m`，SO(3) 姿态误差 RMS
为 `7.07 deg`，无 Offboard 中断或空中 Disarm。

启动命令：

```bash
MicroXRCEAgent udp4 -p 8888
PX4_GZ_WORLD=hnuter_sphere make px4_sitl gz_hnuter

source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
python3 ~/px4_ws_ros2/hnuter_sphere_surface_controller.py
```

## 8. 球面实验数据分析

新增分析工具和输出：

- `Tools/analysis/plot_hnuter_sphere_journal.py`
- `docs/figures/hnuter_sphere_1782820934/`
- `docs/hnuter_sphere_run_1782820934_analysis.md`

分析脚本输出适合论文排版的矢量 PDF 和 `300 dpi` PNG 预览图：

- 估计法向贴附力；
- 期望/实际位置及三维误差；
- 期望/实际姿态及 SO(3) 测地误差；
- 派生指标 CSV 和汇总 Markdown。

日志未包含 Gazebo 接触 wrench，因此贴附力是根据推力、重力和法向加速度估算，
不是力传感器实测值。该轮数据的估计贴附力均值为 `4.59 N`，位置误差范数
RMSE 为 `0.333 m`，SO(3) 姿态误差 RMSE 为 `3.61 deg`。

## 9. 轨迹 5：单侧执行器完全故障

轨迹 5 在 `hnuter_sphere_surface_controller.py` 中实现以下过程：

1. 用五次平滑曲线在 `10 s` 内将横滚期望变为 `90 deg`，俯仰期望变为
   `20 deg`。
2. 保持故障姿态 `10 s`。
3. 用 `10 s` 将选定一侧的两个电机输出平滑衰减到零。
4. 使用另一侧三维推力和可逆 Motor5 尝试保持直升机式姿态与高度。

默认故障侧为左侧 Motor3/4，可通过在线参数
`single_side_fail_left` 修改。当前测试确认故障侧输出能够降为零，高度大部分
时间保持在约 `1.30 m`，完全故障附近横滚约为 `82-86 deg`。

该模式目前不能同时保持三维定点。完全失去一侧后，保守模型只剩另一侧三维
推力和 Motor5 共四个独立输入；横向力又会通过侧向力臂产生力矩，位置和姿态
约束不能独立满足。增益、横滚分支、预补偿和同侧差动推力试验均未消除水平
发散，差动推力版本已回退。

轨迹 5 仅用于 SITL 可控性研究，不是可用于实机的容错飞行模式。

## 10. 编译与验证记录

已完成的主要构建：

```bash
make px4_sitl_default
make px4_sitl gz_hnuter
CCACHE_DISABLE=1 make cuav_7-nano_default
CCACHE_DIR=/tmp/ccache make px4_fmu-v6x_default
```

已记录的硬件固件结果：

- CUAV 7-Nano：`build/cuav_7-nano_default/cuav_7-nano_default.px4`
  - 2026-07-06 构建：FLASH `1846884 B / 1920 KB`，`93.94%`
  - 固件包大小：`1722617 bytes`
  - SHA-256：`ee7545621f8ea866a30404ed548186410a35d97d60f513391dc0a08ab36241d5`
- Pixhawk 6X：`build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`
  - 2026-07-06 构建：FLASH `1942948 B / 1920 KB`，`98.82%`
  - 固件包大小：`1810277 bytes`
  - SHA-256：`ffe73445f592e35e97e2e2078256a266a4c97bba62f2c6dff29195a3b67b6ed2`
  - 构建日志确认 DDS 和 hardfault stream 均参与链接。

SITL 已覆盖：

- Position/Offboard 正常起飞与定点；
- direct 模式 `105/175/180 deg` 俯仰往返；
- 狭窄空间六门连续穿越；
- 球面贴附、往返滑动、多点贴附和侧面全圆；
- 第二级 `-180..180 deg` 倾转；
- 单侧执行器衰减到零的实验流程。

## 11. 当前注意事项

- 一级 `-185..185 deg` 和二级 `-180..180 deg` 是软件与仿真范围。实机使用前
  必须拆桨检查机械干涉、线束缠绕、舵机真实行程和 PWM 零位。
- 旧保存参数可能覆盖 `set-default`，刷写后应重新选择 4051 机型或逐项核对
  `CA_SV_TL*`、`PWM_MAIN_*` 和 `CA_R_REV`。
- 球面轨迹的位置期望故意位于实体球内部，普通位置误差不能直接解释为自由飞行
  跟踪性能。
- 有限 `+/-180 deg` 硬限位机构无法在所有闭合球面轨迹上同时保证机头严格沿
  切向和万向节命令连续。
- 轨迹 5 尚未达到三维定点目标，不应移植到实机。

## 12. 详细文档索引

- `docs/hnuter_flight_log_fix_2026-06-10.md`
  - 实机日志、Motor5、电调校准、参数和硬件构建。
- `docs/hnuter_narrow_world_2026-06-28.md`
  - 狭窄空间场景尺寸、连续轨迹和闭环结果。
- `docs/hnuter_direct_pitch_105_2026-06-28.md`
  - 一级/二级全周倾转、105/175/180 度测试。
- `docs/hnuter_sphere_world_2026-06-29.md`
  - 球面场景、轨迹 1-4、DDS 修复和全圆测试。
- `docs/hnuter_sphere_run_1782820934_analysis.md`
  - 贴附力、位置和姿态误差图表。
- `docs/hnuter_single_side_failure_2026-07-04.md`
  - 轨迹 5、闭环结果和可控性结论。

## 13. 2026-07-07 实机倾转/油门异常修复

针对实机反馈的一级倾转范围有限、二级倾转只向一侧明显动作、所有电机 PWM
只能到约 `1500 us` 的问题，完成以下校对：

- 实机 airframe `4051_gz_hnuter` 的舵机物理输出恢复为已验证的
  `PWM_MAIN_FUNC8..11 = 201..204`，即 `MAIN8-11 -> Servo1-4`。
  这与之前实机 `pwm_out status` 中看到的 MAIN8-11 舵机映射一致，避免
  `actuator_servos[0..3]` 被发布后落不到真实舵机引脚。
- `PWM_MAIN_TIM2/TIM3` 设置为 `50 Hz`，用于 MAIN8-11 舵机；MAIN1-5 电机
  仍保持 `400 Hz`。MAIN8-11 的 min/max/disarmed/failsafe 分别设置为
  `1000/2000/1500/1500 us`。
- Hnuter allocator 的垂向满推力从旧的 `2 * mass * g` 改为四个前电机的物理
  最大推力 `4 * 85.48 N`。旧标定会让满油门分到每个电机约 `22 N`，归一化
  后约 `0.51`，对应 PWM 约 `1500 us`。
- Hnuter allocator 的二级倾转输出归一化与验证过的
  `hnuter_external_direct_controller_debug.py` 对齐，`theta` 使用 `90 deg`
  作为 full-scale servo command，避免内部输出只有外部 direct 控制的一半。
- 集成在 `mc_rate_control` 中的 `runHnuterControl()` 同步取消正常飞行阶段
  `45 deg` 倾转锥限幅，只保留起飞初期 `20/30 deg` 保护；垂向推力发布改用
  与 allocator 相反的同一条 `MPC_THR_HOVER=0.50` 分段曲线。
- `runHnuterControl()` 的力/力矩符号继续保持与外部 direct controller 的
  `_allocator_wrench_from_body_force_torque()` 一致：
  `W = [fx, -fy, -fz, tau_x, -tau_y, -tau_z]`。

验证：

```bash
make px4_sitl_default
CCACHE_DISABLE=1 make cuav_7-nano_default
CCACHE_DISABLE=1 make px4_fmu-v6x_default
```

CUAV 7-Nano 固件：

- 路径：`build/cuav_7-nano_default/cuav_7-nano_default.px4`
- FLASH：`1847460 B / 1920 KB`，`93.97%`
- SHA-256：`2549e4653501afa410ee0647c8e7a2621a419ac47d381c357d63511cbe5b2280`

Pixhawk 6X 固件：

- 路径：`build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`
- FLASH：`1943524 B / 1920 KB`，`98.85%`
- SHA-256：`55d9bdea0cc2d01064d1ed25b2ef8436a469c607d821a3b0e274abd8c854c553`

实机刷写后建议在拆桨状态先核对：

```sh
param show PWM_MAIN_FUNC8
param show PWM_MAIN_FUNC9
param show PWM_MAIN_FUNC10
param show PWM_MAIN_FUNC11
param show PWM_MAIN_TIM2
param show PWM_MAIN_TIM3
param show CA_R_REV
pwm_out status
```

期望结果：MAIN8-11 分别是 Servo1-4，TIM2/TIM3 为 `50 Hz`，`CA_R_REV=16`。

## 14. 2026-07-09 Hnuter 控制后端模块化与 Position 油门限幅排查

针对 CUAV 7-Nano 实机在 Position 模式下姿态误差响应弱、所有电机 PWM 最大
仍停在 `1450 us` 左右，而 Stabilized 模式可以正常拉满的问题，完成以下整理：

- 将原来隐藏在 `MulticopterRateControl::runHnuterControl()` 中的 Hnuter 专用
  控制逻辑整理为 `src/modules/mc_rate_control/HnuterControl.{hpp,cpp}`。
  `mc_rate_control` 现在只负责调度、发布 `vehicle_thrust_setpoint` /
  `vehicle_torque_setpoint` 和状态统计，Hnuter 的位置力、姿态/角速度力矩和起飞
  保护状态由 `HnuterControl` 独立维护。
- 新增 `src/modules/mc_rate_control/hnuter_control_params.c`，引入 QGC 可调参数：
  `HNTR_CTRL_MODE`、`HNTR_MASS`、`HNTR_MAX_ARM_T`、`HNTR_MAX_TAIL_T`、
  `HNTR_L1/L2`、`HNTR_HOV_THR`、`HNTR_XY/Z_P/D/I`、`HNTR_ACC_XY/Z`、
  `HNTR_TILT_MAX`、`HNTR_TO_*`、`HNTR_LOCK_*`、`HNTR_ATT_*`、`HNTR_TAU_*`。
- `HNTR_CTRL_MODE=0` 保留为兼容入口，但 Hnuter position/offboard-position
  路径不能直接复用 PX4 标准 position 控制生成的 roll/pitch rate setpoint；
  Hnuter 机型的水平位移由倾转推力向量完成，机体姿态目标保持水平并只跟踪 yaw。
  `HNTR_CTRL_MODE=1` 用于非 position 姿态入口下的 Hnuter 内部几何姿态误差力矩。
- Hnuter allocator 同步读取 `HNTR_HOV_THR`、`HNTR_MASS`、
  `HNTR_MAX_ARM_T`、`HNTR_MAX_TAIL_T`、`HNTR_L1/L2`，避免控制器和 allocator
  使用不同 hover/推力标定解释同一个归一化 thrust setpoint。
- Position 模式 `1450 us` 问题的主要链路判断：
  Stabilized 能拉满说明 PWM 输出层、`PWM_MAIN_MAX*` 和电调本身不是主限幅；
  Position 模式之前走 Hnuter 位置力映射，`hover=0.50`、最大垂向推力和归一化
  曲线共同使 `vehicle_thrust_setpoint.z` 长时间落在较低区间，allocator 再按推力
  平方根模型映射到电机，表现为 PWM 卡在 `1450 us` 附近。
- 实机和 SITL airframe 新增 `HNTR_*` 默认值，并将 `MPC_THR_HOVER`、
  `MPC_THR_MIN`、`MPC_USE_HTE`、`MPC_*_I/D_ACC`、`MC_*RATE_D` 改为
  `param set-default`，避免 QGC 调参后每次重启被 airframe 强制覆盖。

当前建议实机调参入口：

```sh
param show HNTR_*
param show MC_ROLL_P
param show MC_PITCH_P
param show MC_ROLLRATE*
param show MC_PITCHRATE*
```

如果 Position 模式油门仍偏低，优先小步提高 `HNTR_HOV_THR`，并核对
`HNTR_MAX_ARM_T` 是否高估；如果 position/offboard-position 下姿态误差响应弱，
优先调 `HNTR_ATT_KR_*`、`HNTR_ATT_D_*`、`HNTR_TAU_*`。

追加 SITL 闭环验证：

- 命令：`HEADLESS=1 make px4_sitl gz_hnuter`
- DDS：复用本机 `MicroXRCEAgent udp4 -p 8888`
- 控制器：
  `source /home/hnuter/PX4-Autopilot-Hnuter/px4-venv/bin/activate`
  后运行 `/home/hnuter/px4_ws_ros2/hnuter_external_controller_px4_position.py`
- 第一次验证复现了起飞后姿态发散：position 误差同时进入 PX4 标准 roll/pitch
  attitude setpoint 和 Hnuter 倾转推力向量，导致双重修正，最终
  `vehicle_torque_setpoint` 饱和并触发 Gazebo ODE 崩溃。
- 修复后 position/offboard-position 下只使用 yaw，roll/pitch 目标保持水平；
  `vehicle_thrust_setpoint.xy` 负责水平位移。
- 在线调参将 `HNTR_ATT_KR_R/P` 从 `1.5` 提高到 `8.0`、
  `HNTR_ATT_D_R/P` 从 `1.2` 提高到 `2.0` 后，悬停 pitch 偏差从约 `8 deg`
  降到约 `1.6 deg`。
- 矩形轨迹 `1` 验证通过：高度保持约 `1.30 m`，位置跟踪误差约十几厘米量级，
  控制循环约 `250 Hz`，未再出现姿态发散或 Gazebo 崩溃。

验证：

```bash
make cuav_7-nano_default
```

CUAV 7-Nano 固件：

- 路径：`build/cuav_7-nano_default/cuav_7-nano_default.px4`
- FLASH：`1853324 B / 1920 KB`，`94.26%`
- SHA-256：`66de2dc369983c79fbddf3553753dfb550ced11152c721ffb32fe3582fd75ed8`

## 15. 2026-07-10 Stabilized 改为 Hnuter 姿态+高度模式

针对实机 `Stabilized` 下一级倾转大幅摆动、姿态误差响应链路不清的问题，完成以下修改：

- `src/modules/mc_rate_control/MulticopterRateControl.cpp`
  现在仅在 `CA_AIRFRAME=16` 且导航状态为 `NAVIGATION_STATE_STAB` 时，将
  Hnuter 的 Stabilized 交给 `HnuterControl`，避免误伤 Manual 模式。
- `src/modules/mc_rate_control/HnuterControl.{hpp,cpp}`
  新增 Hnuter manual attitude-altitude 分支：使用 PX4 `mc_att_control`
  发布的 `vehicle_attitude_setpoint` 跟踪遥控器姿态，只保留 Z 轴高度闭环；
  XY 位置误差、XY 速度误差和水平力全部置零。
- 解析 `log_201_2026-7-10-10-43-34.ulg` 后发现，Position 模式可能只发布
  velocity/acceleration setpoint，而 `trajectory_setpoint.position[*]` 全为 NaN。
  旧逻辑只用 position 是否有限判断是否处于 Hnuter 平移控制，导致 velocity-only
  Position 被误判为非平移模式，并落回 PX4 标准 rate torque。现改为按
  `vehicle_control_mode` 的 position/velocity/altitude/climb/offboard 标志判断
  `hnuter_translation_control_active`，避免 Hnuter 平移模式误用标准 rate 控制器。
- 新增可在线调节参数：
  `HNTR_STAB_Z_P`、`HNTR_STAB_Z_D`、`HNTR_STAB_Z_I`、
  `HNTR_STAB_ACC_Z`、`HNTR_STAB_Z_VEL`、`HNTR_STAB_THR_DB`。
  Stabilized 中油门杆不再直接变成推力，而是改变高度期望；中位死区内保持当前高度。
- `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.*`
  将 allocator 里的起飞倾转保护从硬编码 `20/30 deg` 改为读取
  `HNTR_TO_TILT`、`HNTR_LOCK_TILT`，保护时间读取 `HNTR_TO_SUP_T`、
  `HNTR_TO_LOCK_T`。这样 QGC 修改保护角度和时间后 allocator 与控制器一致。
- 实机和 SITL 的 `4051_gz_hnuter` 默认参数改保守：
  `HNTR_ATT_KR_R/P=1.5`、`HNTR_ATT_D_R/P=0.8`、
  `HNTR_TAU_R/P=0.25`，起飞锁定阶段 `HNTR_TO_TILT=8`、
  `HNTR_LOCK_TILT=12`，并降低 `HNTR_ACC_XY/Z` 与 `HNTR_Z_I`。

调参建议：

```sh
param show HNTR_STAB_*
param show HNTR_ATT_*
param show HNTR_TAU_*
param show HNTR_TO_*
param show HNTR_LOCK_*
```

若实机一级倾转仍快速大幅摆动，优先继续降低
`HNTR_TAU_R/P`、`HNTR_ATT_KR_R/P`，并提高 `HNTR_ATT_D_R/P` 做阻尼；
若高度保持过硬或测试架上顶得明显，降低 `HNTR_STAB_ACC_Z`、
`HNTR_STAB_Z_P` 和 `HNTR_STAB_Z_I`。

验证：

- `git diff --check`：通过。
- `make px4_sitl gz_hnuter`：C++ 编译通过；普通沙箱因 `/tmp/px4-sock-0`
  权限无法启动，提升权限后 `HEADLESS=1 make px4_sitl gz_hnuter`
  成功进入 `pxh>`，`gz_bridge` 加载 `hnuter_0` 和 4 个 servo 输出。
- `make cuav_7-nano_default`：通过。

CUAV 7-Nano 固件：

- 路径：`build/cuav_7-nano_default/cuav_7-nano_default.px4`
- FLASH：`1855420 B / 1920 KB`，`94.37%`
- SHA-256：`65347e563002eead5fec7dc1a46d8e832772aeea80b0760f48b9c65f97d95f6f`

### 15.1 Motor5 pitch-only 分配修正

实机装上尾桨后发现 Motor5 转速随油门变化，而不是只随 pitch 误差变化。排查
`ActuatorEffectivenessHnuter` 后确认原因是尾电机推力 `F3` 中包含
`-r_x * W[2]` 垂向总推力补偿项，导致 collective thrust/油门变化会直接改变
Motor5。

已将 Motor5 分配改为只由 pitch torque `W[4]` 生成：

```cpp
float F3 = W[4] / (r_x + l2);
```

总升力继续由 `Fz_front = W[2] - F3` 分给前四个主电机补偿。预期现象是 pitch
误差为零时 Motor5 保持在双向 ESC 中点附近，油门变化主要体现在 MAIN1-4。

### 15.2 Pitch 水平配平偏置

实机尾桨装机后发现，由于重心偏后，水平姿态下 Motor5 不应强制停在双向电调中点
`1450 us`，否则 pitch 控制必须等姿态误差出现后才产生补偿。新增 QGC 可调参数：

- `HNTR_PITCH_BIAS`：归一化 pitch torque 常值偏置，默认 `0.0`。

该偏置叠加在 Hnuter pitch 姿态控制器输出之后，因此水平姿态时可以给尾桨一个基础
推力，同时 pitch 误差闭环仍会在此基础上继续增减。正值会让 Motor5 朝正 pitch
力矩方向偏离中点；若实机现象方向相反，使用负值。

建议实机固定架调试从小量开始：

```sh
param set HNTR_PITCH_BIAS 0.02
```

若抬头趋势更严重，改为负值；若趋势减小但仍不足，按 `0.01` 或 `0.02` 逐步增加绝对值。

### 15.3 Pitch 静差积分修正

解析 `logs/log_213_2026-7-10-20-00-14.ulg`、
`log_214_2026-7-10-20-35-52.ulg` 和
`log_215_2026-7-10-20-43-30.ulg` 后确认：

- `HNTR_PITCH_BIAS` 对 pitch 有明显作用，`log_215` 中 bias 为 `0.095`
  时 Motor5 平均约 `1577 us`。
- pitch 仍不能维持水平，`log_215` 中 pitch 最大误差约 `75 deg`。
- `HNTR_TAU_P` 飞行中多次调到 `15/20/25`，但 pitch 输出大部分时间并未长期撞到
  torque 上限，单纯继续放大 `HNTR_TAU_P` 不是主要解法。
- 当前控制器虽然累计 `_integral_e_R`，但之前没有把姿态积分项加入 `tau_c`，
  因此重心偏后、尾桨安装偏差和 ESC 死区造成的静态 pitch 误差无法自动消除。

新增参数：

- `HNTR_ATT_I_P`：pitch 姿态积分增益，airframe 默认 `0.8`。
- `HNTR_ATT_ILIM_P`：pitch 积分最多贡献的物理力矩，单位 `Nm`，airframe 默认 `3.0`。

实现方式：在 pitch torque `tau_c(1)` 进入 `HNTR_TAU_P` 总限幅前，加入
`-HNTR_ATT_I_P * integral_e_R_pitch`，并用 `HNTR_ATT_ILIM_P` 限制积分力矩。
`HNTR_PITCH_BIAS` 仍作为粗配平量使用，积分项用于消除剩余长期静差。

固定架建议：

```sh
param set HNTR_PITCH_BIAS 0.03
param set HNTR_ATT_I_P 0.5
param set HNTR_ATT_ILIM_P 3.0
param set HNTR_TAU_P 8.0
param save
```

若 pitch 长期误差仍不回零，优先小幅增加 `HNTR_ATT_I_P`；若积分后慢慢越顶越大，
降低 `HNTR_ATT_I_P` 或 `HNTR_ATT_ILIM_P`，并重新检查 `HNTR_PITCH_BIAS` 正负方向。

## 16. 2026-07-13 级联位置/速度控制与实机推力模型

- 将位置误差直通加速度的旧平移环重构为位置 P 外环和速度 PID 内环。
- 增加速度/加速度向量限幅、速度积分条件抗饱和和实测加速度阻尼。
- 修复 `HNTR_ACC_Z` 错误取较大值、Offboard 平移误判及模式入口不完整问题。
- 控制器输出改为物理推力比例，移除会相互抵消的 `HNTR_HOV_THR` 映射。
- SITL 保留 Gazebo 电机常数；硬件新增 `HNTR_MOT_HOV/HNTR_MOT_EXPO`
  悬停点锚定模型，不再把仿真电机常数用于 4112/460 KV/15 寸实机。
- SITL DDS Offboard 起飞稳定，最终位置误差约 `[1.6, -1.7, 0.7] cm`。
- `cuav_7-nano_default` 编译通过，Flash `94.60%`。

完整参数、实机标定流程和验证数据见
`docs/hnuter_cascaded_position_control_2026-07-13.md`。
# 2026-07-14：遥控辅助姿态保持与渐进回平

- Hnuter Position/Velocity 手动模式新增 AUX1 roll 角速度和 AUX2 pitch 角速度积分设定；
- AUX 通道回中后保持当前姿态期望，不再自动返回水平；
- AUX3 上升沿锁存渐进回平，回平速度由 `HNTR_RC_LVL_R` 限制；
- 手动 Position 偏航改为内部角速度积分和航向保持，避免上游 yaw 期望随实际值漂移；
- 新增 `HNTR_RC_ATT_EN`、`HNTR_RC_RATE_R/P`、`HNTR_RC_DB`、
  `HNTR_RC_RATE_Y`、`HNTR_RC_ANG_MAX`、`HNTR_RC_LVL_R` 参数；
- 实机默认最大辅助姿态 45°，参数范围保留到 180°；
- `RC_MAP_AUX1/2/3` 默认映射到物理通道 6/7/8；
- `make px4_sitl_default` 编译通过，`gz_hnuter` Position 闭环测试通过；
- 详细设计、映射和结果见 `docs/hnuter_rc_attitude_hold_2026-07-14.md`。

## 17. 2026-07-14 实机日志分析与空中误着陆修复

- 分析 2026-07-13 两次 Position 实机日志，第二架次定高误差 RMS 为 `2.7 cm`，
  最终参数段 RMS 为 `1.1 cm`，控制分配无饱和。
- 确认一级倾转约 `1 deg` 标准差的慢动作主要来自 Body-X 位置保持力，不是姿态
  环高频自激。
- 确认低油门停转是飞行高度约 `1.1 m` 时误判 landed，随后
  `COM_DISARM_LAND` 自动上锁造成，并非电机 PWM 再次被限幅。
- Hnuter 着陆检测改用 Motor1--4 allocator 后平均控制量，新增
  `HNTR_LND_GC_R`、`HNTR_LND_MIN_R` 两个 QGC 可调阈值。
- 实机和 SITL 的 `LNDMC_TRIG_TIME` 从 `0.5 s` 调整为 `2.0 s`，保留真正落地后的
  自动上锁。
- 完整报告和图表见 `logs/analyze/2026-07-13_flight_analysis.md`。

## 18. 2026-07-14 固件版本标识修复

- 确认初始源码快照来自 2025 年 7 月 PX4 `main`，属于 PX4 v1.17 开发周期，
  早于正式 `v1.17.0` release。
- 初始导入丢失上游 Git tag，导致构建回退为 `v0.0.0`，QGC 主要显示
  Git 哈希。
- 新增 `cmake/hnuter_version.cmake`，按 PX4 标准厂商版本格式设置
  `v1.17.0-1.0.0-dev`。
- QGC 主固件版本为 PX4 `1.17.0 dev`，Hnuter 厂商版本为 `1.0.0 dev`；
  Git 提交仍保留在 MAVLink 自定义版本字段中用于追溯。

## 19. 2026-07-14 日志 9 姿态漂移与一级倾转振荡修复

- 分析 `log_9_2026-7-14-20-40-28.ulg`，排除 EKF 跳变、飞行故障和 active
  actuator 饱和。
- 确认 roll rate、roll torque、一级倾转左右差动和左右电机差动存在共同的
  `3.3-3.7 Hz` 峰值；pitch 增大后 allocator 的 `atan2(Fx,Fz)` 几何关系会放大
  该横滚振荡在一级倾转上的表现。
- AUX1/AUX2/yaw 回中沿改为锁存当前实测姿态，修复机构滞后时继续追赶积分期望、
  回中后仍明显转动的问题。
- 进入 Position 模式时从实测姿态开始，并按 `HNTR_RC_LVL_R` 渐进建立水平目标。
- 修复手动 Hnuter yaw rate 前馈被 `trajectory_setpoint.yawspeed` 覆盖的问题。
- 几何姿态积分扩展为 roll/pitch/yaw 三轴，新增 `HNTR_ATT_I_R/Y` 和
  `HNTR_ATT_ILIM_R/Y`；积分力矩按 `HNTR_ATT_ILIM_*` 正确限幅，并加入落地清零和
  力矩饱和抗积分累积。
- 旧 pitch 积分硬限制下，本次参数最多只能产生 `0.09 Nm`；修复后
  `HNTR_ATT_ILIM_P=0.8 Nm` 可以真正生效，实机不可直接恢复过大的 I 增益。
- 实机 airframe 的 `HNTR_ATT_I_P` 默认值由旧的 `0.8` 改为 `0`，先稳定 P/D
  和分配器后再从小值启用积分。
- 完整报告和图表见
  `logs/analyze/log_9_2026-7-14-20-40-28_analysis.md`。

## 20. 2026-07-31 e0958bbd 实机 Roll 调试参数与安全修正

- 根据 `log_48`/`log_53` 对比，将实机 Roll 参数从已保存的
  `18.2/9.6/56.3` 条件迁移到 `10.0/4.0/15.0`，用于降低 3--4 Hz 横滚振荡。
- 将实机 `HNTR_VEL_I_XY` 从已保存的 `0.39` 条件迁移到 `0.20`。
- 将 Hnuter 着陆阈值改为 `HNTR_LND_GC_R=0.85`、
  `HNTR_LND_MIN_R=0.80`，实机悬停锚点保持 `HNTR_MOT_HOV=0.50`。
- 修复 Hnuter RC/控制消息新鲜度使用旧陀螺仪采样时间的问题，改用当前 HRT 时间，
  避免单周期错误重置 AUX 姿态和航向保持状态。
- 舵机角度反馈、`Fy -> Tz` 几何补偿、真实 allocation residual 和大扰动垂直推力
  裕度暂缓到下一阶段；详细计划见
  `docs/hnuter_e095_roll_debug_followup_2026-07-31.md`。

## 21. 2026-08-04 无延迟实机固件备份基线

- 将 `e0958bbd` 上的 `75fb3965` Roll 初修复明确保存为无舵机延迟模型的实机
  固件基线。
- 记录该版本没有合入 `servo-id` 和 `identified-delay-actuator` 实验分支。
- 汇总相对 `e0958bbd` 的 Roll 参数、XY 积分、着陆阈值、消息时间基准修复和
  条件迁移逻辑。
- 记录 MAIN8--11 实飞保存范围 `800--2200 us` 与舵机完整输入范围
  `500--2500 us` 不一致，以及二级倾转缺少独立齿轮比参数的问题。
- 完整版本身份、历史构建哈希和回退说明见
  `docs/hnuter_no_delay_firmware_baseline_2026-08-04.md`。

## 22. 2026-08-04 舵机 500--2500 us 与二级齿轮比标定

- 放宽 `pwm_out` / `px4io` 参数元数据，使 PWM 端点可配置到
  `500--2500 us`；Hnuter 电机输出范围保持不变。
- Hnuter MAIN8--11 舵机默认端点改为 `500/2500 us`，中立、失效输出继续为
  `1500 us`。
- 只对 `log_48`、`log_53`、`log_55` 中确认的四路 `800/2200 us` 完整旧组合
  做一次性条件迁移，后续逐路实测端点不会被重启覆盖。
- 新增 `HNTR_S2_GEAR`，定义为“舵机轴角 / 二级输出关节角”；实机默认
  `2.0`，当前无齿轮 Gazebo 模型使用 `1.0`。
- 一级角度改为正负 180 deg 对应归一化正负 1；二级归一化包含齿轮比，并将
  齿轮比 2.0 下的物理关节范围限制为正负 90 deg。
- SITL、Gazebo 启动和 CUAV 7-Nano 固件构建均通过；完整公式、迁移、产物哈希和
  台架检查见 `docs/hnuter_servo_pwm_gear_calibration_2026-08-04.md`。

## 23. 2026-08-08 Pitch 与尾电机安全修改

- 对 2026-08-07 日志中的完整 Pitch 激进参数组合做条件迁移：降低 Pitch P/D、
  总力矩、指令速率和最大角度，关闭 Pitch 积分；不覆盖后续非匹配人工调参。
- 将 `HNTR_PITCH_BIAS` 在物理力矩域与 P/D/I 合并后再执行最终
  `HNTR_TAU_P` 限制，并让几何控制抗积分饱和判定包含配平力矩。
- 新增 `HNTR_TAIL_REV_T`；尾电机换向必须先经过中立，当前临时默认 `0.30 s`。
- 新增 `HNTR_S1_RATE`、`HNTR_S2_RATE`，实机按保守舵机轴速率
  `4.7 rad/s` 限制；2:1 二级齿轮后的关节速率为 `2.35 rad/s`。
- 尾电机正反推独立模型和真实 allocation residual 暂缓；辨识依据、残差含义与
  拆桨/系留验证顺序见 `docs/hnuter_pitch_tail_safety_2026-08-08.md`。
