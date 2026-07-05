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
