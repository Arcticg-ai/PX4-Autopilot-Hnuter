# HNUTER 起飞、尾电机模型与 Pitch 抗饱和更新（2026-08-21）

## 修改目的

本次修改合并两个连续阶段的固件工作：修复解锁后直接输出悬停推力、先漂移再拉回的起飞行为；再根据尾部台架数据修正实机 Motor5 模型、几何参数和 Pitch 抗积分饱和。接触飞行模态、扰动力矩观测器和接触力估计不在本次范围内。

## 已实现内容

### 1. 起飞状态机

- Position/Offboard 使用 PX4 `takeoff_status`，不再按解锁后固定时间切换。
- `READY_FOR_TAKEOFF` 阶段控制器输出为零；四个单向主电机对应最小输出，双向 Motor5 对应中点。
- 进入 `RAMPUP` 时锁存实际 XY 位置并清除位置、姿态积分一次。
- 力和力矩使用标准参数 `MPC_TKO_RAMP_T` 从零同步渐增。
- 进入 `FLIGHT` 后，XY 目标从起飞锁存点平滑过渡到 FlightTask 目标。
- 删除 allocator 内第二套固定时间起飞限制，避免两个状态机不同步。
- 删除 `HNTR_LOCK_ACC`、`HNTR_LOCK_KP`、`HNTR_TO_RAMP_T`。保留的 `HNTR_TO_SUP_T/HNTR_TO_LOCK_T` 只在 `takeoff_status` 缺失时作为回退。

### 2. 实机尾电机正反独立模型

根据 4006 电机、12 寸三叶桨、40 A 双向电调台架数据，删除原来单一、对称的 `HNTR_MAX_TAIL_T=85.48 N` 模型，新增：

| 参数 | 实机默认值 | 含义 |
| --- | ---: | --- |
| `HNTR_TAIL_T_POS` | 12.78 N | 正方向最大尾推力 |
| `HNTR_TAIL_T_NEG` | 6.04 N | 负方向最大尾推力幅值 |
| `HNTR_TAIL_EXP_P` | 0.55 | 正方向力到归一化命令的反算指数 |
| `HNTR_TAIL_EXP_N` | 0.68 | 负方向力到归一化命令的反算指数 |

实机 Motor5 使用 `1000–2000 us`，停止/失效中点为 `1500 us`。同一物理力请求现在会按实测曲线产生 PWM，不再使用前电机的通用指数。

Gazebo 配置继续使用自身电机常数和对称 `85.48 N` 能力，不使用上述实机静态曲线；实机和仿真模型保持独立。

### 3. 尾电机几何和换向保护

- `HNTR_L2=0.720 m` 现在明确定义为“重心到 Motor5 推力作用线”的直接距离，分配器不再额外叠加 `HNTR_CG_X`。
- `HNTR_CG_X=0.013 m` 用于姿态相关的重力力矩前馈。
- `HNTR_CG_Z=0`：保留 `CG_Z × Fx` 物理项，但实测竖直偏移接近零，因此默认不产生补偿。
- 最大换向中点等待由 `0.30 s` 调整为 `0.10 s`，小力换向最短等待由 `0.05 s` 调整为 `0.02 s`。
- 高转速换向仍先按斜率降到零、等待中点，再按换向后斜率升高，避免大正推直接跳到大反推。

### 4. Pitch 控制和抗积分饱和

- `HNTR_TAU_P=10 N·m`，P/D/I、重力前馈和 `PITCH_BIAS` 合并后统一经过该软件限幅。
- 实际尾电机能力是不对称的：按 `L2=0.72 m`，正方向约 `9.20 N·m`，负方向约 `4.35 N·m`。因此 `10 N·m` 是控制器统一上限，不代表两个方向的执行器都能达到 10 N·m；超出部分会形成真实模型残差。
- 删除“根据分配残差阻止 AUX2 Pitch 目标变化”的逻辑。AUX2 只保留正常速率限制、连续姿态目标和原有姿态误差 governor。
- Pitch 残差从归一化量换算为物理 `N·m`；进入阈值为 `0.10 N·m`，退出阈值为 `0.05 N·m`。
- Pitch 积分采用方向性抗饱和：仅当新的积分力矩会继续增加同方向不可实现力矩时阻止；反方向消饱和始终允许。
- 位置积分不使用分配残差冻结，只保留原有加速度限幅抗饱和。

### 5. 水平接触力裕度

实机默认 `HNTR_ACC_XY` 从 `3.0` 调整为 `10.0 m/s²`。对 `4.5 kg` 机体，对应理论水平命令包络约 `45 N`。该值只是指令限幅，不等于已测得接触力；实际接触力仍受电机、倾转角、舵机速率、姿态力矩和结构约束影响。

### 6. 新增日志量

`hnuter_control_status` 新增或明确记录：

- `allocator_pitch_residual_nm`
- `pitch_residual_limited`
- `takeoff_state`
- `takeoff_status_valid`
- `takeoff_output_scale`
- `takeoff_release_progress`

`hnuter_allocator_status` 新增 `pitch_unallocated_nm`，并继续记录尾推力请求、受限命令、归一化输出、PWM/RPM估计以及换向状态。

## 烧录后的参数设置

Airframe 使用 `set-default`，不会覆盖飞控中已经保存的旧值。烧录后需要执行一次：

```text
param set HNTR_TAIL_T_POS 12.78
param set HNTR_TAIL_T_NEG 6.04
param set HNTR_TAIL_EXP_P 0.55
param set HNTR_TAIL_EXP_N 0.68
param set HNTR_L2 0.720
param set HNTR_CG_X 0.013
param set HNTR_CG_Z 0.0
param set HNTR_TAIL_REV_T 0.10
param set HNTR_T_REV_MIN 0.02
param set HNTR_ACC_XY 10.0
param set HNTR_TAU_P 10.0
param save
reboot
```

旧的 `HNTR_MAX_TAIL_T`、`HNTR_LOCK_ACC`、`HNTR_LOCK_KP`、`HNTR_TO_RAMP_T` 已从新固件参数表移除。

## 验证顺序

1. 拆桨解锁，确认主电机处于最低输出，Motor5 在 `1500 us` 左右停止。
2. 拆桨缓慢给正负 Pitch，确认 Motor5 方向正确、中点正确且没有正负满幅跳变。
3. 台架或系留验证起飞状态：`takeoff_output_scale` 应从 0 平滑到 1，XY 目标不应在释放瞬间跳变。
4. 低高度自由飞行确认水平姿态误差和两方向 Pitch 余量。
5. 最后再逐级进行接触测试；首次不直接使用 45 N 满指令。

编译通过仅证明源代码和配置可构建，不等于带桨台架、系留或实飞验证。
