# Hnuter 实机 OK 基线：`hnuter-ok-144bd9fe`

## 基线定义

**状态：OK，实机验证通过，可作为当前飞行器的回退基线。**

这个 OK 标记绑定的是“固件源码 + CUAV 7 Nano 硬件 + Hnuter 4051 机型 +
本目录参数快照”这一整套组合，而不只是 Git 提交号。仅切换源码但没有恢复或核对
对应参数，不能视为恢复到本 OK 状态。

| 项目 | 固定值 |
|---|---|
| Git 标签 | `hnuter-ok-144bd9fe` |
| 源码提交 | `144bd9fea6bf7a2b55f9d530809e488292e0d615` |
| 原分支 | `main` |
| 固件内部版本 | `v1.16.0-11-g144bd9fe` |
| 飞控硬件 | `CUAV_7_NANO` |
| 构建目标 | `cuav_7-nano_default` |
| `SYS_AUTOSTART` | `4051`（Hnuter） |
| `CA_AIRFRAME` | `16` |
| 验证日期 | `2026-07-30` |
| 验证范围 | Position 模式起飞、悬停、姿态/位置操纵、着陆和自动上锁 |

当前工作区对应固件包：

```text
build/cuav_7-nano_default/cuav_7-nano_default.px4
size:   1,739,273 bytes
sha256: 3a6e989216151e1974ada4188ec8514577ec045cf0954fba7fd8e7a3da526ec1
```

## 参数快照与实飞证据

完整参数必须以
[`log_48_2026-07-30_parameters.params`](log_48_2026-07-30_parameters.params)
为准。它包含 ULog 中全部 `1154` 个初始参数，飞行中参数变化为 `0`。

```text
parameter snapshot sha256:
f1f15cf273e708a65014b9a1313b017a0ea7f2dae6efabfb45d18036d2c893d8

source ULog:
log_48_2026-7-30-15-43-58.ulg

source ULog sha256:
f8115b30ba16d622a5963d4f9f7273600c35c7266826ccf57ce5c0cdd2f03a73
```

飞行性能、异常点和判定依据见
[`log_48_2026-07-30_analysis.md`](log_48_2026-07-30_analysis.md)。

参数文件还包含传感器标定、RC 标定和输出校准。因此只能直接恢复到本次试飞的
同一架飞机；其他飞行器只能参考控制参数，不能整体导入。

## OK 参数组合

以下数值是本次 OK 实飞实际使用值，不是源码 airframe 默认值。

### 位置、速度和高度控制

```text
HNTR_POS_P_XY     3.75
HNTR_POS_P_Z      3.50
HNTR_VEL_P_XY     9.05
HNTR_VEL_I_XY     0.39
HNTR_VEL_D_XY     0.36
HNTR_VEL_ILIM_XY  1.50
HNTR_VEL_P_Z      4.00
HNTR_VEL_I_Z      0.20
HNTR_VEL_D_Z      0.40
HNTR_VEL_ILIM_Z   2.50
HNTR_VEL_XY       3.00
HNTR_VEL_UP       1.50
HNTR_VEL_DN       1.00
HNTR_ACC_XY       3.00
HNTR_ACC_Z       45.60

HNTR_STAB_Z_P     2.00
HNTR_STAB_Z_I     0.20
HNTR_STAB_Z_D     1.20
HNTR_STAB_Z_VEL   0.50
HNTR_STAB_ACC_Z   5.00
HNTR_STAB_THR_DB  0.20
```

### 姿态、力矩和推力模型

```text
HNTR_ATT_KR_R     18.20
HNTR_ATT_KR_P     20.00
HNTR_ATT_KR_Y      6.00
HNTR_ATT_D_R       9.60
HNTR_ATT_D_P       8.00
HNTR_ATT_D_Y       2.30
HNTR_ATT_I_P       0.06
HNTR_ATT_ILIM_P    0.80

HNTR_TAU_R        56.30
HNTR_TAU_P        10.00
HNTR_TAU_Y         0.80
HNTR_PITCH_BIAS    0.09
HNTR_ROLL_SIGN     1.00
HNTR_TAIL_SIGN     1.00
HNTR_TAIL_COMP     0.00

HNTR_MASS          4.50
HNTR_L1            0.33
HNTR_L2            0.664
HNTR_MAX_ARM_T   170.96
HNTR_MAX_TAIL_T   85.48
HNTR_MOT_HOV       0.50
HNTR_MOT_EXPO      0.50
```

### 起飞、锁定和倾转配置

```text
HNTR_CTRL_MODE     0
HNTR_TO_SUP_T      2.0 s
HNTR_TO_LOCK_T     5.0 s
HNTR_TO_TILT       8.0 deg
HNTR_LOCK_TILT    12.0 deg
HNTR_LOCK_ACC      1.0
HNTR_LOCK_KP       0.4
HNTR_TILT_MAX    185.0 deg

CA_SV_TL0_MINA  -185 deg
CA_SV_TL0_MAXA  +185 deg
CA_SV_TL1_MINA  -185 deg
CA_SV_TL1_MAXA  +185 deg
CA_SV_TL2_MINA  -180 deg
CA_SV_TL2_MAXA  +180 deg
CA_SV_TL3_MINA  -180 deg
CA_SV_TL3_MAXA  +180 deg
```

这里明确保留一级倾转 `±185°`、二级倾转 `±180°` 的配置。本次日志中舵机只运动
到约 `17–27°`，所以 OK 表示正常飞行区间验证通过，不表示机械全角度端点已实飞验证。

## 执行器和遥控映射

### MAIN 输出

| MAIN 通道 | 功能 | PWM 范围 |
|---:|---|---:|
| 1 | Motor 1 (`101`) | `1000–2000` |
| 2 | Motor 2 (`102`) | `1000–2000` |
| 3 | Motor 3 (`103`) | `1000–2000` |
| 4 | Motor 4 (`104`) | `1000–2000` |
| 5 | Motor 5 / 尾桨 (`105`) | `900–2000` |
| 8 | Servo 1 (`201`) | `800–2200` |
| 9 | Servo 2 (`202`) | `800–2200` |
| 10 | Servo 3 (`203`) | `800–2200` |
| 11 | Servo 4 (`204`) | `800–2200` |

`PWM_MAIN_REV=1664` 也属于本机输出方向配置，恢复时不能遗漏。

### RC 和模式

```text
RC_MAP_ROLL      1
RC_MAP_PITCH     2
RC_MAP_THROTTLE  3
RC_MAP_YAW       4
RC_MAP_FLTMODE   5
RC_MAP_AUX1     15
RC_MAP_AUX2     16
RC_MAP_AUX3      0
COM_RC_IN_MODE   5

COM_FLTMODE1     8
COM_FLTMODE2    -1
COM_FLTMODE3    -1
COM_FLTMODE4     2
COM_FLTMODE5    -1
COM_FLTMODE6     7
```

这版固件虽然保留 AUX1/AUX2 的 RC 映射，但没有后续版本的 AUX 姿态控制代码。

### 状态估计与着陆

```text
EKF2_EV_CTRL       11
EKF2_HGT_REF        3
SENS_BOARD_ROT      0
LNDMC_TRIG_TIME     0.5 s
COM_DISARM_LAND     1.0 s
```

## 恢复和核对

1. 使用 `git checkout hnuter-ok-144bd9fe` 恢复实飞源码；该操作会进入 detached
   HEAD，若要继续开发应从标签新建分支。
2. 为 CUAV 7 Nano 构建并烧录 `cuav_7-nano_default`，核对固件内部版本包含
   `g144bd9fe`。
3. 仅在同一架飞机上导入完整参数快照，重启飞控。
4. 核对 `SYS_AUTOSTART=4051`、`CA_AIRFRAME=16`、MAIN 功能/PWM 范围、
   `PWM_MAIN_REV=1664`、倾转角范围和 RC 映射。
5. 拆桨检查五个电机、四个舵机的通道和方向，再进行低风险悬停复验。

## 已知边界

- 该版本不包含 AUX1/AUX2 姿态积分控制。
- 该版本的 Hnuter 着陆检测仍使用不适配的低推力判据；本次没有空中误判。
- 日志存在最大约 `63°` 的偏航瞬态，且 `HNTR_TAU_Y=0.8 Nm` 发生持续限幅。
- OK 是对本次参数组合和已完成飞行范围的认可，不应理解为所有后续修复都已包含。
