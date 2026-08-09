# HNUTER log 134 尾部下垂与 Pitch 重力配平修正（2026-08-09）

## 1. 版本与证据边界

- 修改基线：`a2145202f2c40b1defa2866fbd3bc51dedbced9e`。
- 实飞日志：`log_134_2026-8-8-22-55-02.ulg`。
- 日志内嵌固件：`a2145202f2c40b1defa2866fbd3bc51dedbced9e`，CUAV 7 Nano。
- 本次结论由 ULog 命令、保存参数和源码公式共同得到。飞机没有 Motor5 RPM/推力
  反馈，因此日志可以证明“飞控命令不足”，不能直接测出实际尾推曲线。

## 2. 直接证据

`log_132` 与 `log_134` 的实际 Pitch 参数相同：

```text
HNTR_ATT_KR_P    5.0
HNTR_ATT_D_P     2.5
HNTR_ATT_I_P     0.0
HNTR_ATT_ILIM_P  0.3
HNTR_TAU_P       8.0
HNTR_PITCH_BIAS  0.09
HNTR_RC_RATE_P   8.0
```

`log_134` 的 20--59 s 区间中，实际 Pitch 平均约 `9.04 deg`；姿态初始化阶段约
`1.27 deg`。同时 Pitch 归一化力矩平均约 `0.104`，Motor5 指令约 `0.300`，
MAIN5 PWM 约 `1615 us`，没有换向中立等待，也没有输出到正向极限。

按模型最大尾力矩 `85.48 N * 0.664 m = 56.76 Nm`，固定偏置提供约
`5.11 Nm`。约 `7.8 deg` 误差在 `KR_P=5` 下只增加约 `0.68 Nm`，归一化约
`0.012`，所以总指令约 `0.102`，与日志 `0.104` 一致。控制器并未要求尾电机使用
全部能力；`I_P=0` 使模型误差保留为稳态姿态误差。

`log_133` 中错误的总推力前馈让 Motor5 达到约 `0.379`、PWM `1660 us`，并使飞机
快速倾覆。它不能作为正确控制，但证明当前问题不能优先归因于尾电机已经失去全部
控制能力。

## 3. 控制修改

### 3.1 姿态相关的重力力矩前馈

控制器将世界系重力转到机体系：

```text
F_g_body = R_transpose * [0, 0, mass * g]
tau_gravity_pitch = CG_Z * F_g_body_X - CG_X * F_g_body_Z
```

该 Pitch 前馈只依赖姿态、质量和一级倾转轴到重心的几何关系，不读取油门或总推力。
水平姿态时 `CG_X` 为主要项；接近正负 90 deg Pitch 时，配平自然转为较小的
`CG_Z` 项。它与 P/D/I 和剩余偏置相加后统一受 `HNTR_TAU_P` 最终限幅。

### 3.2 剩余偏置与有限积分

实机默认值改为：

```text
HNTR_CG_X        0.105 m
HNTR_CG_Z       -0.013 m
HNTR_ATT_KR_P    5.0
HNTR_ATT_D_P     2.5
HNTR_ATT_I_P     0.01
HNTR_ATT_ILIM_P  0.5 Nm
HNTR_TAU_P       8.0 Nm
HNTR_PITCH_BIAS  0.02
```

水平时重力项约为 `4.64 Nm`，归一化约 `0.082`；剩余偏置 `0.02` 再提供约
`1.14 Nm`，合计约 `5.77 Nm`，接近 log 134 的稳定需求。积分仅消除剩余的小误差，
并受 `0.5 Nm` 限制，不能替代主配平或越过总力矩限幅。

### 3.3 位置环与起飞渐变

撤销上一版未经要求的弱位置参数，恢复 log 132 实际组合：

```text
HNTR_POS_P_XY   3.75
HNTR_VEL_P_XY  9.05
HNTR_VEL_I_XY  0.20
HNTR_ACC_XY     3.0
HNTR_POS_P_Z    3.5
HNTR_VEL_P_Z    4.0
HNTR_ACC_Z      45.6
```

保留 `HNTR_TO_RAMP_T=4 s`，锁定退出期间仍平滑释放 XY 增益、加速度和倾转限制，
并在渐变开始时只清除一次 XY 积分状态。没有自动参数迁移。

## 4. 日志修复

`hnuter_control_status` 从可选主题改为默认订阅主题，避免 Logger 在首次发布前启动时
将其排除。消息新增：

```text
torque_gravity[3]
torque_bias[3]
```

后续日志可直接区分目标姿态、P/D/I、重力前馈、剩余偏置和最终 Pitch 力矩，无需再
从 PWM 反推内部目标。

## 5. 烧录与验证边界

`param set-default` 不覆盖飞控中已经保存的数值。烧录后必须手动写入并保存第 3.2、
3.3 节参数，然后重新读取确认。不得恢复 `HNTR_TAIL_COMP`。

首次测试必须拆桨确认水平、抬头和低头时 `torque_gravity[1]` 的方向及连续性；随后
系留执行正负 10/20/30 deg，确认松杆稳态误差、Motor5 PWM、Pitch 总力矩限幅和
温度正常后再扩大姿态。编译或 SITL 通过不等于实机配平已验证。
