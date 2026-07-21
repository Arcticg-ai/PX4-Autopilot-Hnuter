# Hnuter 完整控制与控制分配架构

日期：2026-07-19

本文描述当前工作区中的真实调用关系，覆盖 PX4 模式入口、Hnuter 级联控制器、
几何姿态控制、动态非线性控制分配、执行器映射和残差反馈。它可以直接作为控制
架构图的文字输入。

## 1. 总体边界

Hnuter 没有独立的 PX4 可执行模块。它嵌入以下两个现有模块：

1. `mc_rate_control/HnuterControl`：生成归一化三维推力和三维力矩。
2. `control_allocator/ActuatorEffectivenessHnuter`：将六维控制量非线性分配给
   5 个电机和 4 个倾转舵机。

启用条件是：

```text
CA_AIRFRAME == 16
AND flag_control_rates_enabled == true
AND 至少满足以下之一：
    position / velocity / altitude / climb-rate / offboard 已启用
    或当前为 Hnuter 自定义 Stabilized 模式
```

满足条件后，`MulticopterRateControl` 调用 `HnuterControl::update()`，发布 Hnuter
输出并提前返回，不再执行该周期后面的普通 PX4 multicopter rate-control 主路径。

## 2. 坐标系和信号约定

### 2.1 坐标系

- 世界坐标：PX4 NED，`X=North`、`Y=East`、`Z=Down`。
- 机体坐标：PX4 FRD，`X=Forward`、`Y=Right`、`Z=Down`。
- `R`：机体坐标到世界坐标的旋转矩阵。
- `R^T`：世界坐标到机体坐标的旋转矩阵。
- 重力向量：`g_e3 = [0, 0, +9.81]`，因为 NED 的 Z 向下。

### 2.2 六维控制话题

`HnuterControl` 输出：

```text
vehicle_thrust_setpoint.xyz = [uFx, uFy, uFz]
vehicle_torque_setpoint.xyz = [uTx, uTy, uTz]
```

Control Allocator 组合顺序：

```text
control_sp = [uTx, uTy, uTz, uFx, uFy, uFz]
```

这些是归一化量。Hnuter allocator 转换成自定义内部物理 wrench：

```text
W = [Fx, Fy, Fz, Tx, Ty, Tz]

Fx =  uFx * T_arm_max
Fy = -uFy * T_arm_max
Fz = -uFz * (2 * T_arm_max), 保留正负号
Tx =  roll_sign * uTx * T_arm_max * l1
Ty =  tail_sign * uTy * T_tail_max * l2
Tz = -uTz * T_arm_max * l1
```

这里的 `W` 使用 Hnuter 解析逆解内部符号。不要在图中把它与 PX4 FRD topic 的
符号直接画成完全相同。

## 3. 外部输入与最终输出

### 3.1 HnuterControl 输入

| uORB 输入 | 使用内容 | 作用 |
|---|---|---|
| `vehicle_control_mode` | armed、manual、position、velocity、attitude、rates、offboard 等标志 | 模式路由 |
| `vehicle_status` | `nav_state` | 识别自定义 Stabilized |
| `vehicle_land_detected` | `landed`、`maybe_landed` | 地面清积分、起飞保护 |
| `vehicle_odometry` | NED position、velocity | 位置/速度反馈 |
| `vehicle_attitude` | quaternion | 当前 `R` 和几何姿态误差 |
| `vehicle_angular_velocity` | body rates、角加速度 | 姿态阻尼和可选 RateControl |
| `vehicle_local_position` | `ax/ay/az` | 速度环 D 项 |
| `trajectory_setpoint` | position、velocity、acceleration、yaw、yawspeed | Position/Offboard 平移参考 |
| `vehicle_attitude_setpoint` | `q_d`、yaw move rate | 非平移模式姿态参考，或平移模式 yaw 后备 |
| `vehicle_rates_setpoint` | roll/pitch/yaw rate | 非平移模式可选 PX4 RateControl |
| `manual_control_setpoint` | throttle、yaw、AUX1/2/3 | 定高、姿态角速度积分、渐进回平 |
| `control_allocator_status` | 六维未分配残差、achieved 标志 | 三类积分器抗饱和 |

### 3.2 最终输出

| 输出 | 范围 | 物理对象 |
|---|---:|---|
| `actuator_motors[0]` | `[0,1]` | Motor1，右上主电机 |
| `actuator_motors[1]` | `[0,1]` | Motor2，右下主电机 |
| `actuator_motors[2]` | `[0,1]` | Motor3，左上主电机 |
| `actuator_motors[3]` | `[0,1]` | Motor4，左下主电机 |
| `actuator_motors[4]` | `[-1,1]` | Motor5，尾部可逆 pitch 电机 |
| `actuator_servos[0]` | `[-1,1]` | Servo1，右臂一级 `alpha2` |
| `actuator_servos[1]` | `[-1,1]` | Servo2，左臂一级 `alpha1` |
| `actuator_servos[2]` | `[-1,1]` | Servo3，右臂二级 `theta2` |
| `actuator_servos[3]` | `[-1,1]` | Servo4，左臂二级 `theta1` |
| `control_allocator_status` | 六维归一化残差 | 返回上层抗积分饱和 |

7 Nano 物理 PWM 映射：

```text
MAIN1..5  <- Motor1..5
MAIN8..11 <- Servo1..4
```

Motor1--4 为单向 ESC。Motor5 设置 `CA_R_REV=16`，归一化 0 映射到
`(PWM_MAIN_MIN5 + PWM_MAIN_MAX5)/2 = 1450 us`。

## 4. 当前模式路由

| 模式/输入 | 平移控制 | 姿态参考 | 力矩控制 | 重要说明 |
|---|---|---|---|---|
| Position | Hnuter 位置 P + 速度 PID | 默认 roll/pitch=0；手动时 AUX1/2 可积分设角，yaw 来自 trajectory/RC | Hnuter 几何姿态 | 完整世界力矢量，允许倾斜状态保持世界位置 |
| Velocity | Hnuter 速度 PID | 与 Position 相同 | Hnuter 几何姿态 | 无效 position 分量不参与外环 |
| Altitude/Climb | 有效 Z 分量进入 Hnuter 平移环 | 手动平移模式可用 AUX 姿态参考 | Hnuter 几何姿态 | XY 是否受控由 control-mode 和 trajectory 有效字段决定 |
| Stabilized | 只使用 Hnuter 高度目标，不做 XY 位置控制 | 使用上游 `vehicle_attitude_setpoint.q_d` | Hnuter 几何姿态 | throttle 改变高度目标；机体系 Fx/Fy 被清零，大倾角会水平漂移 |
| Offboard position/velocity/acceleration | Hnuter 级联平移环 | roll/pitch 默认 0，yaw 使用 Offboard trajectory yaw | Hnuter 几何姿态 | 同时发送 q_d 时，平移激活期间 q_d 的 roll/pitch 不进入 Hnuter |
| Offboard attitude only | 无位置闭环；仍生成重力悬停力 | 完整 `q_d` | 默认 `HNTR_CTRL_MODE=0` 时，上游 mc_att_control 生成 rate setpoint，再由 PX4 RateControl 覆盖几何力矩 | 不保证 XY 位置保持；当前还要求 trajectory topic 已存在 |
| Offboard body-rate only | 当前不是可靠链路 | rate topic | rate setpoint 实际不会被采用 | attitude flag 为 false，当前 Hnuter `use_rates_sp` 条件不成立，需修复后才能使用 |
| Acro/manual rates | 绕过 HnuterControl | 普通 PX4 rate setpoint | 普通 PX4 RateControl | 仍进入 Hnuter allocator，但不是专用 Hnuter 平移控制 |

注意：当前 `HNTR_CTRL_MODE=0` 不是全局“关闭几何控制器”。Position、Velocity、
Altitude 和自定义 Stabilized 均使用 Hnuter 几何姿态控制。该参数只在非平移、
非自定义 Stabilized 且存在新鲜 `vehicle_rates_setpoint` 时允许 PX4 RateControl
覆盖三轴力矩，并且当前代码还要求 `flag_control_attitude_enabled=true`。

当前 `HnuterControl::update()` 的入口守卫还要求收到过 `trajectory_setpoint`，只有
自定义 Stabilized 例外。因此纯 Offboard attitude 启动时也应发布一个字段为 NaN 的
有效 trajectory 消息，或者先修复该入口守卫；否则控制器会重置并输出零推力/零力矩。

## 5. Hnuter 级联平移控制器

### 5.1 位置外环

对每个有效位置分量：

```text
e_p = p_sp - p
v_sp = v_ff + Kp_pos * e_p
```

然后执行：

- XY 合速度限制 `HNTR_VEL_XY`；
- 上升速度限制 `HNTR_VEL_UP`；
- 下降速度限制 `HNTR_VEL_DN`。

### 5.2 速度内环

```text
e_v = v_sp - v
a_unsat = a_ff + Kp_vel * e_v + I_vel - Kd_vel .* a_measured
a_des = limit(a_unsat, HNTR_ACC_XY, HNTR_ACC_Z, physical_thrust_limit)
```

积分限制：

```text
|I_vel_xy| <= HNTR_VEL_ILIM_XY
|I_vel_z|  <= HNTR_VEL_ILIM_Z
```

### 5.3 世界期望力到机体期望力

```text
f_world = mass * (a_des - [0, 0, g])
f_body  = R^T * f_world
```

Position/Velocity 模式保留完整 `f_body`，因此机体倾斜时倾转机构仍可把合力保持在
世界竖直方向并产生位置修正。

Stabilized 和地面阶段执行：

```text
f_body.x = 0
f_body.y = 0
```

所以 Stabilized 是“高度 + 姿态”控制，不是倾斜位置悬停。大姿态时它允许水平移动。

### 5.4 起飞保护

- 只有 `landed=false && maybe_landed=false` 后才开始离地计时。
- 锁存离地瞬间 XY 位置。
- `HNTR_TO_SUP_T` 内将水平控制权限从 0 线性恢复到 1。
- `HNTR_TO_LOCK_T` 内使用受限 XY 增益、加速度和倾转角。
- 水平权限完全恢复前冻结 XY 速度积分。

### 5.5 归一化推力输出

```text
uFx = clamp(f_body.x / T_arm_max, -1, 1)
uFy = clamp(f_body.y / T_arm_max, -1, 1)
uFz = clamp(f_body.z / (2*T_arm_max), -1, 1)
```

`uFz` 必须保留正负号。超过 90 度时机体系重力补偿分量会反号，主电机仍通过
倾转到相反方向产生正推力，不需要主电机反转。

## 6. 姿态参考生成

### 6.1 Position 手动 AUX 姿态

- AUX1：roll 期望角速度。
- AUX2：pitch 期望角速度。
- 通道回中：锁存当前已经到达的连续姿态角。
- AUX3 上升沿：以受限速度渐进回到 roll=0、pitch=0。
- yaw 摇杆：积分生成 yaw 目标；回中保持当前 yaw。
- `HNTR_RC_ANG_MAX`：roll/pitch 最终角度限幅。

参考变化率同步到倾转机构：

```text
rate_limit = min(requested_rate,
                 0.8 * actuator_rate,
                 HNTR_SYNC_ERR / (delay + tau))
```

roll 使用二级 T2 动态，pitch 使用一级 T1 动态。

### 6.2 超过 90 度的连续姿态

控制器在以下两个等价欧拉表示中选择最接近上一周期命令的一个：

```text
(roll, pitch, yaw)
(roll + pi, sign(pitch)*pi - pitch, yaw + pi)
```

因此 AUX 回中不会把 110 度 pitch 折返成 70 度 pitch 加 180 度 roll/yaw。

## 7. Hnuter 几何姿态控制器

输入：`R`、`R_des`、机体系角速度 `omega`、参考姿态变化率。

姿态误差：

```text
e_R_matrix = 0.5 * (R_des^T R - R^T R_des)
e_R = vee(e_R_matrix)
```

角速度误差：

```text
e_omega = omega - R^T R_des * omega_des
```

物理力矩命令：

```text
tau = -KR .* e_R - Domega .* e_omega - KI .* integral(e_R)
tau = clamp(tau, [-HNTR_TAU_R, -HNTR_TAU_P, -HNTR_TAU_Y],
                 [+HNTR_TAU_R, +HNTR_TAU_P, +HNTR_TAU_Y])
```

归一化：

```text
uTx =  tau.x / (T_arm_max * l1)
uTy = -tau.y / (T_tail_max * l2) + HNTR_PITCH_BIAS
uTz =  tau.z / (T_arm_max * l1)
```

`HNTR_PITCH_BIAS` 是尾电机粗配平；姿态积分用于消除剩余静差。当前 7 Nano
airframe 的三轴几何积分默认均为 0，需要实机确认后在线启用。

## 8. 可选 PX4 RateControl 支路

仅在以下条件同时成立时覆盖上述三轴几何力矩：

```text
HNTR_CTRL_MODE == 0
AND attitude control enabled
AND vehicle_rates_setpoint fresh
AND Hnuter translation control inactive
AND not custom Stabilized
```

RateControl 输入：`rates`、`rates_sp`、角加速度和 allocator 饱和方向；输出归一化
roll/pitch/yaw torque。pitch 在进入 Hnuter allocator 前做一次符号转换。

这意味着默认 Offboard attitude 链为：

```text
q_d -> PX4 mc_att_control -> vehicle_rates_setpoint
    -> Hnuter 内调用 PX4 RateControl -> normalized torque
```

纯 Offboard body-rate 通常关闭 attitude flag，因而不能满足上述条件。当前实现会继续
走 Hnuter 的后备姿态目标，而不是执行所给 body rates；架构图必须将该路径标成未支持。

## 9. Hnuter 非线性解析控制分配

### 9.1 六维 wrench 分解

定义：

```text
r_x = 0.105 m
r_z = -0.013 m
l1  = HNTR_L1
l2  = HNTR_L2
```

先求左右臂目标分量和尾电机力：

```text
u1 = Fx/2 - Tz/(2*l1)
u4 = Fx/2 + Tz/(2*l1)

Ty_parasitic = r_z*Fx - r_x*Fz
F3 = (Ty - HNTR_TAIL_COMP*Ty_parasitic) / (r_x + l2)
Fz_front = Fz - F3

Tx_parasitic = -r_z*Fy
Tx_comp = Tx - Tx_parasitic
u2 = Fz_front/2 + Tx_comp/(2*l1)
u5 = Fz_front/2 - Tx_comp/(2*l1)

u3 = -Fy/2
u6 = -Fy/2
```

左右臂三维目标向量：

```text
v_left  = [u1, u2, u3]
v_right = [u4, u5, u6]
```

物理作用关系：

- `Fx`：主要由左右一级倾转共同分量产生。
- `Fy`：主要由左右二级倾转共同分量产生。
- `Tx`：主要由左右臂垂直推力差产生。
- `Ty`：主要由可逆 Motor5 产生；前四电机同步补偿其附加垂直力。
- `Tz`：主要由左右一级前向分量差产生。
- 实机 `HNTR_TAIL_COMP` 默认 0，Motor5 不随 collective throttle 变化。
- SITL `HNTR_TAIL_COMP` 默认 1，用于补偿 Gazebo 质心模型的寄生 pitch moment。

### 9.2 目标关节角

```text
F_left  = norm(v_left)
F_right = norm(v_right)

alpha1 = atan2(u1, u2)
alpha2 = atan2(u4, u5)
theta1 = asin(u3/F_left)
theta2 = asin(u6/F_right)
```

allocator 根据上一周期估计角选择连续等价云台支路，并限制在物理可达范围：

```text
reachable ~= min(configured_angle_limit,
                 GAIN * servo_command_limit - abs(ZERO))
```

当前实机二级 `GAIN=0.71`，所以命令端 `+-180 deg` 不代表物理关节实际能达到
`+-180 deg`。解析分配会利用一级 `+-185 deg` 与二级至少 `+-90 deg` 的等价支路
覆盖完整推力方向球面。

## 10. 舵机逆模型和倾转状态估计

每个目标物理角先反算等效舵机命令：

```text
q_command = (q_desired - q_zero) / K
servo_sp = angle_to_normalized_servo(q_command)
```

四个关节均经过开环动态估计器：

```text
q_target(t) = q_zero + K*q_command(t-L)
tau*d(q_hat)/dt + q_hat = q_target
|d(q_hat)/dt| <= rate_max
```

离散实现：

```text
delta_lag = (q_target-q_hat)*(1-exp(-dt/tau))
delta = clamp(delta_lag, -rate_max*dt, +rate_max*dt)
q_hat(k+1) = q_hat(k)+delta
```

参数：`HNTR_T1/T2_GAIN`、`ZERO`、`DLY`、`TAU`、`RATE`。

重要边界：`q_hat` 是命令驱动的开环估计角，不是编码器或动捕实际关节角。

## 11. 当前可达方向下的电机推力投影

估计倾角对应的单位方向：

```text
d(alpha,theta) = [cos(theta)*sin(alpha),
                  cos(theta)*cos(alpha),
                  sin(theta)]
```

由于舵机有延迟，当前只能沿 `d_hat` 产生力。每臂电机推力取目标向量在当前可达
方向上的最小二乘投影：

```text
F_alloc = clamp(v_desired dot d_hat, 0, T_arm_max)
```

然后：

```text
Motor1 = Motor2 = right_arm_force/2
Motor3 = Motor4 = left_arm_force/2
Motor5 = clamp(F3, -T_tail_max, +T_tail_max)
```

实机主电机 PWM 模型：

```text
u_motor = HNTR_MOT_HOV * (F_motor/F_hover)^HNTR_MOT_EXPO
F_hover = mass*g/4
```

Motor5 使用带符号幂函数。SITL 使用 Gazebo 电机常数和转速平方模型。

## 12. 六维实际 wrench 反算与残差闭环

使用 `F_alloc*d_hat` 和 Motor5 实际分配力反算：

```text
W_real = [Fx_real, Fy_real, Fz_real, Tx_real, Ty_real, Tz_real]
residual = normalize(W_requested - W_real)
```

输出到：

```text
control_allocator_status.unallocated_thrust[0..2]
control_allocator_status.unallocated_torque[0..2]
```

下一控制周期反馈到三个位置：

1. 速度积分器：将 body force residual 转到 world acceleration，阻止继续向不可实现
   方向积分。
2. 几何姿态积分器：按未分配物理力矩方向冻结会加重饱和的积分步骤。
3. PX4 RateControl：转换成正/负 saturation flags，限制 rate integral。

该通道有一个控制周期延迟，不是代数环。`achieved` 的当前阈值是归一化残差向量
范数小于约 `0.001`。

## 13. Hnuter 着陆检测与安全状态反馈

普通 PX4 land detector 使用 `-vehicle_thrust_setpoint.z` 判断低油门，但 Hnuter 在
倾转状态下该量不等于真实主电机油门。当前 Hnuter 路径改为读取分配后的：

```text
mean_motor = mean(actuator_motors[0..3])
ground_contact_threshold = HNTR_MOT_HOV * HNTR_LND_GC_R
```

`actuator_motors`、下降命令、水平/垂直运动和离地高度共同进入 PX4
`MulticopterLandDetector`，输出 `landed/maybe_landed`。该状态反馈到：

- HnuterControl：清积分、起飞时刻和 XY 锁定；
- Hnuter allocator：地面水平 wrench 抑制和倾转权限斜坡；
- Commander：着陆后自动上锁。

大姿态不是故障姿态：Hnuter airframe 将 `FD_FAIL_R/P` 默认设置为 `180 deg`。

## 14. 当前实现边界

1. Hnuter 内部 `R_des`、`e_R`、速度积分和四个 `q_hat` 没有专用 uORB 状态话题；
   日志中的 `vehicle_attitude_setpoint` 是上游标准 PX4 参考，不一定是 Hnuter 最终参考。
2. 倾转角是开环模型估计，没有编码器或动捕在线校正，不能证明实机舵机实际到位。
3. T1/T2 左右机构分别共用一组动态参数，不能表示左右零位、齿隙和负载差异。
4. `r_x=0.105 m`、`r_z=-0.013 m` 仍硬编码在 allocator；`l1/l2` 才是参数。
5. 实机 PWM-推力模型是悬停点锚定近似，不是台架测得的完整电机曲线。
6. Hnuter 在 `mc_rate_control` 中提前返回，因此普通路径后面的 battery scale 不会
   再应用到 Hnuter thrust/torque；当前没有独立的 Hnuter 电池电压前馈。
7. 纯 Offboard body-rate 当前不被采用；Offboard attitude-only 还依赖 trajectory
   topic 已经存在。

## 15. 建议架构图节点和连线

按从左到右绘制以下主链：

```text
[RC / QGC / ROS2 Offboard]
 -> [PX4 mode + FlightTask reference generation]
 -> [trajectory_setpoint / attitude_setpoint / rates_setpoint]
 -> [Hnuter mode router]
 -> [Position P]
 -> [Velocity PID + limits + anti-windup]
 -> [World desired acceleration]
 -> [Gravity compensation and mass]
 -> [R^T world-to-body force transform]
 -> [Normalized 3D thrust setpoint]

[Attitude reference generator: level / q_d / AUX integration]
 -> [Continuous Euler branch and actuator-synchronized slew]
 -> [SO(3) geometric attitude controller]
 -> [Normalized 3D torque setpoint]

[3D thrust + 3D torque]
 -> [Hnuter analytic 6D inverse allocation]
 -> [Continuous equivalent gimbal branch + reachable-angle limits]
 -> [Servo inverse static model]
 -> [T1/T2 delay-lag-rate observer]
 -> [Force projection on estimated direction]
 -> [Motor thrust model]
 -> [5 motor commands + 4 servo commands]
 -> [PWM output / aircraft plant]
 -> [EKF odometry, attitude, rates]
 -> feedback to position, velocity and attitude controllers
```

用三条虚线反馈：

```text
[Estimated realized 6D wrench]
 -> [control_allocator_status residual]
 -> [Velocity integral anti-windup]
 -> [Geometric attitude integral anti-windup]
 -> [PX4 RateControl saturation feedback]
```

另画一条模型不确定性虚线：

```text
[Physical tilt joints] -- no encoder feedback --> [Open-loop tilt observer]
```

这条线应标注“当前缺少实际关节角闭环；动捕 tilt1/tilt2 仅用于离线辨识”。

## 16. 可直接交给绘图模型的提示词

```text
请根据以下系统定义绘制一张论文风格、从左到右的 Hnuter 双级倾转无人机完整控制
架构图。使用矩形表示控制模块，圆角矩形表示状态/参考生成器，平行四边形表示 uORB
输入输出，实线表示前向信号，虚线表示反馈和估计。分别用不同颜色区分：参考生成、
Hnuter 飞行控制、非线性控制分配、执行器动态模型、物理对象、残差抗饱和。

系统主链包含：RC/QGC/ROS2 Offboard；PX4 mode/FlightTask；trajectory_setpoint、
vehicle_attitude_setpoint、vehicle_rates_setpoint；Hnuter mode router；位置 P 外环；速度
PID 内环；加速度/速度/积分限幅；重力补偿；NED 到 FRD 的 R^T 力变换；SO(3) 几何
姿态控制器；归一化 3D thrust 与 3D torque；Hnuter 解析 6D inverse allocation；连续
等价云台支路和物理可达角限制；T1/T2 舵机逆静态模型；纯延迟 + 一阶惯性 + 速度限制
的开环倾角观测器；按估计倾角进行推力方向投影；实机 PWM-推力模型；Motor1-5 与
Servo1-4；飞机动力学；EKF position/velocity/attitude/rates 反馈。

平移控制公式：e_p=p_sp-p；v_sp=v_ff+Kp_pos e_p；e_v=v_sp-v；
a_des=a_ff+Kp_vel e_v+I_vel-Kd_vel a_measured；f_world=m(a_des-g e3)；
f_body=R^T f_world。姿态控制公式：e_R=vee(0.5(Rd^T R-R^T Rd))；
e_omega=omega-R^T Rd omega_d；tau=-KR e_R-Domega e_omega-KI integral(e_R)。

控制分配输入顺序为 [Tx,Ty,Tz,Fx,Fy,Fz]。左右臂目标力经过 alpha/theta 解析逆解，
舵机状态模型为 q_target=q_zero+K q_command(t-L)，tau_q qdot+q=q_target，并有限速。
电机力为目标臂力在当前估计方向 d(alpha,theta) 上的点积投影。Motor1/2 属于右臂，
Motor3/4 属于左臂，Motor5 是可逆 pitch 尾电机；Servo1/2 是左右一级 alpha，
Servo3/4 是左右二级 theta。

从 estimated realized 6D wrench 到 control_allocator_status 画一条残差反馈，再分别连到
速度积分、几何姿态积分和 PX4 RateControl 积分抗饱和。明确标注该反馈延迟一个控制
周期。另画注释：倾转观测器目前没有编码器反馈，实际关节角误差和齿隙不在闭环内。

增加着陆检测支路：actuator_motors[0..3] 平均值、下降命令和运动状态进入 Hnuter
land detector，输出 landed/maybe_landed，再反馈到控制器积分清零、起飞保护、allocator
地面抑制和 Commander 自动上锁。

在图下方增加模式路由说明：Position/Velocity/Altitude 使用 Hnuter 级联平移控制和
SO(3) 几何姿态控制；Stabilized 只控制高度和姿态并清零 body Fx/Fy，不保持 XY；
Offboard position 忽略 q_d 的 roll/pitch；默认 Offboard attitude 使用完整 q_d，经过
mc_att_control 和 PX4 RateControl；纯 Offboard body-rate 当前因 attitude flag 条件而
未被 Hnuter 采用，应画成待修复路径。Attitude-only 还依赖 trajectory topic 已存在。
```
