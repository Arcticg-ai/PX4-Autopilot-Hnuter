# Hnuter 姿态解耦、尾电机保护与分配残差更新（2026-08-13）

## 版本范围

- 分支：`codex/hnuter-pitch-tail-safety-20260808`
- 基线：`19cacf18`（log 134 Pitch 配平与实飞参数修正版）
- 本次范围：讨论清单第 `9、13、14、15、16、17、18、21` 项。
- 原则：实机和 Gazebo 继续使用独立的 airframe 参数、执行器映射和动态保护尺度；
  未将实机台架数据直接覆盖到仿真物理模型。

## 修改总表

| 编号 | 修改内容 | 本次实现 |
| ---: | --- | --- |
| 9 | 尾电机按换向幅值动态保护 | 增加 `TRACKING/RAMP_DOWN/DWELL/RAMP_UP` 状态机；先卸载至零、小幅换向短等待、大幅换向逐渐接近最长等待，并限制同向大阶跃。 |
| 13 | 用分配残差做抗积分饱和 | 从受保护后的尾推、推力限幅、舵机角度与舵机速率重建六维可实现力/力矩；姿态 Pitch I 和位置速度 I 不再向不可实现方向继续累积。 |
| 14 | AUX1/AUX2/Yaw 语义解耦 | 手动目标改为“世界航向 + 二维倾转 swing”；AUX1 只改第一倾转分量，AUX2 只改第二倾转分量，Yaw 只改世界航向。 |
| 15 | 分轴松杆锁存 | AUX1 或 AUX2 松杆时只锁存对应的实测倾转分量，不再复制完整实测四元数，避免把偏航误差和另一个姿态轴一起写回目标。 |
| 16 | 降低指令侧偏航耦合 | Roll/Pitch 目标不再通过完整机体系角速度增量间接改变航向；保留独立航向目标。机械不对称、舵机误差和气动力造成的实物耦合仍需日志辨识。 |
| 17 | 固定 Pitch 偏置归零 | 实机默认 `HNTR_PITCH_BIAS=0`，不再用未标定常数掩盖重心、推力或执行器模型误差。姿态相关重力力矩前馈保留。 |
| 18 | 近垂直 Pitch 的饱和治理 | 尾电机受换向/斜率/幅值限制时保持姿态 P/D 反馈；冻结 Pitch I，并阻止继续扩大姿态误差的 AUX2 目标增量。Roll/Yaw 仍可控制。 |
| 21 | 增加可诊断日志 | 新增 `hnuter_allocator_status`，记录尾推请求/受限命令、归一化输出、估算 PWM/RPM、保护状态、等待时间、换向次数、饱和标志和 Pitch 残差；扩展 `hnuter_control_status` 记录残差、目标与抗积分状态。 |

## 1. 尾电机动态保护

尾电机不再允许一个控制周期内从较大正推直接跳到较大反推。控制过程为：

```text
原方向跟踪 -> 限斜率卸载到零 -> 中立等待 -> 限斜率建立反向推力
```

中立等待时间按换向前后较大推力的幅值进行二次插值：低负载接近
`HNTR_T_REV_MIN`，达到动态参考推力时接近 `HNTR_TAIL_REV_T`。动态强度和斜率使用
独立参数 `HNTR_T_FORCE_REF`，不直接使用仍待辨识的 `HNTR_MAX_TAIL_T`。这样实机
约 10 N 量级的高负载换向不会因为理论最大推力为 85.48 N 而被误判为小幅换向。

新增参数：

| 参数 | 含义 |
| --- | --- |
| `HNTR_T_REV_MIN` | 小幅换向最短中立时间，s |
| `HNTR_T_FORCE_REF` | 换向强度和斜率的动态参考推力，N；不改变静态分配模型 |
| `HNTR_T_SLEW_UP` | 普通增推斜率，倍动态参考推力/s |
| `HNTR_T_SLEW_DN` | 卸载/减推斜率，倍动态参考推力/s |
| `HNTR_T_REV_SLEW` | 换向等待后的反向建立斜率，倍动态参考推力/s |
| `HNTR_T_RPM_MAX` | 满归一化输出对应的估算转速，仅用于日志 |

## 2. 分配残差与抗积分饱和边界

分配器用最终受限命令重建可实现的六维力/力矩，并通过 PX4
`control_allocator_status` 报告 `requested - achieved`：

- Motor5 换向、斜率或幅值限制会产生真实的模型 Pitch 残差；
- 主电机推力限幅、倾转角限制和舵机指令速率限制会进入力/力矩残差；
- Pitch 残差存在时冻结几何姿态 Pitch 积分，但 P/D 反馈保持；
- 位置速度积分只有在本次积分增量会继续推向分配器不可实现方向时才冻结，其他轴
  和反向消饱和仍允许更新。

这里的 `achieved` 仍是“基于指令和模型的可实现值”，不是电机 RPM、舵机角度或
实际推力传感器的闭环测量值。开环机构的带载不到位、回差或左右不同步仍可能导致
实际残差大于日志中的模型残差。

## 3. 手动大姿态目标表示

旧逻辑在任一 AUX 通道松杆时复制完整实测四元数，实测偏航误差和另一个姿态轴会
一起进入新目标，形成松杆回弹和指令侧耦合。本次将目标改为：

```text
q_target = q_world_heading * q_two_axis_tilt
```

二维倾转采用 swing 向量表示，不依赖 Pitch 欧拉角，在正负 90 度附近不发生
欧拉奇异。AUX2/Pitch 仍没有绝对 30/45 度限制；`HNTR_RC_ANG_MAX` 只限制
AUX1/Roll。若上一周期 Pitch 分配残差表明尾电机受限，只拒绝会继续增大当前姿态
误差的 Pitch 目标增量，不把已达到的 Pitch 目标自动拉回水平。

## 4. 实机与仿真独立参数

| 参数 | 实机 airframe | Gazebo airframe | 说明 |
| --- | ---: | ---: | --- |
| `HNTR_CG_X/Z` | `0.105/-0.013 m` | `0/0 m` | 当前实机值仍待重新测量；仿真重力施加于模型重心 |
| `HNTR_S2_GEAR` | `2.0` | `1.0` | 实机二级 2:1；Gazebo 直接驱动关节 |
| `HNTR_PITCH_BIAS` | `0` | `0` | 两者都不再使用固定偏置 |
| `HNTR_TAIL_REV_T` | `0.30 s` | `0.05 s` | 实机高负载最长保护；仿真使用自身快速转子模型 |
| `HNTR_T_REV_MIN` | `0.05 s` | `0.01 s` | 小幅换向最短等待 |
| `HNTR_T_FORCE_REF` | `12.8 N` | `85.48 N` | 实机按本次台架量级；仿真按当前模型尺度 |
| `HNTR_T_SLEW_UP/DN` | `5/10` | `30/40` | 实机和仿真独立斜率 |
| `HNTR_T_REV_SLEW` | `4` | `25` | 换向后建立推力斜率 |
| `HNTR_T_RPM_MAX` | `20650 rpm` | `9549 rpm` | 仅用于估算日志，不参与控制 |

`ActuatorEffectivenessHnuter` 继续明确区分两条执行器映射：Gazebo 使用仿真电机常数
和角速度范围，实机使用悬停锚点/推力指数映射。两套 airframe 分别发布默认值，
没有把实机 4006/12 寸三叶/40 A 台架模型写进 Gazebo SDF。

## 5. 新增日志字段

`hnuter_allocator_status` 默认以 50 Hz 记录，包含：

- `tail_force_requested/commanded/error`；
- `tail_output_normalized`、`tail_pwm_estimate`、`tail_rpm_estimate`；
- `tail_state`、`tail_limited`、`reversal_count`；
- `reversal_dwell_required/elapsed`、`pitch_unallocated`；
- `simulation_model`，用于区分日志采用的模型路径。

`hnuter_control_status` 增加分配器 Pitch/世界系力残差、二维倾转目标、航向目标，以及
Pitch I、位置 I、Pitch 目标是否被阻止的标志。

PWM 和 RPM 字段是根据最终输出及参数计算的估算值。实机没有转速反馈，因此不能把
`tail_rpm_estimate` 当作桨叶已停转的证据。

## 6. 本次明确未修改

- `HNTR_MAX_TAIL_T` 仍为 `85.48 N`，正反推仍使用对称静态曲线；这两项需要用台架
  推力而不只是 RPM 重新标定，不能在本次保护逻辑中猜测替换。
- 实机 Motor5 PWM 端点和双向电调中点映射未在本次范围内修改。
- 没有增加舵机角度或尾电机 RPM 闭环；所有残差仍受开环模型精度限制。
- 没有恢复参数迁移逻辑，也没有修改 Position 环的实飞参数组合。

## 7. 烧录与验证要求

本版把 `HNTR_PITCH_BIAS` 的固件默认值改为 `0`，但飞控中已保存的 `0.02` 会覆盖
airframe 默认值。烧录后、上桨前需要执行一次：

```text
param set HNTR_PITCH_BIAS 0
param save
```

同时用 `param show HNTR_T_*` 核对实机保护参数。验证顺序仍应为拆桨输出检查、低负载
台架换向、系留正负 10/20/30 度，再逐步扩大姿态。不得把成功编译或 Gazebo 启动
当作实机安全证明。

## 8. 构建与验证记录

- `git diff --check`：通过。
- PX4 AStyle（四个修改的 C/C++ 文件）：通过。
- `make px4_sitl_default`：通过。
- `HEADLESS=1 make px4_sitl gz_hnuter`：成功加载 `SYS_AUTOSTART=4051`，生成
  `hnuter_0`，启动四路倾转舵机、Logger 与 MAVLink，并到达 `pxh>`；随后人工退出。
- `CCACHE_DIR=/tmp/ccache make cuav_7-nano_default`：通过；Flash
  `1880044 / 1966080 bytes (95.62%)`。
