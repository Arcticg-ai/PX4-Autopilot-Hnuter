# Hnuter 悬停 Y/yaw 耦合与通用 Web 调参修复

日期：2026-07-22

## 1. 故障结论

直接控制悬停数据中，Y 位置误差与 yaw 误差具有相同的 `0.216 Hz` 主频，相关系数
为 `0.587`。这不是单纯的定位噪声，而是两个控制链问题叠加：

1. 直接控制器 XY 环使用 `Kp=2.5, Kd=1.8`，对当前慢速二级倾转机构阻尼不足。
2. 分配器使用本周期期望 `Fy` 立即补偿 `Fy` 产生的偏航力矩，但实际二级倾转尚未
   到达该角度，差动一级倾转会提前产生反向偏航力矩。

## 2. 控制链修改

外部直接控制器 `hnuter_external_direct_controller_debug.py`：

- XY 位置环改为可在线配置的 `direct_pos_kp_xy`、`direct_pos_kd_xy` 和
  `direct_pos_ki_xy`，当前 SITL 配置为 `1.2/2.8/0.15`。
- 有 Gazebo 关节反馈时，偏航前馈使用上一周期实际倾角与推力反算出的已实现 `Fy`；
  无反馈时，使用 `direct_yaw_comp_tau_s` 一阶同步状态。
- yaw 几何环当前使用 `KR=1.8`、`D=1.6`、`KI=0.20`，积分限幅为 `3.0`。
  `D=2.8` 与带纯延迟的一级倾转形成约 `1.2 Hz` 饱和振荡，不能继续使用。

PX4 控制分配 `ActuatorEffectivenessHnuter`：

```text
Fy command -> first-order synchronization -> r_yaw_x * Fy_realizable
                                             |
Tz command ----------------------------------(-)--> primary tilt differential
```

新增 `HNTR_YAW_FF_TC`，只同步 `Fy -> yaw` 前馈，不重新启用整套舵机纯延迟/惯性
估计。`HNTR_TDYN_EN=0` 时仍保持原来的静态几何投影，避免在真实舵机和控制器中重复
加入执行器延迟。

实机默认横向环调整为：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `HNTR_POS_P_XY` | 0.50 | 位置误差生成速度期望，降低慢摆激励 |
| `HNTR_VEL_P_XY` | 2.00 | 增加横向速度阻尼 |
| `HNTR_VEL_I_XY` | 0.08 | 只消除慢速静差，避免积分驱动往复 |
| `HNTR_VEL_D_XY` | 0.15 | 抑制速度变化和执行器滞后造成的摆动 |
| `HNTR_ATT_D_Y` | 1.80 | 增加偏航角速度阻尼 |
| `HNTR_ATT_I_Y` | 0.10 | 消除机械/气动固定偏航偏置 |
| `HNTR_YAW_FF_TC` | 0.15 s | 将偏航前馈与二级倾转响应同步 |

## 3. Web 调参修复

旧 Web 服务把参数名和整数类型写死在 Python 文件中。固件增加参数后，服务端会把
它拒绝为未知参数；类型名单过期时，`PARAM_SET` 编码也可能错误，表现为点击 Apply 后
又恢复旧值。

现在服务启动和点击 **Discover** 时通过 MAVLink `PARAM_REQUEST_LIST` 获取当前飞控的
完整参数目录：

- 参数名、当前值和 INT32/REAL32 类型均由飞控返回；
- 默认显示 `HNTR_`，也可输入 `MPC_`、`CA_` 等任意前缀；
- 已知参数继续显示有范围的滑块，未知新参数自动显示直接数值输入框；
- Apply 必须收到 PX4 同名、同值 `PARAM_VALUE` 回读才显示成功；
- 重连或重新发现会清空旧固件目录，防止换固件后残留失效参数。

运行时必须隔离 DDS：

```bash
export ROS_DOMAIN_ID=43
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
cd ~/px4_ws_ros2
python3 hnuter_attitude_tuning_web.py --host 0.0.0.0 --port 8765
```

## 4. 验证结果

- 外部直接控制 baseline：Y/yaw 共同主频 `0.216 Hz`，yaw P95 `4.02 deg`。
- 修改后直接控制：Y/yaw 相关系数由 `0.587` 降至 `-0.079`，yaw P95 降至
  `2.42 deg`，共同低频峰消失。
- PX4 内部 Position Offboard 50 s 稳态：East/Y 误差标准差 `0.020 m`、P95
  `0.050 m`，yaw P95 `2.10 deg`。
- Web 实测发现 `1108` 个 PX4 参数和 `79` 个 `HNTR_*` 参数；新增浮点参数
  `HNTR_YAW_FF_TC` 与动态发现的整数参数 `HNTR_TDYN_EN` 均一次写入并回读成功；
  `MPC_` 前缀发现也通过。
- `make px4_sitl_default` 通过。
- `make cuav_7-nano_default` 通过；Flash `1873332 / 1966080 B (95.28%)`，AXI SRAM
  `98112 / 524288 B (18.71%)`。PX4 文件 SHA-256：
  `a24f026de80df8f72aa6068d8dec547fa3561067e8fad2de1a0cdc6a1b48f56a`。

## 5. 实机迁移

`set-default` 不会覆盖飞控中已经保存的参数。烧录后先在 QGC 或 Web 中核对上表，
再在测试架上从小幅横向扰动开始。`HNTR_YAW_FF_TC` 推荐从 `0.15 s` 起步：偏航补偿
明显领先二级倾转时增大，快速横移时偏航补偿明显滞后时减小，每次只改 `0.02 s`。

## 6. 2026-07-22 参数误调恢复

一次在线调参将横向环提高到 `HNTR_POS_P_XY=1.75`、`HNTR_VEL_P_XY=5.25`、
`HNTR_VEL_I_XY=0.47`、`HNTR_VEL_D_XY=0.47`，同时将 `HNTR_TAU_R/Y` 提高到
`21.1`，并把 yaw `KR/D` 降到 `0.4/0.5`。该组合造成位置环强烈驱动倾转，而姿态
和偏航校正不足，出现大幅姿态误差、位置发散和数据失效。

SITL 已只恢复以下 11 项，没有重置 pitch bias、舵机零位和其他实机标定：

```text
HNTR_ATT_D_R=2.0       HNTR_ATT_D_Y=1.2
HNTR_ATT_KR_R=4.0      HNTR_ATT_KR_Y=1.2
HNTR_POS_P_XY=0.50
HNTR_TAU_R=0.9         HNTR_TAU_Y=1.8
HNTR_VEL_D_XY=0.20     HNTR_VEL_I_XY=0.05
HNTR_VEL_P_XY=2.20     HNTR_VEL_XY=5.0
```

恢复后 `parameters.bson` 中不再包含覆盖 airframe 默认值的 `HNTR_*` 项。Position
Offboard 复测中，35 s 后 East/Y 误差 P95 为 `0.032 m`，Down 误差 P95 为
`0.018 m`，yaw 误差 P95 为 `1.86 deg`，未再出现大姿态发散。恢复前参数文件备份于
`/tmp/hnuter_parameters_before_restore_20260722_1116.bson`。

## 7. 直接外部控制偏航振荡修复

普通启动此前默认关闭 `HNUTER_GZ_TILT_FEEDBACK`。控制器因此使用实机几何和期望倾角
补偿当前 Gazebo 模型，造成 Y/yaw 同周期摆动。现在默认值改为 `auto`：只有收到
Gazebo 专用的四个 `servo_N_dynamic` 话题后，才启用实际倾角分配和 SITL 几何；实机
即使安装了 Gazebo Python 库，只要没有关节话题，也不会切换模型参数。仍可用
`HNUTER_GZ_TILT_FEEDBACK=0` 明确关闭。

实际关节已经包含一级 `0.108 s` 延迟和 `0.071 s` 一阶惯性。旧控制器又叠加
`0.8 * (angle_command - angle_actual)`，使命令角和实际角在振荡频带呈负相关。该项现
改为在线参数 `direct_tilt_tracking_gain`，默认 `0.0`，不再重复闭合延迟关节。

最终直接控制参数：

| 参数 | 值 |
| --- | ---: |
| `direct_KR[2]` | 1.8 |
| `direct_Domega[2]` | 1.6 |
| `direct_KI[2]` | 0.20 |
| `direct_integral_limit[2]` | 3.0 |
| `direct_takeoff_KR/KI/Domega[2]` | 2.0 / 0.20 / 1.6 |
| `direct_xy_lock_KR/KI/Domega[2]` | 2.0 / 0.20 / 1.6 |
| `direct_pos_kp/kd/ki_xy` | 1.2 / 2.8 / 0.15 |

同一 `gz_hnuter` 悬停工况的 CSV 对比：

| 工况 | yaw 标准差 | yaw P95 | yaw 力矩饱和率 | Y 误差 P95 |
| --- | ---: | ---: | ---: | ---: |
| 无关节反馈、激进增益 | 3.76 deg | 7.12 deg | 35.0% | 0.236 m |
| 自动反馈、关闭重复关节闭环 | 0.86 deg | 5.52 deg | 0% | 0.255 m |
| 最终积分/起飞参数 | 0.80 deg | 1.99 deg | 0% | 0.179 m |

最终测试 45 s 后 yaw 标准差为 `0.53 deg`、P95 为 `0.98 deg`；90 s 后 yaw P95
为 `0.90 deg`，XY 合成误差 P95 为 `0.121 m`。原约 19 秒周期偏航摆动未再出现。
