# Hnuter 级联位置控制与实机推力模型修复（2026-07-13）

## 1. 修复原因

旧 Hnuter 平移控制器将位置误差、速度误差和累计位置误差直接相加为期望加速度，
不是真正的位置外环/速度内环。与此同时，实机分配器沿用了 Gazebo 的
`motorConstant=8.54858e-5` 和 `10~1000 rad/s` 电机模型。该常数只描述 SDF
中的仿真电机，不能描述实机的 4112、460 KV、15 寸桨组合。

这会造成两个现象：

- Position 模式的速度误差经固定增益换算后，主电机常停在约 1400~1500 us；
- `HNTR_HOV_THR` 在控制器和分配器中先正向映射、再反向映射，实际上相互抵消，
  调节它无法可靠改变实机悬停油门。

## 2. 新控制链

Position、Velocity、Altitude、Climb-rate 和 Offboard 平移控制均进入 Hnuter 后端：

```text
trajectory_setpoint
  -> 位置 P 外环（位置误差 -> 速度修正）
  -> 速度设定向量限幅
  -> 速度 PID 内环（速度误差、速度积分、实测加速度阻尼）
  -> 加速度向量/物理推力限幅与条件积分抗饱和
  -> 世界系期望力 -> 机体系期望力
  -> Hnuter 几何姿态控制
  -> Hnuter 非线性控制分配
  -> 电机/两级倾转舵机
```

修复内容：

- XY 速度和加速度使用向量模长限幅，避免对角方向多出 `sqrt(2)` 倍控制量；
- Z 加速度取用户上限与可用推力上限的较小值，修复原先参数越调小越不生效的问题；
- 速度积分只在未落地时工作，并在输出饱和方向采用条件积分抗饱和；
- 加速度 D 项使用 `vehicle_local_position.ax/ay/az`；
- Offboard 本身不再被误判为“必定启用平移环”，姿态 Offboard 可继续使用姿态期望；
- 补齐纯速度、纯高度和爬升率模式进入 Hnuter 后端的入口。

## 3. 新参数

| 参数 | 功能 | 实机默认值 |
|---|---|---:|
| `HNTR_POS_P_XY` | XY 位置误差到速度修正 | 0.6 |
| `HNTR_POS_P_Z` | Z 位置误差到速度修正 | 1.0 |
| `HNTR_VEL_P_XY` | XY 速度环 P | 1.5 |
| `HNTR_VEL_P_Z` | Z 速度环 P | 2.5 |
| `HNTR_VEL_I_XY` | XY 速度环 I | 0.10 |
| `HNTR_VEL_I_Z` | Z 速度环 I | 0.40 |
| `HNTR_VEL_D_XY` | XY 实测加速度阻尼 | 0.10 |
| `HNTR_VEL_D_Z` | Z 实测加速度阻尼 | 0.20 |
| `HNTR_VEL_ILIM_XY` | XY 积分最大加速度贡献 | 1.5 m/s^2 |
| `HNTR_VEL_ILIM_Z` | Z 积分最大加速度贡献 | 2.5 m/s^2 |
| `HNTR_VEL_XY` | 最大水平速度设定 | 3.0 m/s |
| `HNTR_VEL_UP` | 最大上升速度设定 | 1.5 m/s |
| `HNTR_VEL_DN` | 最大下降速度设定 | 1.0 m/s |
| `HNTR_ACC_XY` | 最大水平加速度 | 5.0 m/s^2 |
| `HNTR_ACC_Z` | 最大垂向加速度 | 8.0 m/s^2 |

旧的 `HNTR_XY_P/Z_P/XY_D/Z_D/XY_I/Z_I` 仅保留用于读取旧参数文件，新的级联环
不再使用它们，避免飞控中保存的旧高增益在刷入新固件后突然生效。

## 4. 实机 PWM–推力模型

SITL 继续使用 Gazebo SDF 中的精确转速模型。CUAV 7 Nano 等硬件使用悬停点
锚定模型：

```text
motor_command = HNTR_MOT_HOV * (requested_force / hover_force)^HNTR_MOT_EXPO
```

- `HNTR_MOT_HOV`：四个主电机正常悬停时的归一化命令，默认 `0.40`；
- `HNTR_MOT_EXPO`：推力模型指数，默认 `0.50`，对应推力近似与转速平方成正比。

只有电机型号、KV 和桨直径仍不足以确定绝对推力曲线，还缺少电池电压、桨距、ESC
油门映射、电机/桨实际效率及带载压降。新模型不要求先测最大推力，只需要从一次稳定
悬停日志中读取主电机平均命令作为 `HNTR_MOT_HOV`。若只看到 PWM，1000~2000 us
输出可用 `(PWM - 1000) / 1000` 换算。不要在固定测试架上估计悬停点。

`HNTR_HOV_THR` 已成为兼容参数，不再参与实机控制。`HNTR_MAX_ARM_T` 仍用于物理力/
力矩归一化上限，但不再把 Gazebo 电机常数带入实机 PWM。

## 5. 首次实机步骤

1. 保留现有姿态环已调好的参数，先使用上述保守位置环默认值。
2. 在空旷区域短时离地悬停，检查 MAIN1~4 是否同时、连续变化。
3. 从稳定悬停段读取 `actuator_motors.control[0..3]` 平均值，写入
   `HNTR_MOT_HOV` 并 `param save`。
4. 先调 `HNTR_VEL_P_*` 和 `HNTR_VEL_D_*`，使速度响应不振荡；再增加
   `HNTR_VEL_I_*` 消除风、重心和推力模型造成的静差；最后调位置 P。
5. 若命令达到 `HNTR_ACC_Z` 后仍觉得输出不足，确认没有执行器饱和后再逐步提高
   `HNTR_ACC_Z`。该参数是有意的物理加速度限制，不是 1450 us PWM 硬限幅。

建议起始命令：

```sh
param set HNTR_MOT_HOV 0.40
param set HNTR_MOT_EXPO 0.50
param set HNTR_POS_P_XY 0.6
param set HNTR_POS_P_Z 1.0
param set HNTR_VEL_P_XY 1.5
param set HNTR_VEL_P_Z 2.5
param set HNTR_VEL_I_XY 0.10
param set HNTR_VEL_I_Z 0.40
param set HNTR_VEL_D_XY 0.10
param set HNTR_VEL_D_Z 0.20
param save
```

## 6. 验证结果

- `make px4_sitl_default`：通过。
- `HEADLESS=1 make px4_sitl gz_hnuter` + DDS +
  `hnuter_external_controller_px4_position.py`：Offboard 解锁、起飞并稳定收敛。
- 仿真最后 20 秒位置 RMSE：X `0.026 m`、Y `0.008 m`、Z `0.072 m`；最终误差
  为 `[0.016, -0.017, 0.007] m`。
- 仿真前四电机命令范围 `0.31~0.37`，未出现输出平台截断。
- `make cuav_7-nano_default`：通过，Flash `1859908 / 1966080 B`（`94.60%`）。
- 固件：`build/cuav_7-nano_default/cuav_7-nano_default.px4`。
- SHA-256：`3f3b3f27e2537ff6f733a0874cd77cb9cf2b306c53202d00e44186937f84824a`。
