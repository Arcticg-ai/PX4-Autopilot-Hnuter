# Hnuter 遥控辅助姿态保持与渐进回平（2026-07-14）

## 功能

Hnuter Position/Velocity 手动模式支持使用三个额外遥控通道直接调整机体姿态：

- `AUX1`：roll 期望角速度；
- `AUX2`：pitch 期望角速度；
- `AUX3`：上升沿触发渐进回平。

手动 Position 模式的主偏航摇杆同样采用角速度积分：摇杆回中后保持内部航向期望，
避免上游 `trajectory_setpoint.yaw` 随实际航向重置后造成 roll/yaw 耦合漂移。

AUX1/AUX2 偏离中位时，通道值经过死区和重新归一化后形成期望角速度，积分得到
roll/pitch 期望角。通道回中沿会锁存当时的实测姿态，而不是继续追赶超前于机构响应的
积分目标；之后保持锁存角度，不会自动回中。主偏航摇杆采用相同的回中锁存逻辑。

AUX3 从低切到高时锁存回平动作。即使按键随后松开，roll/pitch 期望仍会按
`HNTR_RC_LVL_R` 限制的速度逐渐回到 0°。回平过程中重新操纵 AUX1 或 AUX2 会立即
取消回平，恢复角速度积分控制。

该功能只覆盖手动 Position/Velocity/Altitude 类模式的 Hnuter roll/pitch 期望：

- Stabilized 继续使用主摇杆和 PX4 姿态期望；
- Offboard 不受辅助遥控通道影响；
- 遥控失联或模式切换后清除内部保持状态，再次进入时从实际姿态初始化，并按
  `HNTR_RC_LVL_R` 渐进建立水平 roll/pitch 目标，避免跳变。

## 参数

| 参数 | 功能 | 仿真默认 | 实机默认 |
|---|---|---:|---:|
| `HNTR_RC_ATT_EN` | 启用辅助姿态保持 | 1 | 1 |
| `HNTR_RC_RATE_R` | AUX1 满行程 roll 角速度 | 30 deg/s | 20 deg/s |
| `HNTR_RC_RATE_P` | AUX2 满行程 pitch 角速度 | 30 deg/s | 20 deg/s |
| `HNTR_RC_RATE_Y` | 主偏航摇杆满行程角速度 | 25 deg/s | 25 deg/s |
| `HNTR_RC_DB` | AUX1/AUX2 中位死区 | 0.08 | 0.08 |
| `HNTR_RC_ANG_MAX` | AUX1/Roll 累计姿态绝对值限制 | 90° | 45° |
| `HNTR_RC_LVL_R` | 渐进回平最大速度 | 20 deg/s | 15 deg/s |

自 2026-08-08 起，`HNTR_RC_ANG_MAX` 只限制 AUX1/Roll；AUX2/Pitch 不再受此参数
限制，内部目标按完整姿态圆周期归一化到正负 180°。Roll 的参数范围仍允许到
180°，但实机应保持 45° 或更低，只有在姿态环、控制分配和线束活动范围均验证后
才能逐步放大。

## 遥控器映射

机型默认将物理通道 6、7、8 映射为 AUX1、AUX2、AUX3：

```sh
RC_MAP_AUX1 = 6
RC_MAP_AUX2 = 7
RC_MAP_AUX3 = 8
```

如果遥控器使用其他通道，在 QGC 的参数页面修改这三个映射即可。AUX1/AUX2 必须是
可自动回中的比例通道，校准后中位应接近 0，端点接近 -1/+1。方向相反时优先在 QGC
中反转对应物理 RC 通道。

AUX3 可以使用两段开关或瞬时按键。触发要求从低位切到高位，因此开机或进入 Position
模式前应让开关处于低位。

## SITL 验证

编译与启动：

```bash
make px4_sitl_default
HEADLESS=1 make px4_sitl gz_hnuter
```

使用 MAVLink `MANUAL_CONTROL` 扩展字段以 25 Hz 注入 AUX1/AUX2/AUX3，并在 Position
模式自动起飞到约 4 m。测试顺序为 roll 命令、回中保持、AUX3 回平、pitch 命令、
回中保持、AUX3 回平。

结果：

- roll 保持阶段实际 roll 约 `21.0°`；
- pitch 保持阶段实际 pitch 约 `34.6°`；
- 第一次回平从约 `(20.7°, 24.1°)` 开始，`1.31 s` 后进入 roll/pitch ±5°；
- 第二次回平从约 `pitch 33.1°` 开始，`1.40 s` 后进入 roll/pitch ±5°；
- 最终回平姿态约为 `roll -0.07°`、`pitch 1.66°`；
- 姿态测试阶段高度范围 `0.045 m`，水平位置范围约 `0.15 × 0.23 m`；
- 未发生瞬时回平、失控或位置发散。
- `make cuav_7-nano_default` 编译通过，Flash `1862708 / 1966080 B`（`94.74%`）；
- 固件 SHA-256：`53dba1e4536ba3db1bed4d0fcf90940602f2b1d7a202c9adf2d1666da24c95dc`。

日志：`build/px4_sitl_default/rootfs/log/2026-07-14/02_20_33.ulg`。

进一步检查发现手动 Position 的上游 `trajectory_setpoint.yaw` 会随实际航向移动，已改为
Hnuter 内部偏航角速度积分与航向保持。修复后 pitch 操作能保持 yaw，roll 操作回平后也能
恢复原航向；但约 27° roll 保持期间仍会临时产生约 28° yaw 偏移和约 15° pitch 欧拉角
耦合。在线将 `HNTR_TAU_Y` 从 1.8 Nm 提高到 5 Nm 没有明显改善，说明它不是简单的 yaw
力矩限幅，而是当前 Gazebo 模型/分配器的组合姿态执行器耦合。

该残余不影响通道状态机和渐进回平功能判定。`HNTR_RC_ANG_MAX` 当前只保护 Roll，
首次 Roll 测试仍应设为 10–15°。Pitch 虽已取消该参数限幅，也必须按拆桨、系留
正负 10/20/30° 的顺序验证，不能直接进行 90° 实飞。
