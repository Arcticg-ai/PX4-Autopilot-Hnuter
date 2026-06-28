# Hnuter 一级全周倾转与 105/175/180 度定点俯仰测试

## 问题结论

原来的持续水平漂移不是正常现象，也不是单纯增大姿态增益可以解决的问题。
一级倾转链路在四处被限制为 `-90 deg` 到 `+90 deg`：

- PX4 Hnuter 自定义控制分配的 `alpha_limit`
- PX4 和 Gazebo 之间的 Servo 1/2 角度映射
- Gazebo 模型的 `rj2`、`lj2` 关节限位
- ROS 2 direct 控制器的一级倾转归一化

机体俯仰越过 `90 deg` 后，旧配置无法继续把旋翼合力保持在世界坐标系竖直方向，
因而形成持续水平分力。旧测试中的约 25 m 位移由该限制导致。

## PX4 固件修改

### 控制分配

文件：
`src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.cpp`

- 一级倾转 `alpha1/alpha2` 正常飞行范围改为 `-185..185 deg`，
  为 `-180..180 deg` 姿态输入保留 `5 deg` 控制余量。
- 一级倾转在 `atan2` 的 `-pi/+pi` 分支附近相对上一帧展开，并限制到机械端点，
  防止同一物理方向产生接近 `360 deg` 的舵机命令跳变。
- 一级舵机输出按 `alpha / 185 deg` 归一化。
- 二级倾转 `theta1/theta2` 仍按 `theta / (pi/2)` 归一化。
- 一级和二级分别计算归一化速率限制，物理角速度上限仍为 `50 rad/s`。
- 起飞保护阶段的 `20 deg` 和 `30 deg` 临时限幅保持不变。

### 参数与机型默认值

文件：

- `src/modules/control_allocator/module.yaml`
- `ROMFS/px4fmu_common/init.d/airframes/4051_gz_hnuter`
- `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`

参数元数据 `CA_SV_TLx_MINA/MAXA` 的可配置范围扩展为 `-190..190 deg`。
Hnuter 一级倾转默认值修改如下：

```text
CA_SV_TL0_MINA=-185
CA_SV_TL0_MAXA=185
CA_SV_TL1_MINA=-185
CA_SV_TL1_MAXA=185
```

SITL 的 Servo 1/2 映射同步改为：

```text
SIM_GZ_SV_MINA1=-185
SIM_GZ_SV_MAXA1=185
SIM_GZ_SV_MINA2=-185
SIM_GZ_SV_MAXA2=185
```

Servo 3/4 和 `CA_SV_TL2/3` 仍保持 `-90..90 deg`。

### Gazebo 模型

文件：`Tools/simulation/gz/models/hnuter/model.sdf`

- 一级关节 `rj2`、`lj2`：`-185..185 deg`（`-3.22886..3.22886 rad`）
- 二级关节 `rj1`、`lj1`：继续保持 `-pi/2..+pi/2`

SITL 启动日志已经确认 Servo 0/1 的输出范围为
`-3.229..+3.229 rad`，Servo 2/3 为 `-1.571..+1.571 rad`。

## 外部控制器修改

仓库：`~/px4_ws_ros2`

文件：

- `hnuter_external_direct_controller_debug.py`
- `hnuter_direct_tuning.json`

主要变化：

- `alpha_limit_deg` 从 `90` 改为 `185`。
- 手柄积分俯仰期望范围由 `-90..90 deg` 扩展为 `-180..180 deg`。
- 增加在线参数 `manual_pitch_limit_deg`，默认值为 `180`；即使在线配置写入
  更大的值，也会被硬限制为 `180 deg`。
- 轨迹 3 的 pitch 输入同样在环境变量和在线 JSON 加载入口硬限制为
  `-180..180 deg`。
- 手柄期望到达端点后，发送给姿态控制器的俯仰角速度同步归零，
  不再保留继续穿越端点的角速度前馈。
- 一级倾转同样使用相对上一帧最近的 `atan2` 分支，避免
  `-179 deg` 突然跳到正角分支。
- 增加在线开关 `direct_safety_attitude_check_enabled`，调试阶段默认为 `false`；
  因此手柄跨过 `90 deg` 时不再产生姿态越界告警或触发姿态停机逻辑。
- 水平速度检查、Offboard/DDS 超时和 PX4 自身失效保护保持不变。
- 一级舵机输出由 `alpha / 90 deg` 改为 `alpha / 185 deg`。
- 保留二级 `theta_limit_deg=45` 和 `theta / 90 deg` 输出映射。
- 大姿态阶段恢复水平位置和速度反馈，始终保持轨迹起点。
- 传统 `tan(alpha_limit)` 推力锥只用于小于 `89 deg` 的起飞保护阶段。
- 正常全周倾转阶段不再错误套用该推力锥，仅按二级倾转范围限制侧向力。
- 轨迹结束后继续锁定原悬停点，不再把有瞬时误差的当前位置保存为新目标。
- 轨迹 3 使用 6 s 平滑上升、1 s 峰值保持和 6 s 平滑返回。
- 默认目标为 roll `0 deg`、pitch `105 deg`、yaw `0 deg`。
- pitch 姿态增益为 `KR=5.0`、`Domega=2.5`、力矩上限 `3.0 N m`。
- 保留连续俯仰角日志和 direct 模式 land detector 总推力提示。

手柄相关在线配置：

```json
{
  "manual_pitch_limit_deg": 180.0,
  "direct_safety_attitude_check_enabled": false,
  "direct_safety_attitude_limit_deg": 55.0
}
```

`direct_safety_attitude_limit_deg` 只在
`direct_safety_attitude_check_enabled=true` 时生效。修改 JSON 后控制器会在线重新加载。

## 编译与 SITL 验证

编译：

```bash
make px4_sitl_default
CCACHE_DISABLE=1 make cuav_7-nano_default
```

结果：

- SITL：`402/402`，成功生成 `build/px4_sitl_default/bin/px4`。
- CUAV 7-Nano：最新增量构建 `339/339`，成功生成
  `build/cuav_7-nano_default/cuav_7-nano_default.px4`。
- 7-Nano FLASH 使用 `1846316 B / 1920 KB`，占 `93.91%`。
- 沙箱内全局 ccache 目录只读，因此硬件构建使用 `CCACHE_DISABLE=1`；
  这只关闭编译缓存，不改变固件内容。

运行：

```bash
MicroXRCEAgent udp4 -p 8888
HEADLESS=1 make px4_sitl gz_hnuter
```

```bash
source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
cd ~/px4_ws_ros2
python3 hnuter_external_direct_controller_debug.py
```

按 `o` 起飞，悬停后按 `3`。测试数据：
`~/px4_ws_ros2/hnuter_direct_debug_1782651523.csv`。

2026-06-28 实测指标：

- 目标俯仰峰值：`105.00 deg`
- 实际连续俯仰峰值：`102.61 deg`
- 全活动段俯仰 MAE：`1.50 deg`
- 目标大于等于 `100 deg` 时的俯仰 MAE：`2.45 deg`
- 一级实际命令峰值：`101.28 deg`
- 水平定点误差最大值：`0.206 m`
- 水平定点误差 RMS：`0.135 m`
- 高度误差最大值：`0.153 m`
- 高度误差 RMS：`0.029 m`
- 全程保持 Armed 和 Offboard
- `landed` 误报采样：`0`

## 180 度倾覆原因、175 度验证与余量修复

旧的 180 度手柄测试数据：
`~/px4_ws_ros2/hnuter_direct_debug_1782652607.csv`。

倾覆前的直接证据：

- 目标达到 `180 deg` 时，一级倾转已经接近机械端点 `-179 deg`。
- `atan2` 随后跨过 `-pi/+pi` 分支，一级舵机最大单步跳变达到 `286.48 deg`。
- 左右一级舵机进入不同角度分支，俯仰角速度峰值达到 `23.58 rad/s`。
- pitch 力矩打满 `3.0 N m`，水平误差最大达到 `9.61 m`，随后倾覆。

机械范围达到 `-180..180 deg` 不代表可以把 `180 deg` 当作无余量的稳定工作点。
机体完全倒置时，维持世界竖直合力也要求一级倾转处于端点；位置和姿态控制所需的
正反向微调会跨越该端点，因此必须保留工作余量。

修复后的 175 度测试数据：
`~/px4_ws_ros2/hnuter_direct_debug_1782653169.csv`。

- 目标俯仰峰值：`175.00 deg`
- 实际连续俯仰峰值：`174.40 deg`
- 峰值保持：`3 s`
- 一级倾转峰值约：`174.23 deg`
- 一级舵机最大单步变化：`3.74 deg`
- 俯仰角速度峰值：`0.60 rad/s`
- 水平定点误差最大值：`0.238 m`
- 水平定点误差 RMS：`0.161 m`
- 高度误差最大值：`0.169 m`
- 全程保持 Armed 和 Offboard，`landed` 误报为 0
- 完整返回水平悬停，未发生倾覆

将一级执行器范围扩大到 `-185..185 deg` 后，重新进行严格的 `180 deg`
往返测试，数据为：
`~/px4_ws_ros2/hnuter_direct_debug_1782654836.csv`。

- 目标俯仰峰值：`180.00 deg`
- 峰值保持阶段实际连续俯仰平均值：`179.50 deg`
- 峰值保持阶段俯仰 RMS 误差：`0.50 deg`
- 一级倾转最小值：`-180.21 deg`，距离 `-185 deg` 端点仍有 `4.79 deg`
- 一级舵机最大单步变化：`4.06 deg`
- 俯仰角速度峰值：`0.62 rad/s`
- 水平定点误差最大值：`0.220 m`
- 水平定点误差 RMS：`0.162 m`
- 高度误差最大值：`0.156 m`
- 高度误差 RMS：`0.026 m`
- 全程保持 Armed 和 Offboard，无 `landed` 误报和 direct safety cutoff
- 完整返回水平悬停，未发生倾覆

## 实机注意事项

该配置假定一级机械机构和舵机从最小 PWM 到最大 PWM 能实际覆盖
`-185..+185 deg`。烧写新固件后应先拆除桨叶并检查一级关节方向、零位和全行程，
确认不会拉扯线束或碰撞机架。已有保存参数不会总被 `set-default` 覆盖，必要时重新选择
Hnuter 机型或手动检查 `CA_SV_TL0/1_MINA/MAXA`。
