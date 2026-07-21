# Hnuter 姿态改变时的位置锁定

## 1. 目标

不再要求通过动捕分别辨识四个倾转关节，也不再让辨识得到的纯延迟、时间常数和
速度上限降低控制指令响应。Hnuter 在 Position 模式下改变 roll/pitch 期望时，位置环
保持运行，持续根据实测位置、速度和姿态修正三维合力。

本次没有删除旧的倾转动态模型，而是新增开关 `HNTR_TDYN_EN`，默认值为 `0`：

- `0`：立即使用当前倾转命令对应的静态几何，姿态指令仅受
  `HNTR_RC_RATE_R/P` 或 `HNTR_RC_LVL_R` 限制。
- `1`：保留旧的纯延迟、一阶惯性和速度限制估计，并使用
  `HNTR_SYNC_ERR` 同步姿态参考。只用于需要复现实验或接入可靠关节反馈前的对比。

`HNTR_T1_GAIN/ZERO` 和 `HNTR_T2_GAIN/ZERO` 在两种模式下都生效。它们描述静态
传动关系与机械零位，不会增加时间延迟；二级 1:2 减速比仍可通过 `HNTR_T2_GAIN`
表达，不需要重新做四关节动态辨识。

## 2. 位置锁定控制链

Position 模式下，改变姿态不会关闭位置控制器：

```text
位置/速度期望
      |
      v
级联位置-速度控制器 ---- 实测位置、速度
      |
      v
世界系期望加速度 a_des
      |
      v
F_world = m (a_des - g)
      |
      v
F_body = R_actual^T F_world ---- 实测姿态 R_actual
      |
      v
几何姿态控制器 -> 期望力矩
      |
      v
Hnuter 控制分配 -> 电机与两级倾转命令
      |
      v
机体运动 -> EKF/动捕位置与姿态反馈
```

因此，当飞机保持倾斜姿态时，位置环会增加相应的机体系水平/垂直分量，使世界系
合力继续抵消重力并修正位置误差。`HNTR_TDYN_EN=0` 时，allocator 不再用一个滞后的
虚拟关节角投影电机推力，也不会由该虚拟误差产生额外 residual 和参考降速。

## 3. 实机设置

新固件默认关闭倾转动态补偿。为避免旧参数存储状态不明确，烧录后可显式执行：

```sh
param set HNTR_TDYN_EN 0
param save
reboot
```

检查：

```sh
param show HNTR_TDYN_EN
```

后续姿态变化速度主要由以下参数控制：

- `HNTR_RC_RATE_R`：AUX1 横滚目标角速度，单位 deg/s。
- `HNTR_RC_RATE_P`：AUX2 俯仰目标角速度，单位 deg/s。
- `HNTR_RC_LVL_R`：回平过程的最大角速度，单位 deg/s。
- `HNTR_RC_ANG_MAX`：手动姿态目标绝对角度上限。
- `HNTR_POS_P_*`、`HNTR_VEL_P/I/D_*`：倾斜时的位置和速度恢复能力。
- `HNTR_ACC_XY/Z`：位置环允许输出的加速度上限。

## 4. 验证顺序

1. 测试架上确认四个倾转方向、静态零位和电机方向正确。
2. 小角度悬停，从 `5 deg`、`10 deg`、`20 deg` 逐级增加 roll/pitch，观察位置误差、
   速度误差、`vehicle_thrust_setpoint` 和 allocator residual。
3. 先调姿态环阻尼，再调位置/速度环；不要用位置积分掩盖倾转零位错误。
4. 无绳索约束验证位置锁定。绳索提供的水平约束力会让失败的控制看起来稳定。

## 5. 边界

当前实机没有四个关节角反馈。关闭动态辨识模型后，控制链响应更直接，但 allocator
仍只能知道舵机命令，不能知道齿隙、负载下的实际角度或卡滞。位置闭环会通过动捕/EKF
修正最终误差，但在机构尚未到位的瞬间仍可能出现短暂漂移。若后续需要进一步提高大
姿态瞬态性能，优先接入关节角反馈，而不是重新依赖开环的四关节动态辨识。

## 6. 构建与验证结果

- `make px4_sitl_default`：通过。
- `make cuav_7-nano_default`：通过，Flash `95.22%`，AXI SRAM `18.71%`。
- 固件：`build/cuav_7-nano_default/cuav_7-nano_default.px4`。
- SHA-256：`d0a7da4e7badb1fcc709b199fcd31d4d687a65cb5a314bbfc9746a7ad45c4b09`。
- `HEADLESS=1 make px4_sitl gz_hnuter`：模型、5 电机、4 舵机和 DDS bridge 启动成功；
  `param show HNTR_TDYN_EN` 返回 `0`。

自动模拟 RC 的 Position/AUX 飞行测试未作为通过项：当前固件发布
`vehicle_local_position_v1/vehicle_status_v1`，但现有 `px4_ws_ros2` 安装的消息包仍
订阅旧版本话题；模拟 `manual_control_input` 随后失效并触发标准 `mc_pos_control`
的 `invalid setpoints`。这属于测试入口不一致，不能用于评价本次控制修改。实机应使用
真实 RC、有效动捕/EKF 和本文第 4 节的小角度流程验证。
