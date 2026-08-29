# HNUTER log 133 解锁倾覆与固件纠正（2026-08-08）

## 证据身份

- ULog：`log_133_2026-8-8-21-35-04.ulg`
- ULog SHA-256：
  `7d2db793f80f74e434341d2cf2c7324ba57d32d3a25dd5ef9a084ef15748b722`
- 失效固件：`003349b7bdb3df56a9866ebc194aceb8c93d65c5`
- 硬件：CUAV 7 Nano；日志约 2 s，无 dropout。

`003349b7` 必须标记为禁止实机上桨版本。

## 直接证据与原因

- Pitch 目标约 `0 deg`，遥控 Pitch 基本不变，四个倾转舵机接近中位。
- 尾电机归一化指令在解锁后达到 `0.379`，PWM 达到 `1660 us`；实际 Pitch
  最低约 `-89.45 deg`，最大负 Pitch 角速度约 `-301 deg/s`。
- 飞控实际保存了 `HNTR_PITCH_BIAS=0.09` 和 `HNTR_TAIL_COMP=1`。
- 旧固定偏置约产生 `5.11 Nm` 指令；基于总推力的 CG 补偿又沿同方向增加尾推，
  初始尾推计算值约 `12.29 N`，对应归一化 `0.379`，与 ULog 完全一致。
- 条件迁移先要求 `HNTR_TAIL_COMP=0`，但新 airframe 默认值已经改成 `1`，所以
  `PITCH_BIAS` 未清零。使用参数值组合作为版本判据不可靠。

## 纠正原则

1. 删除 `HNTR_TAIL_COMP` 参数以及尾推随总推力变化的分配路径。
2. Motor5 只由 Pitch 力矩指令产生；继续保留换向过零等待。
3. 保留 `Fz_front=W[2]-F3`，因为它是机体系 Z 方向合力守恒，不是把 Motor5
   永远当作世界坐标升力。
4. 删除 airframe 中全部 `param compare` 条件迁移。固件只提供正确默认值，现有
   飞控烧录后由操作者手动设置并保存发布参数，不在每次启动时覆盖人工调参。
5. 在测得一级倾转轴到重心的准确几何关系前，不加入新的姿态相关重力补偿。

## 新固件默认与烧录后的手动参数表

以下是本次纠正后应核对并手动保存的关键值；仅烧录固件不会覆盖飞控已有参数：

```text
HNTR_ATT_KR_P    5.0
HNTR_ATT_D_P     2.5
HNTR_ATT_I_P     0.0
HNTR_ATT_ILIM_P  0.3
HNTR_TAU_P       8.0
HNTR_PITCH_BIAS  0.09
HNTR_RC_RATE_P   8.0
HNTR_RC_ANG_MAX  30.0

HNTR_POS_P_XY    0.6
HNTR_VEL_P_XY    1.5
HNTR_VEL_I_XY    0.0
HNTR_POS_P_Z     1.0
HNTR_VEL_P_Z     2.5
HNTR_ACC_XY      1.5
HNTR_ACC_Z       8.0

PWM_MAIN_MIN8    500
PWM_MAIN_MIN9    500
PWM_MAIN_MIN10   500
PWM_MAIN_MIN11   500
PWM_MAIN_MAX8    2500
PWM_MAIN_MAX9    2500
PWM_MAIN_MAX10   2500
PWM_MAIN_MAX11   2500
```

`HNTR_TAIL_COMP` 在纠正后的固件中不存在；飞控参数存储中遗留的同名旧值不再被
控制器读取。

## 验证边界

重新编译成功只能证明源码和参数元数据一致。下一次上机必须先拆桨确认 Motor5 在
解锁、低油门和增加总推力时不会因总推力单独改变指令；然后才可进行系留小角度测试。
