# Hnuter 舵机 PWM 与二级齿轮比标定固件（2026-08-04）

## 版本关系

- 上一可回退节点：`hnuter-no-delay-roll-debug-20260804`。
- 上一分支：`codex/hnuter-e095-roll-debug-fixes`，提交 `6727444f`。
- 本修改分支：`codex/hnuter-servo-pwm-gear-calibration-20260804`。
- 本版本仍是无舵机延迟模型固件；没有合入 `servo-id` 或
  `identified-delay-actuator` 分支。

## 问题依据

`log_48`、`log_53` 和 `log_55` 的飞行时参数均为：

```text
PWM_MAIN_MIN8..11 = 800 us
PWM_MAIN_MAX8..11 = 2200 us
PWM_MAIN_DIS8..11 = 1500 us
PWM_MAIN_FAIL8..11 = 1500 us
```

实机位置舵机完整输入范围是 `500--2500 us`，中点为 `1500 us`，完整舵机轴
行程按 `-180--+180 deg` 标定。旧范围只能覆盖完整 PWM 行程的 70%，因此旧固件
即使输出归一化 `+1`，舵机理论上也只到约 `+126 deg`。

此外，二级倾转在舵机轴与输出关节之间存在 2:1 减速传动：舵机轴需要旋转
2 deg，二级输出关节才旋转 1 deg。旧分配器没有该比例，二级实际关节角进一步
只有计算目标的一半。

## 修改内容

### 1. 放宽 PX4 PWM 参数范围

`pwm_out` 和 `px4io` 的参数元数据现在允许：

```text
PWM_*_MINn: 500--1400 us
PWM_*_MAXn: 1600--2500 us
PWM_*_DISn / PWM_*_FAILn: 500--2500 us
```

底层 `PWM_HIGHEST_MAX` 原本就是 `2500 us`，因此本修改放宽的是 PX4 参数层，
不是绕过底层定时器硬限制。电机 MAIN1--5 仍保持原来的电调范围；只有 Hnuter
舵机 MAIN8--11 在 airframe 中设为 `500--2500 us`。

### 2. 新增独立二级齿轮比参数

新增 QGC/PX4 参数：

```text
HNTR_S2_GEAR = 舵机轴角 / 二级输出关节角
```

- 实机默认：`2.0`。
- 当前 Gazebo 直接驱动关节，没有建模齿轮：`1.0`。
- 可调范围：`0.1--10.0`。

该参数只作用于 Servo 3/4（`rj1`、`lj1`），一级 Servo 1/2（`rj2`、`lj2`）
继续按直接驱动处理。分配器同时使用该参数计算二级真实可达关节角，防止齿轮比
2.0 时仍假定二级关节可达正负 180 deg；当前物理可达范围为正负 90 deg。

### 3. 归一化角度与 PWM 对应关系

一级直接驱动：

```text
normalized = primary_angle / 180 deg
PWM = 1500 us + normalized * 1000 us
```

二级齿轮驱动：

```text
servo_angle = secondary_joint_angle * HNTR_S2_GEAR
normalized = servo_angle / 180 deg
PWM = 1500 us + normalized * 1000 us
```

`HNTR_S2_GEAR=2.0` 时的回归表：

| 二级关节目标 | 归一化输出 | PWM | 舵机轴角 | 齿轮后关节角 |
| ---: | ---: | ---: | ---: | ---: |
| -90 deg | -1.0 | 500 us | -180 deg | -90 deg |
| -45 deg | -0.5 | 1000 us | -90 deg | -45 deg |
| 0 deg | 0.0 | 1500 us | 0 deg | 0 deg |
| +45 deg | +0.5 | 2000 us | +90 deg | +45 deg |
| +90 deg | +1.0 | 2500 us | +180 deg | +90 deg |

一级 `CA_SV_TL0/1_MINA/MAXA` 从飞行时保存的正负 185 deg 条件迁移为正负
180 deg，使归一化正负 1 与舵机实测完整轴角严格对应。二级
`CA_SV_TL2/3_MINA/MAXA` 保留正负 180 deg，表示舵机轴在完整 PWM 范围内的
角度；二级输出关节可达角由该值除以 `HNTR_S2_GEAR` 得到。

### 4. 已保存参数的一次性迁移

硬件 airframe 只在四路舵机完整匹配日志旧组合时执行：

```text
PWM_MAIN_MIN8..11: 800 -> 500 us
PWM_MAIN_MAX8..11: 2200 -> 2500 us
```

只有一级左右同时匹配旧的正负 185 deg 时，才迁移到正负 180 deg。这样后续逐路
实测得到的非对称端点不会在重启时被覆盖。

## 验证结果

### 静态与生成物检查

- `git diff --check`：通过。
- 二级五点数学回归：`-90/-45/0/45/90 deg` 均可经 normalized/PWM/齿轮
  正确反算回原关节角。
- CUAV 生成的 `parameters.xml`：
  - `HNTR_S2_GEAR` 默认 `2.0`，范围 `0.1--10.0`；
  - `PWM_MAIN_MIN8` 最小允许 `500`；
  - `PWM_MAIN_MAX8` 最大允许 `2500`。
- 最终 CUAV `romfs_files.tar` 已反向核对，硬件 airframe 中包含 PWM、一级角度
  条件迁移和 `HNTR_S2_GEAR=2.0`。

### 编译与运行

- `make px4_sitl_default`：成功。
- `HEADLESS=1 make px4_sitl gz_hnuter`：成功进入 `pxh>`，模型 `hnuter_0`
  和四路舵机加载；运行时 `HNTR_S2_GEAR=1.0`、`CA_SV_TL2_MAXA=180.0`。
- `CCACHE_DIR=/tmp/ccache make cuav_7-nano_default`：成功；Flash
  `1867028 / 1966080 bytes (94.96%)`。

最终提交完成后重新生成的产物 SHA-256、Flash 使用量和提交号记录在注释标签
`hnuter-servo-pwm-gear-20260804` 中，避免使用提交前 dirty 工作树生成的哈希冒充
最终固件身份。产物路径保持为：

```text
build/px4_sitl_default/bin/px4
build/cuav_7-nano_default/cuav_7-nano_default.px4
```

## 烧录后的台架检查

必须拆除桨叶并逐路检查，不能直接带桨进行大姿态飞行：

1. 确认 MAIN8--11 的 `MIN/MAX/DIS/FAIL` 分别为
   `500/2500/1500/1500 us`。
2. 确认实机 `HNTR_S2_GEAR=2.0`，SITL 才使用 `1.0`。
3. 在 QGC Actuator Test 中按小步增加命令，先检查正负 0.25 和正负 0.5；确认
   `PWM_MAIN_REV` 方向正确且左右不打架。
4. 一级目标正负 90 deg 应约对应 1000/2000 us，正负 180 deg 对应
   500/2500 us。
5. 二级目标正负 45 deg 应约对应 1000/2000 us，正负 90 deg 对应
   500/2500 us。
6. 如果机械限位、舵机内部限位或左右装配偏差导致端点不一致，应缩小对应通道的
   `PWM_MAIN_MIN/MAX`，不能依靠控制器持续顶住机械限位。
7. 完成实际角度核对前，保持小角度、低速 Roll 测试，不增加姿态或位置积分增益。

## 仍未解决

- 本修改校正开环角度比例，但没有增加舵机实际角度传感器；带载下垂、回差、死区
  和左右不同步仍无法由飞控直接观测。
- 若实测齿轮方向与本文定义相反，应按
  `舵机轴角 / 二级输出关节角` 重新填写 `HNTR_S2_GEAR`，不能只按齿轮齿数名称
  猜测数值。
- 后续实飞日志需要同时保存固件提交、完整参数和 MAIN8--11 输出，再评估是否降低
  Roll 积分或增加执行器角度反馈。
