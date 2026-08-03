# Hnuter 无延迟实机固件备份基线（2026-08-04）

## 版本身份

- 基础提交：`e0958bbd`（大倾角 Roll 悬停侧向力限制修复）。
- 本基线代码提交：`75fb3965`（实机 Roll 调试与安全参数修正）。
- 备份分支：`codex/hnuter-e095-roll-debug-fixes`。
- 性质：无舵机延迟模型的实机固件基线。

本分支没有合入以下舵机动力学/延迟实验分支：

- `codex/hnuter-servo-id-20260721`
- `codex/hnuter-identified-delay-actuator-20260728`

因此，本分支的分配器将计算出的倾转角直接转换为
`actuator_servos` 归一化命令，不在飞控内部叠加纯延迟、速率辨识模型或额外的
舵机一阶动态。该版本作为后续 PWM/齿轮比标定修改前的可回退节点。

## 相对 e0958bbd 的修改

### 1. Roll 实机调试参数降级

针对 `log_53_2026-7-30-21-32-46.ulg` 中约 3--4 Hz Roll/二级倾转振荡，将
日志内保存的旧参数组合做一次性条件迁移：

```text
HNTR_ATT_KR_R: 18.2 -> 10.0
HNTR_ATT_D_R:   9.6 ->  4.0
HNTR_TAU_R:    56.3 -> 15.0 Nm
```

条件迁移只匹配上述完整旧组合，避免后续人工调参在每次启动时被覆盖。

### 2. XY 积分和着陆安全参数

```text
HNTR_VEL_I_XY:  0.39 -> 0.20
HNTR_MOT_HOV:   0.40 -> 0.50
HNTR_LND_GC_R:  0.75 -> 0.85
HNTR_LND_MIN_R: 0.60 -> 0.80
```

其中 `HNTR_VEL_I_XY` 和着陆阈值只对日志中确认的旧值执行条件迁移。

### 3. HnuterControl 时间基准修复

消息新鲜度检查、解锁计时和模式内状态保持改用 `hrt_absolute_time()`；传感器的
`timestamp_sample` 仅继续传递给输出消息。该修复避免新发布的 RC/控制消息时间戳
略晚于旧角速度采样时间时，单周期错误清空 AUX Roll/Pitch 和航向保持状态。

### 4. 参数元数据与 SITL 同步

- `HNTR_ATT_KR_R` 默认值改为 `10.0`。
- `HNTR_ATT_D_R` 默认值改为 `4.0`。
- `HNTR_TAU_R` 默认值改为 `15.0 Nm`。
- `HNTR_VEL_I_XY` 默认值改为 `0.20`。
- POSIX/SITL airframe 同步对应的 Roll 调试默认值。

完整测试顺序和暂缓问题见
[`hnuter_e095_roll_debug_followup_2026-07-31.md`](hnuter_e095_roll_debug_followup_2026-07-31.md)。

## 已知未解决问题

1. `actuator_servos` 只记录目标值，没有一级/二级倾转实际角反馈。
2. 实飞保存的 MAIN8--11 PWM 范围为 `800--2200 us`，而位置舵机完整输入范围为
   `500--2500 us`；归一化指令与实际转角比例不一致。
3. 二级倾转存在机械齿轮传动，但本基线没有独立齿轮比参数。
4. 分配器仍按期望倾转角计算推力矢量；舵机不到位会造成大姿态水平力不足和漂移。
5. 在完成实际角度标定前，不应继续提高姿态/位置积分或进行大 Roll 飞行测试。

## 历史编译验证

`75fb3965` 提交时已记录：

- `make px4_sitl_default`：成功；`build/px4_sitl_default/bin/px4` SHA-256：
  `2bab2f4da799cabf65d50b8b9139d8791f4a670ce804cf34533351e5d033d7a1`。
- `CCACHE_DIR=/tmp/ccache make cuav_7-nano_default`：成功；Flash
  `1866428 / 1966080 bytes (94.93%)`；固件 SHA-256：
  `2d0278d3e746d9653246304cb2de382ea9dad82e9a81dbb8b9d98ef95431993c`。
- 最终 `romfs_files.tar` 已反向核对，硬件 airframe 参数和条件迁移已进入固件包。

上述哈希是 `75fb3965` 当时的构建证据；后续 PWM/齿轮比版本必须重新编译并记录
新的产物哈希，不能复用该哈希。

## 回退方法

需要恢复此无延迟基线时，切换到本备份分支或其 Git 标签，不要切换到带
`servo-id` / `identified-delay-actuator` 名称的实验分支。
