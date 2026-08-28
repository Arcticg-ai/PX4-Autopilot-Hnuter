# cfba38c6 无电池 4.8 kg 开门 OK 版本实飞记录

## OK 结论与适用边界

本记录把固件提交 `cfba38c68e403dcc63ee3e5d90e3b238f8918afd` 标记为当前的
**无电池、4.8 kg、现有尾电机/电调/舵机配置、低风险开门任务 OK 版本**。

该结论由 `log_181` 和 `log_182` 两次实飞共同支持：固件身份一致、飞行中没有参数修改、
ULog 没有 dropout、没有 failsafe 或 failure-detector 触发；AUX4 任务段位置跟踪稳定，尾电机
正常同向调节没有再被斜率限制拖慢，Position 模式 AUX2 松杆也没有再次产生历史上的几十度目标阶跃。

OK 不代表以下项目已经完成验证：

- 6 kg 加电池构型；
- 大角度极限姿态和激进水平机动；
- Position/Offboard 模式切换的无扰交接；
- 外部控制程序的具体 Git 提交。ULog 只记录其下发的轨迹，不记录外部仓库版本。

## 固件、产物和日志身份

| 项目 | 值 |
|---|---|
| 分支 | `codex/hnuter-pitch-tail-safety-20260808` |
| 固件提交 | `cfba38c68e403dcc63ee3e5d90e3b238f8918afd` |
| 提交说明 | `hnuter: remove tail tracking lag and release jumps` |
| 飞控硬件 | `CUAV_7_NANO` |
| 编译目标 | `cuav_7-nano_default` |
| `.px4` `git_identity` | `hnuter-servo-pwm-gear-20260804-12-gcfba38c6` |
| `.px4` 编译时间 | `2026-08-27T14:59:01+08:00` |
| `.px4` SHA-256 | `019728f8065d5787f61f6b99aac5f272d41d241fd873ac30d69ce98ab618c07d` |
| `log_181` SHA-256 | `7d7e189bad0850eaa0ee6072cdf7300862dbefbb25684a8f5427a2b109f27079` |
| `log_182` SHA-256 | `3e9d7a143611c90b7d6c7c78ef7be12433a5918357447020b130866d880a83ed` |

两个 ULog 内嵌的 `ver_sw` 都是上述完整固件提交，不是根据当前 checkout 或日志文件名推断。
原始 ULog 各约 16 MB，没有重复复制到 Git；绝对路径和校验值保存在
`analysis_manifest.json`，完整实飞参数分别保存在 `log_181_parameters.csv` 和
`log_182_parameters.csv`。

## 当前 OK 参数组合

| 参数 | 实飞值 | 说明 |
|---|---:|---|
| `HNTR_MASS` | 4.8 kg | 当前无电池整机质量 |
| `HNTR_CG_X` / `HNTR_CG_Z` | 0.013 / 0.0 m | 一级倾转轴到重心偏移 |
| `HNTR_ATT_KR_P` | 5.0 | Pitch 姿态 P |
| `HNTR_ATT_D_P` | 2.5 | Pitch 角速度阻尼 |
| `HNTR_ATT_I_P` | 0.0 | 本版本没有使用 Pitch I |
| `HNTR_PITCH_BIAS` | 0.07 | 尾电机参考推力归一化后的固定 Pitch trim |
| `HNTR_TAU_P` | 5.0 N m | Pitch 力矩限幅 |
| `HNTR_ACC_XY` | 6.0 m/s2 | 水平加速度/接触力上限 |
| `HNTR_RC_RATE_P` / `HNTR_RC_DB` | 6.0 deg/s / 0.08 | Position 手动 Pitch 目标生成 |
| `HNTR_L2` | 0.72 m | 尾电机推力线到重心的直接力臂 |
| `HNTR_TAIL_T_POS` / `HNTR_TAIL_T_NEG` | 12.78 / 6.04 N | 台架正反推力模型 |
| `HNTR_TAIL_REV_T` / `HNTR_T_REV_MIN` | 0.10 s / 0.02 N | 真换向保护 |
| `HNTR_T_REV_SLEW` / `HNTR_T_SLEW_DN` | 4.0 / 10.0 N/s | 反向建立和换向卸载速率 |
| `PWM_MAIN_MIN5` / `DIS5` / `MAX5` | 1000 / 1500 / 2000 us | 新尾电调标定 |

`HNTR_T_SLEW_UP=5` 仍为兼容参数，但提交 `cfba38c6` 后不再限制正常同向尾推力跟踪；
真换向的卸载、过零等待和反向建立保护仍保留。

## 两次飞行表现

### 任务和位置控制

- `log_181` 记录到 2 个 AUX4 高位任务窗口；`log_182` 记录到 5 个命令窗口。AUX4 窗口
  不等价于每次都完成了一次物理开门动作，操作者确认的成功次数应与外部程序日志联合判断。
- 任务段 X 误差 RMS 为约 `0.005--0.023 m`，Z 误差 RMS 为约 `0.010--0.040 m`。
- 最大水平力命令分别为 `15.93 N` 和 `19.40 N`，没有触及当前 4.8 kg、
  `HNTR_ACC_XY=6` 对应的 `28.8 N` 上限。
- 日志内可以看到外部轨迹在部分任务完成后明确反向，说明回退/回弹来自下发轨迹，不能归为
  PX4 自行漂移。TASK_BRAKE/RETURN 状态必须结合外部控制日志确认。

### Pitch、尾电机和目标连续性

- 任务段 Roll、Yaw 的 SO(3) 误差 RMS 大多低于约 `1.2 deg`，但 Pitch 误差 RMS 为
  `6.5--8.8 deg`，因此当前主要剩余误差仍在 Pitch 静态力矩平衡，而不是位置估计失效。
- 正常任务段尾推力中位约 `2.5--2.8 N`，尾 PWM 主要在约 `1628--1790 us`；正常任务
  没有换向，也没有出现持续尾推力限幅。
- 稳定长窗口仍有约 `1.5--1.6 Hz` 的小幅 Pitch 模态，1--2.2 Hz 角度带宽 RMS 约
  `0.30 deg`。目标同频分量接近零，说明它不是遥控目标主动周期摆动；相比旧日志已明显减弱，
  但还不能把 Pitch 内环称为完全调好。
- Position 模式记录到的 6 次 AUX2 松杆事件，目标最大相邻跳变不超过约 `0.11 deg`。
  旧版本 `log_169` 中的 `23--44 deg` 松杆锁存跳变已经消失。

### 剩余风险

- Position/Offboard 切换仍不是 bumpless transfer。两份日志的模式交接可产生约
  `6--10 deg` Pitch 目标变化；`log_182` 最后退出 Offboard 后 Pitch 角速度达到约
  `108 deg/s`，尾电机短时进入反向保护。它没有演化成失控，但不属于 OK 能力范围。
- `log_182` 末段 Pitch 力矩短时触及 `5 N m`。因此本记录不支持立即放大
  `HNTR_TAU_P` 或直接开展激进大角度测试。
- 电池剩余量字段没有有效递减，不能把日志内 `remaining=1` 当成真实续航证据；本次只用电压、
  电流和故障标志判断供电健康。

## 文件说明与复现

- `analysis_manifest.json`：ULog 路径、哈希、固件身份和 AUX4 窗口。
- `log_181_parameters.csv`、`log_182_parameters.csv`：每份 ULog 的全部 1182 个实飞初始参数。
- `event_summary.csv`：每个 AUX4 窗口的位置、姿态、水平力、尾推力和 PWM 统计。
- `position_release_summary.csv`：Position 模式 AUX2 松杆目标连续性。
- `mode_transition_summary.csv`：Position/Offboard 交接冲击。
- `oscillation_summary.csv`：稳定长窗口 Pitch 低频模态。
- `safety_summary.csv`：dropout、故障、供电、力矩、尾电机限幅和残差统计。
- `log_*_full_flight.png`：两次飞行分别绘制的全程图。
- `log_*_event_*.png`：每个任务窗口单独绘制，避免多个任务挤在同一张图上。
- `NEXT_6KG_PREFLIGHT.md`：换成 6 kg 构型后的首飞参数边界和验证顺序。

复现命令：

```bash
cd /home/hnuter/PX4-Hnuter/PX4-Autopilot-Hnuter
MPLCONFIGDIR=/tmp/hnuter-mpl-log181-182 \
python3 docs/flight_test_records/log_181_182_20260827_ok/analyze_logs_181_182.py
```
