# 三种 6S 电池实飞对比与建议参数（2026-08-28）

## 结论

本记录比较三种电池在同一架 Hnuter 飞行器上的实飞表现：

- 22 Ah 半固态电池，1.4 kg，标称 10C：续航最长，但本次没有从明确的满电状态飞到标称容量，且 40–50 A 负载下压降最大。现阶段只能确认约 9–10 min 的保守任务时间，不能据此确认可用 22 Ah。
- 格氏 8 Ah LiPo，1.0 kg：三者中负载能力最好、等效内阻最低，实飞约 10.1 min，适合大水平力或较激进动作。正常任务按 8.5–9 min 规划。
- 21700 组成的 6S 8.4 Ah 电池，0.9 kg：最轻，但高负载后段电压下降更早；适合轻载和续航优先任务，不适合在低电量阶段持续大水平力。正常任务按 6–6.5 min 规划。

下面的控制参数是基于当前日志得到的“下一次实飞推荐起点”，不是已经完成全部包线验证的最终最优值。三块电池的质量和悬停前馈必须分别设置；Pitch 参数只作小幅区分，首次验证应先做水平悬停和小角度阶跃，再测试大角度。

## 统一计算口径

- 仅累计 `vehicle_land_detected.landed == false` 的空中样本。
- 容量由电流积分得到，能量由电压乘电流积分得到。
- 半固态电池的 log184 与 log185 属于同一次连续放电，按时间先后拼接，不能把 log185 当成另一块满电电池。
- 汇总表的峰值和极值使用 ULog 原始采样；对比曲线使用 1 Hz 重采样，电压和电流曲线再作 5 s 移动平均。
- 三个电池总质量按无电池机体质量 4.8 kg 计算。如果当前实测机体质量变化，应优先使用秤重结果。

## 实飞电气表现

| 电池 | 空中时间 | 空中容量 | 空中能量 | 平均/峰值电流 | 最低负载电压 | 在线估计整包内阻中位数 |
|---|---:|---:|---:|---:|---:|---:|
| 22 Ah 半固态 | 11.39 min | 8756.5 mAh | 183.59 Wh | 46.13 / 58.02 A | 18.916 V | 62.76 mΩ |
| 格氏 8 Ah LiPo | 10.11 min | 7190.3 mAh | 153.73 Wh | 42.66 / 50.72 A | 19.615 V | 43.78 mΩ |
| 21700 8.4 Ah | 7.75 min | 5290.8 mAh | 112.94 Wh | 40.96 / 46.93 A | 19.072 V | 53.78 mΩ |

半固态电池平均和峰值放电倍率约为 2.10C 和 2.64C，远低于标称 10C，因此日志没有触及标称倍率上限。它的电压表现更可能受初始荷电状态、整包内阻、连接线/BMS 和具体电芯放电曲线共同影响，不能仅凭“10C”判断一定比 LiPo 稳。

## 三套建议参数

### 1. 22 Ah 半固态电池，1.4 kg

```text
param set HNTR_MASS 6.2
param set HNTR_MOT_HOV 0.58

param set HNTR_ATT_KR_P 4.2
param set HNTR_ATT_D_P 1.7
param set HNTR_ATT_I_P 0.05
param set HNTR_ATT_ILIM_P 1.0
param set HNTR_PITCH_BIAS 0.10
param set HNTR_TAU_P 8.0

param set BAT1_N_CELLS 6
param set BAT1_CAPACITY 10000
param set BAT1_V_CHARGED 4.12
param set BAT1_V_EMPTY 3.60
param set BAT1_R_INTERNAL -1
param save
reboot
```

`BAT1_CAPACITY=10000` 是按当前负载下已验证区间给出的保守“任务可用容量”，并不是说电池物理容量只有 10 Ah。完整充电后必须再做一次 40–50 A 工况的放电验证；验证前不要直接填 22000，否则剩余电量显示可能过于乐观。

### 2. 格氏 8 Ah LiPo，1.0 kg

```text
param set HNTR_MASS 5.8
param set HNTR_MOT_HOV 0.58

param set HNTR_ATT_KR_P 4.2
param set HNTR_ATT_D_P 1.5
param set HNTR_ATT_I_P 0.05
param set HNTR_ATT_ILIM_P 1.0
param set HNTR_PITCH_BIAS 0.10
param set HNTR_TAU_P 8.0

param set BAT1_N_CELLS 6
param set BAT1_CAPACITY 7200
param set BAT1_V_CHARGED 4.18
param set BAT1_V_EMPTY 3.60
param set BAT1_R_INTERNAL -1
param save
reboot
```

这块电池在本次日志中是三者里最适合高负载任务的基准电池。`7200 mAh` 对应标称容量的约 90%，也与本次实际空中积分 7190 mAh 一致。

### 3. 21700 6S 8.4 Ah 电池，0.9 kg

```text
param set HNTR_MASS 5.7
param set HNTR_MOT_HOV 0.57

param set HNTR_ATT_KR_P 4.5
param set HNTR_ATT_D_P 1.5
param set HNTR_ATT_I_P 0.05
param set HNTR_ATT_ILIM_P 1.0
param set HNTR_PITCH_BIAS 0.10
param set HNTR_TAU_P 8.0

param set BAT1_N_CELLS 6
param set BAT1_CAPACITY 5600
param set BAT1_V_CHARGED 4.15
param set BAT1_V_EMPTY 3.60
param set BAT1_R_INTERNAL -1
param save
reboot
```

`5600 mAh` 是当前约 41 A 平均负载下的保守可用容量。标称 8400 mAh 多半来自较低倍率测试；在当前飞行电流下直接填 8400 会高估低电量阶段的可用时间。

## 参数解释和边界

- `HNTR_MASS` 必须随电池总质量变化，它参与力和重力前馈计算。
- `HNTR_MOT_HOV` 是悬停推力前馈，不应靠增大 Z 位置增益来补偿电池重量。建议值需要在稳定悬停段继续用实际油门微调，步长不超过 0.01。
- `HNTR_ATT_I_P=0.05` 只用于缓慢消除稳态 Pitch 误差；如果出现几十秒尺度的周期性摆动，先退回 0，而不是继续增加积分。
- `HNTR_TAU_P=8` 已高于这些日志中的实际 Pitch 力矩使用区间。把它提高到 10 只会放宽限幅，并不会自动增加阻尼；应在确认 8 N·m 确实持续饱和后再提高。
- `HNTR_PITCH_BIAS=0.10` 是当前日志条件下的起点。安装位置改变后应先实测带电池的重心并设置 `HNTR_CG_X/Z`，再微调 Bias，不能用 Bias 长期掩盖错误的重心参数。
- `BAT1_R_INTERNAL=-1` 保留 PX4 在线内阻估计。本次估计结果与负载压降趋势相符，不建议手填固定内阻。
- `BAT1_V_EMPTY=3.60 V/cell` 保持保守值，不应为了延长显示续航而降低。
- 电池型号切换后，在地面执行 `param save` 并重启一次，确保电池估算器和 Hnuter 控制器均从同一套参数开始。

## 曲线

- [电压对比](01_voltage_comparison.png)
- [电流对比](02_current_comparison.png)
- [累计容量](03_capacity_comparison.png)
- [累计能量](04_energy_comparison.png)
- [总质量—平均电流预测](05_mass_current_prediction.png)
- [质量增加但容量不变的续航预测](06_mass_endurance_prediction.png)
- [电池质量和容量同时增加的续航预测](07_mass_endurance_capacity_scaling_prediction.png)
- [按铭牌容量 90% 可用计算的理论上限](08_mass_endurance_nameplate_upper_bound.png)

半固态电压曲线约 4 min 的上跳是 log184 与 log185 的拼接边界；各曲线末端接近零电流的短段是降落/空中状态切段，不代表飞行中突然失去负载。

## 质量、电流和续航预测

多旋翼悬停诱导功率在其他条件不变时近似与总质量的 `1.5` 次方成正比。使用三块电池对应的实测总质量和空中平均电流标定后得到：

```text
平均空中电流 I(A) = 3.0153 × 总质量 m(kg)^1.5
三点拟合 R² = 0.9657
```

### 场景一：增加普通载荷，电池容量不变

该续航预测保持每一种电池在日志中已经观察到的可用能量不变：

```text
t_i(m) = 实测续航_i × (实测质量_i / m)^1.5
```

| 总质量 | 预测平均电流 | 半固态预测续航 | LiPo预测续航 | 21700预测续航 |
|---:|---:|---:|---:|---:|
| 5.0 kg | 33.71 A | 15.73 min | 12.63 min | 9.43 min |
| 5.5 kg | 38.89 A | 13.63 min | 10.95 min | 8.18 min |
| 5.8 kg | 42.12 A | 12.59 min | 10.11 min | 7.55 min |
| 6.0 kg | 44.32 A | 11.96 min | 9.61 min | 7.18 min |
| 6.2 kg | 46.55 A | 11.39 min | 9.15 min | 6.83 min |
| 6.5 kg | 49.97 A | 10.61 min | 8.52 min | 6.36 min |

在 6 kg 附近，每增加 0.5 kg，平均电流预计增加约 5.5 A，续航预计减少约 11%–13%。图中电流给出 ±10%、续航给出 ±15% 的工程不确定带。

只有 5.7–6.2 kg 是当前实测标定范围，区间外属于外推。表中例如“5.0 kg 使用半固态电池”只是保持电池不变、假设机体或其他载荷减重后的数学结果；当前 4.8 kg 机体加 1.4 kg 半固态电池实际不可能达到 5.0 kg。快速平移、持续接触、大姿态、低电压和风扰都会让实际电流高于悬停型预测，并缩短续航。

### 场景二：电池质量和容量同时增加

当前无电池机体质量取 `4.8 kg`。假设增加的是相同化学体系、相同电压的电池并联容量，单位质量可用能量保持不变：

```text
总质量 M = 4.8 kg + 电池质量 mb
电池可用容量 ∝ mb
平均电流 ∝ M^1.5
续航 t ∝ mb / M^1.5
```

每种电池使用自身日志中的单位质量可用能量标定，得到：

| 电池质量 | 总质量 | 平均电流 | 半固态续航 | LiPo续航 | 21700续航 |
|---:|---:|---:|---:|---:|---:|
| 0.9 kg | 5.7 kg | 41.03 A | 8.31 min | 9.34 min | 7.75 min |
| 1.0 kg | 5.8 kg | 42.12 A | 8.99 min | 10.11 min | 8.39 min |
| 1.4 kg | 6.2 kg | 46.55 A | 11.39 min | 12.81 min | 10.63 min |
| 1.8 kg | 6.6 kg | 51.13 A | 13.33 min | 15.00 min | 12.44 min |
| 2.2 kg | 7.0 kg | 55.84 A | 14.92 min | 16.78 min | 13.92 min |

在这个假设下，当前 0.5–2.2 kg 电池范围内，增加电池仍会增加续航，但收益逐渐变小。例如 LiPo 从 1.0 kg 增加到 1.4 kg，电池质量增加 40%，预测续航只从 10.11 min 增加到 12.81 min，约增加 26.7%；增加到 2.2 kg 后，平均电流已经接近 56 A。

按本次日志实际消耗能量计算的单位质量可用能量分别约为：LiPo `153.7 Wh/kg`、半固态 `131.1 Wh/kg`、21700 `125.5 Wh/kg`，所以相同电池质量下 LiPo 预测续航最高。半固态日志并非明确从满电完整放电，它的曲线可能偏保守；只有补充一次真正满充到安全截止的完整飞行或台架数据后，才能体现其标称 22 Ah 的优势。

作为对照，如果假定每块电池都能在当前飞行负载下释放铭牌容量的 90%，原尺寸电池的理论上限为：

| 原尺寸电池 | 实测续航 | 铭牌 90% 理论上限 |
|---|---:|---:|
| 22 Ah 半固态，1.4 kg | 11.39 min | 25.75 min |
| 8 Ah LiPo，1.0 kg | 10.11 min | 10.13 min |
| 8.4 Ah 21700，0.9 kg | 7.75 min | 11.07 min |

其中 LiPo 的实测已经接近铭牌 90%，可信度最高；21700 在约 41 A 下只能使用部分标称容量；半固态理论值与实测相差过大，只能作为乐观上界，不能作为返航时间。

`7.0 kg` 已超出当前实测质量范围。继续增加电池前必须检查主电机平均 PWM、最大推力裕度、结构载荷以及起飞阶段控制裕度；不能只根据数学曲线判断可以安全飞行。

## 可复现文件

- [汇总统计](battery_comparison_summary.csv)
- [推荐参数表](recommended_parameters.csv)
- [1 Hz 对比数据](battery_profiles_1hz.csv)
- [质量—性能预测表](mass_performance_prediction.csv)
- [容量随电池质量增加的预测表](capacity_scaling_prediction.csv)
- [质量预测模型说明](mass_prediction_model.txt)
- [源日志哈希与固件信息](analysis_manifest.json)
- [分析脚本](analyze_batteries.py)
- [质量预测脚本](predict_mass_performance.py)

四份日志都记录为固件提交 `cfba38c68e403dcc63ee3e5d90e3b238f8918afd`，且无 ULog dropout。日志内存在参数修改记录，因此电气对比有效，但不能把三架次直接当成只改变电池的严格 Pitch A/B 试验。
