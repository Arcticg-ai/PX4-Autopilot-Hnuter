# 54 N IEBC 接触实验复验记录

日期：2026-08-16
实验类型：PX4 SITL + Gazebo GUI + ROS 2 closed-loop IEBC
判定：**PASS（按当前力触发数字判据）**

## 实验配置

```text
virtual resistance       54.0 N, world -X
IEBC Emax                200.0 J
push reference rate      0.05 m/s
maximum push distance    4.5 m
maximum push time        110 s
cube mass                0.05 kg
release criterion        filtered contact force >= 54.0 N for 0.04 s
success criterion        force-triggered virtual-resistance release
displacement criterion   diagnostic only; not used for PASS/FAIL
Gazebo                    GUI enabled and retained after completion
```

运行入口：

```bash
HNUTER_GZ_HEADLESS=0 HNUTER_GZ_KEEP_OPEN=1 \
HNUTER_CUBE_FORCE_N=54.0 HNUTER_IEBC_E_MAX_J=200.0 \
HNUTER_CUBE_PUSH_MPS=0.05 HNUTER_CUBE_MAX_PUSH_M=4.5 \
HNUTER_CUBE_MAX_PUSH_TIME_S=110 \
HNUTER_IEBC_EXPERIMENT_VARIANT=closed_loop \
Tools/simulation/gz/run_hnuter_iebc_cube_contact.sh
```

## 结果

| 指标 | 本次结果 |
|---|---:|
| 触发时滤波接触力 | 55.414 N |
| 触发时原始接触力 | 53.915 N |
| 释放前原始力峰值 | 107.742 N |
| 全程原始力峰值 | 790.602 N |
| 滤波力峰值 | 56.474 N |
| PUSH 开始至释放 | 82.32 s |
| LOAD_SETTLE/PUSH 最大航向误差 | 2.543 deg |
| RELEASE_OBSERVE 最大航向误差 | 13.139 deg |
| IEBC 最小 barrier | 104.026 J |
| IEBC 最大 energy | 95.974 J |
| QP 最大 slack | 0.0 |
| 释放后整机峰值速度 | 3.579 m/s |
| 释放后整机峰值位移 | 4.273 m |
| 门体 X 向位移 | 3.997 m |

本次达到 54 N 持续滤波力阈值后，控制器立即撤销虚拟阻力，实验流程正常结束；QP 未使用松弛，IEBC barrier 始终为正，加载阶段也未触发 5 deg 航向几何门限。

## 数据与图

- 原始实验 CSV：`/home/hnuter/px4_ws_ros2/hnuter_logs/external_control/hnuter_iebc_cube_contact_closed_loop_1786874862.csv`
- PX4 ULog：`build/px4_sitl_default/rootfs/log/2026-08-16/10_07_40.ulg`（为保持 Gazebo 打开，PX4 会话仍在运行，最终文件哈希应在关闭会话后补录）
- TRO 风格矢量图：`force_threshold_54n_run_1786874862.pdf`
- TRO 风格 600 dpi 位图：`force_threshold_54n_run_1786874862.png`
- 可复现绘图脚本：`../plot_54n_contact_run.py`

曲线使用色觉友好配色；主线宽约 1.55--1.65 pt，辅助线宽约 1.1--1.35 pt，适合 IEEE TRO 双栏论文进一步排版。

## 解释边界

- 原始接触力包含 Gazebo 接触求解器的离散尖峰。图中主视窗将力轴限制在 72 N，并明确标注被裁剪的 107.7 N 释放前尖峰和 790.6 N 释放后碰撞尖峰；PASS 依据仍是低通力连续越阈值。
- 释放后的 3.579 m/s 前冲和 13.139 deg 航向误差说明瞬时撤载具有明显风险；本次只验证“达到并释放 54 N”，不代表释放过程安全。
- 这是仿真功能与容量证据，不是实机允许载荷，也不是 external-wrench 的严格能量证书。
- 实验 CSV 和图表已经封存并写入 `SOURCE_MANIFEST.sha256`；当前 ULog 仍由保留的 PX4 会话持有，因此不写入一个会随运行变化的伪最终哈希。
