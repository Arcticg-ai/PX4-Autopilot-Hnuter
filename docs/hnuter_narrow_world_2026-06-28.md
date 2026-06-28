# Hnuter 狭窄空间仿真场景与控制任务

## 修改范围

- PX4 world：`Tools/simulation/gz/worlds/hnuter_narrow.sdf`
- ROS2 控制器：`~/px4_ws_ros2/hnuter_narrow_passage_controller.py`
- 本次未修改 PX4 参数、机型控制分配或实机固件配置。

## 障碍物布局

六个门均为包含碰撞体的完整四边框，沿 Gazebo `+X` 方向组成明显的 S 形路线。

| 门 | 中心 `(x, y)` m | 内宽 m | 开口高度范围 m | 飞行目标高度 m |
| --- | --- | --- | --- | --- |
| 1 | `(6.0, 0.0)` | 2.6 | `0.20 - 1.95` | 1.05 |
| 2 | `(11.0, 2.2)` | 2.4 | `0.20 - 1.65` | 0.85 |
| 3 | `(16.5, -2.2)` | 2.4 | `0.45 - 2.25` | 1.30 |
| 4 | `(22.0, 2.4)` | 2.4 | `0.20 - 1.60` | 0.85 |
| 5 | `(27.5, -2.4)` | 2.4 | `0.50 - 2.20` | 1.30 |
| 6 | `(33.0, 1.2)` | 2.3 | `0.25 - 1.85` | 1.05 |

第 3、5 门抬高底边，要求飞机同时完成横向转移和高度变化。相邻两侧门中心的最大横向跨度为 4.8 m。

## 控制任务

- 每个门前增加转向点和对准点。
- 修正 ENU/NED 偏航转换：Gazebo `+X` 对应 PX4 NED `90 deg`，默认偏航偏置由错误的 `90 deg` 改为 `0 deg`。
- 横移过程中机头根据轨迹速度切线连续转动，避免飞机保持固定偏航并侧向飞行。
- 到达对准点后恢复正对 Gazebo `+X`，从门心直线穿过，再进入下一次转向。
- 25 个航点改为三次 Hermite 曲线控制点，位置和速度连续；除起飞和终点外不再在控制点停车。
- 默认启用速度和加速度前馈，门内速度约为 `0.60 - 0.68 m/s`，转场速度约为 `0.82 - 0.90 m/s`。
- 默认轨迹时长约 66 s，完成后在 `(37.0, 0.0, 1.05)` m 悬停并自动着陆。

相关环境变量：

| 变量 | 默认值 | 功能 |
| --- | --- | --- |
| `HNUTER_NARROW_YAW_ENU_OFFSET_DEG` | `0` | 机体模型与 Gazebo ENU 航向之间的附加偏置 |
| `HNUTER_NARROW_FEEDFORWARD` | `true` | 向 PX4 发送轨迹速度和加速度前馈 |
| `HNUTER_NARROW_SPEED_SCALE` | `1.0` | 整体轨迹速度倍率 |
| `HNUTER_NARROW_AUTO_LAND` | `true` | 完成任务后自动着陆 |

## 启动方法

```bash
MicroXRCEAgent udp4 -p 8888
```

带 Gazebo 界面启动：

```bash
PX4_GZ_WORLD=hnuter_narrow make px4_sitl gz_hnuter
```

无界面启动：

```bash
HEADLESS=1 PX4_GZ_WORLD=hnuter_narrow make px4_sitl gz_hnuter
```

启动控制器：

```bash
source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
python3 ~/px4_ws_ros2/hnuter_narrow_passage_controller.py
```

## 验证结果

2026-06-28 使用 `make px4_sitl gz_hnuter`、Micro XRCE-DDS Agent 和连续轨迹控制器完成全程验证：

- PX4 SITL 成功加载 `hnuter_narrow` world 和 `hnuter_0`。
- 六个门全部通过，包括第 3、5 个抬高底边门。
- 全程保持 Offboard 和 Armed，未发生碰撞、姿态发散或 DDS 中断。
- 连续飞行段约 66 s，过程中没有航点停车。
- 实际位置跟踪误差大部分为 `0.05 - 0.19 m`。
- ULog 中直穿门的偏航期望为 NED `90 deg`，实际 heading 平均约 `101.7 deg`；原有 `90 deg` 坐标偏差已消除，剩余约 `12 deg` 为快速轨迹下的动态跟踪偏差。
- 终点实际位置约为 `(37.01, -0.02, 1.05)` m，随后自动着陆并上锁。
