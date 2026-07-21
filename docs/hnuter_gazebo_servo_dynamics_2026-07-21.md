# Hnuter 实测倾转动力学的 Gazebo 集成与闭环验证

## 1. 目的

将 `analysis/` 中由动捕辨识得到的两级倾转参数放到 Gazebo 被控对象中，使仿真舵机
不再瞬时到位。实机固件仍默认 `HNTR_TDYN_EN=0`；只有 `gz_hnuter` 的 POSIX airframe
默认启用同参数的控制分配状态估计，用于补偿仿真被控对象的动态。

## 2. 辨识参数

| 机构 | 方向 | 静态增益 | 纯延迟 (s) | 时间常数 (s) | 速度上限 (deg/s) |
| --- | --- | ---: | ---: | ---: | ---: |
| 一级 T1 | 正向 | 1.404 | 0.110 | 0.076 | 348.509 |
| 一级 T1 | 负向 | 1.423 | 0.106 | 0.065 | 310.391 |
| 二级 T2 | 正向 | 0.705 | 0.156 | 0.153 | 186.362 |
| 二级 T2 | 负向 | 0.695 | 0.137 | 0.149 | 165.338 |

T1 对应 `rj2/lj2`，T2 对应带 1:2 齿轮传动的 `rj1/lj1`。Gazebo 插件保留正负方向
差异；PX4 内部估计器使用两方向均值增益、均值时间常数和较保守的延迟/速度上限：

```text
T1: gain=1.414, delay=0.108 s, tau=0.071 s, rate=310.4 deg/s
T2: gain=0.700, delay=0.147 s, tau=0.151 s, rate=165.3 deg/s
```

## 3. 实现

- `HnuterServoDynamics` 订阅 `/model/<name>/servo_N`，依次执行纯延迟、方向相关静态
  增益、一阶惯性、速度限制和物理角度限制，再发布 `servo_N_dynamic`。
- 四个 Gazebo `JointPositionController` 改为订阅动态输出。插件对非有限命令、非有限
  仿真时间和仿真时间回跳做保护。
- 模型关节速度限制同步为一级负向最慢 `5.419 rad/s`、二级负向最慢
  `2.886 rad/s`。一级关节 PID 原有的 `+-1.57 rad` 隐含限制已改为机构范围
  `+-3.228859 rad`。
- SITL airframe 启用 `HNTR_TDYN_EN=1`，并设置与被控对象匹配的 T1/T2 参数。实机
  airframe 没有改为启用动态估计。
- 外部控制器新增 `HNUTER_CONTROL_MODE=px4_attitude`：同时发布位置和姿态期望，姿态
  输入使用 PX4 1.17 的 `/fmu/in/vehicle_attitude_setpoint_v1`。Arm 状态只以
  `VehicleControlMode.flag_armed` 为准，飞行后意外 Disarm 不会自动重新解锁。

## 4. 启动方法

为避免本机其他 ROS 2 节点抢占 OffboardControlMode，本次测试使用独立 DDS domain：

```sh
ROS_DOMAIN_ID=42 HEADLESS=1 make px4_sitl gz_hnuter
```

另一个终端：

```sh
source /home/hnuter/PX4-Autopilot-Hnuter/px4-venv/bin/activate
source /home/hnuter/px4_ws_ros2/install/setup.bash
ROS_DOMAIN_ID=42 HNUTER_CONTROL_MODE=px4_attitude \
  python3 /home/hnuter/px4_ws_ros2/hnuter_external_direct_controller_debug.py
```

按 `o` 起飞，按 `3` 执行平滑俯仰轨迹。测试配置为 30 deg、单程 6 s、峰值保持 1 s、
高度 1.5 m。

## 5. 闭环结果

日志：`build/px4_sitl_default/rootfs/log/2026-07-21/12_47_57.ulg`。

| 指标 | 较低水平环 | 调整后水平环 |
| --- | ---: | ---: |
| 俯仰峰值期望/实际 | 30.000/28.856 deg | 30.000/29.052 deg |
| 俯仰全段 RMSE | 3.055 deg | 3.057 deg |
| 水平位置最大误差 | 1.325 m | 0.595 m |
| 高度最大误差 | 0.023 m | 0.020 m |
| Roll 绝对峰值 | 1.804 deg | 3.728 deg |
| Yaw 最大误差 | 5.979 deg | 10.868 deg |

最终仿真默认水平参数为：

```text
HNTR_POS_P_XY=0.55
HNTR_VEL_P_XY=1.80
HNTR_VEL_I_XY=0.05
HNTR_VEL_D_XY=0.20
```

姿态默认采用 `KR_R/P/Y=4.0/4.0/1.2`、`D_R/P/Y=2.0/2.0/1.2` 和
`I_R/P/Y=0.15/0.40/0.08`。偏航 D 增益提高到 5.0 会因一级倾转延迟损失相位裕度，
出现约 30 deg 低频摆动；偏航积分提高到 0.25 也会放大大俯仰时的 roll/yaw 耦合，
因此均未固化。

## 6. 结论与边界

实测舵机动态已进入 Gazebo 被控对象，30 deg 俯仰和高度保持闭环通过；强制 Disarm
后控制器也能保持上锁，不会因落地检测状态变化自动重新解锁。
当前位置漂移不再主要由俯仰跟踪误差造成，而是位置纠偏力与姿态力矩共用两级倾转机构
时的瞬态分配残差。提高水平环能把位置误差降低约 55%，但会增加 roll/yaw 耦合。
进一步优化应优先记录或反馈四个实际关节角，并在控制分配中提高力/力矩任务的动态
优先级处理，而不是继续提高带延迟通道的姿态 D 增益。
