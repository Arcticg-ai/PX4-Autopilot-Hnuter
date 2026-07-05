# Hnuter 巨型球面滑行场景

## 文件

PX4 固件仓库：

- `Tools/simulation/gz/worlds/hnuter_sphere.sdf`

ROS 2 外部控制仓库 `~/px4_ws_ros2`：

- `hnuter_sphere_surface_controller.py`
- `hnuter_sphere_tuning.json`

## 场景布局

- 球心 ENU：`[14.0, 0.0, 11.5] m`
- 实体球半径：`10.0 m`
- 控制轨迹半径：`9.90 m`
- 飞机质心期望位置：深入实体球面 `0.10 m`
- Hnuter 碰撞网格最低点距机体原点约 `0.315 m`，因此机体底部与球面的
  指令包含约 `0.415 m` 的接触预载位移，由碰撞约束转换为持续法向压力
- 球体带实体碰撞、底部支座和沿滑行路径布置的彩色标记点
- 球面切向摩擦系数为 `0.02`，允许带法向预载的机体平滑滑动
- 起飞坪位于世界原点

控制器将机体参考原点的期望位置直接置于实体球内部。球体碰撞约束阻止飞机
到达该期望点，持续存在的径向位置误差由位置控制器转换为指向球心的法向力，
从而让机体外形持续压紧球面。球面模式禁用位置积分，避免不可达的球内期望
造成积分饱和；压紧力由比例和速度反馈产生。

## 轨迹

按 `o` 起飞并保持悬停，稳定后选择球面轨迹：

- 按 `1`：连续球面滑行。飞机先垂直爬升，再从球体上方接近球顶，随后沿
  右侧大圆弧往返滑动。机体 `+Z` 轴保持指向球面外法向，俯仰姿态从
  `0 deg` 平滑变化到 `+120 deg`。
- 按 `2`：多点贴附验证。飞机依次到达球面相对球顶
  `0/30/60/90/120 deg` 的五个位置。在每个位置先停在球外 `0.75 m`，
  再沿局部法向将期望位置压入球面，保持 `6 s`，随后退出并沿球外转移到
  下一点；完成五点后循环。
- 按 `3`：在球体下半部侧面贴附画圆。局部圆心位于球体 `+X` 侧、距球顶
  `105 deg` 的法向上，圆的球面角半径为 `10 deg`，因此整圈位于
  `95-115 deg` 极角范围，不再绕世界 Z 轴旋转。飞机从球顶沿球外转移时
  同步将机头转到圆周切向，到达后直接压紧球面并连续画圆，避免在侧下方
  悬空原地转向。画圆阶段机体 `+Z` 指向球面外法向，机头 `+X` 与瞬时
  速度方向一致。
- 按 `4`：执行原姿态角调试轨迹。

位置、速度、加速度和完整期望旋转矩阵均由五次平滑曲线生成，端点速度与
加速度连续。当前路线采用正俯仰分支；测试发现负俯仰接近 `-90 deg` 时，现有
direct 分配会出现 roll/yaw 耦合，因此未将该不稳定分支作为默认路线。
轨迹 3 可通过 `sphere_circle_center_colatitude_deg`、`sphere_circle_center_azimuth_deg`
和 `sphere_circle_radius_deg` 在线改变侧面圆的位置与尺寸。控制器约束圆心
极角为 `100-140 deg`，并限制圆半径，使整圈保持在赤道下方且不接近球底奇点。
轨迹 3 单独使用 `sphere_circle_theta_limit_deg=180`；其他模式继续使用
`theta_limit_deg=45`。

## 在线参数

`hnuter_sphere_tuning.json` 支持运行中更新：

```json
{
  "sphere_center_enu_m": [14.0, 0.0, 11.5],
  "sphere_radius_m": 10.0,
  "sphere_clearance_m": -0.1,
  "sphere_start_angle_deg": -270.0,
  "sphere_end_angle_deg": -390.0,
  "sphere_approach_time_s": 35.0,
  "sphere_attach_time_s": 5.0,
  "sphere_traverse_time_s": 50.0,
  "sphere_point_offsets_deg": [0.0, 30.0, 60.0, 90.0, 120.0],
  "sphere_point_standoff_m": 0.75,
  "sphere_point_press_time_s": 4.0,
  "sphere_point_hold_time_s": 6.0,
  "sphere_point_release_time_s": 4.0,
  "sphere_point_transfer_time_s": 10.0,
  "sphere_circle_center_colatitude_deg": 105.0,
  "sphere_circle_center_azimuth_deg": 0.0,
  "sphere_circle_radius_deg": 10.0,
  "sphere_circle_theta_limit_deg": 180.0,
  "sphere_circle_transfer_time_s": 25.0,
  "sphere_circle_orient_time_s": 20.0,
  "sphere_circle_press_time_s": 5.0,
  "sphere_circle_hold_time_s": 5.0,
  "sphere_circle_period_s": 90.0,
  "sphere_circle_ramp_time_s": 20.0
}
```

World 和控制器的球心、实体半径必须保持一致。

### 第二级倾转与验证状态

第二级倾转的 PX4 参数、SITL 舵机映射、Gazebo `rj1/lj1` 关节和外部控制器
归一化均已扩展为 `-180..180 deg`。逆解会在
`(alpha, theta)` 与 `(alpha + 180 deg, +/-180 deg - theta)` 两组等价解中
选择离上一帧最近的分支，使第二级可以连续穿过 `90 deg`，并在接近奇异区时
保持一级角连续。

SITL 已确认 Servo 2/3 映射为 `-3.142..+3.142 rad`，固件和 SITL 均编译通过。
`110 deg` 圆心、`15 deg` 半径的试验版本已经完成球顶转移、压紧并开始进入
侧面圆，但绕圈启动后发散。随后默认轨迹收缩为 `105 deg` 圆心、`10 deg`
半径、`90 s` 周期并增加分支连续性保护；该最终参数版本尚未完成闭环复测。

## 启动

终端 1，启动 DDS：

```bash
MicroXRCEAgent udp4 -p 8888
```

终端 2，启动带界面的 Gazebo：

```bash
cd ~/PX4-Hnuter/PX4-Autopilot-Hnuter
PX4_GZ_WORLD=hnuter_sphere make px4_sitl gz_hnuter
```

终端 3，启动控制器：

```bash
source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
cd ~/px4_ws_ros2
python3 hnuter_sphere_surface_controller.py
```

控制器启动后先按 `o` 起飞，悬停稳定后按 `1`、`2` 或 `3` 选择球面轨迹。

## 起飞偶发失控修复

排查时 `/fmu/out/vehicle_attitude` 同时存在两个 DDS 发布者，而本机
`/fmu/out/vehicle_local_position_v1` 只有一个发布者。两组姿态四元数被控制器
交替接收，表现为起飞前 yaw 在约 `-16 deg` 和 `-56 deg` 间随机切换，导致相同
起飞操作有时正常、有时乱飞。Micro XRCE-DDS Agent 日志确认本机只有一个 PX4
客户端，第二个姿态发布者来自局域网中的另一 DDS participant。

所有 Hnuter 外部控制器现在会在导入 `rclpy` 前将
`ROS_AUTOMATIC_DISCOVERY_RANGE` 强制设为 `LOCALHOST`，只接收本机 Agent 的
PX4 数据。验证命令：

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ros2 topic info -v /fmu/out/vehicle_attitude
```

正常情况下 `Publisher count` 应为 `1`。确实需要连接远程 DDS Agent 时，可设置
`HNUTER_ALLOW_REMOTE_DDS=1` 取消本机限制；此时应为不同飞行器配置不同的
`ROS_DOMAIN_ID`，避免 PX4 固定话题名相互串扰。

当前固件的 `VehicleStatus` 消息版本为 `1`，而 ROS 2 工作区原先使用版本 `4`。
两者会触发 `Fast CDR exception deserializing VehicleStatus`，导致控制器无法从
本机 `/fmu/out/vehicle_status_v1` 读取 `nav_state`。`src/px4_msgs` 子模块已固定到
PX4 官方提交 `b3f3064c6c210163522cab3fe4a29a5a000d1794`；该提交中的
`VehicleStatus.msg` 与当前固件文件 SHA-256 完全一致。五个控制器的订阅统一改为
`/fmu/out/vehicle_status_v1`。更新子模块后需要在 `~/px4_ws_ros2` 重新执行：

```bash
git submodule update --init --recursive
colcon build --packages-select px4_msgs --allow-overriding px4_msgs
source install/setup.bash
```

## SITL 验证

最终接触压力测试日志：
`~/px4_ws_ros2/hnuter_sphere_direct_1782818172.csv`

测试时仅将接近时间临时缩短到 `10 s`，附着/滑行仍使用 `5/50 s`。
完成后接近时间已恢复为 `35 s`。

- 目标最大连续俯仰：`120.0 deg`
- 实际最大连续俯仰：`116.71 deg`
- 目标大于等于 `115 deg` 时姿态 MAE：`3.31 deg`
- 有效滑行段机体原点到球面的平均距离：`0.231 m`
- 有效滑行段机体原点距离范围：`-0.033-0.443 m`
- 相对球内期望的平均径向误差：`0.331 m`，持续产生法向预载
- 最大归一化电机指令：`0.391`
- 全程保持 Armed 和 Offboard
- 无 `landed` 误报和 direct safety cutoff
- 低摩擦、零反弹球面配置下到达 `120 deg` 并正常反向滑行

## 球面几何连续性

球体的可见表面和碰撞表面使用不同表示：

- 可见表面使用 `meshes/hnuter_sphere_smooth.obj`
- 网格包含 `1986` 个顶点和 `3968` 个三角形
- 每个顶点使用解析球面法向，启用 smooth shading
- 经度/纬度细分为 `64 x 32`
- 物理碰撞继续使用 `<sphere><radius>10.0</radius></sphere>`

碰撞部分没有换成高面数 mesh。Gazebo/ODE 对 `<sphere>` 使用解析球形碰撞，
其法向和曲率在整个表面连续；将碰撞改成高面数网格反而会引入有限数量的三角
面、边和顶点，使接触法向发生离散跳变。

视觉网格可以重新生成：

```bash
python3 Tools/simulation/gz/worlds/generate_hnuter_sphere_mesh.py
```

摩擦、接触刚度和 world 的 `4 ms/250 Hz` 全局物理设置均保持原值。若解析球面
上仍出现物理跳动，离散几何来自 Hnuter 模型自身的 STL 碰撞网格，而不是球体。
这种情况应为球面场景创建专用的平滑机腹接触壳，不能通过继续增加球体碰撞
细分解决。

## 轨迹 3 全圆闭环与倾转限位

2026-07-03 将第二级倾转的固件参数、SITL 舵机映射、Gazebo 关节和控制器映射
统一扩展到 `-180 deg..+180 deg`。一级关节保留 `-185 deg..+185 deg` 机械裕度，
正常输入仍限制在 `-180 deg..+180 deg`。

严格要求机头始终沿圆周切向时，一级关节在圆周约 `34-40 deg` 处必须跨越
`+180/-180 deg`。两个等价倾转解之间只能在第二级恰好为 `+/-90 deg` 时无扰
切换；该轨迹的实际分配角约为 `-76 deg`，因此强制进入奇异位形会瞬间改变约
`6 N` 的水平推力分量。快速换支、慢速换支以及换支期间冻结轨迹都已在 SITL
验证，均会破坏姿态闭环。这是有限转角双轴机构的拓扑约束，不是控制增益或
ENU/NED 坐标错误。

控制器 `hnuter_sphere_surface_controller.py` 因此默认启用机械可实现航向：

- 环形位置轨迹、球面法向和贴附压力保持不变。
- 航向释放一个绕球面法向的自由度，避免一级关节绕转穿越硬限位。
- 强制奇异位形换支默认关闭；仅可通过
  `HNUTER_GIMBAL_BRANCH_TRANSFER=1` 进行实验。
- 连续旋转一级关节硬件可通过 `HNUTER_CIRCLE_HEADING_RELIEF=0` 恢复严格切向
  机头；有限 `+/-180 deg` 硬限位机构不应使用该模式跑完整圆周。

最终闭环使用：

```bash
PX4_GZ_WORLD=hnuter_sphere HEADLESS=1 make px4_sitl gz_hnuter

source ~/PX4-Autopilot-Hnuter/px4-venv/bin/activate
python3 ~/px4_ws_ros2/hnuter_sphere_surface_controller.py
```

按 `o` 起飞，悬停后按 `3`。验证日志：
`~/px4_ws_ros2/hnuter_sphere_direct_1783092838.csv`。

一整圈 `90 s` 的统计结果：

- 位置误差 RMS `0.460 m`，峰值 `0.524 m`
- SO(3) 姿态误差 RMS `7.07 deg`，峰值 `8.34 deg`
- 最大空间速度 `0.182 m/s`
- 一级角范围：`alpha1 97.24..117.07 deg`，
  `alpha2 98.64..123.52 deg`
- 二级角范围：`theta1 -22.21..-16.28 deg`，
  `theta2 -26.46..-19.34 deg`
- `direct_safety_cutoff=0`，无 Offboard 中断，无空中 Disarm

测试在完成一圈后继续进入第二圈，仍保持稳定，随后由测试者主动终止。
