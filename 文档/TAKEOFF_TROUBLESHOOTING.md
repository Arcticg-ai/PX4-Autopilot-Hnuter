# Takeoff 无响应问题诊断指南

## 问题描述

执行 `commander takeoff` 后，Gazebo 中的无人机：
- ❌ 没有怠速解锁
- ❌ 没有起飞
- ❌ ROS2 话题 `/fmu/out/actuator_outputs` 无数据

## 快速诊断步骤

### 步骤 1: 运行自动诊断脚本

```bash
cd ~/px4_ros2_ws
./full_system_check.sh
```

这个脚本会自动检查：
- PX4 和 Gazebo 进程状态
- ROS2 环境和话题
- 数据流状态
- Gazebo 话题状态

### 步骤 2: 在 PX4 控制台检查模块状态

在 PX4 控制台 (nsh>) 执行以下命令：

#### 2.1 检查 uxrce_dds_client 状态

```bash
uxrce_dds_client status
```

**预期输出**:
```
INFO  [uxrce_dds_client] Running
INFO  [uxrce_dds_client] Session: 1
INFO  [uxrce_dds_client] Configured: yes
INFO  [uxrce_dds_client] Connected: yes
```

**如果未连接**:
```bash
uxrce_dds_client stop
uxrce_dds_client start -t udp -p 8888
```

#### 2.2 检查 gz_bridge 状态

```bash
gz_bridge status
```

**预期输出**:
```
INFO  [gz_bridge] Running
INFO  [gz_bridge] Model: hnuter
```

**如果未运行或未连接**:
```bash
gz_bridge stop
gz_bridge start -m hnuter -p gz_bridge
```

#### 2.3 检查解锁状态

```bash
commander status
```

**关键信息**:
- `arming_state`: 应为 `STANDBY` (未解锁) 或 `ARMED` (已解锁)
- `nav_state`: 应为有效状态
- `failsafe`: 应为 `false`

**如果显示解锁失败原因**，按提示解决。

#### 2.4 检查 actuator_motors 数据

```bash
listener actuator_motors
```

**如果解锁后应该看到**:
```
control: [0.000000, 0.000000, 0.000000, 0.000000, 0.000000]
reversible_flags: 0
```

**如果起飞后应该看到**:
```
control: [0.350000, 0.350000, 0.350000, 0.350000, 0.350000]
```

**如果无数据**:
- 说明控制分配器未输出，检查是否已解锁
- 或控制器未运行

### 步骤 3: 检查关键参数

```bash
# 控制分配器配置
param show CA_AIRFRAME    # 应该是 16
param show CA_ROTOR_COUNT # 应该是 5

# Gazebo 电机映射
param show SIM_GZ_EC_FUNC1  # 应该是 101
param show SIM_GZ_EC_FUNC2  # 应该是 102
param show SIM_GZ_EC_FUNC3  # 应该是 103
param show SIM_GZ_EC_FUNC4  # 应该是 104
param show SIM_GZ_EC_FUNC5  # 应该是 105

# Gazebo 使能
param show SIM_GZ_EN  # 应该是 1

# DDS 配置
param show UXRCE_DDS_CFG  # 不应该是 -1
```

## 常见问题和解决方案

### 问题 1: /fmu 话题不存在

**症状**: `ros2 topic list | grep fmu` 无输出

**原因**: uxrce_dds_client 未启动或未连接

**解决方案**:

在 PX4 控制台:
```bash
uxrce_dds_client stop
uxrce_dds_client start -t udp -p 8888
```

等待 2-3 秒，然后检查:
```bash
uxrce_dds_client status
```

应该显示 `Connected: yes`

### 问题 2: /fmu/out/actuator_motors 无数据

**症状**: 话题存在但 `ros2 topic echo /fmu/out/actuator_motors` 无输出

**可能原因**:
1. 未解锁
2. 控制分配器未运行
3. 参数配置错误

**解决方案**:

#### 方案 A: 检查解锁状态
```bash
commander status
```

如果 `arming_state` 不是 `ARMED`:
```bash
commander arm
```

如果解锁失败，使用强制解锁:
```bash
commander arm -f
```

#### 方案 B: 检查控制分配器
```bash
control_allocator status
```

应该显示 `Running`

#### 方案 C: 检查内部 uORB 数据流
```bash
listener actuator_motors
```

如果有数据，说明 PX4 内部正常，问题在 DDS 桥接。
如果无数据，说明控制器未输出。

### 问题 3: Gazebo 电机话题无数据

**症状**: PX4 输出 actuator_motors，但 Gazebo 中 `/hnuter/command/motor_speed` 无数据

**原因**: gz_bridge 未正确转发

**解决方案**:

检查 gz_bridge 状态:
```bash
gz_bridge status
```

如果未运行或模型不匹配:
```bash
gz_bridge stop
gz_bridge start -m hnuter -p gz_bridge
```

检查 Gazebo 话题:
```bash
gz topic -l | grep hnuter
```

应该看到:
```
/hnuter/command/motor_speed
/model/hnuter/joint_state
/model/hnuter/pose
...
```

监听电机命令:
```bash
gz topic -e -t /hnuter/command/motor_speed
```

### 问题 4: 解锁失败

**症状**: `commander arm` 返回错误

**常见原因和解决方案**:

#### 原因 A: EKF 未收敛
```
PREFLIGHT FAIL: EKF INTERNAL CHECKS
```

**解决**: 等待 5-10 秒让 EKF 收敛，或:
```bash
param set COM_ARM_EKF_AB 0.005
param set COM_ARM_EKF_GB 0.002
```

#### 原因 B: GPS 未锁定
```
PREFLIGHT FAIL: GLOBAL POSITION INVALID
```

**解决**:
```bash
param set COM_ARM_WO_GPS 1
```

#### 原因 C: 电池检查
```
PREFLIGHT FAIL: BATTERY
```

**解决**:
```bash
param set CBRK_SUPPLY_CHK 894281
```

#### 原因 D: 模式切换
某些模式不允许解锁

**解决**: 切换到 MANUAL 或 POSCTL 模式

### 问题 5: 解锁成功但电机无输出

**症状**: `commander status` 显示 `ARMED`，但 `listener actuator_motors` 无数据

**原因**: 控制器未生成指令

**解决方案**:

检查飞行模式:
```bash
commander status
```

查看 `nav_state`，确保不是 `MANUAL` 模式（手动模式下需要遥控器输入）

尝试切换模式:
```bash
commander mode posctl  # 位置模式
```

或直接起飞:
```bash
commander takeoff
```

## 完整测试流程

### 流程 A: 标准测试

```bash
# 1. 在终端 1: 启动仿真
cd ~/PX4-Autopilot-Hnuter
make px4_sitl gz_hnuter

# 2. 等待启动完成（看到 nsh> 提示符）

# 3. 在终端 2: 启动监控
cd ~/px4_ros2_ws
python3 monitor_gazebo_motors.py

# 4. 在 PX4 控制台: 检查状态
uxrce_dds_client status
gz_bridge status
commander status

# 5. 在 PX4 控制台: 解锁
commander arm

# 6. 观察监控终端，应该看到怠速输出（~0.1）

# 7. 在 PX4 控制台: 起飞
commander takeoff

# 8. 观察监控终端，应该看到电机输出增加（~0.3-0.5）
```

### 流程 B: 强制解锁测试

如果标准流程解锁失败：

```bash
# 1. 强制解锁
commander arm -f

# 2. 立即检查输出
listener actuator_motors

# 3. 如果有输出，尝试起飞
commander takeoff
```

## 数据流检查清单

使用此清单系统地检查数据流：

```
□ PX4 进程运行
□ Gazebo 进程运行
□ uxrce_dds_client 已连接
□ gz_bridge 已连接
□ ROS2 话题 /fmu/out/actuator_motors 存在
□ PX4 内部 actuator_motors 有数据
□ Gazebo 话题 /hnuter/command/motor_speed 存在
□ Gazebo 收到电机命令
□ 解锁成功 (arming_state = ARMED)
□ 无故障保护激活 (failsafe = false)
```

## 诊断工具使用

### 工具 1: full_system_check.sh

```bash
cd ~/px4_ros2_ws
./full_system_check.sh
```

自动检查所有系统状态，生成诊断报告。

### 工具 2: monitor_gazebo_motors.py

```bash
cd ~/px4_ros2_ws
python3 monitor_gazebo_motors.py
```

实时显示：
- PX4 执行器输出
- Gazebo 电机命令
- 数据流状态

### 工具 3: px4_console_commands.txt

```bash
cat ~/px4_ros2_ws/px4_console_commands.txt
```

包含所有 PX4 控制台诊断命令，可以复制粘贴执行。

## 高级调试

### 启用详细日志

在 PX4 控制台：

```bash
# 启用 gz_bridge 详细日志
gz_bridge stop
gz_bridge start -m hnuter -p gz_bridge -d

# 启用 control_allocator 详细日志
control_allocator stop
control_allocator start -d
```

### 手动测试执行器

如果怀疑执行器映射问题，使用测试脚本：

```bash
cd ~/px4_ros2_ws
python3 test_actuators.py
```

这会直接发送执行器命令，绕过控制器。

### 检查 Gazebo 物理

在 Gazebo GUI 中：
1. 右键点击 hnuter 模型
2. 选择 "View" → "Transparent"
3. 检查关节是否正确连接
4. 查看 "Force Torque" 插件输出

## 联系支持

如果问题仍未解决，收集以下信息：

1. 诊断脚本输出：
   ```bash
   ./full_system_check.sh > diagnosis.txt 2>&1
   ```

2. PX4 控制台输出
3. 监控工具截图
4. 参数导出：
   ```bash
   param show -a > params.txt
   ```

然后查看相关文档：
- `GAZEBO_MOTOR_FIX.md` - 电机问题修复
- `HNUTER_GAZEBO_TESTING_GUIDE.md` - 测试指南
- `PX4_TAKEOFF_COMMAND_FLOW.md` - Takeoff 流程详解

---

**文档创建**: 2026-02-24
**适用版本**: PX4 v1.14+, Gazebo Harmonic
**机型**: Hnuter Tiltrotor
