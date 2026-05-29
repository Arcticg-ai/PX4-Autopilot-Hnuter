# Hnuter Gazebo 仿真完整测试指南

## 问题修复总结

### ✅ 已修复的问题

#### 1. Gazebo 模型电机话题路径错误
**问题**: 电机插件使用了错误的绝对路径 `/hnuter_0/command/motor_speed`
**修复**: 改为相对路径 `command/motor_speed`
**文件**: `Tools/simulation/gz/models/hnuter/model.sdf`

#### 2. ROS2 测试脚本字段名错误
**问题**: `OffboardControlMode.actuator` 字段不存在
**修复**: 改为 `OffboardControlMode.direct_actuator`
**文件**: `~/px4_ros2_ws/test_actuators.py`

#### 3. 机型配置文件检查
**状态**: 配置正确，但倾转舵机当前禁用
**文件**: `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`

---

## 快速验证

运行验证脚本检查所有修复是否正确：

```bash
cd ~/px4_ros2_ws
./verify_motor_fix.sh
```

---

## 测试流程

### 第一步: 重新编译

```bash
cd ~/PX4-Autopilot-Hnuter
make clean
make px4_sitl gz_hnuter
```

**预期输出**:
- PX4 启动成功
- Gazebo 窗口打开
- hnuter 模型加载

### 第二步: 检查 Gazebo 话题（新终端）

```bash
# 列出所有 hnuter 相关话题
gz topic -l | grep hnuter

# 应该看到:
# /hnuter/command/motor_speed  ✅ (不是 /hnuter_0/...)
# /model/hnuter/servo_0        ✅
# /model/hnuter/servo_1        ✅
# /model/hnuter/servo_2        ✅
# /model/hnuter/servo_3        ✅
```

### 第三步: 启动实时监控（新终端）

```bash
cd ~/px4_ros2_ws
python3 monitor_gazebo_motors.py
```

这会显示:
- PX4 执行器输出
- Gazebo 电机速度命令
- 舵机命令

### 第四步: 测试解锁

有两种方法:

#### 方法A: PX4 控制台

在 PX4 SITL 终端输入:
```bash
commander arm
```

#### 方法B: QGroundControl

1. 启动 QGC
2. 连接到 PX4 (自动连接 UDP 14550)
3. 点击工具栏的 "解锁" 按钮

### 第五步: 观察结果

**正常情况应该看到**:

1. **PX4 控制台**:
   ```
   INFO  [commander] Armed by user command
   INFO  [gz_bridge] Publishing motor commands
   ```

2. **监控脚本**:
   ```
   [1] PX4 执行器输出:
   output: [xxx, xxx, xxx, xxx, xxx, ...]

   [2] Gazebo 电机速度命令:
   velocity: [xxx, xxx, xxx, xxx, xxx]
   ```

3. **Gazebo 窗口**:
   - 螺旋桨开始旋转
   - 怠速时缓慢旋转
   - 可以听到物理引擎声音

### 第六步: 测试起飞

在 PX4 控制台或 QGC:
```bash
commander takeoff
```

**预期**:
- 电机加速旋转
- 无人机开始升空
- 高度逐渐增加

---

## 诊断工具

### 1. check_gazebo_topics.py
检查所有话题和连接状态

```bash
cd ~/px4_ros2_ws
python3 check_gazebo_topics.py
```

**输出内容**:
- Gazebo 话题列表
- 电机和舵机话题
- PX4 执行器输出
- 模型信息

### 2. monitor_gazebo_motors.py
实时监控数据流

```bash
cd ~/px4_ros2_ws
python3 monitor_gazebo_motors.py
```

**显示**:
- 实时 actuator_outputs
- 实时电机速度命令
- 实时舵机位置

### 3. test_actuators.py
直接测试执行器

```bash
cd ~/px4_ros2_ws
python3 test_actuators.py
```

**测试序列**:
1. 切换到 Offboard 模式
2. 解锁
3. 低速电机 (20%)
4. 中速电机 (40%)
5. 高速电机 (60%)
6. 舵机测试 (-0.5, 0, 0.5)
7. 停止并上锁

---

## 故障排除

### 问题1: 电机仍然不转

**检查点**:
```bash
# 1. 检查话题是否存在
gz topic -l | grep "command/motor_speed"

# 2. 检查是否有数据发布
gz topic -e -t /hnuter/command/motor_speed

# 3. 检查 PX4 执行器输出
ros2 topic echo /fmu/out/actuator_outputs --once

# 4. 检查 gz_bridge 状态
# 在 PX4 控制台
gz_bridge status
```

**可能原因**:
- [ ] model.sdf 未重新加载 → 清理重编译
- [ ] 话题名称仍然错误 → 检查 model.sdf
- [ ] PX4 未发布命令 → 检查解锁状态
- [ ] gz_bridge 未启动 → 检查 SIM_GZ_EN=1

### 问题2: 解锁失败

**检查**:
```bash
# 在 PX4 控制台查看预检失败原因
commander status
```

**常见原因**:
- EKF 未收敛 → 等待几秒
- GPS 未就绪 → 检查 SIM_GZ_EN_GPS=1
- 电池检查 → CBRK_SUPPLY_CHK=894281
- ESC 检查 → COM_ARM_CHK_ESCS=0

### 问题3: 话题名称不对

如果仍看到 `/hnuter_0/...`:

```bash
# 检查 model.sdf
grep "commandSubTopic" ~/PX4-Autopilot-Hnuter/Tools/simulation/gz/models/hnuter/model.sdf

# 应该都是: <commandSubTopic>command/motor_speed</commandSubTopic>
# 不应该有: /hnuter_0/...
```

修复后重新编译:
```bash
cd ~/PX4-Autopilot-Hnuter
make clean
make px4_sitl gz_hnuter
```

### 问题4: 数据流不通

使用监控脚本诊断数据流:

```bash
# 终端1: 启动仿真
make px4_sitl gz_hnuter

# 终端2: 监控
cd ~/px4_ros2_ws
python3 monitor_gazebo_motors.py

# 终端3: 发送测试命令
ros2 topic pub /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{command: 400, param1: 1.0}" --once
```

检查数据流向:
```
PX4 Controller → actuator_outputs (ROS2)
       ↓
  gz_bridge (PX4 module)
       ↓
  /hnuter/command/motor_speed (Gazebo)
       ↓
  Motor plugins
       ↓
  物理仿真 (螺旋桨旋转)
```

---

## 倾转舵机功能

### 当前状态
倾转舵机配置存在但被禁用 (`CA_SV_TL_COUNT = 0`)

### 启用方法

运行启用脚本:
```bash
cd ~/px4_ros2_ws
./enable_tilt_servos.sh
```

或手动编辑 `4051_gz_hnuter`:
```bash
# 在 "Include base hnuter configuration" 之前添加
param set-default CA_SV_TL_COUNT 4
```

### 验证舵机工作

解锁后在监控脚本中应该看到 `servo_0` 到 `servo_3` 有数据变化。

---

## 参数说明

### 电机配置
```bash
SIM_GZ_EC_FUNC1-5  # 电机功能: 101-105
SIM_GZ_EC_MIN1-5   # 最小速度: 10 RPM
SIM_GZ_EC_MAX1-5   # 最大速度: 1500 RPM

CA_ROTOR_COUNT     # 电机数量: 5
CA_ROTOR0-4_PX/PY/PZ  # 电机位置
CA_ROTOR0-4_KM     # 力矩系数
```

### 舵机配置
```bash
SIM_GZ_SV_FUNC1-4  # 舵机功能: 201-204
SIM_GZ_SV_MAXA1-4  # 最大角度: 1.57 rad (90°)
SIM_GZ_SV_MINA1-4  # 最小角度: -1.57 rad (-90°)

CA_SV_TL_COUNT     # 倾转舵机数量: 0 (当前禁用)
CA_SV_TL0-3_MAXA/MINA  # 舵机角度范围
```

### 安全绕过（仅用于仿真）
```bash
CBRK_SUPPLY_CHK    # 894281 (禁用电源检查)
CBRK_USB_CHK       # 197848 (禁用USB检查)
COM_ARM_CHK_ESCS   # 0 (禁用ESC检查)
COM_ARM_WO_GPS     # 1 (允许无GPS解锁)
```

---

## 成功标志

✅ **所有功能正常时**:

1. **话题检查**:
   - `/hnuter/command/motor_speed` 存在
   - 4个舵机话题存在

2. **解锁**:
   - PX4 成功解锁
   - 电机开始怠速旋转
   - 监控脚本显示数据流动

3. **起飞**:
   - 电机加速
   - 无人机升空
   - 高度控制正常

4. **悬停**:
   - 位置稳定
   - 姿态控制正常
   - 电机速度响应控制输入

---

## 相关文件

### 修改的文件
- `Tools/simulation/gz/models/hnuter/model.sdf` - 电机话题路径
- `~/px4_ros2_ws/test_actuators.py` - 字段名修复

### 配置文件
- `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`
- `ROMFS/px4fmu_common/init.d-posix/airframes/9001_hnuter_tiltrotor`

### 诊断脚本
- `~/px4_ros2_ws/check_gazebo_topics.py`
- `~/px4_ros2_ws/monitor_gazebo_motors.py`
- `~/px4_ros2_ws/test_actuators.py`
- `~/px4_ros2_ws/verify_motor_fix.sh`
- `~/px4_ros2_ws/enable_tilt_servos.sh`

### 文档
- `GAZEBO_MOTOR_FIX.md` - 问题修复总结
- `HNUTER_GAZEBO_TESTING_GUIDE.md` - 本文档

---

## 下一步

修复验证成功后，可以进行:

1. **控制器测试**: 测试姿态控制、位置控制
2. **倾转功能**: 启用并测试倾转舵机
3. **航线飞行**: 通过 QGC 规划和执行任务
4. **自定义控制**: 开发 ROS2 控制节点

---

**祝测试顺利！** 🚁
