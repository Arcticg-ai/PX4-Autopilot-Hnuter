# Gazebo 电机不转问题修复总结

## 问题描述
Gazebo仿真中，解锁和起飞命令无效，螺旋桨不旋转，无人机无反应。

## 发现的问题

### 问题1: Gazebo 模型文件中的话题路径错误 ✅ 已修复

**位置**: `Tools/simulation/gz/models/hnuter/model.sdf`

**问题**: 电机插件使用了错误的绝对路径格式
```xml
<!-- 错误 ❌ -->
<commandSubTopic>/hnuter_0/command/motor_speed</commandSubTopic>

<!-- 正确 ✅ -->
<commandSubTopic>command/motor_speed</commandSubTopic>
```

**原因**:
- PX4的 `GZMixingInterfaceESC.cpp` 会自动构建完整路径: `"/" + model_name + "/command/motor_speed"`
- 如果使用绝对路径，会导致话题名称为 `/hnuter_0/command/motor_speed`
- 但PX4实际发布的是 `/hnuter/command/motor_speed`
- 导致话题不匹配，电机收不到命令

**修复**: 所有5个电机的 `commandSubTopic` 已改为相对路径 `command/motor_speed`

### 问题2: ROS2测试脚本字段名错误 ✅ 已修复

**位置**: `/home/hnuter/px4_ros2_ws/test_actuators.py`

**问题**: 使用了不存在的字段名
```python
# 错误 ❌
msg.actuator = True

# 正确 ✅
msg.direct_actuator = True
```

**原因**: `px4_msgs.msg.OffboardControlMode` 的字段名是 `direct_actuator`，不是 `actuator`

**修复**: 已更新为正确的字段名

### 问题3: 机型配置文件检查 ⚠️ 待验证

**需要检查的文件**:
1. `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`
2. `ROMFS/px4fmu_common/init.d-posix/airframes/9001_hnuter_tiltrotor`

**当前配置**:
- 5个电机配置: ✅ 正确
  - `SIM_GZ_EC_FUNC1-5`: 101-105 (映射到5个电机)
  - 速度限制: MIN=10, MAX=1500

- 4个舵机配置: ✅ 正确
  - `SIM_GZ_SV_FUNC1-4`: 201-204 (映射到4个倾转舵机)
  - 角度限制: -1.57 到 1.57 弧度

## 诊断工具

已创建以下诊断脚本:

### 1. 检查话题脚本
```bash
cd /home/hnuter/px4_ros2_ws
python3 check_gazebo_topics.py
```
这个脚本会检查:
- Gazebo话题列表
- 电机和舵机话题
- PX4执行器输出话题
- 模型信息

### 2. 实时监控脚本
```bash
cd /home/hnuter/px4_ros2_ws
python3 monitor_gazebo_motors.py
```
这个脚本会实时显示:
- PX4执行器输出数据
- Gazebo电机速度命令
- 舵机命令

### 3. 修复后的执行器测试
```bash
cd /home/hnuter/px4_ros2_ws
python3 test_actuators.py
```

## 测试步骤

### 步骤1: 重新编译模型
```bash
cd /home/hnuter/PX4-Autopilot-Hnuter
make px4_sitl gz_hnuter
```

### 步骤2: 检查话题
在另一个终端运行:
```bash
# 检查Gazebo话题
gz topic -l | grep hnuter

# 应该看到:
# /hnuter/command/motor_speed  (不是 /hnuter_0/...)
# /model/hnuter/servo_0
# /model/hnuter/servo_1
# /model/hnuter/servo_2
# /model/hnuter/servo_3
```

### 步骤3: 监控电机命令
```bash
cd /home/hnuter/px4_ros2_ws
python3 monitor_gazebo_motors.py
```

### 步骤4: 测试解锁和起飞
在PX4控制台:
```bash
# 解锁
commander arm

# 或通过QGC解锁和起飞
```

观察监控脚本中是否有数据流动。

## 预期结果

修复后，应该看到:

1. **话题正确创建**:
   - `/hnuter/command/motor_speed` (5个电机共用)
   - `/model/hnuter/servo_0` 到 `servo_3` (4个舵机)

2. **数据流向**:
   ```
   PX4控制分配器
     → actuator_outputs
     → GZ Bridge (gz_bridge模块)
     → /hnuter/command/motor_speed
     → Gazebo电机插件
     → 物理仿真 (螺旋桨旋转)
   ```

3. **可见效果**:
   - 解锁后电机开始怠速旋转
   - 起飞时电机加速
   - 螺旋桨可见旋转
   - 无人机产生升力

## 可能的其他问题

如果修复后仍然不工作，检查:

1. **PX4日志**: 查看 `gz_bridge` 模块是否正常启动
   ```bash
   # 在PX4控制台
   gz_bridge status
   ```

2. **Gazebo插件**: 确认插件加载
   ```bash
   gz plugin -l
   ```

3. **话题连接**: 使用gz工具监听
   ```bash
   gz topic -e -t /hnuter/command/motor_speed
   ```

4. **执行器映射**: 检查控制分配
   ```bash
   # 在PX4控制台
   param show CA_ROTOR*
   param show SIM_GZ_*
   ```

## 参考

- PX4 GZ Bridge源码: `src/modules/simulation/gz_bridge/`
- 电机接口: `GZMixingInterfaceESC.cpp:36-50`
- 舵机接口: `GZMixingInterfaceServo.cpp:106-130`
- 标准参考模型: `Tools/simulation/gz/models/x500/`
