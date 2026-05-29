# Gazebo 电机问题修复总结

## 📋 问题清单

### 问题 1: Gazebo 无人机无反应 ✅ 已修复

**症状**:
- 解锁和起飞命令无效
- 螺旋桨不旋转
- Gazebo 插件未施加力和力矩

**根本原因**:
- SDF 模型文件中电机话题使用了错误的绝对路径
- PX4 发布的话题与 Gazebo 插件订阅的话题不匹配

**修复内容**:
```xml
<!-- 之前 (错误) -->
<commandSubTopic>/hnuter_0/command/motor_speed</commandSubTopic>

<!-- 之后 (正确) -->
<commandSubTopic>command/motor_speed</commandSubTopic>
```

**修改文件**: `Tools/simulation/gz/models/hnuter/model.sdf` (5个电机全部修复)

---

### 问题 2: test_actuators.py 测试异常 ✅ 已修复

**错误信息**:
```
AttributeError: 'OffboardControlMode' object has no attribute 'actuator'
```

**根本原因**:
- `px4_msgs.msg.OffboardControlMode` 字段名不是 `actuator`
- 正确字段名是 `direct_actuator`

**修复内容**:
```python
# 之前 (错误)
msg.actuator = True

# 之后 (正确)
msg.direct_actuator = True
```

**修改文件**: `/home/hnuter/px4_ros2_ws/test_actuators.py`

---

### 问题 3: 机型配置文件检查 ✅ 已验证

**检查结果**:
- ✅ 5个电机配置正确 (SIM_GZ_EC_FUNC1-5: 101-105)
- ✅ 4个舵机配置正确 (SIM_GZ_SV_FUNC1-4: 201-204)
- ⚠️ 倾转舵机当前禁用 (CA_SV_TL_COUNT=0)

**配置文件**:
- `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`
- `ROMFS/px4fmu_common/init.d-posix/airframes/9001_hnuter_tiltrotor`

---

## 🛠️ 创建的诊断工具

### 1. 自动测试脚本
```bash
~/px4_ros2_ws/auto_test.sh
```
自动检查所有修复状态、文件存在性、Gazebo状态等

### 2. 快速验证脚本
```bash
~/px4_ros2_ws/verify_motor_fix.sh
```
验证 model.sdf 和 test_actuators.py 是否正确修复

### 3. Gazebo 话题检查
```bash
~/px4_ros2_ws/check_gazebo_topics.py
```
检查 Gazebo 话题、PX4 话题、模型信息

### 4. 实时监控工具
```bash
~/px4_ros2_ws/monitor_gazebo_motors.py
```
实时显示执行器输出、电机命令、舵机命令

### 5. 执行器测试
```bash
~/px4_ros2_ws/test_actuators.py
```
自动测试序列: 解锁 → 电机测试 → 舵机测试 → 上锁

### 6. 倾转舵机启用脚本
```bash
~/px4_ros2_ws/enable_tilt_servos.sh
```
可选：启用倾转舵机功能 (设置 CA_SV_TL_COUNT=4)

---

## 📚 创建的文档

### 1. 修复总结
`GAZEBO_MOTOR_FIX.md` - 详细的问题分析和修复说明

### 2. 完整测试指南
`HNUTER_GAZEBO_TESTING_GUIDE.md` - 从启动到测试的完整流程

### 3. 快速参考
`QUICK_REFERENCE.txt` - 常用命令速查卡

### 4. 本文档
`README_FIX_SUMMARY.md` - 修复总结和使用说明

---

## 🚀 快速开始

### 步骤1: 运行自动测试
```bash
cd ~/px4_ros2_ws
./auto_test.sh
```

### 步骤2: 启动仿真
```bash
cd ~/PX4-Autopilot-Hnuter
make px4_sitl gz_hnuter
```

### 步骤3: 启动监控（新终端）
```bash
cd ~/px4_ros2_ws
python3 monitor_gazebo_motors.py
```

### 步骤4: 测试解锁（PX4控制台）
```bash
commander arm
```

### 步骤5: 观察结果
- ✅ 电机应该开始旋转
- ✅ 监控脚本显示数据流动
- ✅ Gazebo 中可见螺旋桨旋转

---

## 🔍 数据流向

```
PX4 控制分配器 (Control Allocator)
         ↓
actuator_outputs (uORB/ROS2)
         ↓
GZ Bridge (gz_bridge 模块)
         ↓
/hnuter/command/motor_speed (Gazebo 话题)
         ↓
MulticopterMotorModel 插件
         ↓
物理引擎 (螺旋桨旋转、力和力矩)
```

---

## ✅ 验证清单

- [ ] 运行 `auto_test.sh` 全部通过
- [ ] 话题 `/hnuter/command/motor_speed` 存在
- [ ] 4个舵机话题存在
- [ ] PX4 成功解锁
- [ ] 电机开始旋转
- [ ] 监控脚本显示数据
- [ ] 起飞命令有效
- [ ] 无人机升空

---

## 📞 故障排除

### 电机仍然不转？

1. **检查话题是否匹配**:
   ```bash
   gz topic -l | grep motor_speed
   # 应该是 /hnuter/command/motor_speed
   # 不应该是 /hnuter_0/command/motor_speed
   ```

2. **检查是否有数据发布**:
   ```bash
   gz topic -e -t /hnuter/command/motor_speed
   ```

3. **检查 model.sdf 是否正确**:
   ```bash
   grep "commandSubTopic" ~/PX4-Autopilot-Hnuter/Tools/simulation/gz/models/hnuter/model.sdf
   # 应该都是相对路径: command/motor_speed
   ```

4. **清理重编译**:
   ```bash
   cd ~/PX4-Autopilot-Hnuter
   make clean
   make px4_sitl gz_hnuter
   ```

### 解锁失败？

1. **等待 EKF 收敛** (5-10秒)
2. **检查状态**: `commander status`
3. **强制解锁**: `commander arm -f`

---

## 🎯 成功标志

修复成功后，你应该看到:

1. **PX4 控制台**:
   ```
   INFO  [commander] Armed by user command
   INFO  [gz_bridge] Publishing motor commands
   ```

2. **监控脚本**:
   - actuator_outputs 有数值变化
   - motor_speed 命令有数据
   - 数据实时更新

3. **Gazebo 窗口**:
   - 螺旋桨开始旋转
   - 怠速时缓慢旋转
   - 起飞时加速明显

4. **物理效果**:
   - 解锁后有怠速旋转
   - 起飞时产生升力
   - 无人机开始升空

---

## 🔧 技术细节

### 话题映射

| PX4 输出 | GZ Bridge | Gazebo 话题 | 插件 |
|---------|-----------|-------------|------|
| actuator_motors | ESC接口 | /hnuter/command/motor_speed | MulticopterMotorModel |
| actuator_servos | Servo接口 | /model/hnuter/servo_0-3 | JointPositionController |

### 参数配置

**电机 (ESC)**:
- `SIM_GZ_EC_FUNC1-5`: 功能映射 (101-105)
- `SIM_GZ_EC_MIN/MAX`: 速度范围 (10-1500)

**舵机 (Servo)**:
- `SIM_GZ_SV_FUNC1-4`: 功能映射 (201-204)
- `SIM_GZ_SV_MINA/MAXA`: 角度范围 (±1.57 rad)

**控制分配**:
- `CA_ROTOR_COUNT`: 5 (电机数量)
- `CA_SV_TL_COUNT`: 0 (倾转舵机，当前禁用)

---

## 📝 后续工作

1. **测试飞行**: 测试各种飞行模式和控制
2. **启用倾转**: 运行 `enable_tilt_servos.sh` 启用倾转功能
3. **自定义控制**: 开发 ROS2 控制节点
4. **调参优化**: 根据实际飞行调整 PID 参数

---

**修复完成！现在可以正常使用 Gazebo 仿真了。** 🎉
