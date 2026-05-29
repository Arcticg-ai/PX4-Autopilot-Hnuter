# Hnuter 自定义控制器问题诊断和修复

## 问题症状

- 螺旋桨会转动
- 无人机会乱飞但无法正常起飞
- 自定义的控制分配器和控制器没有被调用

## 诊断发现

### 1. 代码已经存在并编译

✅ 文件已存在：
- `ActuatorEffectivenessHnuter.cpp/hpp`
- `ActuatorEffectivenessFullyActuated.cpp/hpp`
- `HnuterPositionControl.cpp/hpp`

✅ 已在 CMakeLists.txt 中注册编译

✅ 已在 ControlAllocator.cpp 中注册：
```cpp
case EffectivenessSource::HNUTER_TILTROTOR:  // = 16
    tmp = new ActuatorEffectivenessHnuter(this);
    break;
```

### 2. 机型配置问题 ⚠️

**当前配置**：
```bash
# ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter
param set-default CA_AIRFRAME 16  # Multicopter  ← 注释错误！
```

**枚举值定义**：
```cpp
enum class EffectivenessSource {
    MULTIROTOR = 0,          // ← 不是16！
    ...
    FULLY_ACTUATED = 15,
    HNUTER_TILTROTOR = 16,   // ← 这才是16！
};
```

**结论**:
- CA_AIRFRAME=16 **应该**调用 ActuatorEffectivenessHnuter
- 注释说"Multicopter"是错误的，容易误导

### 3. 潜在的Bug

#### Bug 1: 缺少编译后重启

**问题**: 修改C++代码后必须重新编译，但您可能没有重新编译或编译失败了。

**验证**:
```bash
cd ~/PX4-Autopilot-Hnuter
make clean
make px4_sitl gz_hnuter
```

查看编译输出，确认：
- ActuatorEffectivenessHnuter.cpp 被编译
- 无编译错误

#### Bug 2: 控制分配器使用的是默认Multirotor而非Hnuter

**问题**: 即使CA_AIRFRAME=16，可能由于某种原因使用了错误的控制器。

**诊断命令** (在 PX4 控制台):
```bash
# 查看当前使用的控制分配器
control_allocator status
```

**预期输出**:
```
Effectiveness: Hnuter Tiltrotor  ← 应该看到这个
```

如果看到的是 `Effectiveness: Multirotor`，说明控制器选择错误。

#### Bug 3: ActuatorEffectivenessHnuter实现中的问题

从代码分析发现几个潜在问题：

**问题 A: 倾转角度的单位**

hnuter71.py 中：
```python
theta1 = np.arcsin(val1)  # 返回弧度
```

ActuatorEffectivenessHnuter.cpp 中：
```cpp
float theta1 = asinf(val1);  // 返回弧度

// 但是存储到actuator_sp时没有转换！
actuator_sp(_first_tilt_idx + 2) = theta1 + control_collective_tilt;
```

PX4的执行器输出通常是**归一化值 [-1, 1]**，但这里直接使用弧度值（约±1.57），可能导致：
- 舵机角度不正确
- 控制失效

**问题 B: 推力到归一化的映射**

hnuter71.py 中推力单位是牛顿（N）：
```python
F1 = np.clip(F1, 0, T_max)  # T_max = 60 N
```

但PX4的actuator_motors也期望**归一化值 [0, 1]**：

ActuatorEffectivenessHnuter.cpp 中：
```cpp
actuator_sp(0) = T12 / 2.0f;  // T12可能是60N，远大于1！
```

这会导致：
- 控制器输出超出范围
- 电机指令被裁剪
- 无法正常飞行

**问题 C: 控制输入轴的顺序**

ActuatorEffectivenessHnuter.cpp (行 89-97):
```cpp
float W[6];
W[0] = control_sp(3); // Fx  ← PX4的control_sp顺序
W[1] = control_sp(4); // Fy
W[2] = control_sp(5); // Fz
W[3] = control_sp(0); // Tx (roll)
W[4] = control_sp(1); // Ty (pitch)
W[5] = control_sp(2); // Tz (yaw)
```

需要确认 `control_sp` 的顺序是否与 PX4 的 `vehicle_torque_setpoint` 和 `vehicle_thrust_setpoint` 一致。

## 修复方案

### 修复 1: 更正机型配置注释

```bash
# 编辑文件
nano ~/PX4-Autopilot-Hnuter/ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter
```

修改：
```bash
# 从：
param set-default CA_AIRFRAME 16  # Multicopter

# 改为：
param set-default CA_AIRFRAME 16  # Hnuter Tiltrotor (custom controller)
```

### 修复 2: 添加调试日志

在 ActuatorEffectivenessHnuter.cpp 的 `updateSetpoint()` 中添加日志：

```cpp
void ActuatorEffectivenessHnuter::updateSetpoint(...)
{
    // 添加调试日志
    PX4_INFO("Hnuter updateSetpoint called!");  // 确认函数被调用
    PX4_INFO("Control SP: Fx=%.2f, Fy=%.2f, Fz=%.2f, Tx=%.2f, Ty=%.2f, Tz=%.2f",
             (double)control_sp(3), (double)control_sp(4), (double)control_sp(5),
             (double)control_sp(0), (double)control_sp(1), (double)control_sp(2));

    // ... 原有代码 ...
}
```

### 修复 3: 归一化执行器输出

需要将推力（N）和角度（rad）转换为归一化值 [-1, 1] 或 [0, 1]。

**电机推力归一化**:
```cpp
// 原来：
actuator_sp(0) = T12 / 2.0f;  // 可能是30N

// 修改为：
const float T_max_normalized = 60.0f;  // 最大推力
actuator_sp(0) = (T12 / 2.0f) / T_max_normalized;  // 归一化到[0, 1]
actuator_sp(1) = (T12 / 2.0f) / T_max_normalized;
actuator_sp(2) = (T34 / 2.0f) / T_max_normalized;
actuator_sp(3) = (T34 / 2.0f) / T_max_normalized;
actuator_sp(4) = F3 / 15.0f;  // 尾部电机，归一化到[-1, 1]
```

**舵机角度归一化**:
```cpp
// 原来：
actuator_sp(_first_tilt_idx + 0) = alpha1;  // 弧度值 ±1.57

// 修改为：
const float angle_max_rad = M_PI_2;  // 90度 = π/2
actuator_sp(_first_tilt_idx + 0) = alpha1 / angle_max_rad;  // 归一化到[-1, 1]
actuator_sp(_first_tilt_idx + 1) = alpha2 / angle_max_rad;
actuator_sp(_first_tilt_idx + 2) = theta1 / angle_max_rad;
actuator_sp(_first_tilt_idx + 3) = theta2 / angle_max_rad;
```

### 修复 4: 验证控制输入

添加控制输入的范围检查：

```cpp
// 在 updateSetpoint() 开始处
for (int i = 0; i < 6; i++) {
    if (!PX4_ISFINITE(control_sp(i))) {
        PX4_ERR("Invalid control_sp[%d]", i);
        return;
    }
}
```

## 诊断步骤

### 步骤 1: 重新编译

```bash
cd ~/PX4-Autopilot-Hnuter

# 清理
make clean

# 重新编译（查看输出，确认ActuatorEffectivenessHnuter.cpp被编译）
make px4_sitl gz_hnuter 2>&1 | tee build.log

# 检查编译错误
grep -i "error" build.log
```

### 步骤 2: 启动并检查控制器

```bash
# 启动仿真
make px4_sitl gz_hnuter
```

在 PX4 控制台：
```bash
# 检查控制分配器状态
control_allocator status

# 应该显示：
# Effectiveness: Hnuter Tiltrotor  ← 如果看到这个，说明自定义控制器被使用

# 检查参数
param show CA_AIRFRAME  # 应该是 16
param show CA_ROTOR_COUNT  # 应该是 5

# 查看日志
dmesg | grep Hnuter
```

### 步骤 3: 测试控制

```bash
# 解锁
commander arm -f

# 监听执行器输出
listener actuator_motors
listener actuator_servos

# 起飞
commander takeoff
```

**预期结果**:
- 如果使用了自定义控制器，应该看到 actuator_motors 的值在 [0, 1] 范围内
- actuator_servos 也应该在 [-1, 1] 范围内
- 无人机应该能够平稳起飞

## 快速测试脚本

创建测试脚本检查控制器：

```bash
#!/bin/bash
# ~/px4_ros2_ws/check_hnuter_controller.sh

echo "=== 检查 Hnuter 控制器 ==="
echo ""

echo "请在 PX4 控制台执行以下命令："
echo ""
echo "1. control_allocator status"
echo "   查找: 'Effectiveness: Hnuter Tiltrotor'"
echo ""
echo "2. param show CA_AIRFRAME"
echo "   应该是: 16"
echo ""
echo "3. dmesg | grep Hnuter"
echo "   查找调试日志"
echo ""
echo "4. listener actuator_motors"
echo "   解锁后检查输出范围 [0, 1]"
echo ""
```

## 如果控制器仍然没有被调用

### 可能原因 1: 编译配置问题

检查是否被优化掉或条件编译排除：

```bash
cd ~/PX4-Autopilot-Hnuter
grep -r "HNUTER_TILTROTOR" src/modules/control_allocator/
```

### 可能原因 2: 运行时选择逻辑问题

检查 ControlAllocator.cpp 中的 `updateEffectivenessSource()` 函数：

```bash
grep -A 20 "updateEffectivenessSource" src/modules/control_allocator/ControlAllocator.cpp
```

确认CA_AIRFRAME参数正确映射到EffectivenessSource。

### 可能原因 3: 使用了错误的可执行文件

```bash
# 确认px4可执行文件的时间戳
ls -lh build/px4_sitl_default/bin/px4

# 应该是最近编译的时间
```

## 下一步行动

1. **立即执行**: 重新编译并检查 control_allocator status
   ```bash
   cd ~/PX4-Autopilot-Hnuter
   make clean
   make px4_sitl gz_hnuter
   ```

2. **在PX4控制台检查**:
   ```bash
   control_allocator status
   ```

3. **如果显示 "Hnuter Tiltrotor"**:
   - 问题在于归一化，需要修复步骤3的代码

4. **如果显示其他（如 "Multirotor"）**:
   - 问题在于控制器选择，需要调试为什么CA_AIRFRAME=16没有选择正确的控制器

5. **提供编译日志和控制台输出**:
   - 把 `control_allocator status` 的输出告诉我
   - 把 `param show CA_AIRFRAME` 的输出告诉我
   - 把编译过程中与 Hnuter 相关的日志告诉我

---

**创建时间**: 2026-02-24
**优先级**: 高
**预估影响**: 自定义控制器是实现正确飞行的关键
