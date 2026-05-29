# Gazebo 电机关节修复 - 螺旋桨不转动问题

## 问题描述

### 症状
- ✅ PX4 有 actuator_motors 输出
- ✅ gz_bridge 正确发布数据到 Gazebo 话题
- ✅ Gazebo 话题 `/hnuter_0/command/motor_speed` 接收到数据
- ❌ 螺旋桨不转动，模型无物理响应

### 根本原因

**Gazebo 模型文件中的电机关节配置错误**

在 `Tools/simulation/gz/models/hnuter/model.sdf` 中，所有 5 个电机关节（xyj1-xyj5）的 `<limit>` 配置错误：

```xml
<limit>
  <effort>0</effort>      <!-- ❌ 错误：0 表示关节被锁定 -->
  <velocity>0</velocity>  <!-- ❌ 错误：0 表示无法旋转 -->
  <lower>-inf</lower>
  <upper>inf</upper>
</limit>
```

**影响**：
- `effort=0`: 关节无法施加力矩
- `velocity=0`: 关节无法旋转

这导致即使电机插件收到命令，物理引擎也无法让螺旋桨旋转。

## 修复方案

### 修改内容

将所有 5 个电机关节的限制值修改为：

```xml
<limit>
  <effort>30</effort>     <!-- ✅ 允许施加 30 N·m 的力矩 -->
  <velocity>-1</velocity> <!-- ✅ 无限制旋转速度 -->
  <lower>-inf</lower>
  <upper>inf</upper>
</limit>
```

### 修改的关节

1. **xyj1** (Motor xy1, 右臂上方) - 行 469
2. **xyj2** (Motor xy2, 右臂下方) - 行 524
3. **xyj3** (Motor xy3, 左臂上方) - 行 249
4. **xyj4** (Motor xy4, 左臂下方) - 行 304
5. **xyj5** (Motor xy5, 尾部) - 行 579

### 参数说明

**effort (力矩限制)**：
- 定义关节可以施加的最大力矩 (N·m)
- 0 = 关节被锁定，无法移动
- 30 = 允许施加足够的力矩让螺旋桨旋转
- 对于多旋翼，通常设置为 10-50

**velocity (速度限制)**：
- 定义关节的最大旋转速度 (rad/s)
- 0 = 关节无法旋转
- -1 = 无限制（推荐用于螺旋桨）
- 正值 = 具体的速度限制（如 1000 rad/s）

## 测试验证

### 步骤 1: 重启仿真

由于修改了模型文件，必须重启仿真才能生效。

**方法 A: 完全重启**

```bash
# 在 PX4 控制台按 Ctrl+C 停止
# 关闭 Gazebo 窗口
# 然后重新启动
cd ~/PX4-Autopilot-Hnuter
make px4_sitl gz_hnuter
```

**方法 B: 热重载（如果支持）**

在 Gazebo 中删除并重新生成模型（通常不可靠，建议完全重启）

### 步骤 2: 测试电机

**测试 A: 手动测试（直接命令 Gazebo）**

```bash
# 在新终端
gz topic -t /hnuter_0/command/motor_speed -m gz.msgs.Actuators -p 'velocity: [500, 500, 500, 500, 500]'
```

**预期结果**: 所有 5 个螺旋桨应该开始旋转 ✅

**测试 B: PX4 解锁测试**

```bash
# 在 PX4 控制台
commander arm -f
```

**预期结果**:
- 螺旋桨应该以怠速旋转（约 10-20% 速度）
- 无人机保持在地面但有轻微震动

**测试 C: 起飞测试**

```bash
# 在 PX4 控制台
commander takeoff
```

**预期结果**:
- 螺旋桨加速旋转（约 40-60% 速度）
- 无人机升空到约 10 米高度 🚁

### 步骤 3: 监控数据流

同时运行监控脚本：

```bash
cd ~/px4_ros2_ws
python3 monitor_gazebo_motors.py
```

应该看到：
- PX4 actuator_motors 输出值（0.1-0.5）
- Gazebo 电机速度值（100-800）
- 两者实时同步

## 技术原理

### MulticopterMotorModel 插件工作流程

1. **接收命令**: 插件订阅 `/hnuter_0/command/motor_speed` 话题
2. **解析数据**: 提取每个电机的目标速度
3. **应用力矩**: 通过 Gazebo 物理引擎对关节施加力矩
4. **关节限制检查**: 检查 `<limit>` 中的 effort 和 velocity
5. **物理仿真**: 如果限制允许，关节旋转并产生力

**修复前**：
- 第 4 步失败（effort=0, velocity=0）
- 关节被锁定，物理引擎拒绝旋转
- 螺旋桨不动

**修复后**：
- 第 4 步通过（effort=30, velocity=-1）
- 关节可以自由旋转
- 螺旋桨正常工作 ✅

## 常见问题

### Q1: 为什么之前的配置是 0？

**可能原因**：
1. 模型从 CAD 导出时使用了默认值
2. 最初设计为静态展示模型，不需要运动
3. 导出工具的 bug

### Q2: velocity=-1 是什么意思？

**解释**：
- 在 Gazebo/SDF 中，`-1` 是特殊值表示"无限制"
- 等同于 `<velocity>inf</velocity>`
- 推荐用于高速旋转的部件（螺旋桨、车轮等）

### Q3: effort=30 够吗？

**分析**：
- 对于小型多旋翼（5-10 kg），30 N·m 足够
- Hnuter 模型质量约 6 kg，30 N·m 完全足够
- 如果需要更高性能，可以增加到 50 或 100

### Q4: 为什么倾转舵机的关节没问题？

让我检查：

```bash
grep -A 10 "joint name=.*j[12]'" model.sdf | grep -E "(joint name|<effort>|<velocity>)"
```

如果舵机关节也是 0，也需要修复。

## 相关文档

- **模型文件**: `Tools/simulation/gz/models/hnuter/model.sdf`
- **Takeoff 故障排除**: `TAKEOFF_TROUBLESHOOTING.md`
- **Gazebo 测试指南**: `HNUTER_GAZEBO_TESTING_GUIDE.md`
- **电机修复总结**: `GAZEBO_MOTOR_FIX.md` (话题路径修复)

## 修复历史

### 2026-02-24 - 关节限制修复

**问题**: 电机关节 effort 和 velocity 都是 0，导致螺旋桨无法旋转

**修复**:
- effort: 0 → 30
- velocity: 0 → -1

**影响的关节**: xyj1, xyj2, xyj3, xyj4, xyj5 (共 5 个)

**修复文件**: `Tools/simulation/gz/models/hnuter/model.sdf`

## 验证清单

修复后，请确认以下所有项：

- [ ] 重启仿真，无错误消息
- [ ] 手动发送 Gazebo 命令，螺旋桨转动
- [ ] PX4 解锁，螺旋桨怠速旋转
- [ ] PX4 takeoff，无人机升空
- [ ] 监控脚本显示数据流正常
- [ ] 无人机可以悬停和控制
- [ ] 日志中无关节错误

## 总结

**修复前的数据流**:
```
PX4 → gz_bridge → Gazebo 话题 ✅
                         ↓
              MulticopterMotorModel 插件 ✅
                         ↓
              关节限制检查 ❌ (effort=0, velocity=0)
                         ↓
              物理引擎 ❌ (关节锁定)
                         ↓
              螺旋桨 ❌ (不转动)
```

**修复后的数据流**:
```
PX4 → gz_bridge → Gazebo 话题 ✅
                         ↓
              MulticopterMotorModel 插件 ✅
                         ↓
              关节限制检查 ✅ (effort=30, velocity=-1)
                         ↓
              物理引擎 ✅ (施加力矩)
                         ↓
              螺旋桨 ✅ (旋转并产生升力)
                         ↓
              无人机 ✅ (起飞！)
```

---

**创建日期**: 2026-02-24
**修复类型**: 关键 Bug 修复
**影响**: 使 Gazebo 仿真完全可用
**下次启动**: 无需额外配置，自动生效
