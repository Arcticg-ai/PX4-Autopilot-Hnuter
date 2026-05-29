# 📚 Hnuter PX4 仿真和技术文档汇总

本目录包含了 Hnuter 倾转旋翼无人机的 PX4 仿真配置、问题修复、诊断工具和技术文档。

---

## 📋 目录

- [问题修复](#-问题修复)
- [诊断工具](#-诊断工具)
- [技术文档](#-技术文档)
- [快速开始](#-快速开始)
- [参考资料](#-参考资料)

---

## 🔧 问题修复

### 已修复的问题

#### 1. Gazebo 电机不转问题 ✅

**问题描述**: Gazebo 仿真中电机不响应解锁和起飞命令

**根本原因**:
- SDF 模型文件中电机话题路径错误（使用了绝对路径 `/hnuter_0/...` 而非相对路径）
- ROS2 测试脚本字段名错误（`actuator` vs `direct_actuator`）

**修复文件**:
- `Tools/simulation/gz/models/hnuter/model.sdf`
- `~/px4_ros2_ws/test_actuators.py`

**详细说明**: 查看 `GAZEBO_MOTOR_FIX.md` 和 `README_FIX_SUMMARY.md`

---

## 🛠️ 诊断工具

所有工具位于: `~/px4_ros2_ws/`

### 1. 自动测试脚本
```bash
cd ~/px4_ros2_ws
./auto_test.sh
```
**功能**: 自动检查所有修复状态、文件存在性、Gazebo 运行状态、编译状态等

### 2. 快速验证脚本
```bash
./verify_motor_fix.sh
```
**功能**: 验证 model.sdf 和测试脚本是否正确修复

### 3. Gazebo 话题检查
```bash
python3 check_gazebo_topics.py
```
**功能**: 检查 Gazebo 话题、PX4 话题、模型信息

### 4. 实时监控工具
```bash
python3 monitor_gazebo_motors.py
```
**功能**: 实时显示执行器输出、电机命令、舵机命令

### 5. 执行器测试
```bash
python3 test_actuators.py
```
**功能**: 自动测试序列 - 解锁 → 电机测试 → 舵机测试 → 上锁

### 6. 倾转舵机启用脚本
```bash
./enable_tilt_servos.sh
```
**功能**: 启用倾转舵机功能（设置 CA_SV_TL_COUNT=4）

---

## 📖 技术文档

### 0. PX4 Hnuter 4051 新机型集成操作手册
**文件**: `PX4_HNUTER_4051_新机型集成操作手册.md`

**内容**:
- 新增 4051 Hnuter 机型的完整文件清单
- SITL 和实机机型脚本配置
- QGC 无法表达两级倾转时的固定输出映射方案
- Hnuter 专用位置、姿态控制和非线性控制分配集成流程
- 尾部双向电机配置和常见问题排查
- 构建、启动、实机部署和迁移检查表

**适合**: 后续维护、迁移或重新集成 Hnuter 机型时作为主操作手册

### 1. PX4 uORB 消息系统详解
**文件**: `PX4_UORB_MESSAGE_SYSTEM.md`

**内容**:
- uORB 核心架构和工作原理
- 发布-订阅模式实现
- C/C++ API 使用方法
- 消息定义和生成
- 性能特点和优化建议
- 与 ROS2 集成

**适合**: 想要深入理解 PX4 消息系统的开发者

### 2. PX4 Takeoff 命令执行流程详解
**文件**: `PX4_TAKEOFF_COMMAND_FLOW.md`

**内容**:
- 从 GCS 发送命令到电机旋转的完整流程
- 7 个阶段的详细分析：
  1. 命令接收与解析 (Commander)
  2. 状态转换
  3. 导航控制 (Navigator)
  4. 位置控制
  5. 姿态控制
  6. 电机输出 (Control Allocator)
  7. 执行器驱动
- 完整代码追踪（文件路径和行号）
- 时序图和数据流
- 故障处理

**适合**: 想要了解 PX4 控制流程的开发者

### 3. PX4 构建和启动流程详解
**文件**: `PX4_BUILD_AND_LAUNCH_PROCESS.md`

**内容**:
- make px4_sitl gz_hnuter 完整执行流程
- CMake 配置和编译阶段
- ROMFS 生成和结构
- PX4 主程序启动过程
- rcS 启动脚本详解（376 行分析）
- 机型配置加载（4051_gz_hnuter）
- Gazebo 启动过程（px4-rc.gzsim 207 行分析）
- 模型生成和模块初始化
- 完整时间线和流程图
- 关键文件索引

**适合**: 想要深入了解 PX4 构建和启动机制的开发者

### 4. 机型配置修复说明
**文件**: `AIRFRAME_4051_FIX.md`

**内容**:
- 4051_gz_hnuter 机型文件独立化修复
- 问题分析（依赖 9001 导致的参数重复）
- 修复方案详解
- 文件结构对比（修复前 93 行 → 修复后 243 行）
- 配置完整性检查清单
- 验证方法和注意事项

**适合**: 需要了解机型配置修复细节的开发者

### 5. 机型修复总结
**文件**: `AIRFRAME_FIX_SUMMARY.md`

**内容**:
- 机型配置问题和修复的完整总结
- 所有修改的文件和行数
- 修复前后对比
- 验证结果
- 影响分析和后续建议

**适合**: 快速了解机型修复全貌的用户

### 6. Gazebo 仿真测试指南
**文件**: `HNUTER_GAZEBO_TESTING_GUIDE.md`

**内容**:
- 完整测试流程（从编译到起飞）
- 诊断工具使用方法
- 故障排除指南
- 参数说明
- 成功标志

**适合**: 进行仿真测试的用户

### 7. 快速参考卡
**文件**: `QUICK_REFERENCE.txt`

**内容**:
- 常用命令速查
- 启动仿真步骤
- 诊断工具列表
- 故障排除快速指南

**适合**: 日常使用的快速参考

### 8. Gazebo 电机修复总结
**文件**: `GAZEBO_MOTOR_FIX.md`

**内容**:
- 问题分析
- 修复步骤
- 诊断方法
- 参考资料

### 9. 修复总结
**文件**: `README_FIX_SUMMARY.md`

**内容**:
- 所有修复的问题清单
- 诊断工具说明
- 快速开始指南
- 验证清单

---

## 🚀 快速开始

### 步骤 1: 验证修复

```bash
cd ~/px4_ros2_ws
./auto_test.sh
```

### 步骤 2: 启动仿真

```bash
cd ~/PX4-Autopilot-Hnuter
make px4_sitl gz_hnuter
```

### 步骤 3: 启动监控（新终端）

```bash
cd ~/px4_ros2_ws
python3 monitor_gazebo_motors.py
```

### 步骤 4: 测试解锁（PX4 控制台）

```bash
commander arm
commander takeoff
```

### 步骤 5: 观察结果

✅ 电机应该开始旋转
✅ 监控脚本显示数据流动
✅ Gazebo 中可见螺旋桨旋转
✅ 无人机升空

---

## 📁 文件结构

```
PX4-Autopilot-Hnuter/
├── Tools/simulation/gz/models/hnuter/
│   └── model.sdf                          # ✅ 已修复电机话题
│
├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   ├── 4051_gz_hnuter                     # ✅ Gazebo 机型配置（独立完整）
│   └── 9001_hnuter_tiltrotor              # 基础机型配置（保留）
│
├── 📄 PX4_UORB_MESSAGE_SYSTEM.md          # uORB 系统详解
├── 📄 PX4_TAKEOFF_COMMAND_FLOW.md         # Takeoff 流程详解
├── 📄 PX4_BUILD_AND_LAUNCH_PROCESS.md     # 构建和启动流程详解 ⭐
├── 📄 AIRFRAME_4051_FIX.md                # 机型配置修复说明 ⭐
├── 📄 AIRFRAME_FIX_SUMMARY.md             # 机型修复总结 ⭐
├── 📄 HNUTER_GAZEBO_TESTING_GUIDE.md      # 测试指南
├── 📄 GAZEBO_MOTOR_FIX.md                 # 电机修复总结
├── 📄 README_FIX_SUMMARY.md               # 修复和工具总结
├── 📄 QUICK_REFERENCE.txt                 # 快速参考卡
└── 📄 DOCUMENTATION_INDEX.md              # 本文档

~/px4_ros2_ws/
├── 🔧 auto_test.sh                        # 自动测试脚本
├── 🔧 verify_motor_fix.sh                 # 快速验证脚本
├── 🔧 enable_tilt_servos.sh               # 舵机启用脚本
├── 🐍 check_gazebo_topics.py              # 话题检查工具
├── 🐍 monitor_gazebo_motors.py            # 实时监控工具
└── 🐍 test_actuators.py                   # ✅ 已修复执行器测试
```

---

## 🔍 数据流概览

```
外部指令 (QGC/Companion)
    ↓ MAVLink
uORB: vehicle_command
    ↓ Commander
uORB: vehicle_status (nav_state = AUTO_TAKEOFF)
    ↓ Navigator
uORB: position_setpoint_triplet
    ↓ Position Control
uORB: trajectory_setpoint
    ↓ Attitude Control
uORB: vehicle_rates_setpoint, torque_setpoint, thrust_setpoint
    ↓ Control Allocator
uORB: actuator_motors, actuator_servos
    ↓ GZ Bridge / PWM Driver
Gazebo: /hnuter/command/motor_speed, /model/hnuter/servo_0-3
    ↓ Gazebo Physics
物理: 螺旋桨旋转 → 升力 → 起飞
```

---

## 📊 系统架构

### 模块层次

```
┌─────────────────────────────────────────────────────┐
│  应用层: Commander, Navigator                       │
│  (高层决策和路径规划)                               │
└────────────────────┬────────────────────────────────┘
                     │ uORB
┌────────────────────▼────────────────────────────────┐
│  控制层: Position Control, Attitude Control         │
│  (PID 控制器)                                        │
└────────────────────┬────────────────────────────────┘
                     │ uORB
┌────────────────────▼────────────────────────────────┐
│  分配层: Control Allocator                          │
│  (混控矩阵)                                          │
└────────────────────┬────────────────────────────────┘
                     │ uORB
┌────────────────────▼────────────────────────────────┐
│  驱动层: PWM Driver / Gazebo Bridge                 │
│  (硬件接口)                                          │
└─────────────────────────────────────────────────────┘
```

---

## 🎯 学习路径建议

### 新手入门

1. **阅读**: `QUICK_REFERENCE.txt`
2. **运行**: `auto_test.sh`
3. **测试**: 启动仿真并解锁
4. **监控**: 使用 `monitor_gazebo_motors.py`

### 进阶开发

1. **理解消息系统**: 阅读 `PX4_UORB_MESSAGE_SYSTEM.md`
2. **理解控制流程**: 阅读 `PX4_TAKEOFF_COMMAND_FLOW.md`
3. **调试技巧**: 参考 `HNUTER_GAZEBO_TESTING_GUIDE.md`
4. **代码追踪**: 查看流程文档中的文件路径和行号

### 高级定制

1. 修改控制器参数
2. 添加自定义飞行模式
3. 开发 ROS2 控制节点
4. 调整控制分配矩阵

---

## 🆘 故障排除

### 问题: 电机不转

**检查列表**:
1. ✅ 运行 `verify_motor_fix.sh` 确认修复
2. ✅ 检查话题: `gz topic -l | grep hnuter`
3. ✅ 确认解锁: `commander status`
4. ✅ 监控数据: `python3 monitor_gazebo_motors.py`

**解决方案**: 参考 `GAZEBO_MOTOR_FIX.md`

### 问题: 解锁失败

**常见原因**:
- EKF 未收敛 → 等待 5-10 秒
- GPS 未锁定 → 检查 `SIM_GZ_EN_GPS=1`
- 电池检查 → 确认 `CBRK_SUPPLY_CHK=894281`

**解决方案**: 参考 `HNUTER_GAZEBO_TESTING_GUIDE.md` 故障排除章节

### 问题: 测试脚本报错

**检查**:
```bash
cd ~/px4_ros2_ws
python3 test_actuators.py
```

如果有 `AttributeError`，说明修复未生效，重新检查文件。

---

## 🔗 参考资料

### PX4 官方文档
- [PX4 开发指南](https://docs.px4.io/main/en/)
- [uORB 消息系统](https://docs.px4.io/main/en/middleware/uorb.html)
- [Gazebo 仿真](https://docs.px4.io/main/en/sim_gazebo_gz/)

### Hnuter 特定文档
- 机型配置: `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`
- 模型文件: `Tools/simulation/gz/models/hnuter/model.sdf`
- 控制分配: `src/modules/control_allocator/`

### ROS2 集成
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [px4_ros_com](https://github.com/PX4/px4_ros_com)

---

## 📝 更新日志

### 2026-02-24
- ✅ 修复 4051_gz_hnuter 机型配置（独立化，不再依赖 9001）
- ✅ 更新 PX4_BUILD_AND_LAUNCH_PROCESS.md（消除机型加载矛盾）
- ✅ 创建 AIRFRAME_4051_FIX.md（详细修复说明）
- ✅ 创建 AIRFRAME_FIX_SUMMARY.md（修复总结）
- ✅ 更新 DOCUMENTATION_INDEX.md（添加新文档引用）

### 2026-02-23
- ✅ 修复 Gazebo 电机话题路径问题
- ✅ 修复 test_actuators.py 字段名错误
- ✅ 创建 6 个诊断工具脚本
- ✅ 编写完整的技术文档（uORB 和 Takeoff 流程）
- ✅ 创建测试指南和快速参考

---

## 👥 贡献者

- Hnuter Team
- Claude AI (文档生成和问题诊断)

---

## 📄 许可证

本项目遵循 PX4 的 BSD 3-Clause 许可证。

---

**文档版本**: 1.1
**创建日期**: 2026-02-23
**最后更新**: 2026-02-24
**适用 PX4 版本**: v1.14+
**机型**: Hnuter Tiltrotor (5 电机 + 4 倾转舵机)

---

**祝你开发顺利！** 🚁✨
