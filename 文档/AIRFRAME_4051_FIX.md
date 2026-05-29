# 机型文件 4051_gz_hnuter 修复说明

## 问题描述

### 原始问题
原来的 `4051_gz_hnuter` 机型文件在第92行包含了 `9001_hnuter_tiltrotor` 的配置：

```bash
# Include base hnuter configuration
. ${R}etc/init.d-posix/airframes/9001_hnuter_tiltrotor
```

### 造成的问题

1. **参数重复设置**：两个文件都设置了相同的参数（如 `CBRK_SUPPLY_CHK`），导致参数被设置两次
2. **配置冲突**：两个文件都包含 `rc.mc_defaults`，可能导致加载顺序问题
3. **逻辑不清晰**：4051 应该是一个独立的 Gazebo 仿真机型配置，不应该依赖另一个机型文件
4. **维护困难**：修改配置时需要同时考虑两个文件的内容

## 修复方案

### 解决思路
将 `9001_hnuter_tiltrotor` 中的所有必要配置合并到 `4051_gz_hnuter` 中，使其成为一个完整独立的机型文件。

### 修复内容

#### ✅ 删除的内容
- 删除了第92行对 `9001_hnuter_tiltrotor` 的包含语句

#### ✅ 新增的内容
从 `9001_hnuter_tiltrotor` 合并了以下配置：

1. **车辆类型**
   ```bash
   param set-default MAV_TYPE 2  # Quadrotor
   ```

2. **控制分配器配置**
   ```bash
   param set-default CA_AIRFRAME 16  # Multicopter
   param set-default CA_ROTOR_COUNT 5
   ```

3. **5个电机的物理参数**（位置和力矩系数）
   - Motor 0: xy1 (right arm upper, CCW)
   - Motor 1: xy2 (right arm lower, CW)
   - Motor 2: xy3 (left arm upper, CW)
   - Motor 3: xy4 (left arm lower, CCW)
   - Motor 4: xy5 (rear, CW)

4. **倾转舵机配置**
   ```bash
   param set-default CA_SV_TL_COUNT 0  # 测试时禁用
   # 4个倾转舵机的详细参数（TL0-TL3）
   ```

5. **多旋翼控制器参数**
   ```bash
   param set-default MC_ROLL_P 6.0
   param set-default MC_PITCH_P 6.0
   param set-default MC_YAW_P 4.0
   param set-default MC_AIRMODE 1
   param set-default MC_YAWRATE_P 0.4
   ```

6. **VTOL 配置**
   ```bash
   param set-default VT_TYPE 1  # Tiltrotor
   param set-default VT_FWD_THRUST_EN 4  # 使用后置电机作为前向推力
   param set-default VT_FWD_THRUST_SC 0.6
   param set-default VT_TILT_TRANS 0.6
   ```

7. **传感器配置**
   ```bash
   param set-default SYS_HAS_NUM_ASPD 0  # 禁用空速计要求
   ```

8. **EKF2 配置**
   ```bash
   param set-default EKF2_GPS_CHECK 21
   param set-default EKF2_REQ_GPS_H 10
   ```

9. **解锁和安全检查参数**
   ```bash
   param set-default COM_ARM_WO_GPS 1
   param set-default COM_ARM_EKF_VEL 0.5
   param set-default COM_ARM_EKF_POS 1.0
   param set-default COM_ARM_EKF_HGT 1.0
   param set-default COM_ARM_EKF_AB 0.005
   param set-default COM_ARM_EKF_GB 0.002
   param set-default COM_ARM_MAG_ANG 60
   ```

10. **任务参数**
    ```bash
    param set-default MIS_TAKEOFF_ALT 10
    ```

#### ✅ 保留的内容
保留了 4051 原有的 Gazebo 特定配置：
- Gazebo 仿真器设置
- Gazebo 传感器使能
- Gazebo 执行器映射（电机和舵机）
- 电机速度限制
- 舵机位置限制
- 手动控制配置
- 飞行模式配置

## 修复后的文件结构

```
4051_gz_hnuter (243 行)
├── rc.mc_defaults (唯一的外部包含)
├── Gazebo 仿真器配置
│   ├── PX4_SIMULATOR=gz
│   ├── PX4_GZ_WORLD
│   ├── PX4_SIM_MODEL
│   └── 传感器使能
├── Gazebo 执行器映射
│   ├── 5个电机映射 (SIM_GZ_EC_FUNC1-5)
│   ├── 电机速度限制 (SIM_GZ_EC_MIN/MAX)
│   ├── 4个舵机映射 (SIM_GZ_SV_FUNC1-4)
│   └── 舵机位置限制 (SIM_GZ_SV_MAXA/MINA)
├── 车辆类型和控制分配
│   ├── MAV_TYPE
│   ├── CA_AIRFRAME
│   ├── CA_ROTOR_COUNT
│   └── 5个电机的物理参数 (CA_ROTOR0-4)
├── 倾转舵机配置
│   ├── CA_SV_TL_COUNT
│   └── 4个舵机参数 (CA_SV_TL0-3)
├── 多旋翼控制器参数
│   └── MC_* 参数
├── VTOL 配置
│   └── VT_* 参数
├── 传感器配置
│   └── SYS_HAS_NUM_ASPD
├── EKF2 配置
│   └── EKF2_* 参数
├── 解锁和安全检查
│   ├── COM_ARM_* 参数
│   ├── CBRK_* 参数
│   └── FD_* 参数
├── 手动控制配置
│   └── RC_MAP_* 参数
├── 飞行模式配置
│   └── COM_FLTMODE* 参数
├── 安全动作配置
│   ├── COM_LOW_BAT_ACT
│   └── NAV_RCL_ACT
└── 任务参数
    └── MIS_TAKEOFF_ALT
```

## 对比分析

### 修复前
```
4051_gz_hnuter (93 行)
├── Gazebo 特定配置
└── ┗━ 包含 9001_hnuter_tiltrotor (139 行)
    ├── 车辆和控制器配置
    └── 安全检查配置

总配置加载：93 + 139 = 232 行
存在重复参数：CBRK_SUPPLY_CHK, PX4_SIMULATOR 等
```

### 修复后
```
4051_gz_hnuter (243 行)
└── 完整独立的配置（无外部依赖）

总配置加载：243 行
无重复参数，逻辑清晰
```

## 验证方法

### 1. 检查文件内容
```bash
cat /home/hnuter/PX4-Autopilot-Hnuter/ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter
```

### 2. 确认无包含语句
```bash
grep "9001_hnuter_tiltrotor" /home/hnuter/PX4-Autopilot-Hnuter/ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter
# 应该无输出，说明已删除包含语句
```

### 3. 测试仿真启动
```bash
cd /home/hnuter/PX4-Autopilot-Hnuter
make px4_sitl gz_hnuter
```

### 4. 检查参数加载
在 PX4 控制台中：
```bash
param show CA_ROTOR_COUNT    # 应显示 5
param show CA_AIRFRAME       # 应显示 16
param show VT_TYPE           # 应显示 1
param show MAV_TYPE          # 应显示 2
```

## 配置完整性检查清单

- [x] 基础配置 (rc.mc_defaults)
- [x] Gazebo 仿真器设置
- [x] 传感器使能 (GPS, BARO, BAT)
- [x] 电机执行器映射 (5个电机)
- [x] 舵机执行器映射 (4个舵机)
- [x] 车辆类型 (MAV_TYPE)
- [x] 控制分配器 (CA_AIRFRAME, CA_ROTOR_COUNT)
- [x] 电机物理参数 (位置和力矩系数)
- [x] 倾转舵机参数 (已禁用但已配置)
- [x] MC 控制器参数
- [x] VTOL 配置
- [x] EKF2 配置
- [x] 解锁检查配置
- [x] 安全检查配置
- [x] 手动控制配置
- [x] 飞行模式配置
- [x] 任务参数

## 9001 文件状态

`9001_hnuter_tiltrotor` 文件**保持不变**，仍然可以作为：
1. 非 Gazebo 仿真的基础配置
2. 硬件飞控的机型配置
3. 其他机型的参考模板

但 `4051_gz_hnuter` 现在**不再依赖**它，是一个独立完整的 Gazebo 仿真机型配置。

## 优势

### 修复前的问题
- ❌ 参数重复设置
- ❌ 配置依赖关系复杂
- ❌ 难以追踪参数来源
- ❌ 维护困难

### 修复后的优势
- ✅ 配置独立完整
- ✅ 无参数重复
- ✅ 逻辑清晰明了
- ✅ 易于维护和修改
- ✅ 更好的可读性（分节注释）

## 注意事项

1. **编译**：修改机型文件后需要重新编译
   ```bash
   cd /home/hnuter/PX4-Autopilot-Hnuter
   make clean
   make px4_sitl gz_hnuter
   ```

2. **参数持久化**：如果之前运行过仿真，可能有持久化的参数
   ```bash
   # 清除旧参数（可选）
   rm -rf /tmp/rootfs/eeprom.txt
   ```

3. **倾转舵机**：当前 `CA_SV_TL_COUNT=0`（已禁用）
   - 要启用倾转舵机，设置 `CA_SV_TL_COUNT=4`
   - 或使用工具脚本：`~/px4_ros2_ws/enable_tilt_servos.sh`

## 相关文档

- `PX4_BUILD_AND_LAUNCH_PROCESS.md` - PX4 构建和启动流程详解
- `DOCUMENTATION_INDEX.md` - 完整文档索引
- `HNUTER_GAZEBO_TESTING_GUIDE.md` - Gazebo 测试指南

## 修复时间

- **修复日期**: 2026-02-24
- **修复前行数**: 93 行
- **修复后行数**: 243 行
- **增加配置**: 150 行

---

**总结**：4051_gz_hnuter 现在是一个完整、独立、清晰的 Gazebo 仿真机型配置文件，不再依赖 9001_hnuter_tiltrotor。
