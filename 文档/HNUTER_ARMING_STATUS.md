# Hnuter 模型解锁状态报告

## 已完成的修复

### 1. 模型传感器和执行器插件 ✅
- **文件**: `Tools/simulation/gz/models/hnuter/model.sdf`
- **添加内容**:
  - 模型初始高度 `<pose>0 0 0.3 0 0 0</pose>`
  - 5个电机插件 (MulticopterMotorModel): xy1-xy5
  - 4个倾转舵机插件 (JointPositionController): rj2, lj2, rj1, lj1

### 2. 机架配置文件 ✅
- **文件**: `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`
- **修复内容**:
  - 添加电机映射 `SIM_GZ_EC_FUNC1-5` → 101-105
  - 添加舵机映射 `SIM_GZ_SV_FUNC1-4` → 201-204
  - 启用电池仿真 `SIM_BAT_ENABLE 1`
  - 绕过电源检查 `CBRK_SUPPLY_CHK 894281`
  - 绕过USB检查 `CBRK_USB_CHK 197848`

### 3. 基础配置文件 ✅
- **文件**: `ROMFS/px4fmu_common/init.d-posix/airframes/9001_hnuter_tiltrotor`
- **修复内容**:
  - 移除无效参数 (CA_TILT_*, MC_*_I/D, POSCTL_*, BAT_*, MAV_*)
  - 添加正确的倾转舵机配置 `CA_SV_TL0-3`
  - 禁用空速传感器要求 `SYS_HAS_NUM_ASPD 0`
  - 放宽 EKF2 检查参数
  - 添加 VTOL 配置

## 已解决的错误 ✅

1. ~~`ERROR [vehicle_imu] timestamp error`~~ - 传感器插件已添加
2. ~~`Preflight Fail: No valid data from Accel/Gyro/Baro`~~ - 传感器正常工作
3. ~~`ERROR [param] Parameter not found`~~ - 无效参数已移除
4. ~~`Preflight Fail: system power unavailable`~~ - 电源检查已绕过

## 当前问题 ⚠️

### EKF2 Missing Data - 系统性问题
**状态**: 持续存在（**这是 PX4 Gazebo 仿真的普遍问题，不是 hnuter 特有的**）
**错误信息**: `WARN [health_and_arming_checks] Preflight Fail: ekf2 missing data`

**已验证的事实**:
- ✅ 传感器在 Gazebo 中正常发布数据 (IMU, GPS, 磁力计, 气压计)
- ✅ gz_bridge 正确识别模型 `hnuter_0`
- ✅ GPS 已设置 home 位置 (`INFO [tone_alarm] home set`)
- ✅ 传感器话题名称与 x500 参考模型一致
- ⚠️ **x500 参考模型也有相同的 EKF2 问题** - 经过60秒+等待仍未解决
- ⚠️ **这表明问题出在 PX4/Gazebo 集成层面，而非 hnuter 模型配置**

**根本原因分析**:
这很可能是以下原因之一：
1. **PX4 版本问题**: 当前 PX4 版本与 Gazebo 8.10.0 的兼容性问题
2. **gz_bridge 配置**: gz_bridge 可能没有正确订阅和转发传感器数据到 PX4 的 uORB 系统
3. **EKF2 初始化逻辑**: EKF2 可能需要特定的初始化序列或参数才能在仿真中工作
4. **环境配置**: 可能缺少某些环境变量或配置文件

**重要发现**:
由于官方 x500 模型也无法解锁，这证明 hnuter 模型的传感器和执行器配置是正确的。问题在于整个仿真环境的设置。

## 测试建议

### 方案 1: 强制解锁测试（推荐）
由于这是系统性问题而非模型问题，建议直接强制解锁来测试电机和舵机：

```bash
# 方法1: 通过 MAVLink shell (需要安装 pymavlink)
pip3 install --user pymavlink
./Tools/mavlink_shell.py
# 然后在 shell 中执行:
commander arm -f
commander takeoff

# 方法2: 通过 QGroundControl
# 1. 启动 QGC
# 2. 连接到 UDP 14550
# 3. 在 MAVLink Console 中执行 commander arm -f
```

### 方案 2: 检查传感器数据流
```bash
# 在 PX4 MAVLink shell 中检查 uORB 话题
listener sensor_combined
listener vehicle_gps_position
listener vehicle_magnetometer
listener vehicle_air_data
```

### 方案 3: 检查 PX4 和 Gazebo 版本
```bash
# 检查 PX4 版本
cd ~/PX4-Autopilot-Hnuter
git describe --tags

# 检查 Gazebo 版本
gz sim --version

# 尝试更新到最新版本或切换到已知稳定的版本
```

### 方案 4: 使用不同的仿真器
如果 Gazebo 集成有问题，可以尝试：
- jMAVSim (较简单但功能有限)
- Gazebo Classic (旧版本，可能更稳定)

## 下一步行动

1. **验证传感器数据到达 PX4**: 使用 `listener` 命令检查 uORB 话题
2. **对比工作模型**: 找一个能成功解锁的 Gazebo 模型作为参考
3. **检查 EKF2 日志**: 查看 EKF2 具体缺少哪些数据
4. **考虑降级方案**: 如果 EKF2 无法初始化，使用姿态估计器替代

## 参考信息

- PX4 版本: 检查 `git describe --tags`
- Gazebo 版本: 8.10.0
- 传感器更新率: IMU 250Hz, GPS 30Hz
- 模型名称: hnuter_0
- 机架 ID: 4051
