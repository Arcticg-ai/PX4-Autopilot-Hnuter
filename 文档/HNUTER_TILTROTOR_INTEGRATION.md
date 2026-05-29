# HNUTER倾转旋翼无人机PX4集成使用说明

## 1. 项目概述

本文档记录了HNUTER倾转旋翼无人机集成到PX4自动驾驶系统的完整过程，包括控制器框架集成、传感器系统配置、Gazebo仿真环境搭建等内容。

## 2. 系统架构

### 2.1 硬件架构
- **机身类型**: 倾转旋翼无人机
- **旋翼数量**: 5
- **控制模式**: 全驱动控制
- **传感器系统**: IMU、气压计、磁力计、GPS

### 2.2 软件架构
- **PX4版本**: v1.17.0-alpha1
- **控制架构**: 基于`hnuter_controller_frame`的自定义控制器
- **仿真环境**: Gazebo
- **通信协议**: MAVLink、uORB、uXRCE-DDS

## 3. 目录结构

```
PX4-Autopilot-Hnuter/
├── Tools/simulation/gz/models/hnuter/       # Gazebo模型文件
│   ├── model.sdf                            # SDF模型配置
│   └── meshes/                              # 3D模型文件
├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   ├── 4051_gz_hnuter                       # Gazebo仿真配置
│   └── 9001_hnuter_tiltrotor                # 基础机型配置
├── src/modules/control_allocator/           # 控制分配器
│   └── VehicleActuatorEffectiveness/        # 执行器效能模型
└── src/modules/uxrce_dds_client/            # DDS客户端模块
```

## 4. 主要修改内容

### 4.1 控制器框架集成

#### 文件: `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessFullyActuated.cpp`
- **修改内容**: 添加尾部推进器支持，适配HNUTER控制器框架
- **修改原因**: 支持倾转旋翼无人机的特殊控制需求

### 4.2 传感器系统配置

#### 文件: `Tools/simulation/gz/models/hnuter/model.sdf`
- **修改内容**:
  - 添加IMU传感器配置（更新率250Hz）
  - 添加气压计传感器配置（更新率50Hz）
  - 添加磁力计传感器配置（更新率100Hz）
  - 添加GPS传感器配置（更新率30Hz）
  - 配置传感器噪声参数，模拟真实传感器特性
- **修改原因**: 提供完整的传感器数据，支持PX4的状态估计和控制算法

### 4.3 Gazebo仿真配置

#### 文件: `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`
- **修改内容**:
  - 添加Gazebo桥接启用参数（SIM_GZ_EN=1）
  - 配置执行器函数映射（SIM_GZ_EC_FUNC1-7）
  - 设置执行器限制参数（SIM_GZ_EC_MIN/MAX）
  - 启用GPS和气压计传感器数据传输
- **修改原因**: 确保Gazebo与PX4之间的正确通信，支持执行器控制和传感器数据传输

### 4.4 基础机型配置

#### 文件: `ROMFS/px4fmu_common/init.d-posix/airframes/9001_hnuter_tiltrotor`
- **修改内容**:
  - 设置CA_AIRFRAME=16（倾转旋翼）
  - 设置CA_ROTOR_COUNT=5（5个旋翼）
  - 配置控制器参数和仿真参数
- **修改原因**: 定义倾转旋翼无人机的基本特性和控制参数

### 4.5 uXRCE-DDS客户端恢复

#### 文件: `boards/px4/sitl/default.px4board`
- **修改内容**: 将CONFIG_MODULES_UXRCE_DDS_CLIENT从n改为y
- **修改原因**: 恢复DDS通信功能，支持与外部系统的通信

## 5. 使用指南

### 5.1 构建项目

```bash
# 清理构建目录
rm -rf build/px4_sitl_default

# 构建SITL版本
sudo make px4_sitl_default
```

### 5.2 启动Gazebo仿真

```bash
# 进入构建目录
cd build/px4_sitl_default

# 启动Gazebo仿真，使用HNUTER机型
cmake --build . --target gz_hnuter
```

### 5.3 验证传感器数据

在PX4控制台中，使用以下命令检查传感器数据：

```bash
# 查看IMU和气压计数据
listener sensor_combined

# 查看磁力计数据
listener sensor_mag

# 查看气压计数据
listener sensor_baro

# 查看GPS数据
listener vehicle_gps_position
```

### 5.4 测试执行器响应

```bash
# 测试执行器1，设置值为1000（最大值）
actuator_test test 1 1000

# 测试执行器2，设置值为500（中间值）
actuator_test test 2 500

# 测试执行器3，设置值为150（最小值）
actuator_test test 3 150
```

### 5.5 查看话题列表

```bash
topic list
```

### 5.6 检查系统状态

```bash
commander status
```

### 5.7 查看参数配置

```bash
param show CA_
param show MC_
param show SIM_GZ_
```

## 6. 传感器配置详情

### 6.1 IMU传感器
- **更新率**: 250Hz
- **噪声参数**: 
  - 角速度: 0.0008726646 rad/s
  - 加速度: 0.00637 m/s² (X/Y轴), 0.00686 m/s² (Z轴)

### 6.2 气压计传感器
- **更新率**: 50Hz
- **噪声参数**: 3 Pa

### 6.3 磁力计传感器
- **更新率**: 100Hz
- **噪声参数**: 0.0001 T

### 6.4 GPS传感器
- **更新率**: 30Hz
- **配置**: 标准GPS模块

## 7. 执行器配置

### 7.1 执行器函数映射

| 执行器ID | 函数映射 | 最小值 | 最大值 |
|---------|---------|-------|-------|
| 1       | 101     | 150   | 1000  |
| 2       | 102     | 150   | 1000  |
| 3       | 103     | 150   | 1000  |
| 4       | 104     | 150   | 1000  |
| 5       | 105     | 150   | 1000  |
| 6       | 106     | 150   | 1000  |
| 7       | 107     | 150   | 1000  |

### 7.2 执行器类型
- **1-4**: 旋翼电机
- **5**: 尾部推进器
- **6-7**: 倾转轴伺服电机

## 8. 常见问题排查

### 8.1 传感器数据缺失
- **症状**: 控制台显示"Preflight Fail: barometer 0 missing"等错误
- **解决方法**: 检查SDF文件中的传感器配置，确保传感器名称和路径与GZBridge.cpp中的订阅话题匹配

### 8.2 执行器无响应
- **症状**: 执行器测试命令无效果
- **解决方法**: 检查执行器函数映射配置，确保SIM_GZ_EC_FUNC参数设置正确

### 8.3 uXRCE-DDS客户端初始化失败
- **症状**: 控制台显示"uxrce_dds_client: not found"
- **解决方法**: 确保CONFIG_MODULES_UXRCE_DDS_CLIENT=y，并且子模块已正确初始化

### 8.4 模型显示异常
- **症状**: Gazebo中模型显示异常或位置不正确
- **解决方法**: 检查SDF文件中的链接和关节配置，确保坐标系和位置参数正确

## 9. 性能优化建议

1. **传感器更新率调整**: 根据实际硬件性能调整传感器更新率
2. **控制参数调优**: 根据飞行测试结果调整MC_*和POSCTL_*参数
3. **执行器效能模型**: 根据实际电机特性调整执行器效能矩阵
4. **仿真精度**: 增加仿真步长以提高仿真精度

## 10. 未来工作

1. **实机测试**: 进行实机飞行测试，验证控制器性能
2. **参数自动调优**: 实现控制参数的自动调优
3. **故障检测与处理**: 添加故障检测与容错控制功能
4. **高级飞行模式**: 实现自主起飞、着陆、轨迹跟踪等高级飞行模式

## 11. 参考资料

- [PX4官方文档](https://docs.px4.io/)
- [Gazebo仿真指南](https://docs.px4.io/main/en/simulation/gazebo.html)
- [控制分配器文档](https://docs.px4.io/main/en/config_mc/control_allocator.html)
- [uXRCE-DDS文档](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

## 12. 版本历史

| 版本 | 日期 | 描述 |
|------|------|------|
| v1.0 | 2026-01-29 | 初始版本，完成基本集成和仿真环境搭建 |
