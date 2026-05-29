# 使用Hnuter倾转旋翼机型

## 1. 构建PX4固件

首先，确保你已经构建了PX4固件：

```bash
cd /home/hnuter/PX4-Autopilot-Hnuter
make clean
make px4_sitl_default
```

## 2. 启动PX4 SITL

使用以下命令启动PX4 SITL：

```bash
cd build/px4_sitl_default
./bin/px4 ./etc/init.d-posix/rcS -d
```

## 3. 设置Hnuter机型配置

PX4启动后，使用以下命令设置Hnuter机型：

```bash
./bin/px4-param set CA_AIRFRAME 16
```

## 4. 验证配置

检查配置是否生效：

```bash
./bin/px4-param show CA_AIRFRAME
```

你应该看到输出显示CA_AIRFRAME的值为16。

## 5. 检查系统状态

使用以下命令检查系统状态：

```bash
./bin/px4-commander status
./bin/px4-listener vehicle_attitude
```

## 6. 使用QGroundControl连接

打开QGroundControl，它应该自动连接到SITL实例。你可以在QGroundControl中查看和修改参数，以及测试飞行模式。

## 7. 配置说明

Hnuter机型配置了：
- 5个电机（前左上下、前右上下、尾部推进）
- 4个倾转伺服（左倾转俯仰、右倾转俯仰、右倾转滚转、左倾转滚转）
- 自定义的控制分配矩阵
- 优化的PID控制器参数

## 8. 飞行测试

在QGroundControl中，你可以：
- 进行姿态模式测试
- 进行位置模式测试
- 测试从多旋翼模式到固定翼模式的转换
- 测试各种飞行模式

## 9. 日志分析

飞行测试后，你可以使用以下命令下载和分析日志：

```bash
./bin/px4-logger stop
./bin/px4-logger download
```

然后使用PX4日志分析工具（如Flight Review）查看日志。

## 注意事项

- 确保你的系统满足PX4的硬件要求
- 首次飞行前，进行全面的预飞检查
- 从低风险的测试开始，逐步增加复杂度
- 飞行过程中密切关注系统状态

## 10. Gazebo仿真

### 10.1 使用单个命令启动完整仿真

使用以下命令启动完整的Gazebo仿真（推荐方法）。**注意**：必须从项目根目录执行，且需先完成 `make px4_sitl_default` 构建。

```bash
cd /home/hnuter/PX4-Autopilot-Hnuter

# 推荐方式：使用 make 命令（会自动处理工作目录和环境变量）
make px4_sitl gz_hnuter

# 或手动启动（需从 rootfs 目录运行，以便加载 gz_env.sh）
cd build/px4_sitl_default/rootfs
PX4_SYS_AUTOSTART=4051 PX4_SIMULATOR=gz PX4_GZ_WORLD=default PX4_SIM_MODEL=hnuter ../bin/px4
```

这个命令会：
1. 自动启动Gazebo模拟器
2. 加载默认世界环境
3. 生成hnuter模型
4. 启动PX4 SITL
5. 自动连接PX4到Gazebo

### 10.2 验证仿真启动

当你运行上述命令时，你应该看到类似以下的输出：

```
INFO  [init] Gazebo simulator 8.10.0
INFO  [init] Starting gazebo with world: /home/hnuter/PX4-Autopilot-Hnuter/Tools/simulation/gz/worlds/default.sdf
INFO  [init] Starting gz gui
INFO  [init] Gazebo world is ready
INFO  [init] Spawning Gazebo model
INFO  [gz_bridge] world: default, model: hnuter_0
INFO  [lockstep_scheduler] setting initial absolute time to 1100000 us
```

这表明Gazebo仿真已成功启动，并且PX4已经连接到仿真环境。

### 10.3 控制仿真飞行器

使用QGroundControl连接到仿真飞行器：

1. 打开QGroundControl
2. 它应该自动连接到SITL实例
3. 在QGroundControl中，你可以：
   - 进行姿态模式测试
   - 进行位置模式测试
   - 测试从多旋翼模式到固定翼模式的转换
   - 测试各种飞行模式

### 10.4 调整仿真参数

你可以使用以下命令调整仿真参数：

```bash
# 调整控制器参数
./bin/px4-param set MC_ROLL_P 6.0
./bin/px4-param set MC_PITCH_P 6.0
./bin/px4-param set MC_YAW_P 4.0

# 调整位置控制器参数
./bin/px4-param set POSCTL_XY_P 1.0
./bin/px4-param set POSCTL_Z_P 2.0

# 调整倾转参数
./bin/px4-param set CA_TILT_MAX_ANGLE 90.0
./bin/px4-param set CA_TILT_MIN_ANGLE 0.0
```

### 10.5 仿真测试建议

1. **姿态控制测试**：
   - 在姿态模式下测试基本的俯仰、滚转和偏航控制
   - 验证倾转机构是否正常工作

2. **位置控制测试**：
   - 在位置模式下测试悬停
   - 测试位置保持能力
   - 测试定点飞行

3. **模式转换测试**：
   - 测试从多旋翼模式到固定翼模式的转换
   - 测试从固定翼模式到多旋翼模式的转换

4. **风速测试**：
   - 调整风速参数，测试飞行器在有风条件下的稳定性

### 10.6 仿真日志分析

仿真测试后，你可以使用以下命令下载和分析日志：

```bash
./bin/px4-logger stop
./bin/px4-logger download
```

然后使用PX4日志分析工具（如Flight Review）查看日志。

祝你仿真测试愉快！
