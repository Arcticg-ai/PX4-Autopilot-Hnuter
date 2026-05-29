# PX4 uORB 消息系统详解

## 目录
1. [uORB 概述](#uorb-概述)
2. [核心架构](#核心架构)
3. [代码实现](#代码实现)
4. [使用方法](#使用方法)
5. [消息定义](#消息定义)
6. [性能特点](#性能特点)

---

## uORB 概述

### 什么是 uORB？

**uORB** (micro Object Request Broker) 是 PX4 自主研发的轻量级**发布-订阅**消息传递系统，专为嵌入式实时系统设计。

### 设计目标

- ✅ **低延迟**: 适合实时控制系统（微秒级）
- ✅ **高效率**: 最小化内存占用和 CPU 开销
- ✅ **类型安全**: 编译期类型检查
- ✅ **模块解耦**: 发布者和订阅者互不依赖
- ✅ **零拷贝**: 支持直接内存访问（某些场景）

### 核心概念

```
┌─────────────┐     发布      ┌──────────────┐
│  Publisher  │─────────────>│  uORB Topic  │
│  (发布者)   │               │  (消息主题)  │
└─────────────┘               └──────┬───────┘
                                     │
                                     │ 订阅
                                     ▼
                              ┌─────────────┐
                              │ Subscriber  │
                              │  (订阅者)   │
                              └─────────────┘
```

**主题 (Topic)**:
- 每个主题对应一种消息类型（如 `vehicle_command`）
- 系统中可以有多个发布者和订阅者
- 支持多实例（例如多个相同传感器）

---

## 核心架构

### 文件结构

```
PX4-Autopilot/
├── platforms/common/uORB/          # uORB 核心实现
│   ├── uORB.h                       # C/C++ API 头文件 ⭐
│   ├── uORBManager.hpp              # 管理器类 ⭐
│   ├── Publication.hpp              # 发布包装类 ⭐
│   ├── Subscription.hpp             # 订阅包装类 ⭐
│   ├── SubscriptionCallback.hpp    # 回调订阅类
│   ├── SubscriptionInterval.hpp    # 定时订阅类
│   ├── SubscriptionMultiArray.hpp  # 多实例订阅类
│   ├── uORBDeviceNode.hpp/cpp      # 设备节点实现
│   └── uORBDeviceMaster.hpp/cpp    # 设备主控制器
│
├── msg/                             # 消息定义文件（244+个）
│   ├── versioned/                   # 版本化消息
│   │   ├── VehicleCommand.msg      # 飞行器命令
│   │   ├── VehicleStatus.msg       # 飞行器状态
│   │   └── ...
│   ├── PositionSetpoint.msg        # 位置设定点
│   ├── ActuatorOutputs.msg         # 执行器输出
│   └── ...
│
└── src/modules/uORB/               # uORB 实现模块
    ├── uORB_Main.cpp                # 主程序入口
    └── ...
```

### 关键类和组件

#### 1. uORB::Manager

**文件**: `platforms/common/uORB/uORBManager.hpp`

**功能**: 全局单例，管理所有 uORB 主题的注册、发布和订阅

**关键方法**:
```cpp
class Manager {
public:
    static Manager* get_instance();              // 获取单例实例

    orb_advert_t orb_advertise(const orb_metadata *meta,
                               const void *data = nullptr,
                               int *instance = nullptr,
                               unsigned int queue_size = 1);

    int orb_subscribe(const orb_metadata *meta,
                     unsigned instance = 0);

    int orb_publish(const orb_metadata *meta,
                   orb_advert_t handle,
                   const void *data);

    int orb_copy(const orb_metadata *meta,
                int handle,
                void *buffer);
};
```

#### 2. uORB::Publication<T>

**文件**: `platforms/common/uORB/Publication.hpp`

**功能**: 模板化的发布类，提供类型安全的发布接口

**示例**:
```cpp
template<typename T>
class Publication : public PublicationBase {
public:
    Publication(ORB_ID id) : PublicationBase(id) {}

    bool publish(const T &data);     // 发布数据
    bool advertise();                // 广告主题
    void unadvertise();              // 取消广告

private:
    orb_advert_t _handle{nullptr};   // 发布句柄
};
```

**使用示例**:
```cpp
// 创建发布者
uORB::Publication<vehicle_command_s> _cmd_pub{ORB_ID(vehicle_command)};

// 发布数据
vehicle_command_s cmd{};
cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF;
cmd.timestamp = hrt_absolute_time();
_cmd_pub.publish(cmd);
```

#### 3. uORB::Subscription<T>

**文件**: `platforms/common/uORB/Subscription.hpp`

**功能**: 模板化的订阅类，提供类型安全的订阅接口

**示例**:
```cpp
template<typename T>
class Subscription : public SubscriptionBase {
public:
    Subscription(ORB_ID id, unsigned instance = 0);

    bool update(T *data);            // 更新并复制数据
    bool copy(T *data);              // 复制当前数据（不检查更新）
    bool updated();                  // 检查是否有新数据
    uint64_t get_last_generation(); // 获取最后一次更新代数

private:
    int _handle{-1};                 // 订阅句柄
};
```

**使用示例**:
```cpp
// 创建订阅者
uORB::Subscription _cmd_sub{ORB_ID(vehicle_command)};

// 检查并获取新数据
vehicle_command_s cmd;
if (_cmd_sub.update(&cmd)) {
    // 处理新命令
    PX4_INFO("Received command: %u", cmd.command);
}
```

#### 4. uORB::SubscriptionCallback

**文件**: `platforms/common/uORB/SubscriptionCallback.hpp`

**功能**: 当有新数据到达时自动调用回调函数

**使用示例**:
```cpp
class MyModule : public ModuleBase<MyModule> {
    uORB::SubscriptionCallback _cmd_sub_cb{
        {ORB_ID(vehicle_command), 0},
        [this](uint8_t instance) { this->handle_command(); }
    };

    void handle_command() {
        vehicle_command_s cmd;
        if (_cmd_sub_cb.copy(&cmd)) {
            // 处理命令
        }
    }
};
```

---

## 代码实现

### uORB 内部机制

#### 1. 消息存储机制

uORB 使用**环形缓冲区 (Ring Buffer)** 存储消息：

```cpp
// 每个主题有独立的设备节点
class uORBDeviceNode {
private:
    uint8_t *_data;                    // 消息数据缓冲区
    unsigned _queue_size;              // 队列大小
    unsigned _generation;              // 当前代数（用于检测更新）

    struct SubscriberData {
        unsigned last_generation;      // 订阅者上次读取的代数
        // ...
    };

    List<SubscriberData*> _subscribers;
};
```

**工作原理**:
1. 每次发布新数据时，`_generation` 递增
2. 订阅者记录自己最后读取的 `last_generation`
3. 通过比较两者判断是否有新数据

#### 2. 零拷贝机制

对于单个订阅者的场景，uORB 支持零拷贝：

```cpp
int orb_copy(const orb_metadata *meta, int handle, void *buffer) {
    // 直接返回指向内部缓冲区的指针（某些实现）
    // 或者使用 memcpy（安全实现）
    memcpy(buffer, node->_data, meta->o_size);
}
```

#### 3. 多实例支持

同一类型的传感器可以有多个实例：

```cpp
// 订阅特定实例
uORB::Subscription gps_sub0{ORB_ID(sensor_gps), 0};  // GPS 0
uORB::Subscription gps_sub1{ORB_ID(sensor_gps), 1};  // GPS 1

// 订阅所有实例
uORB::SubscriptionMultiArray<sensor_gps_s> gps_subs{ORB_ID::sensor_gps};
for (auto &gps : gps_subs) {
    if (gps.updated()) {
        sensor_gps_s data;
        gps.copy(&data);
        // 处理数据
    }
}
```

---

## 使用方法

### 基本工作流程

#### 发布端 (Publisher)

```cpp
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_command.h>

class MyPublisher {
public:
    MyPublisher() {
        // 构造时自动创建发布者
    }

    void send_command() {
        vehicle_command_s cmd{};
        cmd.timestamp = hrt_absolute_time();
        cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF;
        cmd.param7 = 10.0f;  // 起飞高度 10米

        // 发布（自动广告）
        _cmd_pub.publish(cmd);
    }

private:
    uORB::Publication<vehicle_command_s> _cmd_pub{ORB_ID(vehicle_command)};
};
```

#### 订阅端 (Subscriber)

**方法1: 轮询方式**

```cpp
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>

class MySubscriber {
public:
    void check_commands() {
        vehicle_command_s cmd;

        // update() 返回 true 表示有新数据
        if (_cmd_sub.update(&cmd)) {
            process_command(cmd);
        }
    }

private:
    void process_command(const vehicle_command_s &cmd) {
        switch (cmd.command) {
            case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
                PX4_INFO("Takeoff command received!");
                break;
            // ...
        }
    }

    uORB::Subscription _cmd_sub{ORB_ID(vehicle_command)};
};
```

**方法2: 回调方式**

```cpp
#include <uORB/SubscriptionCallback.hpp>

class MySubscriber : public ModuleBase<MySubscriber> {
public:
    MySubscriber() {
        // 注册到事件循环
        _cmd_sub_cb.registerCallback();
    }

private:
    void handle_command() {
        vehicle_command_s cmd;
        if (_cmd_sub_cb.copy(&cmd)) {
            process_command(cmd);
        }
    }

    uORB::SubscriptionCallback _cmd_sub_cb{
        {ORB_ID(vehicle_command), 0},
        [this](uint8_t instance) { this->handle_command(); }
    };
};
```

**方法3: 定时轮询**

```cpp
#include <uORB/SubscriptionInterval.hpp>

class MySubscriber {
public:
    void run() {
        // 每 100ms 检查一次
        if (_battery_sub.updated()) {
            battery_status_s battery;
            _battery_sub.copy(&battery);
            PX4_INFO("Battery: %.1f%%", battery.remaining * 100);
        }
    }

private:
    // 构造时指定更新间隔（微秒）
    uORB::SubscriptionInterval _battery_sub{ORB_ID(battery_status), 100_ms};
};
```

### C API (低级接口)

虽然推荐使用 C++ API，但 C API 仍然可用：

```c
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>

// 发布
orb_advert_t cmd_pub = orb_advertise(ORB_ID(vehicle_command), NULL);
vehicle_command_s cmd = {};
cmd.command = VEHICLE_CMD_NAV_TAKEOFF;
orb_publish(ORB_ID(vehicle_command), cmd_pub, &cmd);

// 订阅
int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
vehicle_command_s cmd_recv;
bool updated;
orb_check(cmd_sub, &updated);
if (updated) {
    orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd_recv);
    // 处理命令
}

// 清理
orb_unadvertise(cmd_pub);
orb_unsubscribe(cmd_sub);
```

---

## 消息定义

### 消息文件格式

消息定义使用 `.msg` 文件，语法类似 ROS：

**示例**: `msg/versioned/VehicleCommand.msg`

```
# Vehicle command message
# Used to send commands to vehicles

uint64 timestamp        # 时间戳 (microseconds since system start)

float32 param1          # 参数 1
float32 param2          # 参数 2
float32 param3          # 参数 3
float32 param4          # 参数 4
float64 param5          # 参数 5 (通常是纬度)
float64 param6          # 参数 6 (通常是经度)
float32 param7          # 参数 7 (通常是高度)

uint32 command          # 命令 ID

uint8 target_system     # 目标系统 ID
uint8 target_component  # 目标组件 ID
uint8 source_system     # 来源系统 ID
uint16 source_component # 来源组件 ID

uint8 confirmation      # 0: 首次发送, 1-255: 重发次数
bool from_external      # true: 来自外部源 (GCS, companion computer)

# 命令常量
uint32 VEHICLE_CMD_NAV_TAKEOFF = 22
uint32 VEHICLE_CMD_NAV_LAND = 21
uint32 VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
# ... 更多命令
```

### 消息生成

消息定义文件在编译时自动生成 C/C++ 头文件：

```bash
# 编译时生成
msg/versioned/VehicleCommand.msg
    ↓ (CMake + genmsg)
build/uORB/topics/vehicle_command.h  # C/C++ 结构体定义

# 生成的结构体
struct vehicle_command_s {
    uint64_t timestamp;
    float param1;
    float param2;
    // ...
    uint32_t command;
    // ...

    static constexpr uint32_t VEHICLE_CMD_NAV_TAKEOFF = 22;
};
```

### 常用消息类型

| 消息名 | 文件 | 用途 | 发布频率 |
|--------|------|------|----------|
| `vehicle_command` | VehicleCommand.msg | 命令输入 | 事件驱动 |
| `vehicle_status` | VehicleStatus.msg | 系统状态 | ~2 Hz |
| `sensor_combined` | SensorCombined.msg | IMU 数据 | ~250 Hz |
| `vehicle_local_position` | VehicleLocalPosition.msg | 局部位置 | ~50 Hz |
| `vehicle_attitude` | VehicleAttitude.msg | 姿态 | ~250 Hz |
| `actuator_outputs` | ActuatorOutputs.msg | 电机输出 | ~400 Hz |
| `battery_status` | BatteryStatus.msg | 电池状态 | ~1 Hz |
| `position_setpoint_triplet` | PositionSetpointTriplet.msg | 位置设定点 | ~10 Hz |

---

## 性能特点

### 延迟分析

**典型延迟**:
- 发布到订阅通知: **< 10 微秒**
- 内存拷贝: **< 1 微秒** (小消息)
- 上下文切换: **5-20 微秒**

**实测数据** (NuttX on STM32H7):
```
Publish latency:     2-8 μs
Subscribe check:     0.5 μs
Copy data:           0.3 μs (64 bytes)
Total loop time:     ~3-10 μs
```

### 内存占用

**每个主题的内存开销**:
```
基础开销:
  - DeviceNode 结构: ~200 bytes
  - 消息缓冲区: 消息大小 × 队列长度
  - 订阅者列表: ~20 bytes/订阅者

示例计算 (vehicle_command):
  - 消息大小: 112 bytes
  - 队列长度: 8
  - 订阅者: 3

  总计 = 200 + (112 × 8) + (20 × 3)
       = 200 + 896 + 60
       = 1156 bytes (~1.1 KB)
```

### CPU 占用

**发布操作** (~10-20 CPU cycles):
1. 获取设备节点
2. 递增 generation
3. 复制数据到环形缓冲区
4. 通知等待的订阅者

**订阅检查** (~5 CPU cycles):
1. 比较 generation
2. 返回结果

### 优化建议

#### 1. 减少发布频率
```cpp
// 不好: 每帧都发布
void run() {
    vehicle_status_s status{};
    _status_pub.publish(status);  // 400 Hz
}

// 好: 按需发布
void run() {
    if (status_changed) {
        vehicle_status_s status{};
        _status_pub.publish(status);  // ~2 Hz
    }
}
```

#### 2. 使用 update() 而不是 updated() + copy()
```cpp
// 不好: 两次系统调用
if (_cmd_sub.updated()) {
    vehicle_command_s cmd;
    _cmd_sub.copy(&cmd);
}

// 好: 一次系统调用
vehicle_command_s cmd;
if (_cmd_sub.update(&cmd)) {
    // 处理
}
```

#### 3. 使用回调避免轮询
```cpp
// 不好: 主循环轮询
void run() {
    while (true) {
        if (_cmd_sub.updated()) { /* ... */ }
        usleep(1000);  // 浪费 CPU
    }
}

// 好: 事件驱动
uORB::SubscriptionCallback _cmd_sub_cb{
    {ORB_ID(vehicle_command), 0},
    [this](uint8_t) { this->handle_command(); }
};
```

---

## 调试和监控

### uORB 命令行工具

```bash
# 列出所有主题
uorb top

# 监听特定主题
uorb listener vehicle_command

# 打印主题统计
uorb status

# 测试发布
uorb test
```

### 日志记录

所有 uORB 消息都可以记录到 ULog 文件：

```bash
# 启用日志
logger on

# 下载日志
# 使用 FlightPlot 或 PlotJuggler 分析
```

---

## 与 ROS2 集成

PX4 uORB 可以通过 **px4_ros2_bridge** 桥接到 ROS2:

```
PX4 uORB                    ROS2 DDS
    │                           │
    ├─ vehicle_command ────────>├─ px4_msgs/VehicleCommand
    ├─ vehicle_status ─────────>├─ px4_msgs/VehicleStatus
    └─ sensor_combined ────────>└─ px4_msgs/SensorCombined
```

**示例**: 从 ROS2 发送命令到 PX4

```python
# ROS2 节点
import rclpy
from px4_msgs.msg import VehicleCommand

class CommandPublisher:
    def __init__(self):
        self.pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

    def send_takeoff(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        cmd.param7 = 10.0  # 10米高度
        self.pub.publish(cmd)
```

---

## 总结

**uORB 优势**:
- ✅ 极低延迟 (< 10μs)
- ✅ 类型安全的 C++ API
- ✅ 轻量级内存占用
- ✅ 支持多实例
- ✅ 与 ROS2 兼容

**适用场景**:
- 嵌入式实时系统
- 飞行控制器
- 机器人控制系统
- 任何需要低延迟消息传递的场景

**参考资源**:
- [PX4 uORB 官方文档](https://docs.px4.io/main/en/middleware/uorb.html)
- [消息定义文件](https://github.com/PX4/PX4-Autopilot/tree/main/msg)
- [uORB 源码](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/common/uORB)

---

## 从 uORB 消息到执行器输出的完整流程

本章详细说明 PX4 如何从接收控制命令（uORB 消息）到最终输出执行器指令的完整处理流程。

### 整体数据流架构

```
控制器输出 (Attitude/Position Controllers)
    ↓
vehicle_torque_setpoint + vehicle_thrust_setpoint (uORB)
    ↓
Control Allocator 模块 (混控计算)
    ↓
actuator_motors + actuator_servos (uORB)
    ↓
Mixing Output 模块 (输出管理)
    ↓
驱动层 (PWM/Gazebo Bridge)
    ↓
物理执行 (电机/舵机)
```

---

### 阶段一: 控制器输出

#### 1.1 姿态控制器输出

**文件**: `src/modules/mc_att_control/mc_att_control_main.cpp`

姿态控制器计算所需的转矩和推力：

```cpp
void MulticopterAttitudeControl::Run() {
    // 计算转矩设定值
    vehicle_torque_setpoint_s torque_sp{};
    torque_sp.timestamp = hrt_absolute_time();
    torque_sp.xyz[0] = roll_torque;    // Roll 转矩
    torque_sp.xyz[1] = pitch_torque;   // Pitch 转矩
    torque_sp.xyz[2] = yaw_torque;     // Yaw 转矩
    _vehicle_torque_setpoint_pub.publish(torque_sp);

    // 计算推力设定值
    vehicle_thrust_setpoint_s thrust_sp{};
    thrust_sp.timestamp = hrt_absolute_time();
    thrust_sp.xyz[0] = 0.0f;           // X 轴推力
    thrust_sp.xyz[1] = 0.0f;           // Y 轴推力
    thrust_sp.xyz[2] = -thrust_z;      // Z 轴推力（向下为正）
    _vehicle_thrust_setpoint_pub.publish(thrust_sp);
}
```

#### 1.2 消息定义

**vehicle_torque_setpoint** (`msg/VehicleTorqueSetpoint.msg`):
```
uint64 timestamp        # 系统启动以来的微秒数
uint64 timestamp_sample # 数据采样时间戳

float32[3] xyz          # 转矩设定值 [Nm] (归一化)
                        # [0]: Roll 转矩
                        # [1]: Pitch 转矩
                        # [2]: Yaw 转矩
```

**vehicle_thrust_setpoint** (`msg/VehicleThrustSetpoint.msg`):
```
uint64 timestamp        # 系统启动以来的微秒数
uint64 timestamp_sample # 数据采样时间戳

float32[3] xyz          # 推力设定值 [N] (归一化, [-1, 1])
                        # [0]: X 轴推力 (前进)
                        # [1]: Y 轴推力 (右侧)
                        # [2]: Z 轴推力 (向下)
```

---

### 阶段二: 控制分配 (Control Allocator)

#### 2.1 模块概述

**文件位置**: `src/modules/control_allocator/`

**核心职责**:
1. 接收转矩和推力设定值
2. 通过混控矩阵计算执行器指令
3. 处理执行器饱和和故障
4. 发布执行器控制指令

#### 2.2 主循环

**文件**: `src/modules/control_allocator/ControlAllocator.cpp` (第 393 行)

```cpp
void ControlAllocator::Run() {
    bool do_update = false;

    // 1️⃣ 接收转矩设定值
    vehicle_torque_setpoint_s vehicle_torque_setpoint;
    if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
        _torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);
        do_update = true;
        _timestamp_sample = vehicle_torque_setpoint.timestamp_sample;
    }

    // 2️⃣ 接收推力设定值
    vehicle_thrust_setpoint_s vehicle_thrust_setpoint;
    if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
        _thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);
        do_update = true;
    }

    if (do_update) {
        // 3️⃣ 执行混控分配
        allocate_control();

        // 4️⃣ 发布执行器指令
        publish_actuator_controls();
    }
}
```

#### 2.3 混控矩阵分配

**文件**: `src/modules/control_allocator/ControlAllocator.cpp` (第 413 行)

```cpp
void ControlAllocator::allocate_control() {
    // 构建控制设定值向量
    matrix::Vector<float, NUM_AXES> control_sp;
    control_sp(0) = _torque_sp(0);    // Roll 转矩
    control_sp(1) = _torque_sp(1);    // Pitch 转矩
    control_sp(2) = _torque_sp(2);    // Yaw 转矩
    control_sp(3) = _thrust_sp(0);    // X 推力
    control_sp(4) = _thrust_sp(1);    // Y 推力
    control_sp(5) = _thrust_sp(2);    // Z 推力

    // 更新执行器效果矩阵（如果需要）
    _actuator_effectiveness->updateSetpoint(
        control_sp,
        matrix_index,
        actuator_sp,
        ...
    );

    // 执行控制分配算法
    _control_allocation[matrix_index]->setControlSetpoint(control_sp);
    _control_allocation[matrix_index]->allocate();

    // 应用斜率限制（平滑输出）
    if (_has_slew_rate) {
        _control_allocation[matrix_index]->applySlewRateLimit(dt);
    }

    // 裁剪到执行器范围
    _control_allocation[matrix_index]->clipActuatorSetpoint();
}
```

#### 2.4 混控矩阵原理

控制分配的核心是求解以下方程：

```
τ = B * u

其中：
τ = [τ_roll, τ_pitch, τ_yaw, F_x, F_y, F_z]^T  (6×1) 控制向量
B = 效果矩阵 (6×N)                              每列表示一个执行器的贡献
u = [u_1, u_2, ..., u_N]^T                     (N×1) 执行器指令

求解：u = B^† * τ + trim

其中 B^† 是 B 的伪逆矩阵
```

**以 Hnuter 为例** (5 电机):

```
[τ_roll ]   [b00 b01 b02 b03 b04]   [u_motor1]
[τ_pitch]   [b10 b11 b12 b13 b14]   [u_motor2]
[τ_yaw  ] = [b20 b21 b22 b23 b24] * [u_motor3]
[F_x    ]   [b30 b31 b32 b33 b34]   [u_motor4]
[F_y    ]   [b40 b41 b42 b43 b44]   [u_motor5]
[F_z    ]   [b50 b51 b52 b53 b54]

每列 b_i = [电机 i 对各轴的贡献]
b_i 由电机位置、方向、推力系数、力矩系数决定
```

#### 2.5 执行器效果矩阵

**文件**: `src/lib/control_allocation/actuator_effectiveness/ActuatorEffectiveness.cpp`

效果矩阵的每一列由以下参数计算：

```cpp
// 对于旋翼 i：
Vector3f position(CA_ROTOR${i}_PX, CA_ROTOR${i}_PY, CA_ROTOR${i}_PZ);
Vector3f axis(CA_ROTOR${i}_AX, CA_ROTOR${i}_AY, CA_ROTOR${i}_AZ);
float thrust_coef = CA_ROTOR${i}_CT;
float moment_coef = CA_ROTOR${i}_KM;

// 计算力矩贡献
Vector3f torque = position.cross(axis * thrust_coef) + axis * moment_coef;

// 效果矩阵列
effectiveness_matrix(0, i) = torque(0);      // Roll
effectiveness_matrix(1, i) = torque(1);      // Pitch
effectiveness_matrix(2, i) = torque(2);      // Yaw
effectiveness_matrix(3, i) = axis(0) * thrust_coef;  // F_x
effectiveness_matrix(4, i) = axis(1) * thrust_coef;  // F_y
effectiveness_matrix(5, i) = axis(2) * thrust_coef;  // F_z
```

#### 2.6 控制分配算法

**伪逆法** (`src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp`):

```cpp
void ControlAllocationPseudoInverse::allocate() {
    // u = B^† * τ
    _actuator_sp = _effectiveness_inv * _control_sp;

    // 裁剪到 [min, max]
    for (int i = 0; i < _num_actuators; i++) {
        _actuator_sp(i) = math::constrain(
            _actuator_sp(i),
            _actuator_min(i),
            _actuator_max(i)
        );
    }
}
```

**顺序去饱和法** (`src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp`):

```cpp
void ControlAllocationSequentialDesaturation::allocate() {
    // 步骤1: 初始分配
    _actuator_sp = _effectiveness_inv * _control_sp;

    // 步骤2: 识别饱和执行器
    bool saturated[MAX_ACTUATORS];
    for (int i = 0; i < _num_actuators; i++) {
        if (_actuator_sp(i) < _actuator_min(i)) {
            _actuator_sp(i) = _actuator_min(i);
            saturated[i] = true;
        } else if (_actuator_sp(i) > _actuator_max(i)) {
            _actuator_sp(i) = _actuator_max(i);
            saturated[i] = true;
        }
    }

    // 步骤3: 重新分配未饱和执行器（迭代）
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        // 构建缩减矩阵（只包含未饱和执行器）
        MatrixXf B_reduced = buildReducedMatrix(saturated);

        // 计算残差控制
        VectorXf control_residual = _control_sp - B * _actuator_sp;

        // 重新分配
        VectorXf delta = B_reduced_inv * control_residual;

        // 更新未饱和执行器
        updateUnsaturatedActuators(delta, saturated);

        // 检查收敛
        if (control_residual.norm() < TOLERANCE) break;
    }
}
```

#### 2.7 发布执行器指令

**文件**: `src/modules/control_allocator/ControlAllocator.cpp` (第 658 行)

```cpp
void ControlAllocator::publish_actuator_controls() {
    // 发布电机指令
    actuator_motors_s actuator_motors{};
    actuator_motors.timestamp = hrt_absolute_time();
    actuator_motors.timestamp_sample = _timestamp_sample;

    for (int i = 0; i < _num_motors; i++) {
        float value = _control_allocation[0]->getActuatorSetpoint()(i);
        actuator_motors.control[i] = PX4_ISFINITE(value) ? value : NAN;
    }

    // 设置可反向标志
    actuator_motors.reversible_flags = _reversible_mask;

    _actuator_motors_pub.publish(actuator_motors);

    // 发布舵机指令
    if (_num_servos > 0) {
        actuator_servos_s actuator_servos{};
        actuator_servos.timestamp = hrt_absolute_time();
        actuator_servos.timestamp_sample = _timestamp_sample;

        for (int i = 0; i < _num_servos; i++) {
            int servo_idx = _num_motors + i;
            float value = _control_allocation[0]->getActuatorSetpoint()(servo_idx);
            actuator_servos.control[i] = PX4_ISFINITE(value) ? value : NAN;
        }

        _actuator_servos_pub.publish(actuator_servos);
    }
}
```

#### 2.8 消息定义

**actuator_motors** (`msg/ActuatorMotors.msg`):
```
uint64 timestamp         # [us] 系统启动以来的微秒数
uint64 timestamp_sample  # [us] 数据采样时间戳

uint16 reversible_flags  # 位掩码，指示哪些电机可反向
                         # bit i = 1: 电机 i 可反向

float32[12] control      # 归一化推力 [-1, 1] 或 [0, 1]
                         # 1 = 最大正推力
                         # -1 = 最大负推力 (如果支持)
                         # 0 = 怠速
                         # NaN = 消音 (停止电机)
```

**actuator_servos** (`msg/ActuatorServos.msg`):
```
uint64 timestamp         # [us] 系统启动以来的微秒数
uint64 timestamp_sample  # [us] 数据采样时间戳

float32[8] control       # 归一化位置 [-1, 1]
                         # 1 = 最大正位置 (CA_SV_TL*_MAXA)
                         # -1 = 最大负位置 (CA_SV_TL*_MINA)
                         # 0 = 中立位置
                         # NaN = 消音
```

---

### 阶段三: 混合输出 (Mixing Output)

#### 3.1 模块概述

**文件位置**: `src/lib/mixer_module/`

**核心职责**:
1. 管理输出功能映射（每个输出通道的功能）
2. 处理最小/最大值、故障安全值、消音值
3. 应用斜率限制器（防止突变）
4. 调用驱动特定的 `updateOutputs()` 方法
5. 支持执行器测试

#### 3.2 MixingOutput 类

**文件**: `src/lib/mixer_module/mixer_module.hpp`

```cpp
class MixingOutput {
public:
    // 更新输出
    bool update();

    // 设置输出功能映射
    void setFunction(int output_idx, int function);

    // 应用斜率限制
    void applySlewRateLimit(float *output, float dt);

    // 订阅执行器消息
    void updateSubscriptions(bool force = false);

private:
    // 输出功能数组
    OutputFunction _function[MAX_ACTUATORS];

    // 执行器限制
    float _actuator_min[MAX_ACTUATORS];
    float _actuator_max[MAX_ACTUATORS];
    float _disarmed_value[MAX_ACTUATORS];
    float _failsafe_value[MAX_ACTUATORS];

    // 斜率限制器
    SlewRate<float> _slew_rate[MAX_ACTUATORS];
};
```

#### 3.3 输出功能映射

每个输出通道可以配置为不同的功能：

```cpp
enum OutputFunction {
    None = 0,

    // 电机 (101-112)
    Motor1 = 101,
    Motor2 = 102,
    // ...
    Motor12 = 112,

    // 舵机 (201-208)
    Servo1 = 201,
    Servo2 = 202,
    // ...
    Servo8 = 208,

    // 其他功能
    Gimbal_Roll = 301,
    Gimbal_Pitch = 302,
    // ...
};
```

**配置示例** (Hnuter):
```bash
# 电机映射
param set SIM_GZ_EC_FUNC1 101  # 输出 1 -> 电机 1
param set SIM_GZ_EC_FUNC2 102  # 输出 2 -> 电机 2
# ...

# 舵机映射
param set SIM_GZ_SV_FUNC1 201  # 舵机输出 1 -> 舵机 1
param set SIM_GZ_SV_FUNC2 202  # 舵机输出 2 -> 舵机 2
# ...
```

#### 3.4 混合输出更新流程

```cpp
bool MixingOutput::update() {
    // 1️⃣ 订阅执行器消息
    updateSubscriptions();

    // 2️⃣ 处理电机输出
    actuator_motors_s motors;
    if (_actuator_motors_sub.update(&motors)) {
        for (int i = 0; i < MAX_MOTORS; i++) {
            int output_idx = functionToOutputIndex(Motor1 + i);
            if (output_idx >= 0) {
                float value = motors.control[i];

                // 应用斜率限制
                if (_slew_rate_enabled[output_idx]) {
                    value = _slew_rate[output_idx].update(value, dt);
                }

                // 映射到输出范围
                _output[output_idx] = mapToOutputRange(
                    value,
                    _actuator_min[output_idx],
                    _actuator_max[output_idx]
                );
            }
        }
    }

    // 3️⃣ 处理舵机输出
    actuator_servos_s servos;
    if (_actuator_servos_sub.update(&servos)) {
        for (int i = 0; i < MAX_SERVOS; i++) {
            int output_idx = functionToOutputIndex(Servo1 + i);
            if (output_idx >= 0) {
                float value = servos.control[i];

                // 应用斜率限制
                if (_slew_rate_enabled[output_idx]) {
                    value = _slew_rate[output_idx].update(value, dt);
                }

                // 映射到输出范围
                _output[output_idx] = mapToOutputRange(
                    value,
                    _actuator_min[output_idx],
                    _actuator_max[output_idx]
                );
            }
        }
    }

    // 4️⃣ 调用驱动特定的输出函数
    return updateOutputs(_output, num_outputs);
}
```

---

### 阶段四: 驱动层输出

#### 4.1 Gazebo 仿真桥接

**文件位置**: `src/modules/simulation/gz_bridge/`

##### 4.1.1 GZBridge 架构

**文件**: `src/modules/simulation/gz_bridge/GZBridge.cpp`

```cpp
class GZBridge : public ModuleBase<GZBridge> {
public:
    void Run() override;

private:
    // 混合接口
    GZMixingInterfaceESC *_mixing_interface_esc{nullptr};
    GZMixingInterfaceServo *_mixing_interface_servo{nullptr};
    GZMixingInterfaceWheel *_mixing_interface_wheel{nullptr};
};

void GZBridge::Run() {
    // 根据配置初始化混合接口
    if (_param_sim_gz_en_esc.get()) {
        _mixing_interface_esc = new GZMixingInterfaceESC(_model_name);
        _mixing_interface_esc->init();
    }

    if (_param_sim_gz_en_servo.get()) {
        _mixing_interface_servo = new GZMixingInterfaceServo(_model_name);
        _mixing_interface_servo->init();
    }

    // 主循环
    while (!should_exit()) {
        if (_mixing_interface_esc) {
            _mixing_interface_esc->Run();
        }
        if (_mixing_interface_servo) {
            _mixing_interface_servo->Run();
        }
    }
}
```

##### 4.1.2 电机输出接口

**文件**: `src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp`

```cpp
bool GZMixingInterfaceESC::init(const std::string &model_name) {
    // 订阅 Gazebo 反馈话题
    std::string motor_speed_topic = "/" + model_name + "/command/motor_speed";
    _node.Subscribe(motor_speed_topic, &GZMixingInterfaceESC::motorSpeedCallback, this);

    // 发布 Gazebo 命令话题
    _actuators_pub = _node.Advertise<gz::msgs::Actuators>(motor_speed_topic);

    return true;
}

bool GZMixingInterfaceESC::updateOutputs(
    uint16_t outputs[MAX_ACTUATORS],
    unsigned num_outputs,
    unsigned num_control_groups_updated)
{
    // 1️⃣ 构建 Gazebo Actuators 消息
    gz::msgs::Actuators rotor_velocity_message;
    rotor_velocity_message.mutable_velocity()->Resize(num_outputs, 0);

    // 2️⃣ 填充电机速度
    for (unsigned i = 0; i < num_outputs; i++) {
        if (_mixing_output.isFunctionSet(i)) {
            // outputs[i] 已经是实际速度值（来自参数映射）
            rotor_velocity_message.set_velocity(i, outputs[i]);
        }
    }

    // 3️⃣ 发布到 Gazebo
    if (_actuators_pub.Valid()) {
        return _actuators_pub.Publish(rotor_velocity_message);
    }

    return false;
}

void GZMixingInterfaceESC::motorSpeedCallback(const gz::msgs::Actuators &actuators) {
    // 接收 Gazebo 反馈（实际电机速度）
    esc_status_s esc_status{};
    esc_status.esc_count = actuators.velocity_size();

    for (int i = 0; i < actuators.velocity_size(); i++) {
        esc_status.esc[i].timestamp = hrt_absolute_time();
        esc_status.esc[i].esc_rpm = actuators.velocity(i);
        esc_status.esc_online_flags |= 1 << i;

        if (actuators.velocity(i) > 0) {
            esc_status.esc_armed_flags |= 1 << i;
        }
    }

    // 发布 ESC 状态到 PX4
    _esc_status_pub.publish(esc_status);
}
```

##### 4.1.3 舵机输出接口

**文件**: `src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp`

```cpp
bool GZMixingInterfaceServo::init(const std::string &model_name) {
    // 为每个舵机创建 Gazebo 话题
    for (int i = 0; i < MAX_SERVOS; i++) {
        std::string joint_name = "servo_" + std::to_string(i);
        std::string servo_topic = "/model/" + model_name + "/" + joint_name;

        _servos_pub.push_back(_node.Advertise<gz::msgs::Double>(servo_topic));

        // 获取角度范围
        double min_angle = get_servo_angle_min(i);  // CA_SV_TL*_MINA
        double max_angle = get_servo_angle_max(i);  // CA_SV_TL*_MAXA
        _angle_min_rad.push_back(min_angle);
        _angular_range_rad.push_back(max_angle - min_angle);
    }

    return true;
}

bool GZMixingInterfaceServo::updateOutputs(
    uint16_t outputs[MAX_ACTUATORS],
    unsigned num_outputs,
    unsigned num_control_groups_updated)
{
    bool updated = false;

    for (int i = 0; i < _servos_pub.size(); i++) {
        if (_mixing_output.isFunctionSet(i)) {
            gz::msgs::Double servo_output;

            // 将归一化值 [-1, 1] 映射到角度 [min, max]
            double output_range = _mixing_output.maxValue(i) - _mixing_output.minValue(i);
            double angle = _angle_min_rad[i] +
                          _angular_range_rad[i] *
                          (outputs[i] - _mixing_output.minValue(i)) / output_range;

            servo_output.set_data(angle);

            if (_servos_pub[i].Valid()) {
                _servos_pub[i].Publish(servo_output);
                updated = true;
            }
        }
    }

    return updated;
}
```

##### 4.1.4 Gazebo 话题

**电机话题**: `/<model_name>/command/motor_speed`
- 消息类型: `gz::msgs::Actuators`
- 字段: `repeated double velocity`

**舵机话题**: `/model/<model_name>/servo_<N>`
- 消息类型: `gz::msgs::Double`
- 字段: `double data` (角度, 弧度)

**Hnuter 示例**:
```bash
# 电机话题
/hnuter/command/motor_speed

# 舵机话题
/model/hnuter/servo_0  # rj2 (右臂主倾转)
/model/hnuter/servo_1  # lj2 (左臂主倾转)
/model/hnuter/servo_2  # rj1 (右臂副倾转)
/model/hnuter/servo_3  # lj1 (左臂副倾转)
```

#### 4.2 实际硬件 PWM 输出

**文件位置**: `src/drivers/pwm_out/`

**文件**: `src/drivers/pwm_out/pwm_out.cpp`

```cpp
void PWMOut::Run() {
    // 订阅执行器输出
    actuator_outputs_s outputs;
    if (_outputs_sub.update(&outputs)) {
        // 转换为 PWM 值
        for (int i = 0; i < _num_outputs; i++) {
            float normalized = outputs.output[i];  // [-1, 1] 或 [0, 1]

            // 映射到 PWM 范围 (通常 1000-2000 μs)
            uint16_t pwm = _pwm_min[i] +
                          (normalized - _output_min) / (_output_max - _output_min) *
                          (_pwm_max[i] - _pwm_min[i]);

            // 限幅
            pwm = math::constrain(pwm, _pwm_min[i], _pwm_max[i]);

            // 输出到硬件定时器
            up_pwm_servo_set(i, pwm);
        }
    }
}
```

---

### 配置参数详解

#### 控制分配参数 (CA_*)

**文件**: `src/modules/control_allocator/module.yaml`

| 参数分类 | 参数名 | 类型 | 说明 |
|---------|--------|------|------|
| **机型配置** | `CA_AIRFRAME` | enum | 机型选择 (0=多旋翼, 3=倾转旋翼, 16=全驱动, 17=Hnuter) |
| | `CA_METHOD` | enum | 分配算法 (0=伪逆, 1=顺序去饱和, 2=自动) |
| **电机配置** | `CA_ROTOR_COUNT` | int | 旋翼总数 (Hnuter: 5) |
| | `CA_R_REV` | bitmask | 可反向电机位掩码 |
| **电机位置** | `CA_ROTOR${i}_PX` | float | 第 i 个旋翼 X 坐标 (m, 前向) |
| | `CA_ROTOR${i}_PY` | float | 第 i 个旋翼 Y 坐标 (m, 右侧) |
| | `CA_ROTOR${i}_PZ` | float | 第 i 个旋翼 Z 坐标 (m, 向下) |
| **电机方向** | `CA_ROTOR${i}_AX` | float | 第 i 个旋翼推力方向 X 分量 |
| | `CA_ROTOR${i}_AY` | float | 第 i 个旋翼推力方向 Y 分量 |
| | `CA_ROTOR${i}_AZ` | float | 第 i 个旋翼推力方向 Z 分量 |
| **电机系数** | `CA_ROTOR${i}_CT` | float | 第 i 个旋翼推力系数 |
| | `CA_ROTOR${i}_KM` | float | 第 i 个旋翼力矩系数 (正=CCW, 负=CW) |
| **倾转舵机** | `CA_SV_TL_COUNT` | int | 倾转舵机数量 (Hnuter: 4) |
| | `CA_SV_TL${i}_MINA` | float | 第 i 个舵机最小角度 (度) |
| | `CA_SV_TL${i}_MAXA` | float | 第 i 个舵机最大角度 (度) |
| **斜率限制** | `CA_R${i}_SLEW` | float | 第 i 个电机斜率限制 (1/s) |
| | `CA_SV${i}_SLEW` | float | 第 i 个舵机斜率限制 (1/s) |

**Hnuter 配置示例**:
```bash
# 机型和算法
param set CA_AIRFRAME 17      # Hnuter
param set CA_METHOD 2         # 自动选择

# 电机数量
param set CA_ROTOR_COUNT 5

# 电机 0 (xy1, 右臂上方, CCW)
param set CA_ROTOR0_PX 0.0
param set CA_ROTOR0_PY 0.2
param set CA_ROTOR0_PZ 0.05
param set CA_ROTOR0_KM -0.05   # CCW: 负值

# 电机 1 (xy2, 右臂下方, CW)
param set CA_ROTOR1_PX 0.0
param set CA_ROTOR1_PY 0.2
param set CA_ROTOR1_PZ -0.05
param set CA_ROTOR1_KM 0.05    # CW: 正值

# ... (其他电机类似)

# 倾转舵机
param set CA_SV_TL_COUNT 4
param set CA_SV_TL0_MINA -90   # 舵机 0 最小角度
param set CA_SV_TL0_MAXA 90    # 舵机 0 最大角度
```

#### Gazebo 仿真参数 (SIM_GZ_*)

| 参数分类 | 参数名 | 说明 |
|---------|--------|------|
| **电机功能** | `SIM_GZ_EC_FUNC1-12` | 电机输出功能映射 (101-112) |
| **电机范围** | `SIM_GZ_EC_MIN1-12` | 电机最小值 (通常 10 RPM) |
| | `SIM_GZ_EC_MAX1-12` | 电机最大值 (通常 1500 RPM) |
| | `SIM_GZ_EC_DIS1-12` | 电机消音值 (0 RPM) |
| **舵机功能** | `SIM_GZ_SV_FUNC1-8` | 舵机输出功能映射 (201-208) |
| **舵机角度** | `SIM_GZ_SV_MINA1-8` | 舵机最小角度 (度) |
| | `SIM_GZ_SV_MAXA1-8` | 舵机最大角度 (度) |
| **舵机范围** | `SIM_GZ_SV_MIN1-8` | 舵机最小输出值 |
| | `SIM_GZ_SV_MAX1-8` | 舵机最大输出值 |

**Hnuter Gazebo 配置示例**:
```bash
# 电机功能映射
param set SIM_GZ_EC_FUNC1 101  # 输出 1 -> 电机 1 (xy1)
param set SIM_GZ_EC_FUNC2 102  # 输出 2 -> 电机 2 (xy2)
param set SIM_GZ_EC_FUNC3 103  # 输出 3 -> 电机 3 (xy3)
param set SIM_GZ_EC_FUNC4 104  # 输出 4 -> 电机 4 (xy4)
param set SIM_GZ_EC_FUNC5 105  # 输出 5 -> 电机 5 (xy5)

# 电机速度范围
param set SIM_GZ_EC_MIN1 10    # 最小 10 RPM
param set SIM_GZ_EC_MAX1 1500  # 最大 1500 RPM

# 舵机功能映射
param set SIM_GZ_SV_FUNC1 201  # 舵机 1 (rj2)
param set SIM_GZ_SV_FUNC2 202  # 舵机 2 (lj2)
param set SIM_GZ_SV_FUNC3 203  # 舵机 3 (rj1)
param set SIM_GZ_SV_FUNC4 204  # 舵机 4 (lj1)

# 舵机角度范围
param set SIM_GZ_SV_MINA1 -1.57  # -90° (弧度)
param set SIM_GZ_SV_MAXA1 1.57   # +90° (弧度)
```

---

### 完整数据流示例

#### 场景: 四旋翼执行 Roll 倾斜

```
步骤 1: 姿态控制器输出
────────────────────────────────────────
vehicle_torque_setpoint:
  xyz[0] = 0.1    # Roll 转矩 (向右倾斜)
  xyz[1] = 0.0    # Pitch 转矩
  xyz[2] = 0.0    # Yaw 转矩

vehicle_thrust_setpoint:
  xyz[2] = -0.5   # 垂直推力 (悬停)

步骤 2: Control Allocator 接收
────────────────────────────────────────
control_sp = [0.1, 0.0, 0.0, 0.0, 0.0, -0.5]^T

步骤 3: 混控矩阵计算
────────────────────────────────────────
效果矩阵 B (6×4, 四旋翼):
  Motor1 (前右):  [+, +, -, 0, 0, +]^T
  Motor2 (后左):  [-, -, -, 0, 0, +]^T
  Motor3 (前左):  [-, +, +, 0, 0, +]^T
  Motor4 (后右):  [+, -, +, 0, 0, +]^T

伪逆计算: u = B^† * control_sp

结果:
  u_motor1 = 0.55  (增加推力 - 右侧上升)
  u_motor2 = 0.55  (增加推力 - 左侧下降)
  u_motor3 = 0.45  (减少推力 - 左侧下降)
  u_motor4 = 0.45  (减少推力 - 右侧上升)

步骤 4: 发布 actuator_motors
────────────────────────────────────────
actuator_motors_s:
  control[0] = 0.55
  control[1] = 0.55
  control[2] = 0.45
  control[3] = 0.45
  control[4-11] = NAN

步骤 5: Gazebo Bridge 处理
────────────────────────────────────────
GZMixingInterfaceESC::updateOutputs():
  归一化值 [0.55, 0.55, 0.45, 0.45]

  映射到 RPM (SIM_GZ_EC_MIN=10, MAX=1500):
    motor1: 10 + 0.55 * (1500-10) = 829 RPM
    motor2: 10 + 0.55 * (1500-10) = 829 RPM
    motor3: 10 + 0.45 * (1500-10) = 680 RPM
    motor4: 10 + 0.45 * (1500-10) = 680 RPM

步骤 6: 发布到 Gazebo
────────────────────────────────────────
gz::msgs::Actuators:
  velocity[0] = 829
  velocity[1] = 829
  velocity[2] = 680
  velocity[3] = 680

话题: /x500/command/motor_speed

步骤 7: Gazebo 物理仿真
────────────────────────────────────────
MulticopterMotorModel 插件接收:
  计算每个电机的推力和扭矩
  施加到飞机刚体

结果:
  右侧电机推力 > 左侧电机推力
  → 产生 Roll 转矩
  → 飞机向右倾斜

步骤 8: 反馈闭环
────────────────────────────────────────
Gazebo → vehicle_attitude_groundtruth
        ↓
       EKF → vehicle_attitude
        ↓
  Attitude Controller → 调整转矩设定值
        ↓
    重复步骤 1...
```

---

### 性能和延迟分析

#### 延迟来源

| 阶段 | 延迟 | 说明 |
|------|------|------|
| 控制器计算 | ~1-2 ms | 姿态/位置控制器 PID 计算 |
| uORB 发布 | ~0.1 ms | 消息发布延迟 |
| Control Allocator | ~1-5 ms | 矩阵运算（取决于算法和执行器数量） |
| uORB 发布 | ~0.1 ms | actuator_motors/servos 发布 |
| Mixing Output | ~0.5 ms | 功能映射和限制处理 |
| Gazebo Bridge | ~1-5 ms | Gazebo Transport 消息传递 |
| Gazebo 物理 | ~10-50 ms | 物理引擎更新（取决于时间步长） |
| **总延迟** | **~15-65 ms** | 命令到物理响应 |

#### 优化建议

1. **使用回调驱动调度**:
```cpp
// 设置 MixingOutput 为自动调度
_mixing_output.setSchedulingPolicy(MixingOutput::SchedulingPolicy::Auto);
```

2. **缓存效果矩阵**:
```cpp
// 仅在参数改变时更新
if (parameters_updated()) {
    update_effectiveness_matrix();
}
```

3. **限制分配频率**:
```cpp
// Control Allocator 以 400 Hz 运行（足够）
// 不需要每个控制器周期都分配
```

4. **使用斜率限制器**:
```cpp
// 平滑输出变化，减少高频抖动
param set CA_R0_SLEW 20.0  # 20 rad/s 斜率限制
```

---

### 调试和监控

#### uORB 命令行工具

```bash
# 监听转矩设定值
uorb listener vehicle_torque_setpoint

# 监听推力设定值
uorb listener vehicle_thrust_setpoint

# 监听电机输出
uorb listener actuator_motors

# 监听舵机输出
uorb listener actuator_servos

# 监听分配状态
uorb listener control_allocator_status
```

#### Gazebo 话题监控

```bash
# 列出所有话题
gz topic -l

# 监听电机命令
gz topic -e -t /hnuter/command/motor_speed

# 监听舵机命令
gz topic -e -t /model/hnuter/servo_0
```

#### 分配状态诊断

```cpp
control_allocator_status_s status;
if (_status_sub.update(&status)) {
    if (!status.torque_setpoint_achieved) {
        PX4_WARN("未完全分配转矩: [%.2f, %.2f, %.2f]",
                 status.unallocated_torque[0],
                 status.unallocated_torque[1],
                 status.unallocated_torque[2]);
    }

    for (int i = 0; i < num_actuators; i++) {
        if (status.actuator_saturation[i] != 0) {
            PX4_WARN("执行器 %d 饱和: %d", i, status.actuator_saturation[i]);
        }
    }
}
```

---

### 故障处理

#### 电机故障

**检测** (Failure Detector):
```cpp
// 文件: src/modules/commander/failure_detector.cpp
if (motor_current[i] < MIN_CURRENT && throttle > 0.5) {
    // 电机可能故障
    motor_failure_detected[i] = true;
}
```

**处理** (Control Allocator):
```cpp
// 从效果矩阵中移除故障电机
if (_motor_failure_mask & (1 << motor_idx)) {
    // 将该电机的列设为 0
    for (int axis = 0; axis < NUM_AXES; axis++) {
        _effectiveness_matrix(axis, motor_idx) = 0.0f;
    }

    // 重新计算伪逆
    compute_pseudo_inverse();
}
```

#### 执行器饱和

**检测**:
```cpp
if (actuator_sp[i] > _actuator_max[i]) {
    actuator_saturation[i] = ACTUATOR_SATURATION_UPPER;
    actuator_sp[i] = _actuator_max[i];
} else if (actuator_sp[i] < _actuator_min[i]) {
    actuator_saturation[i] = ACTUATOR_SATURATION_LOWER;
    actuator_sp[i] = _actuator_min[i];
}
```

**影响**:
- 控制性能下降（无法实现期望的转矩/推力）
- `control_allocator_status` 中 `*_setpoint_achieved = false`
- 飞行器可能失去控制（极端情况）

---

### 文件路径总结

| 组件 | 文件路径 |
|------|---------|
| **Control Allocator** | `src/modules/control_allocator/ControlAllocator.{hpp,cpp}` |
| **Actuator Effectiveness** | `src/lib/control_allocation/actuator_effectiveness/ActuatorEffectiveness.{hpp,cpp}` |
| **Hnuter Effectiveness** | `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.{hpp,cpp}` |
| **Control Allocation Algorithms** | `src/lib/control_allocation/control_allocation/` |
| **Mixing Output** | `src/lib/mixer_module/mixer_module.{hpp,cpp}` |
| **Gazebo Bridge** | `src/modules/simulation/gz_bridge/GZBridge.{hpp,cpp}` |
| **ESC Interface** | `src/modules/simulation/gz_bridge/GZMixingInterfaceESC.{hpp,cpp}` |
| **Servo Interface** | `src/modules/simulation/gz_bridge/GZMixingInterfaceServo.{hpp,cpp}` |
| **PWM Driver** | `src/drivers/pwm_out/pwm_out.cpp` |
| **消息定义** | `msg/ActuatorMotors.msg`, `msg/ActuatorServos.msg` |
| **参数定义** | `src/modules/control_allocator/module.yaml` |

---

**文档版本**: 2.0
**更新日期**: 2026-02-23
**适用版本**: PX4 v1.14+
**机型**: Hnuter Tiltrotor (5 电机 + 4 倾转舵机)
