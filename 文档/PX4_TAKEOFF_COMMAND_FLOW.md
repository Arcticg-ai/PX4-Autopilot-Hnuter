# PX4 Takeoff 命令执行完整流程详解

## 目录
1. [流程概览](#流程概览)
2. [阶段一: 命令接收与解析](#阶段一-命令接收与解析)
3. [阶段二: 状态转换](#阶段二-状态转换)
4. [阶段三: 导航控制](#阶段三-导航控制)
5. [阶段四: 位置控制](#阶段四-位置控制)
6. [阶段五: 姿态控制](#阶段五-姿态控制)
7. [阶段六: 电机输出](#阶段六-电机输出)
8. [完整代码追踪](#完整代码追踪)
9. [时序图](#时序图)
10. [故障处理](#故障处理)

---

## 流程概览

### 整体架构

```
外部指令 (GCS/Companion)
    │
    ├─ MAVLink → uORB 转换
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│  STAGE 1: Commander - 命令接收与验证                        │
│  文件: src/modules/commander/Commander.cpp                  │
│  功能: 接收命令 → 验证 → 改变导航状态                       │
└──────────────────────┬──────────────────────────────────────┘
                       │ vehicle_status (nav_state = AUTO_TAKEOFF)
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  STAGE 2: Navigator - 起飞序列规划                          │
│  文件: src/modules/navigator/takeoff.cpp                    │
│  功能: 生成起飞轨迹 → 发布位置设定点                        │
└──────────────────────┬──────────────────────────────────────┘
                       │ position_setpoint_triplet
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  STAGE 3: Position Control - 位置速度控制                   │
│  文件: src/modules/mc_pos_control/                          │
│  功能: 计算速度/加速度指令 → 生成轨迹设定点                 │
└──────────────────────┬──────────────────────────────────────┘
                       │ trajectory_setpoint
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  STAGE 4: Attitude Control - 姿态控制                       │
│  文件: src/modules/mc_att_control/                          │
│  功能: 计算所需姿态 → 计算角速率指令                        │
└──────────────────────┬──────────────────────────────────────┘
                       │ vehicle_rates_setpoint
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  STAGE 5: Rate Control - 角速率控制                         │
│  文件: src/modules/mc_att_control/                          │
│  功能: 计算扭矩/推力指令                                     │
└──────────────────────┬──────────────────────────────────────┘
                       │ vehicle_torque_setpoint / vehicle_thrust_setpoint
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  STAGE 6: Control Allocator - 控制分配                      │
│  文件: src/modules/control_allocator/                       │
│  功能: 分配到各电机 → 输出 PWM/RPM 指令                     │
└──────────────────────┬──────────────────────────────────────┘
                       │ actuator_motors / actuator_servos
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  STAGE 7: 执行器/驱动 - 物理输出                            │
│  文件: src/drivers/pwm_out/, Gazebo Bridge                  │
│  功能: 发送到 ESC → 电机旋转 → 飞行器起飞                   │
└─────────────────────────────────────────────────────────────┘
```

### 数据流总览

```
vehicle_command (TAKEOFF)
    → vehicle_status (nav_state=AUTO_TAKEOFF)
    → position_setpoint_triplet (起飞点 + 目标高度)
    → trajectory_setpoint (位置 + 速度 + 加速度)
    → vehicle_attitude_setpoint (姿态目标)
    → vehicle_rates_setpoint (角速率目标)
    → vehicle_torque_setpoint + vehicle_thrust_setpoint
    → actuator_motors (电机转速)
    → 物理执行 (螺旋桨旋转)
```

---

## 阶段一: 命令接收与解析

### 涉及模块: Commander

**主文件**: `src/modules/commander/Commander.cpp`

### 1.1 命令输入

外部系统（如 QGroundControl）通过 MAVLink 发送 `MAV_CMD_NAV_TAKEOFF` 命令：

```
MAVLink 消息:
  msg_id: COMMAND_LONG (76)
  command: MAV_CMD_NAV_TAKEOFF (22)
  param7: 起飞高度 (AMSL - Above Mean Sea Level)
```

MAVLink 接收器将其转换为 uORB 消息：

**文件**: `src/modules/mavlink/mavlink_receiver.cpp`

```cpp
void MavlinkReceiver::handle_message_command_long(mavlink_message_t *msg) {
    mavlink_command_long_t cmd_mavlink;
    mavlink_msg_command_long_decode(msg, &cmd_mavlink);

    vehicle_command_s vcmd{};
    vcmd.timestamp = hrt_absolute_time();
    vcmd.command = cmd_mavlink.command;      // 22 (TAKEOFF)
    vcmd.param1 = cmd_mavlink.param1;
    // ...
    vcmd.param7 = cmd_mavlink.param7;        // 目标高度
    vcmd.from_external = true;

    // 发布到 uORB
    _vehicle_command_pub.publish(vcmd);
}
```

### 1.2 Commander 主循环

**文件**: `src/modules/commander/Commander.cpp`

**主循环**: `Commander::run()` (第 349 行开始)

```cpp
void Commander::run() {
    // 主循环以 10ms 周期运行
    const hrt_abstime COMMANDER_MONITORING_INTERVAL = 10_ms;

    while (!should_exit()) {
        perf_begin(_loop_perf);

        // 1️⃣ 订阅并处理 vehicle_command
        vehicle_command_s cmd;
        if (_vehicle_command_sub.update(&cmd)) {
            if (handle_command(cmd)) {
                // 命令已处理
            }
        }

        // 2️⃣ 更新系统状态
        update_control_mode();

        // 3️⃣ 发布系统状态
        _vehicle_status_pub.publish(_vehicle_status);

        perf_end(_loop_perf);
        px4_usleep(COMMANDER_MONITORING_INTERVAL);
    }
}
```

### 1.3 命令处理函数

**文件**: `src/modules/commander/Commander.cpp` (第 755 行开始)

```cpp
bool Commander::handle_command(const vehicle_command_s &cmd) {
    bool cmd_result = false;
    uint8_t cmd_ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;

    switch (cmd.command) {

    // ... 其他命令处理

    case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF: {
        // 第 1076-1086 行

        // 检查系统是否已解锁
        if (_vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
            cmd_ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
            PX4_WARN("Takeoff rejected: not armed");
            break;
        }

        // 检查起飞点是否已设置
        if (!_home_position.valid()) {
            cmd_ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
            PX4_WARN("Takeoff rejected: home position not valid");
            break;
        }

        // 尝试改变导航状态为 AUTO_TAKEOFF
        if (_user_mode_intention.change(
                vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF,
                getSourceFromCommand(cmd))) {

            cmd_ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
            cmd_result = true;

            PX4_INFO("Takeoff command accepted");
        } else {
            printRejectMode(vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF);
            cmd_ack_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
        }
    }
    break;

    // ... 更多命令

    } // switch 结束

    // 发送命令应答
    answer_command(cmd, cmd_ack_result);

    return cmd_result;
}
```

### 1.4 命令应答

**文件**: `src/modules/commander/Commander.cpp`

```cpp
void Commander::answer_command(const vehicle_command_s &cmd, uint8_t result) {
    vehicle_command_ack_s command_ack{};
    command_ack.timestamp = hrt_absolute_time();
    command_ack.command = cmd.command;
    command_ack.result = result;
    command_ack.target_system = cmd.source_system;
    command_ack.target_component = cmd.source_component;

    // 发布应答消息
    _vehicle_command_ack_pub.publish(command_ack);
}
```

**应答结果**返回给 GCS：
- `VEHICLE_CMD_RESULT_ACCEPTED` (0) - 命令已接受
- `VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED` (4) - 暂时拒绝
- `VEHICLE_CMD_RESULT_DENIED` (3) - 命令被拒绝

---

## 阶段二: 状态转换

### 2.1 导航状态更新

**文件**: `src/modules/commander/ModeManagement.cpp`

```cpp
bool UserModeIntention::change(uint8_t new_nav_state, uint8_t source) {
    // 检查是否允许该状态转换
    if (isNavStateValid(new_nav_state)) {
        _nav_state = new_nav_state;
        _source = source;

        // 触发状态机更新
        return true;
    }
    return false;
}
```

### 2.2 发布新状态

Commander 更新 `vehicle_status` 并发布：

```cpp
// 在 Commander::run() 中
_vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF;
_vehicle_status.nav_state_timestamp = hrt_absolute_time();
_vehicle_status_pub.publish(_vehicle_status);
```

**关键字段**:
```cpp
struct vehicle_status_s {
    uint8_t nav_state;     // 新值: NAVIGATION_STATE_AUTO_TAKEOFF (17)
    uint8_t arming_state;  // ARMING_STATE_ARMED (2)
    // ...
};
```

---

## 阶段三: 导航控制

### 涉及模块: Navigator

**主文件**: `src/modules/navigator/navigator_main.cpp`

### 3.1 Navigator 主循环

**文件**: `src/modules/navigator/navigator_main.cpp` (第 152 行)

```cpp
void Navigator::run() {
    // 运行频率: 20 Hz (50ms)
    const hrt_abstime NAVIGATOR_UPDATE_INTERVAL = 50_ms;

    while (!should_exit()) {
        perf_begin(_loop_perf);

        // 1️⃣ 订阅 vehicle_status
        vehicle_status_s vstatus;
        if (_vehicle_status_sub.update(&vstatus)) {
            _vstatus = vstatus;
        }

        // 2️⃣ 检测导航状态变化
        if (_vstatus.nav_state != _nav_state_prev) {
            switch_navigation_mode(_vstatus.nav_state);
            _nav_state_prev = _vstatus.nav_state;
        }

        // 3️⃣ 运行当前导航任务
        if (_navigation_mode) {
            _navigation_mode->run(_local_pos_valid, _global_pos_valid);
        }

        // 4️⃣ 发布位置设定点
        if (_pos_sp_triplet_updated) {
            _pos_sp_triplet_pub.publish(_pos_sp_triplet);
            _pos_sp_triplet_updated = false;
        }

        perf_end(_loop_perf);
        px4_usleep(NAVIGATOR_UPDATE_INTERVAL);
    }
}
```

### 3.2 切换到 Takeoff 模式

**文件**: `src/modules/navigator/navigator_main.cpp`

```cpp
void Navigator::switch_navigation_mode(uint8_t new_nav_state) {
    // 停止之前的模式
    if (_navigation_mode) {
        _navigation_mode->on_inactive();
    }

    // 根据导航状态选择模式
    switch (new_nav_state) {

    case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
        _navigation_mode = &_takeoff;  // 激活 Takeoff 模式
        break;

    case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
        _navigation_mode = &_loiter;
        break;

    // ... 更多模式

    default:
        _navigation_mode = nullptr;
        break;
    }

    // 激活新模式
    if (_navigation_mode) {
        _navigation_mode->on_activation();
    }
}
```

### 3.3 Takeoff 模式实现

**文件**: `src/modules/navigator/takeoff.cpp`

#### 激活阶段 (on_activation)

```cpp
void Takeoff::on_activation() {
    // 第 51-59 行

    // 1. 设置起飞位置（当前位置）
    _takeoff_position.setLatLon(
        _navigator->get_global_position()->lat,
        _navigator->get_global_position()->lon
    );

    // 2. 设置起飞高度
    // 从 vehicle_command.param7 获取，或使用默认值
    float takeoff_alt_amsl = _navigator->get_takeoff_min_alt();

    if (_navigator->get_vehicle_command_params().param7 > 0.001f) {
        takeoff_alt_amsl = _navigator->get_vehicle_command_params().param7;
    }

    _takeoff_position.setAlt(takeoff_alt_amsl);

    // 3. 重置巡航速度
    _navigator->reset_cruising_speed();

    PX4_INFO("Takeoff to %.1f m AMSL", (double)takeoff_alt_amsl);
}
```

#### 执行阶段 (on_active)

```cpp
void Takeoff::on_active() {
    // 第 62-150+ 行

    // 多旋翼起飞逻辑
    if (_navigator->get_vstatus()->vehicle_type ==
        vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {

        // 1. 获取当前位置
        const vehicle_local_position_s &local_pos =
            *_navigator->get_local_position();

        // 2. 生成位置设定点
        position_setpoint_triplet_s pos_sp_triplet{};

        // 当前设定点（起飞中）
        pos_sp_triplet.current.valid = true;
        pos_sp_triplet.current.type =
            position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

        pos_sp_triplet.current.lat = _takeoff_position.lat();
        pos_sp_triplet.current.lon = _takeoff_position.lon();
        pos_sp_triplet.current.alt = _takeoff_position.alt();

        pos_sp_triplet.current.vx = NAN;  // 不指定速度
        pos_sp_triplet.current.vy = NAN;
        pos_sp_triplet.current.vz = -_navigator->get_takeoff_cruise_speed();  // 向上速度

        pos_sp_triplet.current.yaw = NAN;  // 保持当前偏航角

        // 3. 检查是否达到目标高度
        if (local_pos.z <= -(_takeoff_position.alt() -
                             _navigator->get_global_position()->alt - 0.5f)) {
            // 达到目标高度，切换到 LOITER 模式
            _navigator->set_position_setpoint_triplet_updated(true);

            vehicle_command_s cmd{};
            cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
            cmd.param1 = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
            _navigator->publish_vehicle_cmd(&cmd);

            return;
        }

        // 4. 发布位置设定点
        _navigator->set_position_setpoint_triplet(pos_sp_triplet);
        _navigator->set_position_setpoint_triplet_updated(true);
    }

    // 固定翼起飞逻辑
    else if (_navigator->get_vstatus()->vehicle_type ==
             vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
        // 固定翼起飞包含两阶段:
        // 1. CLIMBOUT - 爬升到目标高度
        // 2. GO_TO_LOITER - 飞向盘旋点

        // ... (固定翼特定逻辑)
    }
}
```

### 3.4 发布位置设定点

**消息类型**: `position_setpoint_triplet_s`

```cpp
struct position_setpoint_triplet_s {
    position_setpoint_s previous;  // 上一个设定点
    position_setpoint_s current;   // 当前设定点 ⭐
    position_setpoint_s next;      // 下一个设定点
};

struct position_setpoint_s {
    bool valid;
    uint8_t type;          // SETPOINT_TYPE_TAKEOFF = 2

    double lat;            // 纬度 (度)
    double lon;            // 经度 (度)
    float alt;             // 高度 AMSL (米)

    float vx, vy, vz;      // 速度 NED (米/秒)
    float yaw;             // 偏航角 (弧度)
    // ...
};
```

---

## 阶段四: 位置控制

### 涉及模块: Multicopter Position Control

**主文件**: `src/modules/mc_pos_control/MulticopterPositionControl.cpp`

### 4.1 位置控制器主循环

```cpp
void MulticopterPositionControl::run() {
    // 运行频率: ~100 Hz (10ms)

    while (!should_exit()) {
        // 1️⃣ 订阅位置设定点
        position_setpoint_triplet_s pos_sp_triplet;
        if (_pos_sp_triplet_sub.update(&pos_sp_triplet)) {
            _pos_sp_triplet = pos_sp_triplet;
        }

        // 2️⃣ 订阅当前位置
        vehicle_local_position_s local_pos;
        if (_local_pos_sub.update(&local_pos)) {
            _local_pos = local_pos;
        }

        // 3️⃣ 计算控制指令
        if (_control_mode.flag_control_altitude_enabled &&
            _control_mode.flag_control_position_enabled) {

            PositionControl::updateStateEstimate(_local_pos);
            PositionControl::updateSetpoint(_pos_sp_triplet.current);
            PositionControl::generateTrajectorySetpoint();
        }

        // 4️⃣ 发布轨迹设定点
        _traj_sp_pub.publish(_trajectory_setpoint);
    }
}
```

### 4.2 位置控制算法

**文件**: `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`

```cpp
void PositionControl::generateTrajectorySetpoint() {
    // 1. 获取当前状态
    Vector3f pos_current = _position;
    Vector3f vel_current = _velocity;

    // 2. 获取目标状态
    Vector3f pos_target = _pos_sp;
    Vector3f vel_target = _vel_sp;

    // 3. 计算位置误差
    Vector3f pos_error = pos_target - pos_current;

    // 4. P控制器 - 计算期望速度
    Vector3f vel_desired = pos_error * _gain_pos_p;

    // 限幅
    vel_desired(0) = math::constrain(vel_desired(0),
                                     -_vel_max_xy, _vel_max_xy);
    vel_desired(1) = math::constrain(vel_desired(1),
                                     -_vel_max_xy, _vel_max_xy);
    vel_desired(2) = math::constrain(vel_desired(2),
                                     -_vel_max_down, _vel_max_up);

    // 5. 计算速度误差
    Vector3f vel_error = vel_desired - vel_current;

    // 6. PI控制器 - 计算期望加速度
    Vector3f acc_desired = vel_error * _gain_vel_p + _vel_int;

    // 积分抗饱和
    _vel_int += vel_error * _gain_vel_i * _dt;
    _vel_int(0) = math::constrain(_vel_int(0), -_acc_max_xy, _acc_max_xy);
    _vel_int(1) = math::constrain(_vel_int(1), -_acc_max_xy, _acc_max_xy);
    _vel_int(2) = math::constrain(_vel_int(2), -_acc_max_z, _acc_max_z);

    // 7. 组装轨迹设定点
    _trajectory_setpoint.position[0] = pos_target(0);
    _trajectory_setpoint.position[1] = pos_target(1);
    _trajectory_setpoint.position[2] = pos_target(2);

    _trajectory_setpoint.velocity[0] = vel_desired(0);
    _trajectory_setpoint.velocity[1] = vel_desired(1);
    _trajectory_setpoint.velocity[2] = vel_desired(2);

    _trajectory_setpoint.acceleration[0] = acc_desired(0);
    _trajectory_setpoint.acceleration[1] = acc_desired(1);
    _trajectory_setpoint.acceleration[2] = acc_desired(2);

    _trajectory_setpoint.yaw = _yaw_sp;
    _trajectory_setpoint.yawspeed = 0.0f;
}
```

### 4.3 发布轨迹设定点

**消息类型**: `trajectory_setpoint_s`

```cpp
struct trajectory_setpoint_s {
    uint64_t timestamp;

    float position[3];      // NED 位置 (米)
    float velocity[3];      // NED 速度 (米/秒)
    float acceleration[3];  // NED 加速度 (米/秒²)
    float jerk[3];          // NED jerk (米/秒³)

    float yaw;              // 偏航角 (弧度)
    float yawspeed;         // 偏航角速率 (弧度/秒)
};
```

---

## 阶段五: 姿态控制

### 涉及模块: Multicopter Attitude Control

**主文件**: `src/modules/mc_att_control/mc_att_control_main.cpp`

### 5.1 姿态控制器主循环

```cpp
void MulticopterAttitudeControl::run() {
    // 运行频率: ~250 Hz (4ms)

    while (!should_exit()) {
        // 1️⃣ 订阅轨迹设定点
        trajectory_setpoint_s traj_sp;
        if (_traj_sp_sub.update(&traj_sp)) {
            _traj_sp = traj_sp;
        }

        // 2️⃣ 订阅当前姿态
        vehicle_attitude_s att;
        if (_vehicle_attitude_sub.update(&att)) {
            _vehicle_attitude = att;
        }

        // 3️⃣ 计算期望姿态
        Quatf q_desired = computeDesiredAttitude(_traj_sp);

        // 4️⃣ 计算姿态误差
        Quatf q_error = _vehicle_attitude.q.inversed() * q_desired;

        // 5️⃣ 姿态控制器 → 角速率设定值
        Vector3f rates_sp = attitudeController(q_error);

        // 6️⃣ 角速率控制器 → 扭矩指令
        Vector3f torque = rateController(rates_sp);

        // 7️⃣ 发布扭矩和推力设定值
        publishTorqueSetpoint(torque);
        publishThrustSetpoint(_traj_sp.acceleration[2]);
    }
}
```

### 5.2 期望姿态计算

```cpp
Quatf MulticopterAttitudeControl::computeDesiredAttitude(
    const trajectory_setpoint_s &traj_sp) {

    // 1. 从加速度计算期望的机体Z轴方向
    Vector3f acc_desired(traj_sp.acceleration);
    acc_desired(2) += CONSTANTS_ONE_G;  // 补偿重力

    Vector3f body_z = acc_desired.normalized();

    // 2. 从偏航角计算期望的机体X轴方向
    Vector3f body_x_desired(
        cosf(traj_sp.yaw),
        sinf(traj_sp.yaw),
        0.0f
    );

    // 3. 计算机体Y轴 (右手系)
    Vector3f body_y = body_z.cross(body_x_desired).normalized();

    // 4. 重新计算机体X轴
    Vector3f body_x = body_y.cross(body_z);

    // 5. 构建旋转矩阵
    Matrix3f R;
    R.setCol(0, body_x);
    R.setCol(1, body_y);
    R.setCol(2, body_z);

    // 6. 转换为四元数
    return Quatf(R);
}
```

### 5.3 姿态控制器

```cpp
Vector3f MulticopterAttitudeControl::attitudeController(
    const Quatf &q_error) {

    // 将四元数误差转换为角度误差
    Vector3f e_R = q_error.getEuler();

    // P控制器 - 计算角速率设定值
    Vector3f rates_sp;
    rates_sp(0) = e_R(0) * _att_p(0);  // Roll rate
    rates_sp(1) = e_R(1) * _att_p(1);  // Pitch rate
    rates_sp(2) = e_R(2) * _att_p(2);  // Yaw rate

    // 限幅
    rates_sp(0) = math::constrain(rates_sp(0), -_rate_max(0), _rate_max(0));
    rates_sp(1) = math::constrain(rates_sp(1), -_rate_max(1), _rate_max(1));
    rates_sp(2) = math::constrain(rates_sp(2), -_rate_max(2), _rate_max(2));

    return rates_sp;
}
```

### 5.4 角速率控制器

```cpp
Vector3f MulticopterAttitudeControl::rateController(
    const Vector3f &rates_sp) {

    // 1. 获取当前角速率
    Vector3f rates_current = _vehicle_angular_velocity;

    // 2. 计算角速率误差
    Vector3f rate_error = rates_sp - rates_current;

    // 3. PID控制器
    Vector3f torque = rate_error.emult(_rate_p)           // P项
                    + _rate_int                           // I项
                    + (_rate_deriv).emult(_rate_d);       // D项

    // 4. 更新积分项
    _rate_int += rate_error.emult(_rate_i) * _dt;
    _rate_int(0) = math::constrain(_rate_int(0), -_rate_int_lim(0), _rate_int_lim(0));
    _rate_int(1) = math::constrain(_rate_int(1), -_rate_int_lim(1), _rate_int_lim(1));
    _rate_int(2) = math::constrain(_rate_int(2), -_rate_int_lim(2), _rate_int_lim(2));

    // 5. 更新微分项
    _rate_deriv = (rate_error - _rate_error_prev) / _dt;
    _rate_error_prev = rate_error;

    return torque;
}
```

### 5.5 发布扭矩和推力设定值

```cpp
void MulticopterAttitudeControl::publishTorqueSetpoint(
    const Vector3f &torque) {

    vehicle_torque_setpoint_s torque_sp{};
    torque_sp.timestamp = hrt_absolute_time();
    torque_sp.xyz[0] = torque(0);
    torque_sp.xyz[1] = torque(1);
    torque_sp.xyz[2] = torque(2);

    _vehicle_torque_setpoint_pub.publish(torque_sp);
}

void MulticopterAttitudeControl::publishThrustSetpoint(
    float thrust_z) {

    vehicle_thrust_setpoint_s thrust_sp{};
    thrust_sp.timestamp = hrt_absolute_time();
    thrust_sp.xyz[0] = 0.0f;
    thrust_sp.xyz[1] = 0.0f;
    thrust_sp.xyz[2] = -thrust_z;  // 向下为正

    _vehicle_thrust_setpoint_pub.publish(thrust_sp);
}
```

---

## 阶段六: 电机输出

### 涉及模块: Control Allocator

**主文件**: `src/modules/control_allocator/ControlAllocator.cpp`

### 6.1 控制分配器主循环

```cpp
void ControlAllocator::run() {
    // 运行频率: ~400 Hz (2.5ms)

    while (!should_exit()) {
        // 1️⃣ 订阅扭矩设定值
        vehicle_torque_setpoint_s torque_sp;
        if (_torque_sp_sub.update(&torque_sp)) {
            _torque_sp = torque_sp;
        }

        // 2️⃣ 订阅推力设定值
        vehicle_thrust_setpoint_s thrust_sp;
        if (_thrust_sp_sub.update(&thrust_sp)) {
            _thrust_sp = thrust_sp;
        }

        // 3️⃣ 组合控制向量
        Vector4f control_sp;
        control_sp(0) = _torque_sp.xyz[0];   // Roll torque
        control_sp(1) = _torque_sp.xyz[1];   // Pitch torque
        control_sp(2) = _torque_sp.xyz[2];   // Yaw torque
        control_sp(3) = -_thrust_sp.xyz[2];  // Thrust (向上为正)

        // 4️⃣ 分配到电机
        MatrixXf actuator_sp = allocate(control_sp);

        // 5️⃣ 发布电机指令
        publishActuatorControls(actuator_sp);
    }
}
```

### 6.2 控制分配算法

对于 **Hnuter** 的 5 电机配置：

```cpp
MatrixXf ControlAllocator::allocate(const Vector4f &control_sp) {
    // 控制分配矩阵 B (4×5)
    // 每列代表一个电机对 [τx, τy, τz, Fz] 的贡献

    // Hnuter 配置 (5个电机):
    //   Motor 0 (xy1): 右臂上方, CCW
    //   Motor 1 (xy2): 右臂下方, CW
    //   Motor 2 (xy3): 左臂上方, CW
    //   Motor 3 (xy4): 左臂下方, CCW
    //   Motor 4 (xy5): 尾部, CCW

    // 混控矩阵来自配置参数:
    //   CA_ROTOR0_PX, PY, PZ: 电机位置
    //   CA_ROTOR0_KM: 力矩系数

    // 控制分配: u = B^+ * v
    // 其中 B^+ 是 B 的伪逆矩阵
    // u: 电机指令 (5×1)
    // v: 控制指令 (4×1)

    VectorXf actuator_sp = _allocation_matrix * control_sp;

    // 归一化和限幅
    for (int i = 0; i < _num_motors; i++) {
        actuator_sp(i) = math::constrain(
            actuator_sp(i),
            _actuator_min(i),
            _actuator_max(i)
        );
    }

    return actuator_sp;
}
```

### 6.3 发布执行器指令

```cpp
void ControlAllocator::publishActuatorControls(
    const MatrixXf &actuator_sp) {

    // 发布电机指令
    actuator_motors_s motors{};
    motors.timestamp = hrt_absolute_time();
    for (int i = 0; i < _num_motors; i++) {
        motors.control[i] = actuator_sp(i);
    }
    _actuator_motors_pub.publish(motors);

    // 发布舵机指令 (如果有)
    if (_num_servos > 0) {
        actuator_servos_s servos{};
        servos.timestamp = hrt_absolute_time();
        for (int i = 0; i < _num_servos; i++) {
            servos.control[i] = _servo_sp(i);
        }
        _actuator_servos_pub.publish(servos);
    }
}
```

---

## 阶段七: 执行器驱动

### 7.1 Gazebo 仿真

**文件**: `src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp`

```cpp
bool GZMixingInterfaceESC::updateOutputs(
    uint16_t outputs[MAX_ACTUATORS],
    unsigned num_outputs,
    unsigned num_control_groups_updated) {

    // 1. 构建 Gazebo Actuators 消息
    gz::msgs::Actuators rotor_velocity_message;
    rotor_velocity_message.mutable_velocity()->Resize(num_outputs, 0);

    // 2. 填充电机速度
    for (unsigned i = 0; i < num_outputs; i++) {
        if (_mixing_output.isFunctionSet(i)) {
            rotor_velocity_message.set_velocity(i, outputs[i]);
        }
    }

    // 3. 发布到 Gazebo
    if (_actuators_pub.Valid()) {
        return _actuators_pub.Publish(rotor_velocity_message);
    }

    return false;
}
```

**Gazebo 话题**: `/hnuter/command/motor_speed`

### 7.2 实际硬件 (PWM 输出)

**文件**: `src/drivers/pwm_out/pwm_out.cpp`

```cpp
void PWMOut::update_pwm_outputs() {
    // 1. 订阅 actuator_outputs
    actuator_outputs_s outputs;
    if (_outputs_sub.update(&outputs)) {

        // 2. 转换为 PWM 值 (1000-2000 μs)
        for (int i = 0; i < _num_outputs; i++) {
            float normalized = outputs.output[i];  // [-1, 1]

            uint16_t pwm = _pwm_min[i] +
                          (normalized + 1.0f) / 2.0f *
                          (_pwm_max[i] - _pwm_min[i]);

            // 3. 输出到硬件定时器
            up_pwm_servo_set(i, pwm);
        }
    }
}
```

---

## 完整代码追踪

### 关键文件清单

| 模块 | 文件路径 | 关键函数 | 行号 |
|------|---------|---------|------|
| **MAVLink** | src/modules/mavlink/mavlink_receiver.cpp | handle_message_command_long() | ~2500 |
| **Commander** | src/modules/commander/Commander.cpp | run() | 349 |
| | | handle_command() | 755 |
| | | VEHICLE_CMD_NAV_TAKEOFF处理 | 1076 |
| | | answer_command() | ~1200 |
| **Navigator** | src/modules/navigator/navigator_main.cpp | run() | 152 |
| | | switch_navigation_mode() | ~350 |
| **Takeoff** | src/modules/navigator/takeoff.cpp | on_activation() | 51 |
| | | on_active() | 62 |
| **Pos Control** | src/modules/mc_pos_control/MulticopterPositionControl.cpp | run() | ~180 |
| | src/modules/mc_pos_control/PositionControl/PositionControl.cpp | generateTrajectorySetpoint() | ~250 |
| **Att Control** | src/modules/mc_att_control/mc_att_control_main.cpp | run() | ~120 |
| | | computeDesiredAttitude() | ~400 |
| | | attitudeController() | ~500 |
| | | rateController() | ~600 |
| **Allocator** | src/modules/control_allocator/ControlAllocator.cpp | run() | ~150 |
| | | allocate() | ~300 |
| **GZ Bridge** | src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp | updateOutputs() | 65 |

### uORB 消息流

```
vehicle_command
    ↓ (Commander 处理)
vehicle_command_ack
vehicle_status (nav_state = AUTO_TAKEOFF)
    ↓ (Navigator 处理)
position_setpoint_triplet
    ↓ (Position Control 处理)
trajectory_setpoint
    ↓ (Attitude Control 处理)
vehicle_rates_setpoint
vehicle_torque_setpoint
vehicle_thrust_setpoint
    ↓ (Control Allocator 处理)
actuator_motors
actuator_servos
    ↓ (驱动/Gazebo 处理)
物理执行 (螺旋桨旋转)
```

---

## 时序图

```
时间轴 →

T0: GCS 发送 MAV_CMD_NAV_TAKEOFF
    │
    ├─ MAVLink 接收器转换为 vehicle_command (< 1ms)
    │
T1: Commander 接收 vehicle_command (10ms 循环)
    │
    ├─ handle_command() 验证 (< 0.1ms)
    ├─ 改变 nav_state = AUTO_TAKEOFF
    ├─ 发布 vehicle_command_ack
    ├─ 发布 vehicle_status
    │
T2: Navigator 检测状态变化 (50ms 循环)
    │
    ├─ switch_navigation_mode() 切换到 Takeoff (< 0.1ms)
    ├─ Takeoff::on_activation() 设置起飞点 (< 0.1ms)
    │
T3: Navigator 开始生成设定点 (50ms 循环)
    │
    ├─ Takeoff::on_active() 每帧执行
    ├─ 发布 position_setpoint_triplet (50ms 周期)
    │
T4: Position Control 处理设定点 (10ms 循环)
    │
    ├─ generateTrajectorySetpoint() 计算速度/加速度
    ├─ 发布 trajectory_setpoint (10ms 周期)
    │
T5: Attitude Control 处理轨迹 (4ms 循环)
    │
    ├─ computeDesiredAttitude() 计算姿态
    ├─ attitudeController() 计算角速率
    ├─ rateController() 计算扭矩
    ├─ 发布 vehicle_torque_setpoint
    ├─ 发布 vehicle_thrust_setpoint
    │
T6: Control Allocator 分配到电机 (2.5ms 循环)
    │
    ├─ allocate() 混控算法
    ├─ 发布 actuator_motors (2.5ms 周期)
    │
T7: 执行器驱动输出 (< 1ms)
    │
    ├─ Gazebo: 发布到 /hnuter/command/motor_speed
    ├─ 硬件: PWM 输出到 ESC
    │
T8: 物理效果
    │
    └─ 电机加速 → 产生升力 → 飞行器起飞

总延迟 (命令到电机响应): ~70-100ms
```

---

## 故障处理

### 常见失败场景

#### 1. 未解锁 (Not Armed)

```cpp
// Commander::handle_command()
if (_vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
    return vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
}
```

**解决**: 先发送 `VEHICLE_CMD_COMPONENT_ARM_DISARM` (param1=1.0)

#### 2. 起飞点无效 (Home Position Invalid)

```cpp
if (!_home_position.valid()) {
    PX4_WARN("Takeoff rejected: home position not valid");
    return VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
}
```

**解决**: 等待 GPS 锁定，系统自动设置起飞点

#### 3. EKF 未收敛

```cpp
// Commander 检查
if (!_ekf2_status.healthy) {
    // 拒绝起飞
}
```

**解决**: 等待 5-10 秒让 EKF 收敛

#### 4. 电池电量低

```cpp
if (_battery_status.remaining < _low_battery_threshold) {
    printRejectReason("Battery too low");
    return VEHICLE_CMD_RESULT_DENIED;
}
```

**解决**: 更换/充电电池

---

## 参数配置

### Takeoff 相关参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `MIS_TAKEOFF_ALT` | 2.5m | 默认起飞高度 (相对) |
| `MIS_TAKEOFF_SPEED` | 1.5 m/s | 起飞爬升速度 |
| `MPC_TKO_SPEED` | 1.5 m/s | 多旋翼起飞速度 |
| `MPC_Z_VEL_MAX_UP` | 3.0 m/s | 最大上升速度 |
| `COM_TAKEOFF_ACT` | 0 | 起飞动作 (0=无, 1=跳跃) |
| `NAV_ACC_RAD` | 5.0m | 航点接受半径 |

### 控制器参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `MC_ROLLRATE_P` | 0.15 | Roll 角速率 P 增益 |
| `MC_PITCHRATE_P` | 0.15 | Pitch 角速率 P 增益 |
| `MC_YAWRATE_P` | 0.2 | Yaw 角速率 P 增益 |
| `MPC_Z_P` | 1.0 | 高度 P 增益 |
| `MPC_Z_VEL_P_ACC` | 4.0 | 垂直速度 P 增益 |

---

## 总结

### 完整流程回顾

1. **命令接收**: GCS → MAVLink → uORB → Commander
2. **状态转换**: Commander → vehicle_status (AUTO_TAKEOFF)
3. **轨迹规划**: Navigator → position_setpoint_triplet
4. **位置控制**: Position Control → trajectory_setpoint
5. **姿态控制**: Attitude Control → torque/thrust setpoint
6. **控制分配**: Control Allocator → actuator_motors
7. **执行输出**: 驱动 → 物理电机

### 关键时间节点

- **命令延迟**: ~10-20ms (MAVLink → Commander)
- **规划延迟**: ~50ms (Navigator 周期)
- **控制延迟**: ~10-20ms (Position + Attitude)
- **输出延迟**: ~3-5ms (Allocator)
- **总延迟**: **~70-100ms** (命令到电机)

### 涉及的核心模块

- ✅ Commander (命令管理)
- ✅ Navigator (轨迹规划)
- ✅ Position Control (位置控制)
- ✅ Attitude Control (姿态控制)
- ✅ Control Allocator (控制分配)
- ✅ uORB (消息系统)

---

**文档版本**: 1.0
**创建日期**: 2026-02-23
**适用版本**: PX4 v1.14+
**机型**: Hnuter Tiltrotor (5 电机配置)
