#!/usr/bin/env python3
"""
Hnuter Tiltrotor PX4 External Controller with Gamepad Manual Control

核心修改：
1. 将 hnuter104.py 中的 GamepadManager 移植到 ROS2/PX4 外部控制节点。
2. 手柄输出为机体系速度 vx_b/vy_b、世界系垂直速度 vz、偏航角速度 yaw_rate。
3. LT/RT 触发器输出期望俯仰角速度，积分为目标 pitch 姿态。
4. 速度指令通过欧拉积分生成期望位置与期望偏航，送入原几何控制器与执行器分配。
4. 修复 Offboard/Arm 启动逻辑：先连续发布 OffboardControlMode，再切 Offboard，最后 Arm。
5. 避免 hover_only/xy_lock 永久覆盖手动目标点。
"""

import os
import time
import math

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, Float64MultiArray

from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import ActuatorServos
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleStatus

try:
    import pygame
except Exception:  # 允许没有手柄/没有 pygame 时保持悬停
    pygame = None


# ============================================================
# 手柄管理器：从 hnuter104.py 移植，加入异常保护
# ============================================================
class GamepadManager:
    def __init__(self,
                 max_vxy: float = 1.0,
                 max_vz: float = 0.5,
                 max_yaw_rate: float = 0.6,
                 max_pitch_rate: float = math.radians(20.0),
                 deadzone: float = 0.10,
                 expo: float = 0.40,
                 filter_tau: float = 0.20,
                 lt_axis: int = 2,
                 rt_axis: int = 5,
                 trigger_mode: str = 'minus_one_to_one',
                 logger=None):
        self.logger = logger
        self.joystick = None
        self.max_vxy = float(max_vxy)
        self.max_vz = float(max_vz)
        self.max_yaw_rate = float(max_yaw_rate)
        self.max_pitch_rate = float(max_pitch_rate)
        self.deadzone = float(deadzone)
        self.expo = float(expo)
        self.filter_tau = float(filter_tau)
        self.lt_axis = int(lt_axis)
        self.rt_axis = int(rt_axis)
        # 常见 Xbox/XInput 手柄 LT/RT: 未按=-1，按满=+1。
        # 若你的手柄是未按=0，按满=1，把 trigger_mode 改为 'zero_to_one'。
        # 若你的手柄是未按=+1，按满=-1，把 trigger_mode 改为 'one_to_minus_one'。
        self.trigger_mode = str(trigger_mode)
        self.filtered_cmds = {
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'pitch_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
        }

        if pygame is None:
            self._log_warn('未导入 pygame，手柄不可用，控制器将保持悬停。')
            return

        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self._log_info(f'🎮 成功连接控制外设: {self.joystick.get_name()}')
            else:
                self._log_warn('⚠️ 未检测到手柄，控制器将保持悬停。')
        except Exception as exc:
            self._log_warn(f'⚠️ 手柄初始化失败: {exc}，控制器将保持悬停。')
            self.joystick = None

    def _log_info(self, text: str):
        if self.logger:
            self.logger.info(text)
        else:
            print(text)

    def _log_warn(self, text: str):
        if self.logger:
            self.logger.warn(text)
        else:
            print(text)

    def close(self):
        if pygame is not None:
            try:
                pygame.quit()
            except Exception:
                pass

    def _apply_deadzone(self, val: float) -> float:
        return float(val) if abs(float(val)) > self.deadzone else 0.0

    def _apply_expo(self, val: float) -> float:
        return self.expo * (val ** 3) + (1.0 - self.expo) * val

    def _trigger_to_unit(self, raw: float) -> float:
        """将 LT/RT 原始轴值转换为 [0, 1]，并施加死区与 EXPO。"""
        raw = float(raw)
        if self.trigger_mode == 'zero_to_one':
            val = raw
        elif self.trigger_mode == 'one_to_minus_one':
            val = 0.5 * (1.0 - raw)
        else:
            # 默认 Xbox/XInput: -1 未按，+1 按满
            val = 0.5 * (raw + 1.0)

        val = float(np.clip(val, 0.0, 1.0))
        if val <= self.deadzone:
            return 0.0

        # 把死区之后的行程重新归一化到 [0, 1]
        val = (val - self.deadzone) / max(1.0 - self.deadzone, 1e-6)
        return float(np.clip(self._apply_expo(val), 0.0, 1.0))

    def get_velocity_commands(self, dt: float) -> dict:
        if pygame is None or self.joystick is None:
            return self.filtered_cmds.copy()

        try:
            pygame.event.pump()
            num_axes = self.joystick.get_numaxes()

            # Xbox/PS 常用轴映射：0 左摇杆左右；1 左摇杆上下；3 右摇杆左右；4 右摇杆上下
            raw_yaw = self.joystick.get_axis(0) if num_axes > 0 else 0.0
            raw_throttle = self.joystick.get_axis(1) if num_axes > 1 else 0.0
            raw_roll = self.joystick.get_axis(3) if num_axes > 3 else 0.0
            raw_pitch = self.joystick.get_axis(4) if num_axes > 4 else 0.0
            raw_lt = self.joystick.get_axis(self.lt_axis) if num_axes > self.lt_axis else -1.0
            raw_rt = self.joystick.get_axis(self.rt_axis) if num_axes > self.rt_axis else -1.0

            yaw_expo = self._apply_expo(self._apply_deadzone(raw_yaw))
            thr_expo = self._apply_expo(self._apply_deadzone(raw_throttle))
            roll_expo = self._apply_expo(self._apply_deadzone(raw_roll))
            pitch_expo = self._apply_expo(self._apply_deadzone(raw_pitch))
            lt_expo = self._trigger_to_unit(raw_lt)
            rt_expo = self._trigger_to_unit(raw_rt)

            # FLU 机体系：x 前，y 左，z 上；上推为正向前/上升
            target_vx_b = -pitch_expo * self.max_vxy
            target_vy_b = -roll_expo * self.max_vxy
            target_vz_w = -thr_expo * self.max_vz
            target_yaw_rate = -yaw_expo * self.max_yaw_rate

            # LT 增大期望 pitch，RT 减小期望 pitch。
            # 输出是 pitch 角速度，后面在 update_trajectory() 中积分为目标俯仰角。
            target_pitch_rate = (lt_expo - rt_expo) * self.max_pitch_rate

            alpha = dt / (self.filter_tau + dt) if self.filter_tau > 1e-3 else 1.0
            alpha = float(np.clip(alpha, 0.0, 1.0))

            self.filtered_cmds['vx_b'] += alpha * (target_vx_b - self.filtered_cmds['vx_b'])
            self.filtered_cmds['vy_b'] += alpha * (target_vy_b - self.filtered_cmds['vy_b'])
            self.filtered_cmds['vz'] += alpha * (target_vz_w - self.filtered_cmds['vz'])
            self.filtered_cmds['yaw_rate'] += alpha * (target_yaw_rate - self.filtered_cmds['yaw_rate'])
            self.filtered_cmds['pitch_rate'] += alpha * (target_pitch_rate - self.filtered_cmds['pitch_rate'])
            self.filtered_cmds['lt'] = lt_expo
            self.filtered_cmds['rt'] = rt_expo
            return self.filtered_cmds.copy()
        except Exception as exc:
            self._log_warn(f'读取手柄失败: {exc}，本周期保持上一指令。')
            return self.filtered_cmds.copy()


class HnuterController(Node):
    def __init__(self):
        super().__init__('hnuter_controller_gamepad')

        qos_profile_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_profile_command = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.actuator_motors_pub = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile_in)
        self.actuator_servos_pub = self.create_publisher(
            ActuatorServos, '/fmu/in/actuator_servos', qos_profile_in)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_command)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile_command)

        self.gz_servo0_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_0', 10)
        self.gz_servo1_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_1', 10)
        self.gz_servo2_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_2', 10)
        self.gz_servo3_pub = self.create_publisher(Float64, '/model/hnuter_0/servo_3', 10)
        self.plot_pub = self.create_publisher(Float64MultiArray, '/plot_data', 10)
        self.publish_gz_servos_direct = False

        self.odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile_out)
        self.odometry_sub_v1 = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry_v1', self.odometry_callback, qos_profile_out)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos_profile_out)
        self.attitude_sub_v1 = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude_v1', self.attitude_callback, qos_profile_out)
        self.angular_velocity_sub = self.create_subscription(
            VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', self.angular_velocity_callback, qos_profile_out)
        self.angular_velocity_sub_v1 = self.create_subscription(
            VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity_v1', self.angular_velocity_callback, qos_profile_out)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile_out)
        self.vehicle_status_sub_v1 = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_callback, qos_profile_out)

        # PX4 常量，兼容不同 px4_msgs 版本
        self.CMD_DO_SET_MODE = getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176)
        self.CMD_COMPONENT_ARM_DISARM = getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400)
        self.NAVIGATION_STATE_OFFBOARD = getattr(VehicleStatus, 'NAVIGATION_STATE_OFFBOARD', 14)
        self.ARMING_STATE_ARMED = getattr(VehicleStatus, 'ARMING_STATE_ARMED', 2)

        # State variables
        self.position = np.zeros(3)       # ENU: x East, y North, z Up
        self.velocity = np.zeros(3)       # ENU
        self.attitude_q = np.array([1.0, 0.0, 0.0, 0.0])
        self.angular_velocity = np.zeros(3)  # FLU body angular velocity
        self.R = np.eye(3)                # ENU <- FLU
        self.nav_state = None
        self.armed = False
        self.data_received = False
        self.px4_timestamp = 0

        # Offboard/Arm 启动状态机
        self.offboard_setpoint_counter = 0
        self._last_offboard_cmd_time = 0.0
        self._last_arm_cmd_time = 0.0
        self._offboard_request_sent = False
        self._arm_request_sent = False

        # ====== 启动策略配置：防止 PX4 自动 disarm 后被程序反复 arm ======
        # True : 节点启动后自动尝试切 Offboard，并只自动 Arm 一次。
        # False: 节点只维持 OffboardControlMode 心跳，需要你用 QGC/遥控器手动 Arm。
        self.auto_arm_enabled = True

        # 强烈建议 False。PX4 如果因为预起飞超时、落地检测或 failsafe disarm，
        # 程序不应立刻再次解锁，否则会出现“反复 arm / 反复起落”的循环。
        self.rearm_after_auto_disarm = False

        # 自动 Arm 最多尝试次数。调试期建议 1；若想完全手动解锁，设 auto_arm_enabled=False。
        self.max_auto_arm_attempts = 1
        self.auto_arm_attempts = 0
        self.was_armed_once = False
        self._last_armed_state = False
        self.startup_blocked_after_disarm = False

        # PX4 要求进入 Offboard 前先连续发送 >1s 的 OffboardControlMode。
        # 这里 20Hz * 30 = 1.5s，留出裕量。
        self.offboard_warmup_ticks = 30
        self.mode_request_period_s = 1.0
        self.arm_request_period_s = 1.0

        # Debug variables
        self.last_motor_cmd = np.zeros(12)
        self.last_servo_cmd = np.zeros(8)
        self.last_F1 = 0.0
        self.last_F2 = 0.0
        self.last_F3 = 0.0
        self.last_velocity_left = 0.0
        self.last_velocity_right = 0.0
        self.last_velocity_tail = 0.0
        self.control_loop_count = 0
        self.last_W = np.zeros(6)
        self._last_manual_cmd = {
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'pitch_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
        }

        # Physical parameters
        self.mass = 4.5
        self.gravity = 9.81
        self.J = np.diag([0.2456, 0.1276, 0.3264])
        self.l1 = 0.33
        self.l2 = 0.664
        self.tail_thrust_scale = 0.08
        self.tail_control_limit = 1.0

        # Actuator limits
        self.alpha_limit_rad = np.radians(60.0)
        self.theta_limit_rad = np.radians(45.0)
        self.servo_rate_limit_rad_s = 50.0
        self.takeoff_tilt_suppress_time_s = 1.0
        self.takeoff_tilt_limit_rad = np.radians(20.0)
        self.disable_tail_at_takeoff = False
        self.takeoff_xy_lock_time_s = 3.0
        self.xy_lock_max_acc_xy = 3.0
        self.xy_lock_tilt_limit_rad = np.radians(30.0)
        self.xy_lock_kp_scale = 0.8
        self._xy_lock_initialized = False
        self._xy_lock_position = np.zeros(2)
        self._xy_lock_active = False

        # 不再永久 hover_only，否则会覆盖手柄 XY 目标点
        self.hover_only = False

        # Yaw variables
        self._yaw_initialized = False
        self.initial_yaw = 0.0

        self._alpha1_cmd = 0.0
        self._alpha2_cmd = 0.0
        self._theta1_cmd = 0.0
        self._theta2_cmd = 0.0

        # Position loop
        self.Kp = np.diag([2.5, 2.5, 8.0])
        self.Dp = np.diag([1.8, 1.8, 4.0])
        self.K_pos_I = np.array([0.0, 0.0, 3.0])
        self.integral_pos_error = np.zeros(3)

        # Attitude loop
        self.KR = np.array([1.5, 1.5, 1.5])
        self.Domega = np.array([1.2, 1.2, 1.2])
        self.KI = np.array([0.0, 0.0, 0.0])
        self.integral_e_R = np.zeros(3)

        self.max_acc_xy = 20.0
        self.max_acc_z = 20.0
        self.max_climb_rate = 1.0

        self.target_position = np.array([0.0, 0.0, 1.3])
        self.target_velocity = np.zeros(3)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, 0.0, 0.0])
        self.target_attitude_rate = np.zeros(3)

        self.takeoff_height = 1.3
        self.max_altitude = 5.0
        self.min_altitude = 0.25
        self.manual_enabled = True
        self.manual_pos_initialized = False
        self.manual_des_pos = np.zeros(3)   # [x_enu, y_enu, z_relative]
        self.manual_des_yaw = 0.0
        # LT/RT 积分得到的俯仰姿态期望。
        # 正号严格按 euler_to_rotation_matrix() 的 pitch 正方向；若实机观察方向相反，
        # 只需要在 GamepadManager 中把 target_pitch_rate 改成 (rt_expo - lt_expo)。
        self.manual_des_pitch = 0.0
        self.manual_pitch_limit_rad = np.radians(15.0)
        self._z0_initialized = False
        self._z0 = 0.0
        self._z_sp = 0.0

        # Time
        self.sim_start_time_s = 0.0
        self._last_timestamp_s = 0.0

        # Timers: Offboard heartbeat should be comfortably > 2 Hz
        self.offboard_timer = self.create_timer(0.05, self.offboard_startup_tick)
        self.status_timer = self.create_timer(1.0, self.print_status)
        self.debug_print_period_s = 1.0
        self._last_debug_print_time = 0.0

        # Gamepad: 实机建议先用低速度，确认方向后再加大
        self.gamepad = GamepadManager(
            max_vxy=1.0,
            max_vz=0.5,
            max_yaw_rate=0.6,
            max_pitch_rate=math.radians(20.0),
            deadzone=0.10,
            expo=0.40,
            filter_tau=0.20,
            lt_axis=2,
            rt_axis=5,
            trigger_mode='minus_one_to_one',
            logger=self.get_logger()
        )

        # Logs
        self.log_time = []
        self.log_motors = {0: [], 1: [], 2: [], 3: [], 4: []}
        self.log_servos = {0: [], 1: [], 2: [], 3: []}
        self.log_attitude = {'roll': [], 'pitch': [], 'yaw': []}
        self.log_start_time = None

        self.get_logger().info('Hnuter PX4 External Controller initialized: Gamepad + fixed Offboard/Arm state machine')

    # ============================================================
    # PX4 callbacks
    # ============================================================
    def odometry_callback(self, msg):
        self.px4_timestamp = int(msg.timestamp)
        pos_ned = msg.position
        vel_ned = msg.velocity
        self.position = np.array([pos_ned[1], pos_ned[0], -pos_ned[2]], dtype=float)
        self.velocity = np.array([vel_ned[1], vel_ned[0], -vel_ned[2]], dtype=float)
        self.data_received = True

    def attitude_callback(self, msg):
        self.px4_timestamp = int(msg.timestamp)
        w, x, y, z = msg.q
        self.attitude_q = np.array([w, x, y, z], dtype=float)
        R_ned_frd = np.array([
            [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x ** 2 + y ** 2)]
        ])
        R_enu_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_frd_flu = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.R = R_enu_ned @ R_ned_frd @ R_frd_flu

        if not self._yaw_initialized:
            self.initial_yaw = float(np.arctan2(self.R[1, 0], self.R[0, 0]))
            self.target_attitude[2] = self.initial_yaw
            self.manual_des_yaw = self.initial_yaw
            self._yaw_initialized = True

        self.data_received = True
        self.control_loop()

    def angular_velocity_callback(self, msg):
        # PX4 FRD -> FLU
        self.angular_velocity = np.array([msg.xyz[0], -msg.xyz[1], -msg.xyz[2]], dtype=float)

    def status_callback(self, msg):
        self.armed = (int(msg.arming_state) == self.ARMING_STATE_ARMED)
        self.nav_state = int(getattr(msg, 'nav_state', -1))

    # ============================================================
    # Offboard/Arm startup logic
    # ============================================================
    def is_offboard(self) -> bool:
        return self.nav_state == self.NAVIGATION_STATE_OFFBOARD

    def timestamp_now_us(self) -> int:
        return int(self.px4_timestamp) if self.px4_timestamp > 0 else int(self.get_clock().now().nanoseconds / 1000)

    def offboard_startup_tick(self):
        # 1) 始终发送 OffboardControlMode 作为 proof-of-life，频率 20Hz。
        #    这是维持 Offboard 的心跳，不等价于重复 arm。
        self.publish_offboard_control_mode()

        # 2) 未收到状态数据前不切模式、不解锁。
        if not self.data_received or self.px4_timestamp <= 0:
            return

        self.offboard_setpoint_counter += 1
        now = time.time()

        # 3) 检测 PX4 是否从 armed 变成 disarmed。
        #    如果已经成功 arm 过一次，之后又被 PX4 自动上锁，默认禁止自动二次 arm。
        if self._last_armed_state and not self.armed:
            self.was_armed_once = True
            self.manual_pos_initialized = False
            self.integral_pos_error[:] = 0.0
            self.integral_e_R[:] = 0.0
            if not self.rearm_after_auto_disarm:
                self.startup_blocked_after_disarm = True
                self.get_logger().warn(
                    'PX4 已从 armed 变为 disarmed。已阻止自动二次 Arm。'
                    '请检查是否触发 COM_DISARM_PRFLT、COM_DISARM_LAND、land detector 或 failsafe；'
                    '确认安全后重启本节点或手动 Arm。'
                )
        self._last_armed_state = self.armed

        if self.startup_blocked_after_disarm:
            return

        # 4) 至少连续发送 1s 以上 OffboardControlMode 后，再请求 Offboard。
        stream_ready = self.offboard_setpoint_counter >= self.offboard_warmup_ticks
        if stream_ready and not self.is_offboard():
            if now - self._last_offboard_cmd_time > self.mode_request_period_s:
                self.set_offboard_mode()
                self._last_offboard_cmd_time = now
                self._offboard_request_sent = True
                self.get_logger().info('请求切换到 Offboard 模式...')
            return

        # 5) 已进入 Offboard 后再 Arm；并且默认只自动 Arm 一次。
        if self.is_offboard() and not self.armed:
            if not self.auto_arm_enabled:
                return
            if self.was_armed_once and not self.rearm_after_auto_disarm:
                return
            if self.auto_arm_attempts >= self.max_auto_arm_attempts:
                return
            if now - self._last_arm_cmd_time > self.arm_request_period_s:
                self.arm()
                self.auto_arm_attempts += 1
                self._last_arm_cmd_time = now
                self._arm_request_sent = True
                self.get_logger().info(
                    f'请求 Arm 解锁... ({self.auto_arm_attempts}/{self.max_auto_arm_attempts})'
                )

        if self.armed:
            self.was_armed_once = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        # 兼容不同 px4_msgs 版本
        if hasattr(msg, 'thrust_and_torque'):
            msg.thrust_and_torque = False
        if hasattr(msg, 'direct_actuator'):
            msg.direct_actuator = True
        msg.timestamp = self.timestamp_now_us()
        self.offboard_control_mode_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0,
                                param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp_now_us()
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(self.CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.publish_vehicle_command(self.CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def set_offboard_mode(self):
        # VEHICLE_CMD_DO_SET_MODE: param1=1(custom), param2=6(OFFBOARD)
        self.publish_vehicle_command(self.CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    # ============================================================
    # Manual trajectory: gamepad velocity -> desired position/yaw
    # ============================================================
    def update_trajectory(self, current_time: float, dt: float):
        if not self._z0_initialized:
            self._z0 = float(self.position[2])
            self._z0_initialized = True

        # 未进入 offboard 或未解锁前，目标点贴住当前点，清积分，避免一解锁就猛冲
        if (not self.is_offboard()) or (not self.armed):
            self.integral_pos_error[:] = 0.0
            self.integral_e_R[:] = 0.0
            self.manual_pos_initialized = False
            self._z_sp = 0.0
            self.manual_des_pitch = 0.0
            self.target_position = np.array([self.position[0], self.position[1], 0.0])
            self.target_velocity = np.zeros(3)
            self.target_acceleration = np.zeros(3)
            self.target_attitude = np.array([0.0, 0.0, self.initial_yaw])
            self.target_attitude_rate = np.zeros(3)
            return

        if not self.manual_pos_initialized:
            self.manual_des_pos = np.array([self.position[0], self.position[1], max(0.0, self.position[2] - self._z0)])
            self.manual_des_yaw = self.initial_yaw if self._yaw_initialized else 0.0
            self.manual_des_pitch = 0.0
            self._z_sp = float(self.manual_des_pos[2])
            self._xy_lock_position = self.position[:2].copy()
            self._xy_lock_initialized = True
            self.manual_pos_initialized = True

        cmds = self.gamepad.get_velocity_commands(dt) if self.manual_enabled else {
            'vx_b': 0.0,
            'vy_b': 0.0,
            'vz': 0.0,
            'yaw_rate': 0.0,
            'pitch_rate': 0.0,
            'lt': 0.0,
            'rt': 0.0,
        }
        self._last_manual_cmd = cmds.copy()

        # 初始爬升：若手柄不动，则自动缓慢爬到 takeoff_height；若手柄给 z，则叠加人工指令
        z_auto_vel = 0.0
        if self.manual_des_pos[2] < self.takeoff_height:
            z_err = self.takeoff_height - self.manual_des_pos[2]
            z_auto_vel = float(np.clip(z_err, 0.0, self.max_climb_rate))

        yaw_ref = self.manual_des_yaw
        vx_w = cmds['vx_b'] * math.cos(yaw_ref) - cmds['vy_b'] * math.sin(yaw_ref)
        vy_w = cmds['vx_b'] * math.sin(yaw_ref) + cmds['vy_b'] * math.cos(yaw_ref)
        vz_w = cmds['vz'] + z_auto_vel
        yaw_rate = cmds['yaw_rate']
        pitch_rate = cmds.get('pitch_rate', 0.0)

        self.manual_des_pos[0] += vx_w * dt
        self.manual_des_pos[1] += vy_w * dt
        self.manual_des_pos[2] += vz_w * dt
        self.manual_des_pos[2] = float(np.clip(self.manual_des_pos[2], self.min_altitude, self.max_altitude))
        self.manual_des_yaw = float(np.arctan2(math.sin(self.manual_des_yaw + yaw_rate * dt), math.cos(self.manual_des_yaw + yaw_rate * dt)))
        self.manual_des_pitch = float(np.clip(
            self.manual_des_pitch + pitch_rate * dt,
            -self.manual_pitch_limit_rad,
            self.manual_pitch_limit_rad
        ))

        self.target_position = self.manual_des_pos.copy()
        self.target_velocity = np.array([vx_w, vy_w, vz_w], dtype=float)
        self.target_acceleration = np.zeros(3)
        self.target_attitude = np.array([0.0, self.manual_des_pitch, self.manual_des_yaw], dtype=float)
        self.target_attitude_rate = np.array([0.0, pitch_rate, yaw_rate], dtype=float)

    # ============================================================
    # Geometry controller and allocation
    # ============================================================
    def euler_to_rotation_matrix(self, euler):
        roll, pitch, yaw = euler
        R_x = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        return R_z @ R_y @ R_x

    @staticmethod
    def vee_map(S):
        return np.array([S[2, 1], S[0, 2], S[1, 0]])

    def compute_control_wrench(self, dt):
        pos_curr = self.position.copy()
        pos_curr[2] = float(pos_curr[2] - self._z0)
        pos_error = self.target_position - pos_curr
        vel_error = self.target_velocity - self.velocity

        self.integral_pos_error += pos_error * dt
        self.integral_pos_error[:2] = np.clip(self.integral_pos_error[:2], -1.0, 1.0)
        self.integral_pos_error[2] = float(np.clip(self.integral_pos_error[2], -2.0, 2.0))

        Kp = self.Kp * float(self.xy_lock_kp_scale) if self._xy_lock_active else self.Kp
        acc_des = self.target_acceleration + Kp @ pos_error + self.Dp @ vel_error + self.K_pos_I * self.integral_pos_error

        max_acc_xy = float(self.xy_lock_max_acc_xy) if self._xy_lock_active else float(self.max_acc_xy)
        acc_des[0] = float(np.clip(acc_des[0], -max_acc_xy, max_acc_xy))
        acc_des[1] = float(np.clip(acc_des[1], -max_acc_xy, max_acc_xy))
        acc_des[2] = float(np.clip(acc_des[2], -self.max_acc_z, self.max_acc_z))
        f_c_world = self.mass * (acc_des + np.array([0, 0, self.gravity]))

        R_des = self.euler_to_rotation_matrix(self.target_attitude)
        e_R = 0.5 * self.vee_map(R_des.T @ self.R - self.R.T @ R_des)
        self.integral_e_R += e_R * dt
        self.integral_e_R = np.clip(self.integral_e_R, -1.5, 1.5)

        omega_error = self.angular_velocity - self.R.T @ R_des @ self.target_attitude_rate
        tau_c = -self.KR * e_R - self.KI * self.integral_e_R - self.Domega * omega_error
        tau_c[2] = float(np.clip(tau_c[2], -0.5, 0.5))

        f_c_body = self.R.T @ f_c_world
        return f_c_body, tau_c, e_R, f_c_world

    def inverse_nonlinear_mapping(self, W):
        l1, l2 = self.l1, self.l2
        r_x = 0.105
        r_z = -0.013

        u1 = W[0] / 2.0 - W[5] / (2.0 * l1)
        u4 = W[0] / 2.0 + W[5] / (2.0 * l1)

        Ty_parasitic = r_z * W[0] - r_x * W[2]
        Ty_comp = W[4] - Ty_parasitic
        F3 = float(Ty_comp / (r_x + l2))
        Fz_front = float(W[2] - F3)

        Tx_parasitic = -r_z * W[1]
        Tx_comp = W[3] - Tx_parasitic
        u2 = Fz_front / 2.0 + Tx_comp / (2.0 * l1)
        u5 = Fz_front / 2.0 - Tx_comp / (2.0 * l1)
        u3 = -W[1] / 2.0
        u6 = -W[1] / 2.0

        F1 = np.sqrt(u1 ** 2 + u2 ** 2 + u3 ** 2)
        F2 = np.sqrt(u4 ** 2 + u5 ** 2 + u6 ** 2)
        eps = 1e-8
        F1_safe = max(F1, eps)
        F2_safe = max(F2, eps)

        alpha1 = np.arctan2(u1, u2)
        alpha2 = np.arctan2(u4, u5)
        theta1 = np.arcsin(np.clip(u3 / F1_safe, -0.99, 0.99))
        theta2 = np.arcsin(np.clip(u6 / F2_safe, -0.99, 0.99))

        F1 = np.clip(F1, 0.0, 50.0)
        F2 = np.clip(F2, 0.0, 50.0)
        F3 = np.clip(F3, -50.0, 50.0)
        alpha1 = np.clip(alpha1, -self.alpha_limit_rad, self.alpha_limit_rad)
        alpha2 = np.clip(alpha2, -self.alpha_limit_rad, self.alpha_limit_rad)
        theta1 = np.clip(theta1, -self.theta_limit_rad, self.theta_limit_rad)
        theta2 = np.clip(theta2, -self.theta_limit_rad, self.theta_limit_rad)
        return F1, F2, F3, alpha1, alpha2, theta1, theta2

    def control_loop(self):
        if not self.data_received or self.px4_timestamp <= 0:
            return

        now_s = self.px4_timestamp / 1_000_000.0
        if self.sim_start_time_s == 0.0:
            self.sim_start_time_s = now_s
            self._last_timestamp_s = now_s
            return

        dt = now_s - self._last_timestamp_s
        if dt <= 0.0001 or dt > 0.2:
            self._last_timestamp_s = now_s
            return

        self._last_timestamp_s = now_s
        current_time = now_s - self.sim_start_time_s

        self.update_trajectory(current_time, dt)

        # 没进入 Offboard 时不向 actuator topics 写入外部 setpoint，避免和 PX4 内部控制器冲突
        if not self.is_offboard():
            return

        if not self._xy_lock_initialized:
            self._xy_lock_position = self.position[:2].copy()
            self._xy_lock_initialized = True

        suppress_tilts = (current_time < self.takeoff_tilt_suppress_time_s)
        xy_lock = (current_time >= self.takeoff_tilt_suppress_time_s and current_time < self.takeoff_xy_lock_time_s)
        self._xy_lock_active = bool(xy_lock)

        if xy_lock:
            self.target_position[0] = float(self._xy_lock_position[0])
            self.target_position[1] = float(self._xy_lock_position[1])
            self.target_velocity[0] = 0.0
            self.target_velocity[1] = 0.0

        f_c_body, tau_c, e_R, f_c_world = self.compute_control_wrench(dt)
        W = np.array([f_c_body[0], f_c_body[1], f_c_body[2], tau_c[0], tau_c[1], tau_c[2]])

        if suppress_tilts:
            W[0] = 0.0
            W[1] = 0.0

        F1, F2, F3, alpha1, alpha2, theta1, theta2 = self.inverse_nonlinear_mapping(W)

        if suppress_tilts:
            alpha1 = np.clip(alpha1, -self.takeoff_tilt_limit_rad, self.takeoff_tilt_limit_rad)
            alpha2 = np.clip(alpha2, -self.takeoff_tilt_limit_rad, self.takeoff_tilt_limit_rad)
            theta1 = np.clip(theta1, -self.takeoff_tilt_limit_rad, self.takeoff_tilt_limit_rad)
            theta2 = np.clip(theta2, -self.takeoff_tilt_limit_rad, self.takeoff_tilt_limit_rad)
            if self.disable_tail_at_takeoff:
                F3 = 0.0

        if self._xy_lock_active:
            alpha1 = np.clip(alpha1, -self.xy_lock_tilt_limit_rad, self.xy_lock_tilt_limit_rad)
            alpha2 = np.clip(alpha2, -self.xy_lock_tilt_limit_rad, self.xy_lock_tilt_limit_rad)
            theta1 = np.clip(theta1, -self.xy_lock_tilt_limit_rad, self.xy_lock_tilt_limit_rad)
            theta2 = np.clip(theta2, -self.xy_lock_tilt_limit_rad, self.xy_lock_tilt_limit_rad)

        self.last_F1 = F1
        self.last_F2 = F2
        self.last_F3 = F3
        self.last_W = W
        self.control_loop_count += 1
        self.publish_actuator_commands(F1, F2, F3, alpha1, alpha2, theta1, theta2, dt)

        now = time.time()
        if now - self._last_debug_print_time >= self.debug_print_period_s:
            self.get_logger().info(f'控制 dt={dt * 1000:.1f}ms | Offboard={self.is_offboard()} | Armed={self.armed}')
            self._last_debug_print_time = now

    @staticmethod
    def _slew_limit(current, target, rate_limit, dt):
        delta = target - current
        max_delta = rate_limit * dt
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def publish_actuator_commands(self, F1, F2, F3, alpha1, alpha2, theta1, theta2, dt):
        motor_constant = 8.54858e-05
        min_velocity = 10.0
        max_velocity = 1000.0

        T_single_left = F1 / 2.0
        T_single_right = F2 / 2.0
        velocity_left = np.sqrt(max(T_single_left, 0.0) / motor_constant)
        velocity_right = np.sqrt(max(T_single_right, 0.0) / motor_constant)
        velocity_tail = np.sqrt(abs(F3) / motor_constant)

        self.last_velocity_left = velocity_left
        self.last_velocity_right = velocity_right
        self.last_velocity_tail = velocity_tail

        normalized_left = (velocity_left - min_velocity) / (max_velocity - min_velocity)
        normalized_right = (velocity_right - min_velocity) / (max_velocity - min_velocity)
        self._alpha1_cmd = self._slew_limit(self._alpha1_cmd, alpha1, self.servo_rate_limit_rad_s, dt)
        self._alpha2_cmd = self._slew_limit(self._alpha2_cmd, alpha2, self.servo_rate_limit_rad_s, dt)
        self._theta1_cmd = self._slew_limit(self._theta1_cmd, theta1, self.servo_rate_limit_rad_s, dt)
        self._theta2_cmd = self._slew_limit(self._theta2_cmd, theta2, self.servo_rate_limit_rad_s, dt)

        motor_msg = ActuatorMotors()
        motor_msg.timestamp = self.timestamp_now_us()
        motor_msg.control = [float('nan')] * 12
        tail_is_reversible = hasattr(motor_msg, 'reversible_flags')
        if tail_is_reversible:
            motor_msg.reversible_flags = 1 << 4
            normalized_tail = math.copysign(velocity_tail / max_velocity, F3) if velocity_tail > 0.0 else 0.0
        else:
            normalized_tail = (velocity_tail - min_velocity) / (max_velocity - min_velocity)

        motor_msg.control[0] = float(np.clip(normalized_right, 0.0, 1.0))
        motor_msg.control[1] = float(np.clip(normalized_right, 0.0, 1.0))
        motor_msg.control[2] = float(np.clip(normalized_left, 0.0, 1.0))
        motor_msg.control[3] = float(np.clip(normalized_left, 0.0, 1.0))
        motor_msg.control[4] = float(np.clip(
            normalized_tail,
            -self.tail_control_limit if tail_is_reversible else 0.0,
            self.tail_control_limit
        ))
        self.last_motor_cmd = np.array(motor_msg.control)
        self.actuator_motors_pub.publish(motor_msg)

        servo_msg = ActuatorServos()
        servo_msg.timestamp = motor_msg.timestamp
        servo_msg.control = [float('nan')] * 8
        angle_max = np.radians(90.0)
        servo_msg.control[0] = float(np.clip(self._alpha2_cmd / angle_max, -1.0, 1.0))
        servo_msg.control[1] = float(np.clip(self._alpha1_cmd / angle_max, -1.0, 1.0))
        servo_msg.control[2] = float(np.clip(self._theta2_cmd / angle_max, -1.0, 1.0))
        servo_msg.control[3] = float(np.clip(self._theta1_cmd / angle_max, -1.0, 1.0))
        self.last_servo_cmd = np.array(servo_msg.control)
        self.actuator_servos_pub.publish(servo_msg)

        current_pitch = float(np.arcsin(np.clip(-self.R[2, 0], -1.0, 1.0)))
        plot_msg = Float64MultiArray()
        plot_msg.data = [
            float(F1), float(F2), float(F3),
            float(np.degrees(alpha1)), float(np.degrees(alpha2)),
            float(np.degrees(current_pitch)),
            float(self._last_manual_cmd['vx_b']),
            float(self._last_manual_cmd['vy_b']),
            float(self._last_manual_cmd['vz']),
            float(self._last_manual_cmd['yaw_rate']),
            float(np.degrees(self._last_manual_cmd.get('pitch_rate', 0.0))),
            float(np.degrees(self.manual_des_pitch)),
            float(self._last_manual_cmd.get('lt', 0.0)),
            float(self._last_manual_cmd.get('rt', 0.0))
        ]
        self.plot_pub.publish(plot_msg)
        self._record_log(motor_msg, servo_msg)

    def _record_log(self, motor_msg, servo_msg):
        if self.log_start_time is None:
            self.log_start_time = time.time()
        current_t = time.time() - self.log_start_time
        self.log_time.append(current_t)
        for i in range(5):
            self.log_motors[i].append(motor_msg.control[i])
        for i in range(4):
            self.log_servos[i].append(servo_msg.control[i])
        roll = np.arctan2(self.R[2, 1], self.R[2, 2])
        pitch = np.arcsin(np.clip(-self.R[2, 0], -1.0, 1.0))
        yaw = np.arctan2(self.R[1, 0], self.R[0, 0])
        self.log_attitude['roll'].append(np.degrees(roll))
        self.log_attitude['pitch'].append(np.degrees(pitch))
        self.log_attitude['yaw'].append(np.degrees(yaw))

    # ============================================================
    # Status/plot/shutdown
    # ============================================================
    def print_status(self):
        if not self.data_received:
            self.get_logger().info('等待 PX4 odometry/attitude/status 数据...')
            return

        control_hz = self.control_loop_count
        self.control_loop_count = 0
        pos_curr_rel_z = self.position[2] - self._z0 if self._z0_initialized else self.position[2]
        current_pitch_deg = float(np.degrees(np.arcsin(np.clip(-self.R[2, 0], -1.0, 1.0))))
        self.get_logger().info(
            f"\n{'=' * 72}\n"
            f"Mode: Offboard={self.is_offboard()} | Armed={self.armed} | nav_state={self.nav_state} | ctrl≈{control_hz}Hz\n"
            f"Target ENU/Zrel: [{self.target_position[0]:6.2f}, {self.target_position[1]:6.2f}, {self.target_position[2]:6.2f}] m\n"
            f"Current ENU/Zrel: [{self.position[0]:6.2f}, {self.position[1]:6.2f}, {pos_curr_rel_z:6.2f}] m\n"
            f"Gamepad: vx_b={self._last_manual_cmd['vx_b']:+4.2f}, vy_b={self._last_manual_cmd['vy_b']:+4.2f}, "
            f"vz={self._last_manual_cmd['vz']:+4.2f}, yaw_rate={self._last_manual_cmd['yaw_rate']:+4.2f}, "
            f"LT={self._last_manual_cmd.get('lt', 0.0):4.2f}, RT={self._last_manual_cmd.get('rt', 0.0):4.2f}\n"
            f"Pitch: des={np.degrees(self.manual_des_pitch):+5.1f}° | current={current_pitch_deg:+5.1f}° | "
            f"pitch_rate={np.degrees(self._last_manual_cmd.get('pitch_rate', 0.0)):+5.1f}°/s\n"
            f"Wrench: Fx={self.last_W[0]:+5.2f}N, Fy={self.last_W[1]:+5.2f}N, Fz={self.last_W[2]:+5.2f}N\n"
            f"Thrust: F1={self.last_F1:5.2f}N | F2={self.last_F2:5.2f}N | F3={self.last_F3:5.2f}N\n"
            f"Tilt: A1={np.degrees(self._alpha1_cmd):+5.1f}° | A2={np.degrees(self._alpha2_cmd):+5.1f}° | "
            f"T1={np.degrees(self._theta1_cmd):+5.1f}° | T2={np.degrees(self._theta2_cmd):+5.1f}°\n"
            f"{'=' * 72}"
        )

    def plot_results(self):
        if not self.log_time:
            self.get_logger().info('没有记录到足够的数据，无法绘图。')
            return

        save_dir = 'hnuter_saved_plots'
        os.makedirs(save_dir, exist_ok=True)
        self.get_logger().info(f'正在生成图表，图片将保存至: {save_dir}')

        plt.figure('Motor Commands', figsize=(10, 6))
        for i in range(5):
            plt.plot(self.log_time, self.log_motors[i], label=f'Motor {i}')
        plt.title('Motor Control Commands over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Normalized Command [0, 1]')
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.savefig(os.path.join(save_dir, 'motor_commands.png'), dpi=300, bbox_inches='tight')

        plt.figure('Servo Commands', figsize=(10, 6))
        for i in range(4):
            plt.plot(self.log_time, self.log_servos[i], label=f'Servo {i}')
        plt.title('Servo Control Commands over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Normalized Command [-1, 1]')
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.savefig(os.path.join(save_dir, 'servo_commands.png'), dpi=300, bbox_inches='tight')

        plt.figure('Attitude Angles', figsize=(10, 6))
        plt.plot(self.log_time, self.log_attitude['roll'], label='Roll (deg)')
        plt.plot(self.log_time, self.log_attitude['pitch'], label='Pitch (deg)')
        plt.plot(self.log_time, self.log_attitude['yaw'], label='Yaw (deg)')
        plt.title('Attitude Angles over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (deg)')
        plt.legend(loc='upper right')
        plt.grid(True)
        plt.savefig(os.path.join(save_dir, 'attitude_angles.png'), dpi=300, bbox_inches='tight')
        self.get_logger().info('图表保存完成。')
        plt.show()

    def destroy_node(self):
        try:
            self.gamepad.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('接收到终止信号，准备绘制历史曲线...')
        controller.plot_results()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
