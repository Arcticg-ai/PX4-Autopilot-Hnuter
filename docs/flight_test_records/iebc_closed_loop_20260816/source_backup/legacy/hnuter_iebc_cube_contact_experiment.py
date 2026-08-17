#!/usr/bin/env python3
"""Automated Gazebo-only IEBC cube-contact experiment for HNUTER.

This entry point deliberately stays separate from the real-aircraft IEBC
controller.  It reuses :class:`InteractionEnergyBarrierFilter` and the PX4
position-setpoint conversion from
``hnuter_external_controller_px4_position_hardware_iebc.py``, but adds a
simulation-only state machine, Gazebo contact/wrench transport, and PX4
VehicleCommand messages so the complete experiment can be reproduced without
a transmitter.

Experiment sequence (Gazebo world X == controller ENU X):

1. Arm, enter Offboard and rise to the cube centre height.
2. Hold position and yaw until the probe's physical +X axis faces the cube.
3. Approach with the lightweight cylindrical probe until contact is measured.
4. Apply a persistent constant -X virtual force to the rail-mounted cube.
5. Increase the forward position reference slowly so contact force rises.
6. The cube starts against the rail's -X stop. Once it moves continuously in
   +X, the aircraft has physically overcome the virtual force; clear that
   force and hold the reference. Contact force is logged diagnostically but
   collision impulses are not used as the release trigger.
7. Record the vehicle/cube response and IEBC energy state to CSV.

This file must never be used on real hardware.  It is guarded by an explicit
environment variable and by the expected Gazebo world name.
"""

import csv
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

# Conservative, explicit defaults for this one-dimensional contact experiment.
# They must be set before importing the base IEBC module because its constructor
# reads the environment.
os.environ.setdefault('HNUTER_IEBC_ENABLE', '1')
os.environ.setdefault('HNUTER_IEBC_MASS_KG', '4.5')
os.environ.setdefault('HNUTER_IEBC_LAMBDA_BAR_KG', '4.5')
os.environ.setdefault('HNUTER_IEBC_E_MAX_J', '1.2')
os.environ.setdefault('HNUTER_IEBC_AXIS_X', '1.0')
os.environ.setdefault('HNUTER_IEBC_AXIS_Y', '0.0')
os.environ.setdefault('HNUTER_IEBC_AXIS_Z', '0.0')
os.environ.setdefault('HNUTER_IEBC_CBF_GAMMA', '4.0')
os.environ.setdefault('HNUTER_IEBC_MAX_ACCEL_MPS2', '0.8')
os.environ.setdefault('HNUTER_IEBC_ENGAGE_POWER_W', '0.05')
os.environ.setdefault('HNUTER_IEBC_POWER_MARGIN_W', '0.05')
os.environ.setdefault('HNUTER_IEBC_FORCE_ERROR_BOUND_N', '0.20')
os.environ.setdefault('HNUTER_IEBC_WRENCH_SOURCE', 'proxy')

from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleCommand

from hnuter_external_controller_px4_position_hardware_iebc import HnuterController
from hnuter_log_paths import diagnostic_csv_path


def smoothstep01(value: float) -> tuple:
    """Return cubic smooth-step position, first and second derivatives."""
    u = float(np.clip(value, 0.0, 1.0))
    return (3.0 * u ** 2 - 2.0 * u ** 3,
            6.0 * u * (1.0 - u),
            6.0 * (1.0 - 2.0 * u))


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return float(math.atan2(math.sin(angle_rad), math.cos(angle_rad)))


class ContactForceFilter:
    """Age-gated first-order filter for Gazebo's contact-wrench samples."""

    def __init__(self, tau_s: float = 0.08, timeout_s: float = 0.15):
        self.tau_s = max(float(tau_s), 0.0)
        self.timeout_s = max(float(timeout_s), 0.01)
        self.raw_n = 0.0
        self.filtered_n = 0.0
        self.last_sample_monotonic = -math.inf

    def feed(self, force_n: float, received_s: float = None) -> None:
        self.raw_n = max(float(force_n), 0.0)
        self.last_sample_monotonic = (
            time.monotonic() if received_s is None else float(received_s))

    def update(self, dt: float, now_s: float = None) -> float:
        now_s = time.monotonic() if now_s is None else float(now_s)
        target = self.raw_n if now_s - self.last_sample_monotonic <= self.timeout_s else 0.0
        alpha = 1.0 if self.tau_s <= 1e-6 else float(np.clip(dt / (self.tau_s + dt), 0.0, 1.0))
        self.filtered_n += alpha * (target - self.filtered_n)
        return self.filtered_n


class HnuterIebcCubeContactExperiment(HnuterController):
    """SITL-only controller and Gazebo experiment coordinator."""

    EXPECTED_WORLD = 'hnuter_cube_contact'
    CUBE_MODEL = 'interaction_cube'
    VEHICLE_MODEL_PREFIX = 'hnuter_contact_'
    CONTACT_TOPIC = '/hnuter/cube_contact'

    STAGE_WAIT = 'WAIT_CONTROL'
    STAGE_TAKEOFF = 'TAKEOFF'
    STAGE_ALIGN = 'ALIGN_YAW'
    STAGE_APPROACH = 'APPROACH'
    STAGE_LOAD_SETTLE = 'LOAD_SETTLE'
    STAGE_PUSH = 'PUSH_RAMP'
    STAGE_RELEASE = 'RELEASE_OBSERVE'
    STAGE_COMPLETE = 'COMPLETE'
    STAGE_FAILED = 'FAILED'

    def __init__(self):
        if os.environ.get('HNUTER_IEBC_CUBE_SIM', '0') != '1':
            raise RuntimeError(
                'Refusing to start: set HNUTER_IEBC_CUBE_SIM=1 only for the '
                'HNUTER Gazebo cube-contact experiment.')

        self.world_name = os.environ.get('HNUTER_GZ_WORLD', self.EXPECTED_WORLD)
        if self.world_name != self.EXPECTED_WORLD:
            raise RuntimeError(
                f'Expected Gazebo world {self.EXPECTED_WORLD!r}, got {self.world_name!r}.')

        super().__init__()

        # Free-flight actuator power is not environment interaction. Keep the
        # reference filter transparent through takeoff and approach, then arm
        # and reset IEBC exactly at measured probe contact.
        self._iebc_requested = bool(self.iebc.enabled)
        self.iebc.enabled = False
        self.iebc.reset()

        qos_command = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_command)

        self.cmd_set_mode = getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176)
        self.cmd_arm_disarm = getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400)
        self._startup_ticks = 0
        self._last_mode_request_s = -math.inf
        self._last_arm_request_s = -math.inf

        self.virtual_force_n = abs(float(os.environ.get('HNUTER_CUBE_FORCE_N', '2.0')))
        # A short positive cube displacement is a physics-based breakaway
        # detector: while the -X load pins the cube against the lower rail
        # stop, sustained +X motion is only possible after aircraft force has
        # exceeded the imposed load. This avoids false releases from Gazebo's
        # very large single-step collision-force spikes.
        self.release_travel_m = max(float(os.environ.get('HNUTER_CUBE_RELEASE_TRAVEL_M', '0.02')), 0.002)
        self.release_hold_s = max(float(os.environ.get('HNUTER_CUBE_RELEASE_HOLD_S', '0.10')), 0.04)
        self.barrier_tolerance_j = max(float(os.environ.get('HNUTER_CUBE_BARRIER_TOL_J', '0.02')), 0.0)
        self.takeoff_height_m = max(float(os.environ.get('HNUTER_CUBE_TAKEOFF_M', '1.10')), 0.3)
        self.takeoff_time_s = max(float(os.environ.get('HNUTER_CUBE_TAKEOFF_TIME_S', '5.0')), 1.0)
        self.approach_distance_m = max(float(os.environ.get('HNUTER_CUBE_APPROACH_M', '1.65')), 0.5)
        self.approach_speed_mps = max(float(os.environ.get('HNUTER_CUBE_APPROACH_MPS', '0.12')), 0.02)
        self.push_speed_mps = max(float(os.environ.get('HNUTER_CUBE_PUSH_MPS', '0.035')), 0.005)
        self.max_push_distance_m = max(float(os.environ.get('HNUTER_CUBE_MAX_PUSH_M', '0.80')), 0.1)
        self.load_settle_s = max(float(os.environ.get('HNUTER_CUBE_LOAD_SETTLE_S', '1.5')), 0.2)
        self.release_observe_s = max(float(os.environ.get('HNUTER_CUBE_OBSERVE_S', '7.0')), 1.0)
        self.max_push_time_s = max(float(os.environ.get('HNUTER_CUBE_MAX_PUSH_TIME_S', '25.0')), 2.0)
        self.yaw_tolerance_rad = math.radians(max(
            float(os.environ.get('HNUTER_CUBE_YAW_TOL_DEG', '3.0')), 0.5))
        self.yaw_hold_s = max(float(os.environ.get('HNUTER_CUBE_YAW_HOLD_S', '1.0')), 0.2)
        self.yaw_timeout_s = max(float(os.environ.get('HNUTER_CUBE_YAW_TIMEOUT_S', '12.0')), 2.0)
        self.yaw_loss_tolerance_rad = math.radians(max(
            float(os.environ.get('HNUTER_CUBE_YAW_LOSS_TOL_DEG', '5.0')), 1.0))
        self.yaw_loss_hold_s = max(
            float(os.environ.get('HNUTER_CUBE_YAW_LOSS_HOLD_S', '0.25')), 0.05)
        self.yaw_command_bias_rad = math.radians(
            float(os.environ.get('HNUTER_CUBE_YAW_CMD_BIAS_DEG', '-3.5')))

        # Gazebo and the base controller's ENU representation share world X.
        # This was confirmed from /world/.../pose/info against PX4 odometry;
        # using ENU Y makes the vehicle pass beside the cube.
        self.interaction_axis_enu = np.array([1.0, 0.0, 0.0], dtype=float)
        # The probe is fixed to body +X and the cube rail lies on Gazebo world
        # +X. This HNUTER SITL model's physical Gazebo yaw was calibrated
        # against the position-controller input: its world yaw follows the
        # controller's ENU yaw value directly (the PX4 bridge handles the
        # internal NED conversion). Keep this model-specific mapping here,
        # isolated from the real-aircraft controller.
        self.desired_world_yaw = 0.0
        self.desired_controller_yaw = wrap_pi(
            self.desired_world_yaw + self.yaw_command_bias_rad)
        self.stage = self.STAGE_WAIT
        self.stage_start_s = 0.0
        self.experiment_origin_enu = None
        self.contact_origin_enu = None
        self.release_target_enu = None
        self.terminal_hold_enu = None
        self.release_vehicle_position_enu = None
        self.release_vehicle_velocity_enu = None
        self.release_cube_x = math.nan
        self.loaded_cube_x = math.nan
        self.release_threshold_since_s = None
        self.yaw_aligned_since_s = None
        self.yaw_loss_since_s = None
        self.virtual_force_active = False
        self.release_event_seen = False
        self.peak_post_release_speed_mps = 0.0
        self.peak_post_release_position_delta_m = 0.0
        self.min_interaction_barrier_j = math.inf

        self._transport_lock = threading.Lock()
        self.contact_filter = ContactForceFilter()
        self.cube_x_m = math.nan
        self.cube_y_m = math.nan
        self.vehicle_gz_position = np.full(3, math.nan)
        self.vehicle_gz_yaw = math.nan
        self.gz_node = GazeboNode()
        self.gz_node.subscribe(Contacts, self.CONTACT_TOPIC, self._contact_callback)
        self.gz_node.subscribe(
            Pose_V, f'/world/{self.world_name}/pose/info', self._pose_callback)
        self._persistent_wrench_pub = self.gz_node.advertise(
            f'/world/{self.world_name}/wrench/persistent', EntityWrench)
        self._clear_wrench_pub = self.gz_node.advertise(
            f'/world/{self.world_name}/wrench/clear', Entity)

        self.csv_path = Path(diagnostic_csv_path('hnuter_iebc_cube_contact'))
        self._csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow([
            'px4_time_s', 'stage', 'vehicle_enu_x_m', 'vehicle_enu_y_m',
            'vehicle_enu_z_m', 'vehicle_enu_vx_mps', 'vehicle_enu_vy_mps',
            'vehicle_enu_vz_mps', 'target_enu_x_m', 'target_enu_y_m',
            'target_z_relative_m', 'vehicle_gz_yaw_deg', 'target_gz_yaw_deg',
            'yaw_error_deg', 'cube_world_x_m', 'contact_force_raw_n',
            'contact_force_filtered_n', 'cube_breakaway_m', 'virtual_force_active',
            'virtual_force_n', 'iebc_active', 'iebc_power_w',
            'iebc_storage_j', 'iebc_energy_j', 'iebc_barrier_j',
            'min_interaction_barrier_j',
            'iebc_accel_nom_mps2', 'iebc_accel_safe_mps2',
        ])
        self._last_csv_flush_s = time.monotonic()

        self.get_logger().warn(
            'GAZEBO-ONLY IEBC cube experiment enabled: this node may Arm and '
            'enter Offboard automatically. Never run it with a real flight controller.')
        self.get_logger().info(
            f'Virtual cube load={self.virtual_force_n:.2f} N; breakaway travel='
            f'{self.release_travel_m:.3f} m; yaw tolerance='
            f'{math.degrees(self.yaw_tolerance_rad):.1f} deg; yaw command bias='
            f'{math.degrees(self.yaw_command_bias_rad):+.1f} deg; yaw loss limit='
            f'{math.degrees(self.yaw_loss_tolerance_rad):.1f} deg for '
            f'{self.yaw_loss_hold_s:.2f} s; CSV={self.csv_path}')

    # Gazebo transport -------------------------------------------------
    @staticmethod
    def _vector_x(vector) -> float:
        return float(getattr(vector, 'x', 0.0))

    def _contact_callback(self, message: Contacts) -> None:
        max_force_x = 0.0
        for contact in message.contact:
            force_body_1 = 0.0
            force_body_2 = 0.0
            for wrench in contact.wrench:
                if wrench.HasField('body_1_wrench'):
                    force_body_1 += self._vector_x(wrench.body_1_wrench.force)
                if wrench.HasField('body_2_wrench'):
                    force_body_2 += self._vector_x(wrench.body_2_wrench.force)
            max_force_x = max(max_force_x, abs(force_body_1), abs(force_body_2))

        with self._transport_lock:
            self.contact_filter.feed(max_force_x)

    def _pose_callback(self, message: Pose_V) -> None:
        cube_x = math.nan
        cube_y = math.nan
        vehicle_position = None
        vehicle_yaw = math.nan
        for pose in message.pose:
            name = str(pose.name)
            if name == self.CUBE_MODEL or name.endswith(f'::{self.CUBE_MODEL}'):
                cube_x = float(pose.position.x)
                cube_y = float(pose.position.y)
            elif name.startswith(self.VEHICLE_MODEL_PREFIX):
                vehicle_position = np.array([
                    float(pose.position.x), float(pose.position.y), float(pose.position.z)])
                q = pose.orientation
                vehicle_yaw = math.atan2(
                    2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
                    1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2))
        if math.isfinite(cube_x):
            with self._transport_lock:
                self.cube_x_m = cube_x
                self.cube_y_m = cube_y
                if vehicle_position is not None:
                    self.vehicle_gz_position = vehicle_position
                    self.vehicle_gz_yaw = vehicle_yaw

    def _set_virtual_force(self) -> None:
        message = EntityWrench()
        message.entity.name = self.CUBE_MODEL
        message.entity.type = Entity.MODEL
        message.wrench.force.x = -self.virtual_force_n
        published = bool(self._persistent_wrench_pub.publish(message))
        self.virtual_force_active = True
        self.get_logger().info(
            f'Applied persistent cube virtual force Fx={-self.virtual_force_n:.2f} N '
            f'(Gazebo publish={published}).')

    def _clear_virtual_force(self) -> None:
        message = Entity()
        message.name = self.CUBE_MODEL
        message.type = Entity.MODEL
        published = bool(self._clear_wrench_pub.publish(message))
        self.virtual_force_active = False
        self.get_logger().warn(
            f'CLEARED cube virtual force (Gazebo publish={published}); observing vehicle response.')

    # PX4 simulation-only authority -----------------------------------
    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        message = VehicleCommand()
        message.command = int(command)
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        message.timestamp = self.timestamp_now_us()
        self.vehicle_command_pub.publish(message)

    def offboard_startup_tick(self):
        self.publish_offboard_control_mode()
        self._startup_ticks += 1
        self._update_hardware_control_gate()

        if self.data_received and self.px4_timestamp > 0:
            if not self._hardware_control_active:
                self._hold_current_position()
            self.publish_px4_trajectory_setpoint()

        if not self.data_received or self._startup_ticks < 30:
            return

        now_s = time.monotonic()
        if not self.is_offboard() and now_s - self._last_mode_request_s >= 1.0:
            self._publish_vehicle_command(self.cmd_set_mode, param1=1.0, param2=6.0)
            self._last_mode_request_s = now_s
            self.get_logger().info('SITL experiment requesting Offboard mode.')

        if not self.armed and now_s - self._last_arm_request_s >= 1.0:
            self._publish_vehicle_command(self.cmd_arm_disarm, param1=1.0)
            self._last_arm_request_s = now_s
            self.get_logger().info('SITL experiment requesting Arm.')

    # Experiment state machine ----------------------------------------
    def _set_stage(self, stage: str, current_time: float) -> None:
        previous = self.stage
        self.stage = stage
        self.stage_start_s = float(current_time)
        if stage in (self.STAGE_COMPLETE, self.STAGE_FAILED):
            self.terminal_hold_enu = self.position.copy()
        self.get_logger().warn(f'Experiment stage: {previous} -> {stage}')

    def _begin_hardware_control(self):
        super()._begin_hardware_control()
        self.experiment_origin_enu = self.position.copy()
        self.contact_origin_enu = None
        self.release_target_enu = None
        self.release_threshold_since_s = None
        self.loaded_cube_x = math.nan
        self.yaw_aligned_since_s = None
        self.yaw_loss_since_s = None
        self._set_stage(self.STAGE_TAKEOFF, self.px4_timestamp / 1_000_000.0 - self.sim_start_time_s)

    def _set_reference(
            self, position_enu: np.ndarray, velocity_enu=None, acceleration_enu=None,
            yaw_enu: float = None) -> None:
        position_enu = np.asarray(position_enu, dtype=float).reshape(3)
        self.target_position = position_enu.copy()
        self.target_position[2] -= self._z0
        self.target_velocity = (np.zeros(3) if velocity_enu is None
                                else np.asarray(velocity_enu, dtype=float).reshape(3))
        self.target_acceleration = (np.zeros(3) if acceleration_enu is None
                                    else np.asarray(acceleration_enu, dtype=float).reshape(3))
        commanded_yaw = (self.desired_controller_yaw if yaw_enu is None
                         else float(yaw_enu))
        self.target_attitude = np.array([0.0, 0.0, commanded_yaw], dtype=float)
        # PX4 position mode publishes manual_des_yaw, not target_attitude[2].
        # Both must be updated or the aircraft preserves its arbitrary startup
        # heading while translating toward the cube.
        self.manual_des_yaw = commanded_yaw
        self.target_attitude_rate = np.zeros(3)

    def _gazebo_yaw_error(self) -> float:
        with self._transport_lock:
            vehicle_yaw = self.vehicle_gz_yaw
        return (wrap_pi(self.desired_world_yaw - vehicle_yaw)
                if math.isfinite(vehicle_yaw) else math.nan)

    def _contact_force(self, dt: float) -> tuple:
        with self._transport_lock:
            filtered = self.contact_filter.update(dt)
            raw = self.contact_filter.raw_n
            cube_x = self.cube_x_m
        return raw, filtered, cube_x

    def update_trajectory(self, current_time: float, dt: float):
        if not self._hardware_control_active or self.experiment_origin_enu is None:
            self._hold_current_position()
            return

        elapsed = max(0.0, current_time - self.stage_start_s)
        raw_force, contact_force, cube_x = self._contact_force(dt)
        origin = self.experiment_origin_enu
        hover_position = origin + np.array([0.0, 0.0, self.takeoff_height_m])

        # A valid force-limit trial must remain a head-on push, not merely be
        # aligned once before approach. Abort if the physical Gazebo yaw leaves
        # the allowed cone long enough to represent real loss of alignment.
        if self.stage in (self.STAGE_APPROACH, self.STAGE_LOAD_SETTLE, self.STAGE_PUSH):
            yaw_error = self._gazebo_yaw_error()
            yaw_lost = (not math.isfinite(yaw_error)
                        or abs(yaw_error) > self.yaw_loss_tolerance_rad)
            if yaw_lost:
                if self.yaw_loss_since_s is None:
                    self.yaw_loss_since_s = current_time
                elif current_time - self.yaw_loss_since_s >= self.yaw_loss_hold_s:
                    if self.virtual_force_active:
                        self._clear_virtual_force()
                    self._set_stage(self.STAGE_FAILED, current_time)
                    self.get_logger().error(
                        'Physical probe yaw alignment was lost during interaction; '
                        f'error={math.degrees(yaw_error):.2f} deg, limit='
                        f'{math.degrees(self.yaw_loss_tolerance_rad):.2f} deg for '
                        f'{self.yaw_loss_hold_s:.2f} s.')
                    self._write_csv(current_time, raw_force, contact_force, cube_x)
                    return
            else:
                self.yaw_loss_since_s = None

        if self.stage in (self.STAGE_LOAD_SETTLE, self.STAGE_PUSH) and self.iebc.enabled:
            barrier_j = float(self.iebc.debug.get('h_i', self.iebc.e_max))
            self.min_interaction_barrier_j = min(self.min_interaction_barrier_j, barrier_j)
            if barrier_j < -self.barrier_tolerance_j:
                self._clear_virtual_force()
                self._set_stage(self.STAGE_FAILED, current_time)
                self.get_logger().error(
                    f'IEBC barrier violated during interaction: h={barrier_j:.3f} J, '
                    f'tolerance={self.barrier_tolerance_j:.3f} J.')
                self._write_csv(current_time, raw_force, contact_force, cube_x)
                return

        if self.stage == self.STAGE_TAKEOFF:
            u, du, ddu = smoothstep01(elapsed / self.takeoff_time_s)
            position = origin + np.array([0.0, 0.0, self.takeoff_height_m * u])
            velocity = np.array([0.0, 0.0, self.takeoff_height_m * du / self.takeoff_time_s])
            acceleration = np.array([0.0, 0.0, self.takeoff_height_m * ddu / self.takeoff_time_s ** 2])
            self._set_reference(position, velocity, acceleration, yaw_enu=self.initial_yaw)
            if elapsed >= self.takeoff_time_s and abs(self.position[2] - hover_position[2]) < 0.20:
                self.yaw_aligned_since_s = None
                self._set_stage(self.STAGE_ALIGN, current_time)

        elif self.stage == self.STAGE_ALIGN:
            self._set_reference(hover_position)
            yaw_error = self._gazebo_yaw_error()
            if math.isfinite(yaw_error) and abs(yaw_error) <= self.yaw_tolerance_rad:
                if self.yaw_aligned_since_s is None:
                    self.yaw_aligned_since_s = current_time
                elif current_time - self.yaw_aligned_since_s >= self.yaw_hold_s:
                    self.yaw_loss_since_s = None
                    self._set_stage(self.STAGE_APPROACH, current_time)
            else:
                self.yaw_aligned_since_s = None

            if elapsed >= self.yaw_timeout_s:
                self._set_stage(self.STAGE_FAILED, current_time)
                self.get_logger().error(
                    'Physical probe yaw did not align with the cube before timeout; '
                    f'error={math.degrees(yaw_error):.2f} deg.')

        elif self.stage == self.STAGE_APPROACH:
            distance = min(self.approach_speed_mps * elapsed, self.approach_distance_m)
            position = hover_position + self.interaction_axis_enu * distance
            velocity = (self.interaction_axis_enu * self.approach_speed_mps
                        if distance < self.approach_distance_m else np.zeros(3))
            self._set_reference(position, velocity)

            if contact_force >= 0.20:
                self.contact_origin_enu = self.position.copy()
                self.iebc.enabled = self._iebc_requested
                self.iebc.always_active = self._iebc_requested
                self.iebc.reset()
                self._set_virtual_force()
                self.contact_filter.filtered_n = 0.0
                self.release_threshold_since_s = None
                self._set_stage(self.STAGE_LOAD_SETTLE, current_time)
            elif distance >= self.approach_distance_m and elapsed > (
                    self.approach_distance_m / self.approach_speed_mps + 3.0):
                self._set_stage(self.STAGE_FAILED, current_time)
                self.get_logger().error('No probe/cube contact detected within the configured approach distance.')

        elif self.stage == self.STAGE_LOAD_SETTLE:
            self._set_reference(self.contact_origin_enu)
            if elapsed >= self.load_settle_s:
                self.loaded_cube_x = cube_x
                self._set_stage(self.STAGE_PUSH, current_time)

        elif self.stage == self.STAGE_PUSH:
            push_distance = min(self.push_speed_mps * elapsed, self.max_push_distance_m)
            nominal = self.contact_origin_enu + self.interaction_axis_enu * push_distance
            velocity = (self.interaction_axis_enu * self.push_speed_mps
                        if push_distance < self.max_push_distance_m else np.zeros(3))
            self._set_reference(nominal, velocity)

            cube_breakaway_m = (cube_x - self.loaded_cube_x if
                                math.isfinite(cube_x) and math.isfinite(self.loaded_cube_x)
                                else -math.inf)
            if cube_breakaway_m >= self.release_travel_m:
                if self.release_threshold_since_s is None:
                    self.release_threshold_since_s = current_time
                elif current_time - self.release_threshold_since_s >= self.release_hold_s:
                    self.release_target_enu = nominal.copy()
                    self.release_vehicle_position_enu = self.position.copy()
                    self.release_vehicle_velocity_enu = self.velocity.copy()
                    self.release_cube_x = cube_x
                    self._clear_virtual_force()
                    # The environment interaction ends at virtual-load release.
                    # Leaving the actuator-wrench proxy active in free flight
                    # incorrectly integrates hover/position-control power as
                    # interaction energy and contaminates the recovery result.
                    self.iebc.enabled = False
                    self.iebc.always_active = False
                    self.iebc.reset()
                    self.release_event_seen = True
                    self._set_stage(self.STAGE_RELEASE, current_time)
            else:
                self.release_threshold_since_s = None

            if elapsed >= self.max_push_time_s or (
                    push_distance >= self.max_push_distance_m and
                    cube_breakaway_m < self.release_travel_m):
                self._clear_virtual_force()
                self._set_stage(self.STAGE_FAILED, current_time)
                self.get_logger().error(
                    f'Cube did not break away against {self.virtual_force_n:.2f} N before the push limit; '
                    f'travel={cube_breakaway_m:.3f} m, filtered contact force={contact_force:.2f} N.')

        elif self.stage == self.STAGE_RELEASE:
            self._set_reference(self.release_target_enu)
            speed = float(np.linalg.norm(self.velocity))
            displacement = float(np.linalg.norm(self.position - self.release_vehicle_position_enu))
            self.peak_post_release_speed_mps = max(self.peak_post_release_speed_mps, speed)
            self.peak_post_release_position_delta_m = max(
                self.peak_post_release_position_delta_m, displacement)
            if elapsed >= self.release_observe_s:
                self._set_stage(self.STAGE_COMPLETE, current_time)
                cube_delta = cube_x - self.release_cube_x if (
                    math.isfinite(cube_x) and math.isfinite(self.release_cube_x)) else math.nan
                self.get_logger().warn(
                    'EXPERIMENT COMPLETE: virtual load was cleared after measured cube breakaway; '
                    f'post-release peak vehicle speed={self.peak_post_release_speed_mps:.3f} m/s, '
                    f'peak vehicle displacement={self.peak_post_release_position_delta_m:.3f} m, '
                    f'cube dx={cube_delta:.3f} m, CSV={self.csv_path}')

        elif self.stage in (self.STAGE_COMPLETE, self.STAGE_FAILED):
            hold = (self.release_target_enu if self.release_target_enu is not None
                    else self.terminal_hold_enu)
            self._set_reference(hold)

        self._write_csv(current_time, raw_force, contact_force, cube_x)

    def _write_csv(self, current_time: float, raw_force: float, filtered_force: float, cube_x: float) -> None:
        debug = self.iebc.debug
        yaw_error = self._gazebo_yaw_error()
        cube_breakaway_m = (cube_x - self.loaded_cube_x if
                            math.isfinite(cube_x) and math.isfinite(self.loaded_cube_x)
                            else math.nan)
        self._csv.writerow([
            f'{current_time:.6f}', self.stage,
            *(f'{value:.6f}' for value in self.position),
            *(f'{value:.6f}' for value in self.velocity),
            *(f'{value:.6f}' for value in self.target_position),
            f'{math.degrees(self.vehicle_gz_yaw):.6f}',
            f'{math.degrees(self.desired_world_yaw):.6f}',
            f'{math.degrees(yaw_error):.6f}',
            f'{cube_x:.6f}', f'{raw_force:.6f}', f'{filtered_force:.6f}',
            f'{cube_breakaway_m:.6f}', int(self.virtual_force_active), f'{self.virtual_force_n:.6f}',
            int(bool(debug.get('active', False))), f"{debug.get('p_hat', 0.0):.6f}",
            f"{debug.get('s_bar', 0.0):.6f}", f"{debug.get('e_i', 0.0):.6f}",
            f"{debug.get('h_i', 0.0):.6f}", f'{self.min_interaction_barrier_j:.6f}',
            f"{debug.get('a_nom_i', 0.0):.6f}",
            f"{debug.get('a_safe_i', 0.0):.6f}",
        ])
        now_s = time.monotonic()
        if now_s - self._last_csv_flush_s >= 1.0:
            self._csv_file.flush()
            self._last_csv_flush_s = now_s

    def print_status(self):
        super().print_status()
        if self.data_received:
            _, filtered_force, cube_x = self._contact_force(0.02)
            self.get_logger().info(
                f'Cube experiment: stage={self.stage} | contact={filtered_force:.2f} N | '
                f'virtual_load={self.virtual_force_active} | cube_x={cube_x:.3f} m | '
                f'yaw={math.degrees(self.vehicle_gz_yaw):+.1f} deg -> '
                f'{math.degrees(self.desired_world_yaw):+.1f} deg | '
                f'release_seen={self.release_event_seen}')

    def destroy_node(self):
        try:
            if self.virtual_force_active:
                self._clear_virtual_force()
        except Exception:
            pass
        try:
            self._csv_file.flush()
            self._csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = HnuterIebcCubeContactExperiment()
    exit_code = 1
    try:
        while rclpy.ok() and controller.stage not in (
                controller.STAGE_COMPLETE, controller.STAGE_FAILED):
            rclpy.spin_once(controller, timeout_sec=0.1)
        exit_code = 0 if controller.stage == controller.STAGE_COMPLETE else 1
    except KeyboardInterrupt:
        controller.get_logger().info('Cube-contact experiment interrupted.')
        exit_code = 130
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
