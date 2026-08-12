/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "HnuterControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>

#include <cstring>

using namespace matrix;
using namespace time_literals;

static Quatf normalizedCanonicalQuaternion(const Quatf &input)
{
	Quatf output{input};
	output.normalize();
	output.canonicalize();
	return output;
}

static Quatf normalizedQuaternion(const Quatf &input)
{
	Quatf output{input};
	output.normalize();
	return output;
}

static float wrapPi(float angle)
{
	return atan2f(sinf(angle), cosf(angle));
}

static Quatf attitudeFromHeadingTilt(float heading, const Vector2f &tilt)
{
	const Quatf q_heading{Eulerf{0.f, 0.f, heading}};
	const Quatf q_tilt{AxisAnglef{Vector3f{tilt(0), tilt(1), 0.f}}};
	return normalizedQuaternion(Quatf{q_heading * q_tilt});
}

static Vector2f tiltForFixedHeading(const Quatf &attitude, float heading)
{
	const Quatf q_heading{Eulerf{0.f, 0.f, heading}};
	const Quatf q_relative = normalizedQuaternion(Quatf{q_heading.inversed() * attitude});
	Quatf q_twist{q_relative(0), 0.f, 0.f, q_relative(3)};

	if (q_twist.norm() < 1e-5f) {
		q_twist = Quatf{};

	} else {
		q_twist.normalize();
	}

	const Quatf q_swing = normalizedCanonicalQuaternion(Quatf{q_relative * q_twist.inversed()});
	const Vector3f swing_vector{AxisAnglef{q_swing}};
	return Vector2f{swing_vector(0), swing_vector(1)};
}

static float headingFromAttitude(const Quatf &attitude, float fallback)
{
	const Quatf q = normalizedQuaternion(attitude);

	if (fabsf(q(0)) + fabsf(q(3)) < 1e-5f) {
		return fallback;
	}

	return wrapPi(2.f * atan2f(q(3), q(0)));
}

HnuterControl::HnuterControl(ModuleParams *parent) :
	ModuleParams(parent)
{
}

float HnuterControl::forceToNormalizedThrust(float force, float max_force)
{
	return math::constrain(force / math::max(max_force, 1.f), 0.f, 1.f);
}

void HnuterControl::constrainXY(Vector3f &vector, float limit)
{
	const float constrained_limit = math::max(limit, 0.f);
	const float xy_norm = vector.xy().norm();

	if (xy_norm > constrained_limit && xy_norm > FLT_EPSILON) {
		const float scale = constrained_limit / xy_norm;
		vector(0) *= scale;
		vector(1) *= scale;
	}
}

float HnuterControl::applyDeadband(float input, float deadband)
{
	const float constrained_deadband = math::constrain(deadband, 0.f, 0.9f);
	const float magnitude = fabsf(input);

	if (magnitude <= constrained_deadband) {
		return 0.f;
	}

	return math::signNoZero(input) * (magnitude - constrained_deadband) / (1.f - constrained_deadband);
}

void HnuterControl::reset()
{
	_velocity_integral.setZero();
	_integral_e_R.setZero();
	_xy_lock_initialized = false;
	_takeoff_ramp_started = false;
	_manual_altitude_initialized = false;
	_rc_attitude_initialized = false;
	_rc_level_return_active = false;
	_rc_level_switch_previous = false;
	_rc_roll_input_previous = false;
	_rc_pitch_input_previous = false;
	_rc_yaw_input_previous = false;
	_prev_armed = false;
	_armed_time = 0;
	_manual_altitude_sp = 0.f;
	_rc_yaw_sp = 0.f;
	_rc_governor_scale = 1.f;
	_rc_tilt_sp.setZero();
	_rc_attitude_q_sp = Quatf{};
}

bool HnuterControl::update(const vehicle_angular_velocity_s &angular_velocity,
			   const vehicle_control_mode_s &vehicle_control_mode, bool landed, bool maybe_landed,
			   RateControl &rate_control, AlphaFilter<float> &yaw_torque_filter, float dt,
			   const matrix::Vector3f &rates, Output &output)
{
	// Topic timestamps are publication times in the HRT clock domain. Use the
	// current HRT time for freshness and elapsed-time checks: timestamp_sample
	// can legitimately precede a newly published manual/control setpoint and
	// must only be propagated as the output sample timestamp.
	const hrt_abstime now = hrt_absolute_time();

	vehicle_odometry_s odom{};
	vehicle_attitude_s att{};
	trajectory_setpoint_s traj_sp{};
	vehicle_attitude_setpoint_s att_sp{};
	vehicle_local_position_s local_pos{};
	manual_control_setpoint_s manual_sp{};

	const bool have_odom = _vehicle_odometry_sub.copy(&odom);
	const bool have_att = _vehicle_attitude_sub.copy(&att);
	const bool have_traj = _trajectory_setpoint_sub.copy(&traj_sp);
	const bool have_att_sp = _vehicle_attitude_setpoint_sub.copy(&att_sp);
	const bool have_local_pos = _vehicle_local_position_sub.copy(&local_pos);
	const bool have_manual_sp = _manual_control_setpoint_sub.copy(&manual_sp);

	const bool manual_attitude_altitude_mode = vehicle_control_mode.flag_control_manual_enabled
			&& vehicle_control_mode.flag_control_attitude_enabled
			&& vehicle_control_mode.flag_control_rates_enabled
			&& !vehicle_control_mode.flag_control_position_enabled
			&& !vehicle_control_mode.flag_control_velocity_enabled
			&& !vehicle_control_mode.flag_control_altitude_enabled
			&& !vehicle_control_mode.flag_control_climb_rate_enabled
			&& !vehicle_control_mode.flag_control_offboard_enabled;

	if (!vehicle_control_mode.flag_armed || !have_odom || !have_att || (!have_traj && !manual_attitude_altitude_mode)) {
		reset();

		output.thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
		output.thrust_setpoint.timestamp = hrt_absolute_time();
		output.torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
		output.torque_setpoint.timestamp = output.thrust_setpoint.timestamp;
		return true;
	}

	if (vehicle_control_mode.flag_armed && !_prev_armed) {
		_armed_time = now;
		_xy_lock_initialized = false;
		_takeoff_ramp_started = false;
		_manual_altitude_initialized = false;
		_velocity_integral.setZero();
		_integral_e_R.setZero();
	}

	if (_armed_time == 0) {
		_armed_time = now;
	}

	_prev_armed = vehicle_control_mode.flag_armed;

	const float mass = math::max(_param_hntr_mass.get(), 0.1f);
	const float gravity = 9.81f;
	const float max_thrust_per_arm = math::max(_param_hntr_max_arm_t.get(), 1.f);
	const float max_tail_thrust = math::max(_param_hntr_max_tail_t.get(), 1.f);
	const float max_front_vertical_thrust = max_thrust_per_arm * 2.f;
	const float l1 = math::max(_param_hntr_l1.get(), 0.01f);
	const float l2 = math::max(_param_hntr_l2.get(), 0.01f);
	const Dcmf R{Quatf(att.q)};

	control_allocator_status_s allocator_status{};
	const bool allocator_status_valid = _control_allocator_status_sub.copy(&allocator_status)
					    && allocator_status.timestamp > 0 && now >= allocator_status.timestamp
					    && (now - allocator_status.timestamp) < 100_ms;
	const float allocator_pitch_residual = allocator_status_valid ? allocator_status.unallocated_torque[1] : 0.f;
	const bool pitch_actuator_limited = allocator_status_valid && fabsf(allocator_pitch_residual) > 1e-3f;
	Vector3f allocator_force_residual_world{};

	if (allocator_status_valid) {
		const Vector3f allocator_force_residual_body{
			allocator_status.unallocated_thrust[0] *max_thrust_per_arm,
			-allocator_status.unallocated_thrust[1] *max_thrust_per_arm,
			-allocator_status.unallocated_thrust[2] *max_front_vertical_thrust
		};
		allocator_force_residual_world = R * allocator_force_residual_body / mass;
	}

	bool position_integrator_blocked = false;
	bool pitch_integrator_blocked = false;
	bool pitch_target_blocked = false;

	const Vector3f pos{odom.position};
	const Vector3f vel{odom.velocity};
	Vector3f measured_acceleration{};

	if (have_local_pos && local_pos.timestamp > 0 && now >= local_pos.timestamp
	    && (now - local_pos.timestamp) < 500_ms
	    && PX4_ISFINITE(local_pos.ax) && PX4_ISFINITE(local_pos.ay) && PX4_ISFINITE(local_pos.az)) {
		measured_acceleration = Vector3f{local_pos.ax, local_pos.ay, local_pos.az};
	}

	Vector3f pos_sp{pos};
	Vector3f vel_sp{};
	Vector3f acc_ff{};
	bool position_sp_valid[3] {false, false, false};
	bool velocity_sp_valid[3] {false, false, false};

	if (!manual_attitude_altitude_mode) {
		for (int i = 0; i < 3; i++) {
			if (PX4_ISFINITE(traj_sp.position[i])) {
				pos_sp(i) = traj_sp.position[i];
				position_sp_valid[i] = true;
			}

			if (PX4_ISFINITE(traj_sp.velocity[i])) {
				vel_sp(i) = traj_sp.velocity[i];
				velocity_sp_valid[i] = true;
			}

			if (PX4_ISFINITE(traj_sp.acceleration[i])) {
				acc_ff(i) = traj_sp.acceleration[i];
			}
		}
	}

	const bool hnuter_translation_control_active = !manual_attitude_altitude_mode
			&& (vehicle_control_mode.flag_control_position_enabled
			    || vehicle_control_mode.flag_control_velocity_enabled
			    || vehicle_control_mode.flag_control_altitude_enabled
			    || vehicle_control_mode.flag_control_climb_rate_enabled);

	if (manual_attitude_altitude_mode) {
		if (!_manual_altitude_initialized) {
			_manual_altitude_sp = pos(2);
			_manual_altitude_initialized = true;
			_velocity_integral.setZero();
		}

		float throttle_stick = 0.f;

		if (have_manual_sp && manual_sp.valid && PX4_ISFINITE(manual_sp.throttle)
		    && (now >= manual_sp.timestamp) && (now - manual_sp.timestamp) < 500_ms) {
			throttle_stick = math::constrain(manual_sp.throttle, -1.f, 1.f);
		}

		const float throttle_deadband = math::constrain(_param_hntr_stab_thr_db.get(), 0.f, 0.8f);

		if (fabsf(throttle_stick) < throttle_deadband) {
			throttle_stick = 0.f;

		} else {
			const float sign = throttle_stick > 0.f ? 1.f : -1.f;
			throttle_stick = sign * (fabsf(throttle_stick) - throttle_deadband) / (1.f - throttle_deadband);
		}

		const float max_climb_rate = math::max(_param_hntr_stab_z_vel.get(), 0.f);
		_manual_altitude_sp -= throttle_stick * max_climb_rate * dt;
		pos_sp = pos;
		pos_sp(2) = _manual_altitude_sp;
		position_sp_valid[2] = true;
		vel_sp.zero();
		acc_ff.zero();
	}

	const float takeoff_tilt_suppress_time_s = math::max(_param_hntr_to_sup_t.get(), 0.f);
	const float takeoff_xy_lock_time_s = math::max(_param_hntr_to_lock_t.get(), takeoff_tilt_suppress_time_s);
	const float takeoff_release_ramp_time_s = math::max(_param_hntr_to_ramp_t.get(), 0.f);
	const float xy_lock_kp_scale = math::constrain(_param_hntr_lock_kp.get(), 0.f, 1.f);
	const float max_acc_xy_default = math::max(_param_hntr_acc_xy.get(), 0.1f);
	const float xy_lock_max_acc_xy = math::max(_param_hntr_lock_acc.get(), 0.1f);
	const float takeoff_tilt_limit_rad = math::radians(math::constrain(_param_hntr_to_tilt.get(), 0.f, 185.f));
	const float xy_lock_tilt_limit_rad = math::radians(math::constrain(_param_hntr_lock_tilt.get(), 0.f, 185.f));
	const float default_tilt_limit_rad = math::radians(math::constrain(_param_hntr_tilt_max.get(), 0.f, 185.f));

	const float time_since_armed_s = (_armed_time != 0) ? math::constrain(((now - _armed_time) * 1e-6f), 0.f, 100.f) : 100.f;
	const bool tilt_suppress_active = time_since_armed_s < takeoff_tilt_suppress_time_s;
	const bool xy_lock_active = (time_since_armed_s >= takeoff_tilt_suppress_time_s)
				    && (time_since_armed_s < takeoff_xy_lock_time_s);
	const float takeoff_release_progress = takeoff_release_ramp_time_s > FLT_EPSILON
					       ? math::constrain((time_since_armed_s - takeoff_xy_lock_time_s) / takeoff_release_ramp_time_s, 0.f, 1.f)
					       : 1.f;
	const bool takeoff_release_ramp_active = time_since_armed_s >= takeoff_xy_lock_time_s
			&& takeoff_release_progress < 1.f;
	const float takeoff_xy_gain_scale = xy_lock_active ? xy_lock_kp_scale
					    : (takeoff_release_ramp_active
					       ? xy_lock_kp_scale + (1.f - xy_lock_kp_scale) * takeoff_release_progress : 1.f);

	if (takeoff_release_ramp_active && !_takeoff_ramp_started) {
		// Do not carry the locked-phase XY integrator into the newly released
		// position target. The normal controller rebuilds it smoothly afterwards.
		_velocity_integral(0) = 0.f;
		_velocity_integral(1) = 0.f;
		_takeoff_ramp_started = true;
	}

	if (xy_lock_active) {
		if (!_xy_lock_initialized) {
			_xy_lock_position = pos.xy();
			_xy_lock_initialized = true;
		}

		pos_sp(0) = _xy_lock_position(0);
		pos_sp(1) = _xy_lock_position(1);
		position_sp_valid[0] = true;
		position_sp_valid[1] = true;
	}

	Vector3f pos_error{};

	for (int i = 0; i < 3; i++) {
		if (position_sp_valid[i]) {
			pos_error(i) = pos_sp(i) - pos(i);
			velocity_sp_valid[i] = true;
		}
	}

	SquareMatrix<float, 3> position_p;
	position_p.setZero();
	position_p(0, 0) = manual_attitude_altitude_mode ? 0.f : _param_hntr_pos_p_xy.get();
	position_p(1, 1) = manual_attitude_altitude_mode ? 0.f : _param_hntr_pos_p_xy.get();
	position_p(2, 2) = manual_attitude_altitude_mode ? _param_hntr_stab_z_p.get() : _param_hntr_pos_p_z.get();

	position_p(0, 0) *= takeoff_xy_gain_scale;
	position_p(1, 1) *= takeoff_xy_gain_scale;

	vel_sp += position_p * pos_error;
	constrainXY(vel_sp, manual_attitude_altitude_mode ? 0.f : math::max(_param_hntr_vel_xy.get(), 0.f));
	const float max_velocity_up = manual_attitude_altitude_mode ? math::max(_param_hntr_stab_z_vel.get(), 0.f) :
				      math::max(_param_hntr_vel_up.get(), 0.f);
	const float max_velocity_down = manual_attitude_altitude_mode ? math::max(_param_hntr_stab_z_vel.get(), 0.f) :
					math::max(_param_hntr_vel_dn.get(), 0.f);
	vel_sp(2) = math::constrain(vel_sp(2), -max_velocity_up, max_velocity_down);

	Vector3f vel_error{};

	for (int i = 0; i < 3; i++) {
		if (velocity_sp_valid[i]) {
			vel_error(i) = vel_sp(i) - vel(i);

		} else {
			_velocity_integral(i) = 0.f;
		}
	}

	SquareMatrix<float, 3> velocity_p;
	velocity_p.setZero();
	velocity_p(0, 0) = manual_attitude_altitude_mode ? 0.f : _param_hntr_vel_p_xy.get();
	velocity_p(1, 1) = manual_attitude_altitude_mode ? 0.f : _param_hntr_vel_p_xy.get();
	velocity_p(2, 2) = manual_attitude_altitude_mode ? _param_hntr_stab_z_d.get() : _param_hntr_vel_p_z.get();

	const Vector3f velocity_d{
		manual_attitude_altitude_mode ? 0.f : _param_hntr_vel_d_xy.get(),
		manual_attitude_altitude_mode ? 0.f : _param_hntr_vel_d_xy.get(),
		_param_hntr_vel_d_z.get()
	};
	const Vector3f acc_unsaturated = acc_ff + velocity_p * vel_error + _velocity_integral
					 - velocity_d.emult(measured_acceleration);
	Vector3f acc_des = acc_unsaturated;

	const float takeoff_max_acc_xy = xy_lock_active ? xy_lock_max_acc_xy
					 : (takeoff_release_ramp_active
					    ? xy_lock_max_acc_xy + (max_acc_xy_default - xy_lock_max_acc_xy) * takeoff_release_progress
					    : max_acc_xy_default);
	const float max_acc_xy = manual_attitude_altitude_mode ? 0.f : takeoff_max_acc_xy;
	constrainXY(acc_des, max_acc_xy);
	const float requested_max_acc_z = manual_attitude_altitude_mode ? math::max(_param_hntr_stab_acc_z.get(), 0.1f) :
					  math::max(_param_hntr_acc_z.get(), 0.1f);
	const float physical_max_acc_up = math::max(max_front_vertical_thrust / mass - gravity, 0.1f);
	const float max_acc_up = math::min(requested_max_acc_z, physical_max_acc_up);
	const float max_acc_down = math::min(requested_max_acc_z, gravity);
	acc_des(2) = math::constrain(acc_des(2), -max_acc_up, max_acc_down);

	if (landed || maybe_landed) {
		_velocity_integral.setZero();

	} else {
		const Vector3f velocity_i{
			manual_attitude_altitude_mode ? 0.f : _param_hntr_vel_i_xy.get(),
			manual_attitude_altitude_mode ? 0.f : _param_hntr_vel_i_xy.get(),
			manual_attitude_altitude_mode ? _param_hntr_stab_z_i.get() : _param_hntr_vel_i_z.get()
		};
		const Vector3f saturation_residual = acc_unsaturated - acc_des;

		for (int i = 0; i < 3; i++) {
			const bool drives_further_into_saturation = saturation_residual(i) * vel_error(i) > 0.f;
			const float integral_step = velocity_i(i) * vel_error(i) * dt;
			const bool drives_further_into_allocator_limit = allocator_status_valid
					&& allocator_force_residual_world(i) * integral_step > 0.f;

			if (velocity_sp_valid[i] && !drives_further_into_saturation
			    && !drives_further_into_allocator_limit) {
				_velocity_integral(i) += integral_step;

			} else if (velocity_sp_valid[i] && drives_further_into_allocator_limit) {
				position_integrator_blocked = true;
			}
		}

		constrainXY(_velocity_integral, math::max(_param_hntr_vel_ilim_xy.get(), 0.f));
		const float velocity_integral_z_limit = math::max(_param_hntr_vel_ilim_z.get(), 0.f);
		_velocity_integral(2) = math::constrain(_velocity_integral(2), -velocity_integral_z_limit,
							velocity_integral_z_limit);
	}

	const Vector3f gravity_vec{0.f, 0.f, gravity};
	const Vector3f f_world = mass * (acc_des - gravity_vec);
	Vector3f f_body = R.transpose() * f_world;

	if (manual_attitude_altitude_mode || tilt_suppress_active) {
		f_body(0) = 0.f;
		f_body(1) = 0.f;
	}

	const bool takeoff_force_limit_active = tilt_suppress_active || xy_lock_active || takeoff_release_ramp_active;
	const float takeoff_force_limit = tilt_suppress_active ? takeoff_tilt_limit_rad
					  : (xy_lock_active ? xy_lock_tilt_limit_rad
					     : xy_lock_tilt_limit_rad + (default_tilt_limit_rad - xy_lock_tilt_limit_rad)
					     * takeoff_release_progress);

	if (takeoff_force_limit_active && takeoff_force_limit < math::radians(89.f)) {
		const float fz_abs = fabsf(f_body(2));

		if (fz_abs > 1e-3f) {
			const float max_xy = fz_abs * tanf(takeoff_force_limit);
			const float fxy_norm = f_body.xy().norm();

			if (fxy_norm > max_xy && fxy_norm > 1e-5f) {
				f_body(0) *= max_xy / fxy_norm;
				f_body(1) *= max_xy / fxy_norm;
			}

		} else {
			f_body(0) = 0.f;
			f_body(1) = 0.f;
		}

	} else if (!takeoff_force_limit_active && default_tilt_limit_rad < math::radians(89.f)) {
		// HNTR_TILT_MAX describes the primary (pitch/forward) tilt. Applying
		// the same cone to body Y incorrectly removes the lateral force needed
		// to hold position at large roll angles.
		const float max_x = fabsf(f_body(2)) * tanf(default_tilt_limit_rad);
		f_body(0) = math::constrain(f_body(0), -max_x, max_x);
	}

	Dcmf R_des{};
	bool r_des_valid = false;
	float yaw_rate_sp = 0.f;
	Vector3f target_attitude_rate{};
	bool rc_attitude_control_active = false;

	const bool use_attitude_sp = vehicle_control_mode.flag_control_attitude_enabled
				     && have_att_sp
				     && att_sp.timestamp > 0
				     && (now >= att_sp.timestamp)
				     && (now - att_sp.timestamp) < 500_ms;

	if (use_attitude_sp && !hnuter_translation_control_active) {
		const Quatf q_d{att_sp.q_d};

		if (q_d.isAllFinite()) {
			R_des = Dcmf{q_d};
			r_des_valid = true;
		}

		if (PX4_ISFINITE(att_sp.yaw_sp_move_rate)) {
			yaw_rate_sp = att_sp.yaw_sp_move_rate;
		}
	}

	if (!r_des_valid) {
		const Eulerf euler_cur{R};
		float yaw_sp = euler_cur.psi();

		if (have_traj && PX4_ISFINITE(traj_sp.yaw)) {
			yaw_sp = traj_sp.yaw;

		} else if (use_attitude_sp) {
			const Quatf q_d{att_sp.q_d};

			if (q_d.isAllFinite()) {
				yaw_sp = Eulerf{Dcmf{q_d}}.psi();
			}
		}

		Vector2f attitude_sp_rp{};
		const bool manual_sp_fresh = have_manual_sp && manual_sp.valid && manual_sp.timestamp > 0
					     && now >= manual_sp.timestamp && (now - manual_sp.timestamp) < 500_ms;
		rc_attitude_control_active = _param_hntr_rc_att_en.get()
					     && hnuter_translation_control_active
					     && vehicle_control_mode.flag_control_manual_enabled
					     && manual_sp_fresh;

		if (rc_attitude_control_active) {
			const Quatf q_cur = normalizedQuaternion(Quatf{att.q});

			if (!_rc_attitude_initialized) {
				// Represent the target as heading plus a two-axis swing vector. This
				// stays regular through +/-90 degree Pitch while keeping AUX1/AUX2
				// independent from the held world-heading command.
				_rc_yaw_sp = headingFromAttitude(q_cur, euler_cur.psi());
				_rc_tilt_sp = tiltForFixedHeading(q_cur, _rc_yaw_sp);
				_rc_attitude_q_sp = attitudeFromHeadingTilt(_rc_yaw_sp, _rc_tilt_sp);
				_rc_attitude_initialized = true;
				_rc_level_return_active = true;
				_rc_level_switch_previous = PX4_ISFINITE(manual_sp.aux3) && manual_sp.aux3 > 0.5f;
				_rc_roll_input_previous = false;
				_rc_pitch_input_previous = false;
				_rc_yaw_input_previous = false;
			}

			const float deadband = _param_hntr_rc_db.get();
			const float roll_input = PX4_ISFINITE(manual_sp.aux1) ? applyDeadband(manual_sp.aux1, deadband) : 0.f;
			const float pitch_input = PX4_ISFINITE(manual_sp.aux2) ? applyDeadband(manual_sp.aux2, deadband) : 0.f;
			const float yaw_input = PX4_ISFINITE(manual_sp.yaw) ? applyDeadband(manual_sp.yaw, deadband) : 0.f;
			const bool level_switch = PX4_ISFINITE(manual_sp.aux3) && manual_sp.aux3 > 0.5f;

			if (level_switch && !_rc_level_switch_previous) {
				_rc_level_return_active = true;
				_rc_yaw_sp = headingFromAttitude(q_cur, _rc_yaw_sp);
			}

			_rc_level_switch_previous = level_switch;
			const bool roll_input_active = fabsf(roll_input) > FLT_EPSILON;
			const bool pitch_input_active = fabsf(pitch_input) > FLT_EPSILON;
			const bool yaw_input_active = fabsf(yaw_input) > FLT_EPSILON;
			const bool attitude_input_active = roll_input_active || pitch_input_active;

			if (attitude_input_active) {
				_rc_level_return_active = false;
			}

			// Latch only the released swing component. The measured Yaw error and
			// the other AUX target are deliberately not copied into the setpoint.
			if ((!roll_input_active && _rc_roll_input_previous)
			    || (!pitch_input_active && _rc_pitch_input_previous)) {
				const Vector2f measured_tilt = tiltForFixedHeading(q_cur, _rc_yaw_sp);

				if (!roll_input_active && _rc_roll_input_previous) {
					_rc_tilt_sp(0) = measured_tilt(0);
				}

				if (!pitch_input_active && _rc_pitch_input_previous) {
					_rc_tilt_sp(1) = measured_tilt(1);
				}

				_rc_attitude_q_sp = attitudeFromHeadingTilt(_rc_yaw_sp, _rc_tilt_sp);
			}

			const Quatf previous_target = _rc_attitude_q_sp;
			Vector2f requested_tilt_delta{};
			float requested_yaw_delta = 0.f;
			bool level_return_would_finish = false;

			if (_rc_level_return_active) {
				const float level_rate = math::radians(math::max(_param_hntr_rc_lvl_r.get(), 0.f));
				const float tilt_angle = _rc_tilt_sp.norm();

				if (tilt_angle < 1e-4f || level_rate <= FLT_EPSILON) {
					_rc_tilt_sp.setZero();
					_rc_attitude_q_sp = attitudeFromHeadingTilt(_rc_yaw_sp, _rc_tilt_sp);
					_rc_level_return_active = false;

				} else {
					const float level_step = math::min(level_rate * dt, tilt_angle);
					requested_tilt_delta = -_rc_tilt_sp * (level_step / tilt_angle);
					level_return_would_finish = level_step >= tilt_angle - 1e-5f;
				}

			} else {
				requested_tilt_delta(0) = roll_input_active
							  ? roll_input * math::radians(math::max(_param_hntr_rc_rate_r.get(), 0.f)) * dt : 0.f;
				requested_tilt_delta(1) = pitch_input_active
							  ? pitch_input * math::radians(math::max(_param_hntr_rc_rate_p.get(), 0.f)) * dt : 0.f;
				requested_yaw_delta = yaw_input_active
						      ? yaw_input * math::radians(math::max(_param_hntr_rc_rate_y.get(), 0.f)) * dt : 0.f;

				// HNTR_RC_ANG_MAX remains an AUX1-only swing guard. Pitch is not
				// given an artificial absolute angle limit.
				const float roll_limit = math::radians(math::constrain(_param_hntr_rc_ang_max.get(), 0.f, 180.f));
				const float next_roll = math::constrain(_rc_tilt_sp(0) + requested_tilt_delta(0),
									-roll_limit, roll_limit);
				requested_tilt_delta(0) = next_roll - _rc_tilt_sp(0);

				// When the previous-cycle allocator residual says Motor5 could not
				// realize Pitch, reject only a Pitch target step that increases the
				// target-to-aircraft attitude error. Roll and held heading stay active.
				if (pitch_actuator_limited && fabsf(requested_tilt_delta(1)) > FLT_EPSILON) {
					const Quatf target_without_pitch = attitudeFromHeadingTilt(
							wrapPi(_rc_yaw_sp + requested_yaw_delta),
							Vector2f{_rc_tilt_sp(0) + requested_tilt_delta(0), _rc_tilt_sp(1)});
					const Quatf target_with_pitch = attitudeFromHeadingTilt(
										wrapPi(_rc_yaw_sp + requested_yaw_delta), _rc_tilt_sp + requested_tilt_delta);
					const float error_without_pitch = AxisAnglef{normalizedCanonicalQuaternion(
							Quatf{target_without_pitch.inversed() * q_cur})}.norm();
					const float error_with_pitch = AxisAnglef{normalizedCanonicalQuaternion(
							Quatf{target_with_pitch.inversed() * q_cur})}.norm();

					if (error_with_pitch > error_without_pitch + 1e-4f) {
						requested_tilt_delta(1) = 0.f;
						pitch_target_blocked = true;
					}
				}
			}

			const Vector2f candidate_tilt = _rc_tilt_sp + requested_tilt_delta;
			const float candidate_yaw = wrapPi(_rc_yaw_sp + requested_yaw_delta);
			const Quatf full_candidate = attitudeFromHeadingTilt(candidate_yaw, candidate_tilt);
			const float current_error = AxisAnglef{normalizedCanonicalQuaternion(
					Quatf{previous_target.inversed() * q_cur})}.norm();
			const float candidate_error = AxisAnglef{normalizedCanonicalQuaternion(
					Quatf{full_candidate.inversed() * q_cur})}.norm();
			const float governor_soft = math::radians(math::constrain(_param_hntr_rc_err_s.get(), 0.f, 179.f));
			const float governor_hard = math::radians(math::constrain(_param_hntr_rc_err_h.get(),
						    math::degrees(governor_soft) + 1.f, 180.f));
			_rc_governor_scale = 1.f;

			if (candidate_error > current_error && current_error > governor_soft) {
				_rc_governor_scale = math::constrain((governor_hard - current_error)
								     / math::max(governor_hard - governor_soft, math::radians(1.f)), 0.f, 1.f);
			}

			_rc_tilt_sp += requested_tilt_delta * _rc_governor_scale;
			_rc_yaw_sp = wrapPi(_rc_yaw_sp + requested_yaw_delta * _rc_governor_scale);
			_rc_attitude_q_sp = attitudeFromHeadingTilt(_rc_yaw_sp, _rc_tilt_sp);

			if (dt > FLT_EPSILON) {
				target_attitude_rate = Vector3f{AxisAnglef{normalizedCanonicalQuaternion(
									Quatf{previous_target.inversed() * _rc_attitude_q_sp})}} / dt;
			}

			if (level_return_would_finish && _rc_governor_scale > 0.999f) {
				_rc_tilt_sp.setZero();
				_rc_attitude_q_sp = attitudeFromHeadingTilt(_rc_yaw_sp, _rc_tilt_sp);
				_rc_level_return_active = false;
			}

			R_des = Dcmf{_rc_attitude_q_sp};
			r_des_valid = true;
			_rc_roll_input_previous = roll_input_active;
			_rc_pitch_input_previous = pitch_input_active;
			_rc_yaw_input_previous = yaw_input_active;

		} else {
			_rc_attitude_initialized = false;
			_rc_level_return_active = false;
			_rc_level_switch_previous = false;
			_rc_roll_input_previous = false;
			_rc_pitch_input_previous = false;
			_rc_yaw_input_previous = false;
			_rc_governor_scale = 1.f;

			// Hnuter extension for Offboard: trajectory_setpoint carries
			// translation, and jerk[0]/jerk[1] carry roll/pitch attitude
			// setpoints. This avoids racing mc_pos_control on the shared
			// vehicle_attitude_setpoint topic in position/velocity mode.
			if (hnuter_translation_control_active && vehicle_control_mode.flag_control_offboard_enabled
			    && have_traj && (PX4_ISFINITE(traj_sp.jerk[0]) || PX4_ISFINITE(traj_sp.jerk[1]))) {
				if (PX4_ISFINITE(traj_sp.jerk[0])) {
					attitude_sp_rp(0) = math::constrain(traj_sp.jerk[0], -M_PI_F, M_PI_F);
				}

				if (PX4_ISFINITE(traj_sp.jerk[1])) {
					attitude_sp_rp(1) = math::constrain(traj_sp.jerk[1], -M_PI_F, M_PI_F);
				}

			} else if (use_attitude_sp && hnuter_translation_control_active
				   && vehicle_control_mode.flag_control_offboard_enabled) {
				const Quatf q_d{att_sp.q_d};

				if (q_d.isAllFinite()) {
					const Eulerf euler_sp{Dcmf{q_d}};
					attitude_sp_rp(0) = euler_sp.phi();
					attitude_sp_rp(1) = euler_sp.theta();

					if (!have_traj || !PX4_ISFINITE(traj_sp.yaw)) {
						yaw_sp = euler_sp.psi();
					}
				}

				if (PX4_ISFINITE(att_sp.yaw_sp_move_rate)) {
					yaw_rate_sp = att_sp.yaw_sp_move_rate;
				}
			}
		}

		if (!rc_attitude_control_active) {
			// Translation is generated by thrust-vectoring, so body roll and pitch
			// can be commanded independently without mc_pos_control's tilt target.
			R_des = Dcmf{Eulerf{attitude_sp_rp(0), attitude_sp_rp(1), yaw_sp}};
			r_des_valid = true;
		}

		if (!rc_attitude_control_active && have_traj && PX4_ISFINITE(traj_sp.yawspeed)) {
			yaw_rate_sp = traj_sp.yawspeed;
		}
	}

	if (!rc_attitude_control_active) {
		// yaw_rate_sp is about the world vertical axis. Express it directly in
		// desired-body coordinates without an Euler-angle round trip.
		target_attitude_rate = R_des.transpose() * Vector3f{0.f, 0.f, yaw_rate_sp};
	}

	// Shortest-path SO(3) logarithm retains control authority through 90 degrees
	// and up to 180 degrees. The previous skew/sine error went back to zero at
	// 180 degrees and could command no recovery torque.
	const Quatf q_error = normalizedCanonicalQuaternion(Quatf{Quatf{R_des}.inversed() * Quatf{R}});
	const Vector3f e_R{AxisAnglef{q_error}};

	const Vector3f KR{_param_hntr_att_kr_r.get(), _param_hntr_att_kr_p.get(), _param_hntr_att_kr_y.get()};
	const Vector3f Domega{_param_hntr_att_d_r.get(), _param_hntr_att_d_p.get(), _param_hntr_att_d_y.get()};
	const Vector3f Ki{_param_hntr_att_i_r.get(), _param_hntr_att_i_p.get(), _param_hntr_att_i_y.get()};
	const Vector3f integral_torque_limit{_param_hntr_att_ilim_r.get(), _param_hntr_att_ilim_p.get(),
					     _param_hntr_att_ilim_y.get()};
	const Eulerf euler_des{R_des};
	const Vector3f omega_error = rates - R.transpose() * R_des * target_attitude_rate;
	const Vector3f torque_p = -KR.emult(e_R);
	const Vector3f torque_d = -Domega.emult(omega_error);
	const Vector3f attitude_torque = torque_p + torque_d;
	const Vector3f tau_limit{_param_hntr_tau_r.get(), _param_hntr_tau_p.get(), _param_hntr_tau_y.get()};
	const float pitch_bias_normalized = math::constrain(_param_hntr_pitch_bias.get(), -1.f, 1.f);
	const Vector3f gravity_force_body = R.transpose() * Vector3f{0.f, 0.f, mass * gravity};
	const float cg_x = math::constrain(_param_hntr_cg_x.get(), -1.f, 1.f);
	const float cg_z = math::constrain(_param_hntr_cg_z.get(), -1.f, 1.f);
	Vector3f gravity_torque{};
	// Cancel the attitude-dependent gravity moment about the primary tilt axis.
	// This feed-forward depends on attitude, mass and CG geometry, not collective
	// thrust. At vertical Pitch it naturally falls toward the much smaller CG-Z
	// contribution instead of retaining the level-flight tail command.
	gravity_torque(1) = cg_z * gravity_force_body(0) - cg_x * gravity_force_body(2);
	Vector3f bias_torque{};
	bias_torque(1) = -pitch_bias_normalized * max_tail_thrust * l2;
	Vector3f trim_torque{};
	trim_torque = gravity_torque + bias_torque;

	if (landed || maybe_landed) {
		_integral_e_R.setZero();

	} else {
		for (int i = 0; i < 3; i++) {
			const float integral_gain = math::max(Ki(i), 0.f);
			const float torque_limit = math::max(tau_limit(i), 0.f);
			const float integral_limit = math::max(integral_torque_limit(i), 0.f);

			if (integral_gain <= FLT_EPSILON || integral_limit <= FLT_EPSILON) {
				_integral_e_R(i) = 0.f;
				continue;
			}

			if (i == 1 && pitch_actuator_limited) {
				// The allocator status is one control cycle old. Freeze geometric
				// Pitch I while Motor5 is ramping, dwelling or saturated so a missing
				// actuator command cannot be stored and released later as a torque step.
				pitch_integrator_blocked = true;
				continue;
			}

			const float candidate_integral = math::constrain(_integral_e_R(i) + e_R(i) * dt,
							 -integral_limit / integral_gain,
							 integral_limit / integral_gain);
			const float candidate_torque = attitude_torque(i) + trim_torque(i)
						       - integral_gain * candidate_integral;
			const float integral_torque_step = -integral_gain * (candidate_integral - _integral_e_R(i));
			const bool saturated = fabsf(candidate_torque) > torque_limit;
			const bool drives_further_into_saturation = saturated && candidate_torque * integral_torque_step > 0.f;

			if (!drives_further_into_saturation) {
				_integral_e_R(i) = candidate_integral;
			}
		}
	}

	// Apply the CG/tail trim in physical torque before the final per-axis limit.
	// Previously HNTR_PITCH_BIAS was added after this limit in normalized units,
	// allowing the real pitch command to exceed HNTR_TAU_P.
	const Vector3f torque_i = -Ki.emult(_integral_e_R);
	Vector3f tau_c = attitude_torque + trim_torque + torque_i;
	tau_c(0) = math::constrain(tau_c(0), -tau_limit(0), tau_limit(0));
	tau_c(1) = math::constrain(tau_c(1), -tau_limit(1), tau_limit(1));
	tau_c(2) = math::constrain(tau_c(2), -tau_limit(2), tau_limit(2));

	Vector3f torque_setpoint_normalized{
		math::constrain(tau_c(0) / (max_thrust_per_arm * l1), -1.f, 1.f),
		math::constrain(-tau_c(1) / (max_tail_thrust * l2), -1.f, 1.f),
		math::constrain(tau_c(2) / (max_thrust_per_arm * l1), -1.f, 1.f)
	};

	vehicle_rates_setpoint_s vehicle_rates_setpoint{};
	const bool use_rates_sp = (_param_hntr_ctrl_mode.get() == 0)
				  && vehicle_control_mode.flag_control_attitude_enabled
				  && _vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)
				  && vehicle_rates_setpoint.timestamp > 0
				  && (now >= vehicle_rates_setpoint.timestamp)
				  && (now - vehicle_rates_setpoint.timestamp) < 500_ms;
	const bool geometric_control_active = !(use_rates_sp && !hnuter_translation_control_active
						&& !manual_attitude_altitude_mode);

	if (use_rates_sp && !hnuter_translation_control_active && !manual_attitude_altitude_mode) {
		const Vector3f rates_sp{
			PX4_ISFINITE(vehicle_rates_setpoint.roll) ? vehicle_rates_setpoint.roll : rates(0),
			PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1),
			PX4_ISFINITE(vehicle_rates_setpoint.yaw) ? vehicle_rates_setpoint.yaw : rates(2)
		};

		const Vector3f angular_accel{angular_velocity.xyz_derivative};
		Vector3f rate_torque = rate_control.update(rates, rates_sp, angular_accel, dt, maybe_landed || landed);
		rate_torque(2) = yaw_torque_filter.update(rate_torque(2), dt);

		torque_setpoint_normalized(0) = math::constrain(rate_torque(0), -1.f, 1.f);
		const float normalized_pitch_limit = math::constrain(tau_limit(1) / (max_tail_thrust * l2), 0.f, 1.f);
		const float normalized_pitch_trim = math::constrain(-trim_torque(1) / (max_tail_thrust * l2), -1.f, 1.f);
		torque_setpoint_normalized(1) = math::constrain(-rate_torque(1) + normalized_pitch_trim,
						-normalized_pitch_limit, normalized_pitch_limit);
		torque_setpoint_normalized(2) = math::constrain(rate_torque(2), -1.f, 1.f);

		rate_control.getRateControlStatus(output.rate_ctrl_status);
		output.rate_ctrl_status.timestamp = hrt_absolute_time();
		output.rate_ctrl_status_updated = true;
	}

	const float normalized_vertical_thrust = forceToNormalizedThrust(-f_body(2), max_front_vertical_thrust);

	output.thrust_setpoint.xyz[0] = math::constrain(f_body(0) / max_thrust_per_arm, -1.f, 1.f);
	output.thrust_setpoint.xyz[1] = math::constrain(f_body(1) / max_thrust_per_arm, -1.f, 1.f);
	output.thrust_setpoint.xyz[2] = -normalized_vertical_thrust;
	output.torque_setpoint.xyz[0] = torque_setpoint_normalized(0);
	output.torque_setpoint.xyz[1] = torque_setpoint_normalized(1);
	output.torque_setpoint.xyz[2] = torque_setpoint_normalized(2);

	debug_vect_s hnuter_attitude_setpoint_debug{};
	hnuter_attitude_setpoint_debug.timestamp = hrt_absolute_time();
	strncpy(hnuter_attitude_setpoint_debug.name, "hntr_att", sizeof(hnuter_attitude_setpoint_debug.name));
	hnuter_attitude_setpoint_debug.x = math::degrees(euler_des.phi());
	hnuter_attitude_setpoint_debug.y = math::degrees(euler_des.theta());
	hnuter_attitude_setpoint_debug.z = math::degrees(euler_des.psi());
	_hnuter_attitude_setpoint_debug_pub.publish(hnuter_attitude_setpoint_debug);

	hnuter_control_status_s hnuter_status{};
	hnuter_status.timestamp = hnuter_attitude_setpoint_debug.timestamp;
	hnuter_status.active = geometric_control_active;
	const Quatf q_des{R_des};

	for (int i = 0; i < 4; i++) {
		hnuter_status.attitude_setpoint_q[i] = q_des(i);
	}

	for (int i = 0; i < 3; i++) {
		hnuter_status.attitude_error[i] = e_R(i);
		hnuter_status.rate_setpoint[i] = target_attitude_rate(i);
		hnuter_status.torque_p[i] = torque_p(i);
		hnuter_status.torque_d[i] = torque_d(i);
		hnuter_status.torque_i[i] = torque_i(i);
		hnuter_status.torque_gravity[i] = gravity_torque(i);
		hnuter_status.torque_bias[i] = bias_torque(i);
		hnuter_status.torque_trim[i] = trim_torque(i);
		hnuter_status.torque_command[i] = tau_c(i);
		hnuter_status.force_body[i] = f_body(i);
	}

	hnuter_status.attitude_error_angle = e_R.norm();
	hnuter_status.governor_scale = _rc_governor_scale;
	hnuter_status.allocator_pitch_residual = allocator_pitch_residual;
	hnuter_status.allocator_force_residual_world[0] = allocator_force_residual_world(0);
	hnuter_status.allocator_force_residual_world[1] = allocator_force_residual_world(1);
	hnuter_status.allocator_force_residual_world[2] = allocator_force_residual_world(2);
	hnuter_status.rc_tilt_setpoint[0] = _rc_tilt_sp(0);
	hnuter_status.rc_tilt_setpoint[1] = _rc_tilt_sp(1);
	hnuter_status.rc_yaw_setpoint = _rc_yaw_sp;
	hnuter_status.pitch_integrator_blocked = pitch_integrator_blocked;
	hnuter_status.position_integrator_blocked = position_integrator_blocked;
	hnuter_status.pitch_target_blocked = pitch_target_blocked;
	_hnuter_control_status_pub.publish(hnuter_status);

	output.thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
	output.thrust_setpoint.timestamp = hnuter_attitude_setpoint_debug.timestamp;
	output.torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
	output.torque_setpoint.timestamp = output.thrust_setpoint.timestamp;

	return true;
}
