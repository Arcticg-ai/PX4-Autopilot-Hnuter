/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "HnuterControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;

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

float HnuterControl::slewToZero(float value, float rate_limit, float dt, float &applied_rate)
{
	const float max_step = math::max(rate_limit, 0.f) * dt;
	const float step = math::constrain(-value, -max_step, max_step);
	applied_rate = dt > FLT_EPSILON ? step / dt : 0.f;
	return value + step;
}

void HnuterControl::reset()
{
	_velocity_integral.setZero();
	_integral_e_R.setZero();
	_xy_lock_initialized = false;
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
	_rc_attitude_sp.setZero();
}

bool HnuterControl::update(const vehicle_angular_velocity_s &angular_velocity,
			   const vehicle_control_mode_s &vehicle_control_mode, bool landed, bool maybe_landed,
			   RateControl &rate_control, AlphaFilter<float> &yaw_torque_filter, float dt,
			   const matrix::Vector3f &rates, Output &output)
{
	const hrt_abstime now = angular_velocity.timestamp_sample;

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

	if (xy_lock_active) {
		position_p(0, 0) *= xy_lock_kp_scale;
		position_p(1, 1) *= xy_lock_kp_scale;
	}

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

	const float max_acc_xy = manual_attitude_altitude_mode ? 0.f : (xy_lock_active ? xy_lock_max_acc_xy : max_acc_xy_default);
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

			if (velocity_sp_valid[i] && !drives_further_into_saturation) {
				_velocity_integral(i) += velocity_i(i) * vel_error(i) * dt;
			}
		}

		constrainXY(_velocity_integral, math::max(_param_hntr_vel_ilim_xy.get(), 0.f));
		const float velocity_integral_z_limit = math::max(_param_hntr_vel_ilim_z.get(), 0.f);
		_velocity_integral(2) = math::constrain(_velocity_integral(2), -velocity_integral_z_limit,
					velocity_integral_z_limit);
	}

	const Dcmf R{Quatf(att.q)};
	const Vector3f gravity_vec{0.f, 0.f, gravity};
	const Vector3f f_world = mass * (acc_des - gravity_vec);
	Vector3f f_body = R.transpose() * f_world;

	if (manual_attitude_altitude_mode || tilt_suppress_active) {
		f_body(0) = 0.f;
		f_body(1) = 0.f;
	}

	const float tilt_limit = tilt_suppress_active ? takeoff_tilt_limit_rad :
				 (xy_lock_active ? xy_lock_tilt_limit_rad : default_tilt_limit_rad);

	if (tilt_limit < math::radians(89.f)) {
		const float fz_abs = fabsf(f_body(2));

		if (fz_abs > 1e-3f) {
			const float max_xy = fz_abs * tanf(tilt_limit);
			const float fxy_norm = f_body.xy().norm();

			if (fxy_norm > max_xy && fxy_norm > 1e-5f) {
				f_body(0) *= max_xy / fxy_norm;
				f_body(1) *= max_xy / fxy_norm;
			}

		} else {
			f_body(0) = 0.f;
			f_body(1) = 0.f;
		}
	}

	Dcmf R_des{};
	bool r_des_valid = false;
	float yaw_rate_sp = 0.f;
	Vector2f rc_attitude_rate_sp{};

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
		const bool rc_attitude_control_active = _param_hntr_rc_att_en.get()
						&& hnuter_translation_control_active
						&& vehicle_control_mode.flag_control_manual_enabled
						&& manual_sp_fresh;

		if (rc_attitude_control_active) {
			if (!_rc_attitude_initialized) {
				// Enter position mode at the measured attitude, then use the same
				// bounded slew as AUX3 to establish the level-flight target.
				_rc_attitude_sp = Vector2f{euler_cur.phi(), euler_cur.theta()};
				_rc_yaw_sp = euler_cur.psi();
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
			}

			_rc_level_switch_previous = level_switch;
			const bool roll_input_active = fabsf(roll_input) > FLT_EPSILON;
			const bool pitch_input_active = fabsf(pitch_input) > FLT_EPSILON;
			const bool yaw_input_active = fabsf(yaw_input) > FLT_EPSILON;
			const bool attitude_input_active = roll_input_active || pitch_input_active;

			if (attitude_input_active) {
				_rc_level_return_active = false;
			}

			// AUX1/AUX2 are rate commands. Latch the angle actually reached on
			// release instead of continuing toward an integrated target that may be
			// far ahead of a slow tilt mechanism.
			if (!roll_input_active && _rc_roll_input_previous) {
				_rc_attitude_sp(0) = euler_cur.phi();
			}

			if (!pitch_input_active && _rc_pitch_input_previous) {
				_rc_attitude_sp(1) = euler_cur.theta();
			}

			if (!yaw_input_active && _rc_yaw_input_previous) {
				_rc_yaw_sp = euler_cur.psi();
			}

			if (_rc_level_return_active) {
				const float level_rate = math::radians(math::max(_param_hntr_rc_lvl_r.get(), 0.f));
				_rc_attitude_sp(0) = slewToZero(_rc_attitude_sp(0), level_rate, dt, rc_attitude_rate_sp(0));
				_rc_attitude_sp(1) = slewToZero(_rc_attitude_sp(1), level_rate, dt, rc_attitude_rate_sp(1));

				if (fabsf(_rc_attitude_sp(0)) < 1e-4f && fabsf(_rc_attitude_sp(1)) < 1e-4f) {
					_rc_attitude_sp.setZero();
					rc_attitude_rate_sp.setZero();
					_rc_level_return_active = false;
				}

			} else {
				rc_attitude_rate_sp(0) = roll_input_active
							? roll_input * math::radians(math::max(_param_hntr_rc_rate_r.get(), 0.f)) : 0.f;
				rc_attitude_rate_sp(1) = pitch_input_active
							? pitch_input * math::radians(math::max(_param_hntr_rc_rate_p.get(), 0.f)) : 0.f;
				_rc_attitude_sp += rc_attitude_rate_sp * dt;
			}

			const float attitude_limit = math::radians(math::constrain(_param_hntr_rc_ang_max.get(), 0.f, 180.f));
			_rc_attitude_sp(0) = math::constrain(_rc_attitude_sp(0), -attitude_limit, attitude_limit);
			_rc_attitude_sp(1) = math::constrain(_rc_attitude_sp(1), -attitude_limit, attitude_limit);
			yaw_rate_sp = yaw_input_active
					? yaw_input * math::radians(math::max(_param_hntr_rc_rate_y.get(), 0.f)) : 0.f;
			_rc_yaw_sp = atan2f(sinf(_rc_yaw_sp + yaw_rate_sp * dt), cosf(_rc_yaw_sp + yaw_rate_sp * dt));
			_rc_roll_input_previous = roll_input_active;
			_rc_pitch_input_previous = pitch_input_active;
			_rc_yaw_input_previous = yaw_input_active;
			yaw_sp = _rc_yaw_sp;
			attitude_sp_rp = _rc_attitude_sp;

		} else {
			_rc_attitude_initialized = false;
			_rc_level_return_active = false;
			_rc_level_switch_previous = false;
			_rc_roll_input_previous = false;
			_rc_pitch_input_previous = false;
			_rc_yaw_input_previous = false;
		}

		// Translation is generated by thrust-vectoring, so body roll and pitch can
		// be commanded independently without using mc_pos_control's tilt setpoint.
		R_des = Dcmf{Eulerf{attitude_sp_rp(0), attitude_sp_rp(1), yaw_sp}};
		r_des_valid = true;

		if (!rc_attitude_control_active && have_traj && PX4_ISFINITE(traj_sp.yawspeed)) {
			yaw_rate_sp = traj_sp.yawspeed;
		}
	}

	const Matrix3f e_rm = 0.5f * (R_des.transpose() * R - R.transpose() * R_des);
	const Vector3f e_R{e_rm(2, 1), e_rm(0, 2), e_rm(1, 0)};

	const Vector3f KR{_param_hntr_att_kr_r.get(), _param_hntr_att_kr_p.get(), _param_hntr_att_kr_y.get()};
	const Vector3f Domega{_param_hntr_att_d_r.get(), _param_hntr_att_d_p.get(), _param_hntr_att_d_y.get()};
	const Vector3f Ki{_param_hntr_att_i_r.get(), _param_hntr_att_i_p.get(), _param_hntr_att_i_y.get()};
	const Vector3f integral_torque_limit{_param_hntr_att_ilim_r.get(), _param_hntr_att_ilim_p.get(),
			_param_hntr_att_ilim_y.get()};
	const Eulerf euler_des{R_des};
	const float roll_des = euler_des.phi();
	const float pitch_des = euler_des.theta();
	const Vector3f target_attitude_rate{
		rc_attitude_rate_sp(0) - yaw_rate_sp * sinf(pitch_des),
		rc_attitude_rate_sp(1) * cosf(roll_des) + yaw_rate_sp * sinf(roll_des) * cosf(pitch_des),
		-rc_attitude_rate_sp(1) * sinf(roll_des) + yaw_rate_sp * cosf(roll_des) * cosf(pitch_des)
	};
	const Vector3f omega_error = rates - R.transpose() * R_des * target_attitude_rate;
	const Vector3f attitude_torque = -KR.emult(e_R) - Domega.emult(omega_error);
	const Vector3f tau_limit{_param_hntr_tau_r.get(), _param_hntr_tau_p.get(), _param_hntr_tau_y.get()};

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

			const float candidate_integral = math::constrain(_integral_e_R(i) + e_R(i) * dt,
								   -integral_limit / integral_gain,
								   integral_limit / integral_gain);
			const float candidate_torque = attitude_torque(i) - integral_gain * candidate_integral;
			const float integral_torque_step = -integral_gain * (candidate_integral - _integral_e_R(i));
			const bool saturated = fabsf(candidate_torque) > torque_limit;
			const bool drives_further_into_saturation = saturated && candidate_torque * integral_torque_step > 0.f;

			if (!drives_further_into_saturation) {
				_integral_e_R(i) = candidate_integral;
			}
		}
	}

	Vector3f tau_c = attitude_torque - Ki.emult(_integral_e_R);
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
		torque_setpoint_normalized(1) = math::constrain(-rate_torque(1), -1.f, 1.f);
		torque_setpoint_normalized(2) = math::constrain(rate_torque(2), -1.f, 1.f);

		rate_control.getRateControlStatus(output.rate_ctrl_status);
		output.rate_ctrl_status.timestamp = hrt_absolute_time();
		output.rate_ctrl_status_updated = true;
	}

	torque_setpoint_normalized(1) = math::constrain(torque_setpoint_normalized(1)
				       + math::constrain(_param_hntr_pitch_bias.get(), -1.f, 1.f), -1.f, 1.f);

	const float normalized_vertical_thrust = forceToNormalizedThrust(-f_body(2), max_front_vertical_thrust);

	output.thrust_setpoint.xyz[0] = math::constrain(f_body(0) / max_thrust_per_arm, -1.f, 1.f);
	output.thrust_setpoint.xyz[1] = math::constrain(f_body(1) / max_thrust_per_arm, -1.f, 1.f);
	output.thrust_setpoint.xyz[2] = -normalized_vertical_thrust;
	output.torque_setpoint.xyz[0] = torque_setpoint_normalized(0);
	output.torque_setpoint.xyz[1] = torque_setpoint_normalized(1);
	output.torque_setpoint.xyz[2] = torque_setpoint_normalized(2);

	output.thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
	output.thrust_setpoint.timestamp = hrt_absolute_time();
	output.torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
	output.torque_setpoint.timestamp = output.thrust_setpoint.timestamp;

	return true;
}
