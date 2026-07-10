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

float HnuterControl::forceToNormalizedThrust(float force, float hover_thrust, float hover_force, float max_force)
{
	const float constrained_force = math::constrain(force, 0.f, max_force);
	const float hover = math::constrain(hover_thrust, 0.05f, 0.95f);

	if (constrained_force <= hover_force) {
		return hover * constrained_force / hover_force;
	}

	const float upper_force_range = max_force - hover_force;

	if (upper_force_range <= FLT_EPSILON) {
		return hover;
	}

	return hover + (1.f - hover) * (constrained_force - hover_force) / upper_force_range;
}

void HnuterControl::reset()
{
	_integral_pos_error.setZero();
	_integral_e_R.setZero();
	_xy_lock_initialized = false;
	_prev_armed = false;
	_armed_time = 0;
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

	const bool have_odom = _vehicle_odometry_sub.copy(&odom);
	const bool have_att = _vehicle_attitude_sub.copy(&att);
	const bool have_traj = _trajectory_setpoint_sub.copy(&traj_sp);
	const bool have_att_sp = _vehicle_attitude_setpoint_sub.copy(&att_sp);

	if (!vehicle_control_mode.flag_armed || !have_odom || !have_att || !have_traj) {
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
		_integral_pos_error.setZero();
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

	Vector3f pos_sp{pos};
	Vector3f vel_sp{};
	Vector3f acc_ff{};

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(traj_sp.position[i])) {
			pos_sp(i) = traj_sp.position[i];
		}

		if (PX4_ISFINITE(traj_sp.velocity[i])) {
			vel_sp(i) = traj_sp.velocity[i];
		}

		if (PX4_ISFINITE(traj_sp.acceleration[i])) {
			acc_ff(i) = traj_sp.acceleration[i];
		}
	}

	const bool position_sp_active = PX4_ISFINITE(traj_sp.position[0])
					|| PX4_ISFINITE(traj_sp.position[1])
					|| PX4_ISFINITE(traj_sp.position[2]);

	const float takeoff_tilt_suppress_time_s = math::max(_param_hntr_to_sup_t.get(), 0.f);
	const float takeoff_xy_lock_time_s = math::max(_param_hntr_to_lock_t.get(), takeoff_tilt_suppress_time_s);
	const float xy_lock_kp_scale = math::constrain(_param_hntr_lock_kp.get(), 0.f, 1.f);
	const float max_acc_xy_default = math::max(_param_hntr_acc_xy.get(), 0.1f);
	const float max_acc_z = math::max(_param_hntr_acc_z.get(), max_front_vertical_thrust / mass - gravity);
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
	}

	const Vector3f pos_error = pos_sp - pos;
	const Vector3f vel_error = vel_sp - vel;

	_integral_pos_error += pos_error * dt;
	_integral_pos_error(0) = math::constrain(_integral_pos_error(0), -1.f, 1.f);
	_integral_pos_error(1) = math::constrain(_integral_pos_error(1), -1.f, 1.f);
	_integral_pos_error(2) = math::constrain(_integral_pos_error(2), -2.f, 2.f);

	SquareMatrix<float, 3> Kp;
	Kp.setZero();
	Kp(0, 0) = _param_hntr_xy_p.get();
	Kp(1, 1) = _param_hntr_xy_p.get();
	Kp(2, 2) = _param_hntr_z_p.get();

	if (xy_lock_active) {
		Kp(0, 0) *= xy_lock_kp_scale;
		Kp(1, 1) *= xy_lock_kp_scale;
	}

	SquareMatrix<float, 3> Dp;
	Dp.setZero();
	Dp(0, 0) = _param_hntr_xy_d.get();
	Dp(1, 1) = _param_hntr_xy_d.get();
	Dp(2, 2) = _param_hntr_z_d.get();

	const Vector3f K_pos_I{_param_hntr_xy_i.get(), _param_hntr_xy_i.get(), _param_hntr_z_i.get()};
	Vector3f acc_des = acc_ff + Kp * pos_error + Dp * vel_error + K_pos_I.emult(_integral_pos_error);

	const float max_acc_xy = xy_lock_active ? xy_lock_max_acc_xy : max_acc_xy_default;
	acc_des(0) = math::constrain(acc_des(0), -max_acc_xy, max_acc_xy);
	acc_des(1) = math::constrain(acc_des(1), -max_acc_xy, max_acc_xy);
	acc_des(2) = math::constrain(acc_des(2), -max_acc_z, max_acc_z);

	const Dcmf R{Quatf(att.q)};
	const Vector3f gravity_vec{0.f, 0.f, gravity};
	const Vector3f f_world = mass * (acc_des - gravity_vec);
	Vector3f f_body = R.transpose() * f_world;

	if (tilt_suppress_active) {
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

	const bool use_attitude_sp = vehicle_control_mode.flag_control_attitude_enabled
				     && have_att_sp
				     && att_sp.timestamp > 0
				     && (now >= att_sp.timestamp)
				     && (now - att_sp.timestamp) < 500_ms;

	if (use_attitude_sp && !position_sp_active) {
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

		if (PX4_ISFINITE(traj_sp.yaw)) {
			yaw_sp = traj_sp.yaw;

		} else if (use_attitude_sp) {
			const Quatf q_d{att_sp.q_d};

			if (q_d.isAllFinite()) {
				yaw_sp = Eulerf{Dcmf{q_d}}.psi();
			}
		}

		// In Hnuter position/offboard-position control, horizontal translation is
		// produced by the tilting thrust vector. Keep the body level and use only
		// yaw from PX4's attitude setpoint; otherwise mc_pos_control roll/pitch and
		// Hnuter thrust-vector control fight each other.
		R_des = Dcmf{Eulerf{0.f, 0.f, yaw_sp}};
		r_des_valid = true;

		if (PX4_ISFINITE(traj_sp.yawspeed)) {
			yaw_rate_sp = traj_sp.yawspeed;
		}
	}

	const Matrix3f e_rm = 0.5f * (R_des.transpose() * R - R.transpose() * R_des);
	const Vector3f e_R{e_rm(2, 1), e_rm(0, 2), e_rm(1, 0)};

	_integral_e_R += e_R * dt;
	_integral_e_R(0) = math::constrain(_integral_e_R(0), -1.5f, 1.5f);
	_integral_e_R(1) = math::constrain(_integral_e_R(1), -1.5f, 1.5f);
	_integral_e_R(2) = math::constrain(_integral_e_R(2), -1.5f, 1.5f);

	const Vector3f KR{_param_hntr_att_kr_r.get(), _param_hntr_att_kr_p.get(), _param_hntr_att_kr_y.get()};
	const Vector3f Domega{_param_hntr_att_d_r.get(), _param_hntr_att_d_p.get(), _param_hntr_att_d_y.get()};
	const Vector3f target_attitude_rate{0.f, 0.f, yaw_rate_sp};
	const Vector3f omega_error = rates - R.transpose() * R_des * target_attitude_rate;

	Vector3f tau_c = -KR.emult(e_R) - Domega.emult(omega_error);
	const Vector3f tau_limit{_param_hntr_tau_r.get(), _param_hntr_tau_p.get(), _param_hntr_tau_y.get()};
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

		if (use_rates_sp && !position_sp_active) {
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

	const float hover_thrust = math::constrain(_param_hntr_hov_thr.get(), 0.05f, 0.95f);
	const float hover_force = mass * gravity;
	const float normalized_vertical_thrust = forceToNormalizedThrust(-f_body(2), hover_thrust, hover_force,
			max_front_vertical_thrust);

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
