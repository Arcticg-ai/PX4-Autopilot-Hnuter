/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	_param_ca_airframe = param_find("CA_AIRFRAME");

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	if (_param_ca_airframe != PARAM_INVALID) {
		param_get(_param_ca_airframe, &_ca_airframe);
	}

	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		if (_ca_airframe == 16 && _vehicle_control_mode.flag_control_rates_enabled
		    && (_vehicle_control_mode.flag_control_position_enabled
			|| _vehicle_control_mode.flag_control_offboard_enabled)) {
			if (runHnuterControl(angular_velocity, dt, rates)) {
				perf_end(_loop_perf);
				return;
			}
		}

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				// publish rate setpoint
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// run rate controller
			Vector3f torque_setpoint =
				_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

			// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			// scale setpoints by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		}
	}

	perf_end(_loop_perf);
}

bool MulticopterRateControl::runHnuterControl(const vehicle_angular_velocity_s &angular_velocity, float dt,
		const matrix::Vector3f &rates)
{
	if (_ca_airframe != 16) {
		return false;
	}

	const hrt_abstime now = angular_velocity.timestamp_sample;

	vehicle_odometry_s odom{};
	vehicle_attitude_s att{};
	trajectory_setpoint_s traj_sp{};
	vehicle_attitude_setpoint_s att_sp{};

	const bool have_odom = _vehicle_odometry_sub.copy(&odom);
	const bool have_att = _vehicle_attitude_sub.copy(&att);
	const bool have_traj = _trajectory_setpoint_sub.copy(&traj_sp);
	const bool have_att_sp = _vehicle_attitude_setpoint_sub.copy(&att_sp);

	if (!_vehicle_control_mode.flag_armed || !have_odom || !have_att || !have_traj) {
		_hnuter_integral_pos_error.setZero();
		_hnuter_integral_e_R.setZero();
		_hnuter_xy_lock_initialized = false;
		_hnuter_prev_armed = false;
		_hnuter_prev_landed = _landed;
		_hnuter_armed_time = 0;

		vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
		vehicle_torque_setpoint_s vehicle_torque_setpoint{};

		vehicle_thrust_setpoint.xyz[0] = 0.f;
		vehicle_thrust_setpoint.xyz[1] = 0.f;
		vehicle_thrust_setpoint.xyz[2] = 0.f;
		vehicle_torque_setpoint.xyz[0] = 0.f;
		vehicle_torque_setpoint.xyz[1] = 0.f;
		vehicle_torque_setpoint.xyz[2] = 0.f;

		vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
		vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
		_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

		vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
		vehicle_torque_setpoint.timestamp = vehicle_thrust_setpoint.timestamp;
		_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

		updateActuatorControlsStatus(vehicle_torque_setpoint, dt);
		return true;
	}

	if (_vehicle_control_mode.flag_armed && !_hnuter_prev_armed) {
		_hnuter_armed_time = now;
		_hnuter_xy_lock_initialized = false;
		_hnuter_integral_pos_error.setZero();
		_hnuter_integral_e_R.setZero();
	}

	if (_hnuter_armed_time == 0) {
		_hnuter_armed_time = now;
	}

	_hnuter_prev_armed = _vehicle_control_mode.flag_armed;
	_hnuter_prev_landed = _landed;

	const float mass = 4.5f;
	const float gravity = 9.81f;

	const matrix::Vector3f pos{odom.position};
	const matrix::Vector3f vel{odom.velocity};

	matrix::Vector3f pos_sp{pos};
	matrix::Vector3f vel_sp{};
	matrix::Vector3f acc_ff{};

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

	const float takeoff_tilt_suppress_time_s = 1.f;
	const float takeoff_xy_lock_time_s = 3.f;
	const float xy_lock_kp_scale = 0.8f;
	const float max_acc_xy_default = 20.f;
	const float max_acc_z = 20.f;
	const float xy_lock_max_acc_xy = 3.f;
	const float takeoff_tilt_limit_rad = math::radians(20.f);
	const float xy_lock_tilt_limit_rad = math::radians(30.f);
	const float default_tilt_limit_rad = math::radians(45.f);

	const float time_since_armed_s = (_hnuter_armed_time != 0) ? math::constrain(((now - _hnuter_armed_time) * 1e-6f), 0.f,
					    100.f) : 100.f;

	const bool tilt_suppress_active = time_since_armed_s < takeoff_tilt_suppress_time_s;
	const bool xy_lock_active = (time_since_armed_s >= takeoff_tilt_suppress_time_s) && (time_since_armed_s < takeoff_xy_lock_time_s);

	if (xy_lock_active) {
		if (!_hnuter_xy_lock_initialized) {
			_hnuter_xy_lock_position = pos.xy();
			_hnuter_xy_lock_initialized = true;
		}

		pos_sp(0) = _hnuter_xy_lock_position(0);
		pos_sp(1) = _hnuter_xy_lock_position(1);
	}

	const matrix::Vector3f pos_error = pos_sp - pos;
	const matrix::Vector3f vel_error = vel_sp - vel;

	_hnuter_integral_pos_error += pos_error * dt;
	_hnuter_integral_pos_error(0) = math::constrain(_hnuter_integral_pos_error(0), -1.f, 1.f);
	_hnuter_integral_pos_error(1) = math::constrain(_hnuter_integral_pos_error(1), -1.f, 1.f);
	_hnuter_integral_pos_error(2) = math::constrain(_hnuter_integral_pos_error(2), -2.f, 2.f);

	matrix::SquareMatrix<float, 3> Kp;
	Kp.setZero();
	Kp(0, 0) = 2.5f;
	Kp(1, 1) = 2.5f;
	Kp(2, 2) = 8.f;

	if (xy_lock_active) {
		Kp(0, 0) *= xy_lock_kp_scale;
		Kp(1, 1) *= xy_lock_kp_scale;
	}

	matrix::SquareMatrix<float, 3> Dp;
	Dp.setZero();
	Dp(0, 0) = 1.8f;
	Dp(1, 1) = 1.8f;
	Dp(2, 2) = 4.f;

	const matrix::Vector3f K_pos_I{0.f, 0.f, 3.f};

	matrix::Vector3f acc_des = acc_ff + Kp * pos_error + Dp * vel_error + K_pos_I.emult(_hnuter_integral_pos_error);

	const float max_acc_xy = xy_lock_active ? xy_lock_max_acc_xy : max_acc_xy_default;
	acc_des(0) = math::constrain(acc_des(0), -max_acc_xy, max_acc_xy);
	acc_des(1) = math::constrain(acc_des(1), -max_acc_xy, max_acc_xy);
	acc_des(2) = math::constrain(acc_des(2), -max_acc_z, max_acc_z);

	const Dcmf R{Quatf(att.q)};
	const matrix::Vector3f gravity_vec{0.f, 0.f, gravity};
	const matrix::Vector3f f_world = mass * (acc_des - gravity_vec);
	matrix::Vector3f f_body = R.transpose() * f_world;

	if (tilt_suppress_active) {
		f_body(0) = 0.f;
		f_body(1) = 0.f;
	}

	const float tilt_limit = tilt_suppress_active ? takeoff_tilt_limit_rad : (xy_lock_active ? xy_lock_tilt_limit_rad : default_tilt_limit_rad);
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

	Dcmf R_des{};
	bool r_des_valid = false;
	float yaw_rate_sp = 0.f;

	const bool use_external_attitude_sp = _vehicle_control_mode.flag_control_offboard_enabled
					      && !_vehicle_control_mode.flag_control_position_enabled
					      && !_vehicle_control_mode.flag_control_velocity_enabled
					      && !_vehicle_control_mode.flag_control_altitude_enabled;

	if (have_att_sp && use_external_attitude_sp) {
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
		}

		R_des = Dcmf{Eulerf{0.f, 0.f, yaw_sp}};
		r_des_valid = true;

		if (PX4_ISFINITE(traj_sp.yawspeed)) {
			yaw_rate_sp = traj_sp.yawspeed;
		}
	}

	const matrix::Matrix3f e_rm = 0.5f * (R_des.transpose() * R - R.transpose() * R_des);
	const matrix::Vector3f e_R{e_rm(2, 1), e_rm(0, 2), e_rm(1, 0)};

	_hnuter_integral_e_R += e_R * dt;
	_hnuter_integral_e_R(0) = math::constrain(_hnuter_integral_e_R(0), -1.5f, 1.5f);
	_hnuter_integral_e_R(1) = math::constrain(_hnuter_integral_e_R(1), -1.5f, 1.5f);
	_hnuter_integral_e_R(2) = math::constrain(_hnuter_integral_e_R(2), -1.5f, 1.5f);

	const matrix::Vector3f KR{1.5f, 1.5f, 1.5f};
	const matrix::Vector3f Domega{1.2f, 1.2f, 1.2f};
	const matrix::Vector3f KI{0.f, 0.f, 0.f};

	const matrix::Vector3f target_attitude_rate{0.f, 0.f, yaw_rate_sp};
	const matrix::Vector3f omega_error = rates - R.transpose() * R_des * target_attitude_rate;

	matrix::Vector3f tau_c = -KR.emult(e_R) - KI.emult(_hnuter_integral_e_R) - Domega.emult(omega_error);
	tau_c(2) = math::constrain(tau_c(2), -0.5f, 0.5f);

	vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
	vehicle_torque_setpoint_s vehicle_torque_setpoint{};

	const float max_thrust_per_arm = 85.48f * 2.0f;
	const float max_tail_thrust = 85.48f;
	const float l1 = 0.33f;
	const float l2 = 0.664f;

	vehicle_thrust_setpoint.xyz[0] = f_body(0) / max_thrust_per_arm;
	vehicle_thrust_setpoint.xyz[1] = f_body(1) / max_thrust_per_arm;
	vehicle_thrust_setpoint.xyz[2] = f_body(2) / (mass * gravity * 2.0f);
	vehicle_torque_setpoint.xyz[0] = tau_c(0) / (max_thrust_per_arm * l1);
	vehicle_torque_setpoint.xyz[1] = -tau_c(1) / (max_tail_thrust * l2);
	vehicle_torque_setpoint.xyz[2] = tau_c(2) / (max_thrust_per_arm * l1);

	vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
	vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
	_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

	vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
	vehicle_torque_setpoint.timestamp = vehicle_thrust_setpoint.timestamp;
	_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

	updateActuatorControlsStatus(vehicle_torque_setpoint, dt);
	return true;
}

void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
