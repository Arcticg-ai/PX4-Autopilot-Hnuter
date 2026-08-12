/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
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

/**
 * @file ActuatorEffectivenessHnuter.cpp
 *
 * Actuator effectiveness for Hnuter tiltrotor vehicles
 *
 * @author PX4 Development Team
 */

#include "ActuatorEffectivenessHnuter.hpp"

#include <px4_platform_common/log.h>
#include <mathlib/math/Limits.hpp>

using namespace matrix;

static float thrustToNormalizedMotorControl(float thrust, float motor_constant, float min_velocity, float max_velocity)
{
	if (thrust <= 0.f) {
		return 0.f;
	}

	const float velocity_range = max_velocity - min_velocity;

	if (velocity_range <= 0.001f) {
		return 0.f;
	}

	const float velocity = sqrtf(thrust / motor_constant);
	return math::constrain((velocity - min_velocity) / velocity_range, 0.f, 1.f);
}

static float signedThrustToNormalizedMotorControl(float thrust, float motor_constant, float min_velocity, float max_velocity)
{
	const float sign = (thrust < 0.f) ? -1.f : 1.f;
	return sign * thrustToNormalizedMotorControl(fabsf(thrust), motor_constant, min_velocity, max_velocity);
}

static float normalizedThrustToForce(float normalized_thrust, float max_force)
{
	return math::constrain(normalized_thrust, 0.f, 1.f) * math::max(max_force, 1.f);
}

static float thrustToHoverAnchoredMotorControl(float thrust, float hover_force, float hover_control, float exponent)
{
	if (thrust <= 0.f || hover_force <= FLT_EPSILON) {
		return 0.f;
	}

	const float control = math::constrain(hover_control, 0.05f, 0.95f)
			      * powf(thrust / hover_force, math::constrain(exponent, 0.2f, 1.5f));
	return math::constrain(control, 0.f, 1.f);
}

static float signedForceToNormalizedMotorControl(float thrust, float max_force, float exponent)
{
	const float sign = thrust < 0.f ? -1.f : 1.f;
	const float force_ratio = math::constrain(fabsf(thrust) / math::max(max_force, 1.f), 0.f, 1.f);
	return sign * powf(force_ratio, math::constrain(exponent, 0.2f, 1.5f));
}

static float tiltAngleAbsLimit(const ActuatorEffectivenessTilts &tilts, int tilt_index, float fallback)
{
	if (tilt_index < 0 || tilt_index >= tilts.count()) {
		return fallback;
	}

	const auto &config = tilts.config(tilt_index);
	const float limit = math::max(fabsf(config.min_angle), fabsf(config.max_angle));

	return (limit > math::radians(1.f)) ? limit : fallback;
}

static float tiltAngleToNormalizedServo(float angle, const ActuatorEffectivenessTilts &tilts, int tilt_index,
					float fallback)
{
	if (tilt_index < 0 || tilt_index >= tilts.count()) {
		return math::constrain(angle / fallback, -1.f, 1.f);
	}

	const auto &config = tilts.config(tilt_index);
	const float positive_limit = (config.max_angle > math::radians(1.f)) ? config.max_angle : fallback;
	const float negative_limit = (config.min_angle < -math::radians(1.f)) ? -config.min_angle : fallback;

	if (angle >= 0.f) {
		return math::constrain(angle / positive_limit, 0.f, 1.f);
	}

	return math::constrain(angle / negative_limit, -1.f, 0.f);
}

ActuatorEffectivenessHnuter::ActuatorEffectivenessHnuter(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	  _tilts(this)
{
	_param_sim_gz_ec_min1 = param_find("SIM_GZ_EC_MIN1");
	_param_sim_gz_ec_max1 = param_find("SIM_GZ_EC_MAX1");
	_simulation_motor_model = _param_sim_gz_ec_min1 != PARAM_INVALID && _param_sim_gz_ec_max1 != PARAM_INVALID;
	_param_hntr_mot_hov = param_find("HNTR_MOT_HOV");
	_param_hntr_mot_expo = param_find("HNTR_MOT_EXPO");
	_param_hntr_mass = param_find("HNTR_MASS");
	_param_hntr_max_arm_t = param_find("HNTR_MAX_ARM_T");
	_param_hntr_max_tail_t = param_find("HNTR_MAX_TAIL_T");
	_param_hntr_l1 = param_find("HNTR_L1");
	_param_hntr_l2 = param_find("HNTR_L2");
	_param_hntr_cg_x = param_find("HNTR_CG_X");
	_param_hntr_cg_z = param_find("HNTR_CG_Z");
	_param_hntr_s2_gear = param_find("HNTR_S2_GEAR");
	_param_hntr_roll_sign = param_find("HNTR_ROLL_SIGN");
	_param_hntr_tail_sign = param_find("HNTR_TAIL_SIGN");
	_param_hntr_tail_rev_t = param_find("HNTR_TAIL_REV_T");
	_param_hntr_t_rev_min = param_find("HNTR_T_REV_MIN");
	_param_hntr_t_force_ref = param_find("HNTR_T_FORCE_REF");
	_param_hntr_t_slew_up = param_find("HNTR_T_SLEW_UP");
	_param_hntr_t_slew_dn = param_find("HNTR_T_SLEW_DN");
	_param_hntr_t_rev_slew = param_find("HNTR_T_REV_SLEW");
	_param_hntr_t_rpm_max = param_find("HNTR_T_RPM_MAX");
	_param_pwm_main_min5 = param_find("PWM_MAIN_MIN5");
	_param_pwm_main_max5 = param_find("PWM_MAIN_MAX5");
	_param_hntr_s1_rate = param_find("HNTR_S1_RATE");
	_param_hntr_s2_rate = param_find("HNTR_S2_RATE");
	_param_hntr_to_sup_t = param_find("HNTR_TO_SUP_T");
	_param_hntr_to_lock_t = param_find("HNTR_TO_LOCK_T");
	_param_hntr_to_ramp_t = param_find("HNTR_TO_RAMP_T");
	_param_hntr_to_tilt = param_find("HNTR_TO_TILT");
	_param_hntr_lock_tilt = param_find("HNTR_LOCK_TILT");

	updateParams();
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

void ActuatorEffectivenessHnuter::updateParams()
{
	ModuleParams::updateParams();

	if (_param_sim_gz_ec_min1 != PARAM_INVALID) {
		int32_t min_v = 0;

		if (param_get(_param_sim_gz_ec_min1, &min_v) == 0) {
			_sim_min_velocity = (float)min_v;
		}
	}

	if (_param_sim_gz_ec_max1 != PARAM_INVALID) {
		int32_t max_v = 0;

		if (param_get(_param_sim_gz_ec_max1, &max_v) == 0) {
			_sim_max_velocity = (float)max_v;
		}
	}

	if (_param_hntr_mot_hov != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_mot_hov, &value) == 0) {
			_motor_hover_control = math::constrain(value, 0.05f, 0.95f);
		}
	}

	if (_param_hntr_mot_expo != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_mot_expo, &value) == 0) {
			_motor_thrust_exponent = math::constrain(value, 0.2f, 1.5f);
		}
	}

	if (_param_hntr_mass != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_mass, &value) == 0) {
			_mass = math::max(value, 0.1f);
		}
	}

	if (_param_hntr_max_arm_t != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_max_arm_t, &value) == 0) {
			_max_thrust_per_arm = math::max(value, 1.f);
		}
	}

	if (_param_hntr_max_tail_t != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_max_tail_t, &value) == 0) {
			_max_tail_thrust = math::max(value, 1.f);
		}
	}

	if (_param_hntr_l1 != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_l1, &value) == 0) {
			_l1 = math::max(value, 0.01f);
		}
	}

	if (_param_hntr_l2 != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_l2, &value) == 0) {
			_l2 = math::max(value, 0.01f);
		}
	}

	if (_param_hntr_cg_x != PARAM_INVALID) {
		param_get(_param_hntr_cg_x, &_cg_x);
		_cg_x = math::constrain(_cg_x, -1.f, 1.f);
	}

	if (_param_hntr_cg_z != PARAM_INVALID) {
		param_get(_param_hntr_cg_z, &_cg_z);
		_cg_z = math::constrain(_cg_z, -1.f, 1.f);
	}

	if (_param_hntr_s2_gear != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_s2_gear, &value) == 0) {
			_secondary_servo_gear_ratio = math::constrain(value, 0.1f, 10.f);
		}
	}

	if (_param_hntr_roll_sign != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_roll_sign, &value) == 0) {
			_roll_torque_sign = (value < 0.f) ? -1.f : 1.f;
		}
	}

	if (_param_hntr_tail_sign != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_tail_sign, &value) == 0) {
			_tail_torque_sign = (value < 0.f) ? -1.f : 1.f;
		}
	}

	if (_param_hntr_tail_rev_t != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_tail_rev_t, &value) == 0) {
			_tail_reverse_delay_s = math::constrain(value, 0.f, 2.f);
		}
	}

	if (_param_hntr_t_rev_min != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_t_rev_min, &value) == 0) {
			_tail_reverse_min_delay_s = math::constrain(value, 0.f, _tail_reverse_delay_s);
		}
	}

	if (_param_hntr_t_force_ref != PARAM_INVALID) {
		param_get(_param_hntr_t_force_ref, &_tail_dynamic_force_ref);
		_tail_dynamic_force_ref = math::constrain(_tail_dynamic_force_ref, 0.1f, 200.f);
	}

	if (_param_hntr_t_slew_up != PARAM_INVALID) {
		param_get(_param_hntr_t_slew_up, &_tail_slew_up_norm_s);
		_tail_slew_up_norm_s = math::constrain(_tail_slew_up_norm_s, 0.1f, 100.f);
	}

	if (_param_hntr_t_slew_dn != PARAM_INVALID) {
		param_get(_param_hntr_t_slew_dn, &_tail_slew_down_norm_s);
		_tail_slew_down_norm_s = math::constrain(_tail_slew_down_norm_s, 0.1f, 100.f);
	}

	if (_param_hntr_t_rev_slew != PARAM_INVALID) {
		param_get(_param_hntr_t_rev_slew, &_tail_reverse_slew_norm_s);
		_tail_reverse_slew_norm_s = math::constrain(_tail_reverse_slew_norm_s, 0.1f, 100.f);
	}

	if (_param_hntr_t_rpm_max != PARAM_INVALID) {
		param_get(_param_hntr_t_rpm_max, &_tail_rpm_max);
		_tail_rpm_max = math::constrain(_tail_rpm_max, 100.f, 100000.f);
	}

	int32_t pwm_value = 0;

	if (_param_pwm_main_min5 != PARAM_INVALID && param_get(_param_pwm_main_min5, &pwm_value) == 0) {
		_tail_pwm_min = pwm_value;
	}

	if (_param_pwm_main_max5 != PARAM_INVALID && param_get(_param_pwm_main_max5, &pwm_value) == 0) {
		_tail_pwm_max = pwm_value;
	}

	if (_param_hntr_s1_rate != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_s1_rate, &value) == 0) {
			_primary_servo_rate_rad_s = math::constrain(value, 0.1f, 50.f);
		}
	}

	if (_param_hntr_s2_rate != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_s2_rate, &value) == 0) {
			_secondary_servo_rate_rad_s = math::constrain(value, 0.1f, 50.f);
		}
	}

	if (_param_hntr_to_sup_t != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_to_sup_t, &value) == 0) {
			_takeoff_tilt_suppress_time_s = math::max(value, 0.f);
		}
	}

	if (_param_hntr_to_lock_t != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_to_lock_t, &value) == 0) {
			_takeoff_xy_lock_time_s = math::max(value, _takeoff_tilt_suppress_time_s);
		}
	}

	if (_param_hntr_to_ramp_t != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_to_ramp_t, &value) == 0) {
			_takeoff_release_ramp_time_s = math::max(value, 0.f);
		}
	}

	if (_param_hntr_to_tilt != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_to_tilt, &value) == 0) {
			_takeoff_tilt_limit = math::radians(math::constrain(value, 0.f, 185.f));
		}
	}

	if (_param_hntr_lock_tilt != PARAM_INVALID) {
		float value = 0.f;

		if (param_get(_param_hntr_lock_tilt, &value) == 0) {
			_xy_lock_tilt_limit = math::radians(math::constrain(value, 0.f, 185.f));
		}
	}
}

float ActuatorEffectivenessHnuter::slewTowards(float current, float target, float max_delta)
{
	return current + math::constrain(target - current, -math::max(max_delta, 0.f), math::max(max_delta, 0.f));
}

float ActuatorEffectivenessHnuter::applyTailReversalGuard(float desired_tail_force, hrt_abstime now)
{
	const float max_tail_force = math::max(_max_tail_thrust, 1.f);
	const float dynamic_force_ref = math::constrain(_tail_dynamic_force_ref, 0.1f, max_tail_force);
	const float force_deadband = math::constrain(0.005f * dynamic_force_ref, 0.02f, 0.1f);
	const float dt = (_tail_last_update != 0 && now >= _tail_last_update)
			 ? math::constrain((now - _tail_last_update) * 1e-6f, 0.f, 0.05f) : 0.004f;
	_tail_last_update = now;
	_tail_force_requested = math::constrain(desired_tail_force, -max_tail_force, max_tail_force);
	_tail_limited = false;
	_tail_reverse_dwell_elapsed_s = 0.f;

	auto direction = [force_deadband](float force) {
		return force > force_deadband ? int8_t{1} : (force < -force_deadband ? int8_t{-1} : int8_t{0});
	};

	auto reversalDwell = [&](float from_force, float to_force) {
		const float severity = math::constrain(math::max(fabsf(from_force), fabsf(to_force)) / dynamic_force_ref, 0.f, 1.f);
		return _tail_reverse_min_delay_s
		       + (_tail_reverse_delay_s - _tail_reverse_min_delay_s) * severity * severity;
	};

	int8_t desired_direction = direction(_tail_force_requested);
	int8_t current_direction = direction(_tail_force_command);

	if (_tail_state == hnuter_allocator_status_s::TAIL_STATE_RAMP_DOWN) {
		if (desired_direction != 0 && desired_direction == _tail_last_nonzero_direction) {
			// The request returned to the original direction before reaching zero.
			_tail_pending_direction = 0;
			_tail_state = hnuter_allocator_status_s::TAIL_STATE_TRACKING;

		} else {
			_tail_force_command = slewTowards(_tail_force_command, 0.f,
							  dynamic_force_ref * _tail_slew_down_norm_s * dt);
			_tail_limited = true;

			if (fabsf(_tail_force_command) <= force_deadband) {
				_tail_force_command = 0.f;
				_tail_zero_timestamp = now;
				_tail_state = (_tail_pending_direction != 0)
					      ? hnuter_allocator_status_s::TAIL_STATE_DWELL
					      : hnuter_allocator_status_s::TAIL_STATE_TRACKING;
			}

			return _tail_force_command;
		}
	}

	if (_tail_state == hnuter_allocator_status_s::TAIL_STATE_DWELL) {
		if (desired_direction == 0) {
			_tail_pending_direction = 0;
			_tail_state = hnuter_allocator_status_s::TAIL_STATE_TRACKING;
			return 0.f;
		}

		if (desired_direction == _tail_last_nonzero_direction) {
			_tail_pending_direction = 0;
			_tail_state = hnuter_allocator_status_s::TAIL_STATE_TRACKING;

		} else {
			_tail_reverse_dwell_elapsed_s = (_tail_zero_timestamp != 0 && now >= _tail_zero_timestamp)
							? (now - _tail_zero_timestamp) * 1e-6f : 0.f;

			if (_tail_reverse_dwell_elapsed_s < _tail_reverse_dwell_required_s) {
				_tail_limited = true;
				return 0.f;
			}

			_tail_pending_direction = 0;
			_tail_last_nonzero_direction = desired_direction;
			_tail_state = hnuter_allocator_status_s::TAIL_STATE_RAMP_UP;
		}
	}

	current_direction = direction(_tail_force_command);
	desired_direction = direction(_tail_force_requested);

	if (current_direction != 0 && desired_direction != 0 && desired_direction != current_direction) {
		_tail_last_nonzero_direction = current_direction;
		_tail_pending_direction = desired_direction;
		_tail_reverse_dwell_required_s = reversalDwell(_tail_force_command, _tail_force_requested);
		_tail_reversal_count++;
		_tail_state = hnuter_allocator_status_s::TAIL_STATE_RAMP_DOWN;
		_tail_force_command = slewTowards(_tail_force_command, 0.f,
						  dynamic_force_ref * _tail_slew_down_norm_s * dt);
		_tail_limited = true;

		if (fabsf(_tail_force_command) <= force_deadband) {
			_tail_force_command = 0.f;
			_tail_zero_timestamp = now;
			_tail_state = hnuter_allocator_status_s::TAIL_STATE_DWELL;
		}

		return _tail_force_command;
	}

	if (current_direction == 0 && desired_direction != 0 && _tail_last_nonzero_direction != 0
	    && desired_direction != _tail_last_nonzero_direction) {
		const float required_dwell = reversalDwell(0.f, _tail_force_requested);
		const float neutral_time = (_tail_zero_timestamp != 0 && now >= _tail_zero_timestamp)
					   ? (now - _tail_zero_timestamp) * 1e-6f : 0.f;

		if (neutral_time < required_dwell) {
			_tail_reverse_dwell_required_s = required_dwell;
			_tail_reverse_dwell_elapsed_s = neutral_time;
			_tail_pending_direction = desired_direction;
			_tail_reversal_count++;
			_tail_state = hnuter_allocator_status_s::TAIL_STATE_DWELL;
			_tail_limited = true;
			return 0.f;
		}

		_tail_last_nonzero_direction = desired_direction;
		_tail_state = hnuter_allocator_status_s::TAIL_STATE_RAMP_UP;
	}

	if (desired_direction == 0) {
		_tail_force_command = slewTowards(_tail_force_command, 0.f,
						  dynamic_force_ref * _tail_slew_down_norm_s * dt);
		_tail_limited = fabsf(_tail_force_command) > force_deadband;
		_tail_state = _tail_limited ? hnuter_allocator_status_s::TAIL_STATE_RAMP_DOWN
			      : hnuter_allocator_status_s::TAIL_STATE_TRACKING;
		_tail_pending_direction = 0;

		if (!_tail_limited) {
			_tail_force_command = 0.f;
			_tail_zero_timestamp = now;
		}

		return _tail_force_command;
	}

	const bool post_reversal_ramp = _tail_state == hnuter_allocator_status_s::TAIL_STATE_RAMP_UP;
	const bool increasing_magnitude = fabsf(_tail_force_requested) > fabsf(_tail_force_command);
	const float slew_rate = post_reversal_ramp ? _tail_reverse_slew_norm_s
				: (increasing_magnitude ? _tail_slew_up_norm_s : _tail_slew_down_norm_s);
	_tail_force_command = slewTowards(_tail_force_command, _tail_force_requested, dynamic_force_ref * slew_rate * dt);
	_tail_limited = fabsf(_tail_force_requested - _tail_force_command) > force_deadband;

	if (direction(_tail_force_command) != 0) {
		_tail_last_nonzero_direction = direction(_tail_force_command);
		_tail_zero_timestamp = 0;
	}

	if (!_tail_limited) {
		_tail_force_command = _tail_force_requested;
		_tail_state = hnuter_allocator_status_s::TAIL_STATE_TRACKING;
	}

	return _tail_force_command;
}

bool ActuatorEffectivenessHnuter::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (!_collective_tilt_updated && external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	configuration.selected_matrix = 0;
	_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
	_mc_rotors.enableThreeDimensionalThrust(true);

	const float collective_tilt_control_applied = (external_update == EffectivenessUpdateReason::CONFIGURATION_UPDATE) ?
			-1.f : _last_collective_tilt_control;
	_untiltable_motors = _mc_rotors.updateAxisFromTilts(_tilts, collective_tilt_control_applied)
			     << configuration.num_actuators[(int)ActuatorType::MOTORS];

	const bool mc_rotors_added_successfully = _mc_rotors.addActuators(configuration);
	_motors = _mc_rotors.getMotors();

	_first_tilt_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	_tilts.updateTorqueSign(_mc_rotors.geometry(), false);
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	configuration.effectiveness_matrices[0].setZero();

	_collective_tilt_updated = (external_update == EffectivenessUpdateReason::CONFIGURATION_UPDATE);

	return (mc_rotors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessHnuter::allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp)
{
}

void ActuatorEffectivenessHnuter::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	if (matrix_index != 0) {
		return;
	}

	vehicle_control_mode_s vehicle_control_mode{};

	if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
		_armed = vehicle_control_mode.flag_armed;
		_offboard_enabled = vehicle_control_mode.flag_control_offboard_enabled;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (_armed && !_prev_armed) {
		_armed_time = now;
	}

	_prev_armed = _armed;

	vehicle_land_detected_s land_detected{};

	if (_vehicle_land_detected_sub.update(&land_detected)) {
		_landed = land_detected.landed;
	}

	if (!_armed) {
		const int num_tilts = _tilts.count();

		const int num_rotors = _mc_rotors.geometry().num_rotors;

		for (int i = 0; i < num_rotors; i++) {
			actuator_sp(i) = 0.f;
		}

		if (num_tilts >= 4) {
			actuator_sp(_first_tilt_idx + 0) = 0.0f;
			actuator_sp(_first_tilt_idx + 1) = 0.0f;
			actuator_sp(_first_tilt_idx + 2) = 0.0f;
			actuator_sp(_first_tilt_idx + 3) = 0.0f;
		}

		_last_servo_update = 0;
		_last_servo_sp.setZero();
		_tail_force_command = 0.f;
		_tail_force_requested = 0.f;
		_tail_reverse_dwell_required_s = 0.f;
		_tail_reverse_dwell_elapsed_s = 0.f;
		_tail_zero_timestamp = 0;
		_tail_last_update = 0;
		_tail_last_nonzero_direction = 0;
		_tail_pending_direction = 0;
		_tail_state = hnuter_allocator_status_s::TAIL_STATE_TRACKING;
		_tail_reversal_count = 0;
		_tail_limited = false;
		_unallocated_control.setZero();
		_armed_time = 0;
		_prev_armed = false;
		return;
	}

	updateParams();

	const float time_since_armed_s = (_armed_time != 0) ? math::constrain(((now - _armed_time) * 1e-6f), 0.f, 100.f) : 100.f;
	const bool takeoff_tilt_suppress_active = time_since_armed_s < _takeoff_tilt_suppress_time_s;
	const bool takeoff_xy_lock_active = (time_since_armed_s >= _takeoff_tilt_suppress_time_s)
					    && (time_since_armed_s < _takeoff_xy_lock_time_s);
	const float takeoff_release_progress = _takeoff_release_ramp_time_s > FLT_EPSILON
					       ? math::constrain((time_since_armed_s - _takeoff_xy_lock_time_s) / _takeoff_release_ramp_time_s, 0.f, 1.f)
					       : 1.f;
	const bool takeoff_release_ramp_active = time_since_armed_s >= _takeoff_xy_lock_time_s
			&& takeoff_release_progress < 1.f;

	const float l1 = _l1;
	const float l2 = _l2;
	const float r_x = _cg_x;
	const float r_z = _cg_z;
	const float max_thrust_per_arm = _max_thrust_per_arm;
	const float max_tail_thrust = _max_tail_thrust;
	const float max_front_vertical_thrust = max_thrust_per_arm * 2.0f;
	const float mass = _mass;
	const float gravity = 9.81f;
	const float max_vertical_thrust = max_front_vertical_thrust;

	const float fx =  control_sp(3) * max_thrust_per_arm;
	const float fy = -control_sp(4) * max_thrust_per_arm;
	const float fz = normalizedThrustToForce(-control_sp(5), max_vertical_thrust);
	const float tx =  _roll_torque_sign * control_sp(0) * (max_thrust_per_arm * l1);
	const float ty =  _tail_torque_sign * control_sp(1) * (max_tail_thrust * l2);
	const float tz = -control_sp(2) * (max_thrust_per_arm * l1);

	float W[6] {fx, fy, fz, tx, ty, tz};

	if (takeoff_tilt_suppress_active) {
		W[0] = 0.f;
		W[1] = 0.f;
	}

	float u1 = W[0] / 2.0f - W[5] / (2.0f * l1);
	float u4 = W[0] / 2.0f + W[5] / (2.0f * l1);

	// Motor5 is a dedicated Pitch torque actuator. Its command must not follow
	// collective thrust; any attitude-dependent gravity trim is modelled
	// separately once the primary-axis-to-CG geometry has been measured.
	const float F3_desired = W[4] / (r_x + l2);
	const float F3 = math::constrain(applyTailReversalGuard(F3_desired, now), -max_tail_thrust, max_tail_thrust);

	// Compensate the front vertical force using the guarded tail force that is
	// actually commanded, not the unavailable opposite-direction request.
	float Fz_front = W[2] - F3;

	float Tx_parasitic = - r_z * W[1];
	float Tx_comp = W[3] - Tx_parasitic;

	float u2 = Fz_front / 2.0f + Tx_comp / (2.0f * l1);
	float u5 = Fz_front / 2.0f - Tx_comp / (2.0f * l1);

	float u3 = -W[1] / 2.0f;
	float u6 = -W[1] / 2.0f;

	float F1 = sqrtf(u1 * u1 + u2 * u2 + u3 * u3);
	float F2 = sqrtf(u4 * u4 + u5 * u5 + u6 * u6);

	const float eps = 1e-8f;
	float F1_safe = fmaxf(F1, eps);
	float F2_safe = fmaxf(F2, eps);

	float alpha1 = atan2f(u1, u2);
	float alpha2 = atan2f(u4, u5);
	float theta1 = asinf(math::constrain(u3 / F1_safe, -1.0f, 1.0f));
	float theta2 = asinf(math::constrain(u6 / F2_safe, -1.0f, 1.0f));

	const float alpha2_angle_max = tiltAngleAbsLimit(_tilts, 0, M_PI_F);
	const float alpha1_angle_max = tiltAngleAbsLimit(_tilts, 1, M_PI_F);
	const float theta2_servo_angle_max = tiltAngleAbsLimit(_tilts, 2, M_PI_F);
	const float theta1_servo_angle_max = tiltAngleAbsLimit(_tilts, 3, M_PI_F);
	const float theta2_angle_max = theta2_servo_angle_max / _secondary_servo_gear_ratio;
	const float theta1_angle_max = theta1_servo_angle_max / _secondary_servo_gear_ratio;
	const float alpha_angle_max = math::min(alpha1_angle_max, alpha2_angle_max);
	const float theta_angle_max = math::min(theta1_angle_max, theta2_angle_max);
	float alpha_limit = alpha_angle_max;
	float theta_limit = theta_angle_max;

	if (takeoff_tilt_suppress_active) {
		alpha_limit = _takeoff_tilt_limit;
		theta_limit = _takeoff_tilt_limit;

	} else if (takeoff_xy_lock_active) {
		alpha_limit = _xy_lock_tilt_limit;
		theta_limit = _xy_lock_tilt_limit;

	} else if (takeoff_release_ramp_active) {
		alpha_limit = _xy_lock_tilt_limit + (alpha_angle_max - _xy_lock_tilt_limit) * takeoff_release_progress;
		theta_limit = _xy_lock_tilt_limit + (theta_angle_max - _xy_lock_tilt_limit) * takeoff_release_progress;
	}

	if (_last_servo_update != 0
	    && alpha_limit >= math::radians(179.0f)
	    && theta_limit >= math::radians(179.0f)) {
		auto select_continuous_gimbal_branch = [alpha_limit, theta_limit](
		float & alpha, float & theta, float previous_alpha, float previous_theta) {
			const float base_alpha = matrix::unwrap_pi(previous_alpha, alpha);
			const float alternate_alpha = matrix::unwrap_pi(previous_alpha, alpha + M_PI_F);
			const float alternate_positive_theta = M_PI_F - theta;
			const float alternate_negative_theta = -M_PI_F - theta;
			const bool base_valid = fabsf(base_alpha) <= alpha_limit && fabsf(theta) <= theta_limit;
			const bool alternate_positive_valid = fabsf(alternate_alpha) <= alpha_limit
							      && fabsf(alternate_positive_theta) <= theta_limit;
			const bool alternate_negative_valid = fabsf(alternate_alpha) <= alpha_limit
							      && fabsf(alternate_negative_theta) <= theta_limit;
			const float base_alpha_delta = base_alpha - previous_alpha;
			const float base_theta_delta = theta - previous_theta;
			const float alternate_alpha_delta = alternate_alpha - previous_alpha;
			const float alternate_positive_delta = alternate_positive_theta - previous_theta;
			const float alternate_negative_delta = alternate_negative_theta - previous_theta;
			const float base_cost = base_valid
						? base_alpha_delta * base_alpha_delta + base_theta_delta * base_theta_delta
						: 1e30f;
			const float alternate_positive_cost = alternate_positive_valid
							      ? alternate_alpha_delta * alternate_alpha_delta
							      + alternate_positive_delta * alternate_positive_delta
							      : 1e30f;
			const float alternate_negative_cost = alternate_negative_valid
							      ? alternate_alpha_delta * alternate_alpha_delta
							      + alternate_negative_delta * alternate_negative_delta
							      : 1e30f;

			if (alternate_positive_cost < base_cost
			    && alternate_positive_cost <= alternate_negative_cost) {
				alpha = alternate_alpha;
				theta = alternate_positive_theta;

			} else if (alternate_negative_cost < base_cost) {
				alpha = alternate_alpha;
				theta = alternate_negative_theta;

			} else {
				alpha = base_alpha;
			}
		};

		select_continuous_gimbal_branch(alpha1, theta1,
						_last_servo_sp(1) * alpha1_angle_max,
						_last_servo_sp(3) * theta1_servo_angle_max / _secondary_servo_gear_ratio);
		select_continuous_gimbal_branch(alpha2, theta2,
						_last_servo_sp(0) * alpha2_angle_max,
						_last_servo_sp(2) * theta2_servo_angle_max / _secondary_servo_gear_ratio);
	}

	if (_last_servo_update != 0 && alpha_limit >= math::radians(179.0f)) {
		// Keep the atan2 branch continuous near the primary tilt hard stops.
		const float previous_alpha1 = _last_servo_sp(1) * alpha1_angle_max;
		const float previous_alpha2 = _last_servo_sp(0) * alpha2_angle_max;
		alpha1 = math::constrain(matrix::unwrap_pi(previous_alpha1, alpha1), -alpha_limit, alpha_limit);
		alpha2 = math::constrain(matrix::unwrap_pi(previous_alpha2, alpha2), -alpha_limit, alpha_limit);

	} else {
		alpha1 = math::constrain(alpha1, -alpha_limit, alpha_limit);
		alpha2 = math::constrain(alpha2, -alpha_limit, alpha_limit);
	}

	theta1 = math::constrain(theta1, -theta_limit, theta_limit);
	theta2 = math::constrain(theta2, -theta_limit, theta_limit);

	const int num_rotors = _mc_rotors.geometry().num_rotors;
	const int num_tilts = _tilts.count();
	float normalized_tail_output = 0.f;

	if (num_rotors >= 4) {
		const float right_single = 0.5f * F2;
		const float left_single = 0.5f * F1;

		float norm_right = 0.f;
		float norm_left = 0.f;

		if (_simulation_motor_model) {
			norm_right = thrustToNormalizedMotorControl(right_single, _motor_constant, _sim_min_velocity, _sim_max_velocity);
			norm_left = thrustToNormalizedMotorControl(left_single, _motor_constant, _sim_min_velocity, _sim_max_velocity);
			normalized_tail_output = signedThrustToNormalizedMotorControl(F3, _motor_constant,
						 _sim_min_velocity, _sim_max_velocity);

		} else {
			// A hover anchor is measurable in flight and avoids pretending that the
			// Gazebo motor constant describes the real 4112/460 KV/15-inch setup.
			const float hover_force_per_motor = mass * gravity * 0.25f;
			norm_right = thrustToHoverAnchoredMotorControl(right_single, hover_force_per_motor,
					_motor_hover_control, _motor_thrust_exponent);
			norm_left = thrustToHoverAnchoredMotorControl(left_single, hover_force_per_motor,
					_motor_hover_control, _motor_thrust_exponent);
			normalized_tail_output = signedForceToNormalizedMotorControl(F3, max_tail_thrust,
						 _motor_thrust_exponent);
		}

		actuator_sp(0) = norm_right;
		actuator_sp(1) = norm_right;
		actuator_sp(2) = norm_left;
		actuator_sp(3) = norm_left;

		if (num_rotors >= 5) {
			actuator_sp(4) = normalized_tail_output;
		}
	}

	float dt = 0.004f;

	if (_last_servo_update != 0) {
		dt = math::constrain((now - _last_servo_update) / 1e6f, 0.f, 0.2f);
	}

	if (num_tilts >= 4) {
		matrix::Vector<float, 4> servo_sp;
		servo_sp(0) = tiltAngleToNormalizedServo(alpha2, _tilts, 0, M_PI_F);
		servo_sp(1) = tiltAngleToNormalizedServo(alpha1, _tilts, 1, M_PI_F);
		servo_sp(2) = tiltAngleToNormalizedServo(theta2 * _secondary_servo_gear_ratio, _tilts, 2, M_PI_F);
		servo_sp(3) = tiltAngleToNormalizedServo(theta1 * _secondary_servo_gear_ratio, _tilts, 3, M_PI_F);

		if (dt > 0.f) {
			const float alpha2_max_delta = (_primary_servo_rate_rad_s * dt) / alpha2_angle_max;
			const float alpha1_max_delta = (_primary_servo_rate_rad_s * dt) / alpha1_angle_max;
			const float theta2_max_delta = (_secondary_servo_rate_rad_s * dt) / theta2_servo_angle_max;
			const float theta1_max_delta = (_secondary_servo_rate_rad_s * dt) / theta1_servo_angle_max;
			servo_sp(0) = math::constrain(servo_sp(0), _last_servo_sp(0) - alpha2_max_delta,
						      _last_servo_sp(0) + alpha2_max_delta);
			servo_sp(1) = math::constrain(servo_sp(1), _last_servo_sp(1) - alpha1_max_delta,
						      _last_servo_sp(1) + alpha1_max_delta);
			servo_sp(2) = math::constrain(servo_sp(2), _last_servo_sp(2) - theta2_max_delta,
						      _last_servo_sp(2) + theta2_max_delta);
			servo_sp(3) = math::constrain(servo_sp(3), _last_servo_sp(3) - theta1_max_delta,
						      _last_servo_sp(3) + theta1_max_delta);
		}

		actuator_sp(_first_tilt_idx + 0) = servo_sp(0);
		actuator_sp(_first_tilt_idx + 1) = servo_sp(1);
		actuator_sp(_first_tilt_idx + 2) = servo_sp(2);
		actuator_sp(_first_tilt_idx + 3) = servo_sp(3);

		_last_servo_sp = servo_sp;
		_last_servo_update = now;
	}

	// Reconstruct the wrench from the commands that survive tail reversal,
	// thrust saturation, joint limits and servo-rate limiting. This is a model
	// residual (actual position/RPM sensors are not installed), but unlike the
	// previous all-zero report it exposes commands the allocator cannot realize.
	float alpha1_achieved = alpha1;
	float alpha2_achieved = alpha2;
	float theta1_achieved = theta1;
	float theta2_achieved = theta2;

	if (num_tilts >= 4) {
		alpha2_achieved = _last_servo_sp(0) * alpha2_angle_max;
		alpha1_achieved = _last_servo_sp(1) * alpha1_angle_max;
		theta2_achieved = _last_servo_sp(2) * theta2_servo_angle_max / _secondary_servo_gear_ratio;
		theta1_achieved = _last_servo_sp(3) * theta1_servo_angle_max / _secondary_servo_gear_ratio;
	}

	const float F1_achieved = math::constrain(F1, 0.f, max_thrust_per_arm);
	const float F2_achieved = math::constrain(F2, 0.f, max_thrust_per_arm);
	const float u1_achieved = F1_achieved * cosf(theta1_achieved) * sinf(alpha1_achieved);
	const float u2_achieved = F1_achieved * cosf(theta1_achieved) * cosf(alpha1_achieved);
	const float u3_achieved = F1_achieved * sinf(theta1_achieved);
	const float u4_achieved = F2_achieved * cosf(theta2_achieved) * sinf(alpha2_achieved);
	const float u5_achieved = F2_achieved * cosf(theta2_achieved) * cosf(alpha2_achieved);
	const float u6_achieved = F2_achieved * sinf(theta2_achieved);
	const float fx_achieved = u1_achieved + u4_achieved;
	const float fy_achieved = -(u3_achieved + u6_achieved);
	const float fz_achieved = u2_achieved + u5_achieved + F3;
	const float tx_achieved = l1 * (u2_achieved - u5_achieved) - r_z * fy_achieved;
	const float ty_achieved = F3 * (r_x + l2);
	const float tz_achieved = l1 * (u4_achieved - u1_achieved);
	matrix::Vector<float, NUM_AXES> achieved_control{};
	achieved_control(0) = _roll_torque_sign * tx_achieved / math::max(max_thrust_per_arm * l1, 1.f);
	achieved_control(1) = _tail_torque_sign * ty_achieved / math::max(max_tail_thrust * l2, 1.f);
	achieved_control(2) = -tz_achieved / math::max(max_thrust_per_arm * l1, 1.f);
	achieved_control(3) = fx_achieved / math::max(max_thrust_per_arm, 1.f);
	achieved_control(4) = -fy_achieved / math::max(max_thrust_per_arm, 1.f);
	achieved_control(5) = -fz_achieved / math::max(max_vertical_thrust, 1.f);
	_unallocated_control = control_sp - achieved_control;

	hnuter_allocator_status_s allocator_status{};
	allocator_status.timestamp = now;
	allocator_status.simulation_model = _simulation_motor_model;
	allocator_status.tail_limited = _tail_limited || fabsf(F3_desired - F3) > 0.02f;
	allocator_status.tail_state = _tail_state;
	allocator_status.reversal_count = _tail_reversal_count;
	allocator_status.tail_force_requested = F3_desired;
	allocator_status.tail_force_commanded = F3;
	allocator_status.tail_force_error = F3_desired - F3;
	allocator_status.tail_output_normalized = normalized_tail_output;
	allocator_status.reversal_dwell_required = _tail_reverse_dwell_required_s;
	allocator_status.reversal_dwell_elapsed = _tail_reverse_dwell_elapsed_s;
	allocator_status.pitch_unallocated = _unallocated_control(1);

	if (_simulation_motor_model) {
		const float angular_speed = fabsf(normalized_tail_output) > FLT_EPSILON
					    ? _sim_min_velocity + fabsf(normalized_tail_output)
					    * (_sim_max_velocity - _sim_min_velocity) : 0.f;
		allocator_status.tail_pwm_estimate = NAN;
		allocator_status.tail_rpm_estimate = math::signNoZero(normalized_tail_output)
						     * angular_speed * 60.f / (2.f * M_PI_F);

	} else {
		allocator_status.tail_pwm_estimate = 0.5f * (_tail_pwm_min + _tail_pwm_max)
						     + 0.5f * (_tail_pwm_max - _tail_pwm_min) * normalized_tail_output;
		allocator_status.tail_rpm_estimate = normalized_tail_output * _tail_rpm_max;
	}

	_hnuter_allocator_status_pub.publish(allocator_status);
}

void ActuatorEffectivenessHnuter::setFlightPhase(const FlightPhase &flight_phase)
{
	if (_flight_phase == flight_phase) {
		return;
	}

	ActuatorEffectiveness::setFlightPhase(flight_phase);
	_stopped_motors_mask = 0;
}

const char *ActuatorEffectivenessHnuter::name() const
{
	return "Hnuter Tiltrotor";
}

void ActuatorEffectivenessHnuter::getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const
{
	for (int i = 0; i < MAX_NUM_MATRICES; i++) {
		allocation_method_out[i] = AllocationMethod::NONE;
	}

	allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
}

void ActuatorEffectivenessHnuter::getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const
{
	for (int i = 0; i < MAX_NUM_MATRICES; i++) {
		normalize[i] = false;
	}

	normalize[0] = true;
}

void ActuatorEffectivenessHnuter::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	if (matrix_index != 0) {
		return;
	}

	status.unallocated_torque[0] = _unallocated_control(0);
	status.unallocated_torque[1] = _unallocated_control(1);
	status.unallocated_torque[2] = _unallocated_control(2);
	status.unallocated_thrust[0] = _unallocated_control(3);
	status.unallocated_thrust[1] = _unallocated_control(4);
	status.unallocated_thrust[2] = _unallocated_control(5);
}
