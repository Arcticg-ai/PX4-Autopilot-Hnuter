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
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ActuatorEffectivenessFullyActuated.cpp
 *
 * Actuator effectiveness for fully actuated vehicles
 *
 * @author PX4 Development Team
 */

#include "ActuatorEffectivenessFullyActuated.hpp"

using namespace matrix;

ActuatorEffectivenessFullyActuated::ActuatorEffectivenessFullyActuated(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true),
	  _tilts(this)
{
	updateParams();
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}

void ActuatorEffectivenessFullyActuated::updateParams()
{
	ModuleParams::updateParams();
}

bool
ActuatorEffectivenessFullyActuated::getEffectivenessMatrix(Configuration &configuration,
			EffectivenessUpdateReason external_update)
{
	if (!_collective_tilt_updated && external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// MC motors with tilt support
	configuration.selected_matrix = 0;
	_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
	_mc_rotors.enableThreeDimensionalThrust(true);

	// Update matrix with tilts in vertical position when update is triggered by a manual
	// configuration (parameter) change. This is to make sure the normalization
	// scales are tilt-invariant. Note: configuration updates are only possible when disarmed.
	const float collective_tilt_control_applied = (external_update == EffectivenessUpdateReason::CONFIGURATION_UPDATE) ?
			-1.f : _last_collective_tilt_control;
	_untiltable_motors = _mc_rotors.updateAxisFromTilts(_tilts, collective_tilt_control_applied)
			     << configuration.num_actuators[(int)ActuatorType::MOTORS];

	const bool mc_rotors_added_successfully = _mc_rotors.addActuators(configuration);
	_motors = _mc_rotors.getMotors();

	// Tilts
	_first_tilt_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	_tilts.updateTorqueSign(_mc_rotors.geometry(), false);
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	// Add tail thruster support for hnuter controller frame
	// Configure tail thruster for pitch control (Ty)
	if (configuration.num_actuators[(int)ActuatorType::MOTORS] >= 5) {
		// Get the index of the 5th motor (tail thruster)
		int tail_thruster_idx = configuration.num_actuators_matrix[0] - 1;

		// Update effectiveness matrix for tail thruster
		// Tail thruster contributes to pitch torque (Ty)
		configuration.effectiveness_matrices[0](1, tail_thruster_idx) = 2.0f; // Ty = 2 * T5

		// Tail thruster can also contribute to Fx if needed
		// configuration.effectiveness_matrices[0](3, tail_thruster_idx) = 1.0f; // Fx = T5
	}

	// If it was an update coming from a config change, then make sure to update matrix in
	// the next iteration again with the correct tilt (but without updating the normalization scale).
	_collective_tilt_updated = (external_update == EffectivenessUpdateReason::CONFIGURATION_UPDATE);

	// Update effectiveness matrix for 90° large angle tilt capability
	// Ensure that tilt actuators can handle large angles and provide sufficient control authority
	for (int i = 0; i < _tilts.count(); ++i) {
		int tilt_idx = _first_tilt_idx + i;

		// Increase tilt actuator effectiveness for large angle control
		// This ensures sufficient control authority during 90° tilt transitions
		configuration.effectiveness_matrices[0](0, tilt_idx) *= 1.5f; // Increase roll control authority
		configuration.effectiveness_matrices[0](1, tilt_idx) *= 1.5f; // Increase pitch control authority
		configuration.effectiveness_matrices[0](2, tilt_idx) *= 1.5f; // Increase yaw control authority
	}

	return (mc_rotors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessFullyActuated::allocateAuxilaryControls(const float dt, int matrix_index,
			ActuatorVector &actuator_sp)
{
	// No auxiliary controls for fully actuated vehicles
}

void ActuatorEffectivenessFullyActuated::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
			int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	// Only handle matrix 0 (motors and tilts)
	if (matrix_index != 0) {
		return;
	}

	// Extract control setpoints (W vector)
	// W = [Fx, Fy, Fz, Tx, Ty, Tz]
	float W[6];
	W[0] = control_sp(3); // Fx
	W[1] = control_sp(4); // Fy
	W[2] = control_sp(5); // Fz
	W[3] = control_sp(0); // Tx (roll)
	W[4] = control_sp(1); // Ty (pitch)
	W[5] = control_sp(2); // Tz (yaw)

	// Nonlinear inverse mapping algorithm from hnuter_controller_frame
	// 尾部推力 (由俯仰力矩确定)
	float u7 = (2.0f / 1.0f) * W[4];

	// 左/右旋翼的 X轴分力 (由总Fx和偏航力矩Tz确定)
	float u1 = W[0] / 2.0f - (10.0f / 3.0f) * W[5];
	float u4 = W[0] / 2.0f + (10.0f / 3.0f) * W[5];

	// 左/右旋翼的 Z轴分力 (由总Fz和滚转力矩Tx确定)
	float Fz_front = W[2];
	float u2 = Fz_front / 2.0f - (10.0f / 3.0f) * W[3];
	float u5 = Fz_front / 2.0f + (10.0f / 3.0f) * W[3];

	// 侧向分力均分
	float target_Fy = W[1];
	float u3 = -target_Fy / 2.0f;
	float u6 = -target_Fy / 2.0f;

	// 计算推力和角度
	float F1 = sqrtf(u1 * u1 + u2 * u2 + u3 * u3);
	float F2 = sqrtf(u4 * u4 + u5 * u5 + u6 * u6);
	float F3 = u7;

	// 防止除零保护
	const float eps = 1e-8f;
	float F1_safe = F1 > eps ? F1 : eps;
	float F2_safe = F2 > eps ? F2 : eps;

	// 求解倾转角度
	float alpha1 = atan2f(u1, u2);
	float alpha2 = atan2f(u4, u5);

	float val1 = fmaxf(fminf(u3 / F1_safe, 1.0f - eps), -1.0f + eps);
	float val2 = fmaxf(fminf(u6 / F2_safe, 1.0f - eps), -1.0f + eps);

	float theta1 = asinf(val1);
	float theta2 = asinf(val2);

	// Thrust limits (from hnuter_controller_frame)
	const float T_max = 60.0f;
	const float T5_max = 15.0f;

	// Angle limits (matching SDF joint limits ±90°)
	const float alpha_max = math::radians(90.0f);
	const float theta_max = math::radians(90.0f);

	// Apply limits to thrust
	F1 = fmaxf(fminf(F1, T_max), 0.0f);
	F2 = fmaxf(fminf(F2, T_max), 0.0f);
	F3 = fmaxf(fminf(F3, T5_max), -T5_max);

	// Apply limits to angles
	alpha1 = fmaxf(fminf(alpha1, alpha_max), -alpha_max);
	alpha2 = fmaxf(fminf(alpha2, alpha_max), -alpha_max);
	theta1 = fmaxf(fminf(theta1, theta_max), -theta_max);
	theta2 = fmaxf(fminf(theta2, theta_max), -theta_max);

	// Apply collective tilt if needed
	tiltrotor_extra_controls_s tiltrotor_extra_controls;
	float control_collective_tilt = 0.0f;

	if (_tiltrotor_extra_controls_sub.copy(&tiltrotor_extra_controls)) {
		control_collective_tilt = tiltrotor_extra_controls.collective_tilt_normalized_setpoint * 2.0f - 1.0f;
		control_collective_tilt = control_collective_tilt < -0.99f ? -1.0f : control_collective_tilt;
		control_collective_tilt = control_collective_tilt > 0.99f ? 1.0f : control_collective_tilt;

		if (!PX4_ISFINITE(_last_collective_tilt_control)) {
			_last_collective_tilt_control = control_collective_tilt;
		} else if (fabsf(control_collective_tilt - _last_collective_tilt_control) > 0.01f) {
			_collective_tilt_updated = true;
			_last_collective_tilt_control = control_collective_tilt;
		}
	}

	// Map to actuator setpoints
	// Assuming actuator_sp layout: [motors, tilts, tail_thruster]
	int num_motors = _mc_rotors.getMotors();
	int num_tilts = _tilts.count();

	// Set motor thrusts (front motors)
	if (num_motors >= 4) {
		// Left front motors (upper and lower)
		float T12 = F1;
		actuator_sp(0) = T12 / 2.0f;
		actuator_sp(1) = T12 / 2.0f;

		// Right front motors (upper and lower)
		float T34 = F2;
		actuator_sp(2) = T34 / 2.0f;
		actuator_sp(3) = T34 / 2.0f;

		// Rear motor (if exists)
		if (num_motors >= 5) {
			actuator_sp(4) = F3;
		}
	}

	// Set tilt angles
	if (num_tilts >= 4) {
		// Apply collective tilt to all tilts
		actuator_sp(_first_tilt_idx + 0) = alpha1 + control_collective_tilt;
		actuator_sp(_first_tilt_idx + 1) = alpha2 + control_collective_tilt;
		actuator_sp(_first_tilt_idx + 2) = theta1 + control_collective_tilt;
		actuator_sp(_first_tilt_idx + 3) = theta2 + control_collective_tilt;

		// Clip tilt actuator values to [-1, 1]
		for (int i = 0; i < num_tilts; ++i) {
			actuator_sp(_first_tilt_idx + i) = fmaxf(fminf(actuator_sp(_first_tilt_idx + i), 1.0f), -1.0f);
		}
	}

	// Yaw saturation logic
	bool yaw_saturated_positive = true;
	bool yaw_saturated_negative = true;

	for (int i = 0; i < num_tilts; ++i) {
		if (_tilts.getYawTorqueOfTilt(i) > FLT_EPSILON) {
			if (yaw_saturated_positive && actuator_sp(_first_tilt_idx + i) < actuator_max(_first_tilt_idx + i) - FLT_EPSILON) {
				yaw_saturated_positive = false;
			}

			if (yaw_saturated_negative && actuator_sp(_first_tilt_idx + i) > actuator_min(_first_tilt_idx + i) + FLT_EPSILON) {
				yaw_saturated_negative = false;
			}
		} else if (_tilts.getYawTorqueOfTilt(i) < -FLT_EPSILON) {
			if (yaw_saturated_negative && actuator_sp(_first_tilt_idx + i) < actuator_max(_first_tilt_idx + i) - FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

			if (yaw_saturated_positive && actuator_sp(_first_tilt_idx + i) > actuator_min(_first_tilt_idx + i) + FLT_EPSILON) {
				yaw_saturated_positive = false;
			}
		}
	}

	_yaw_tilt_saturation_flags.tilt_yaw_neg = yaw_saturated_negative;
	_yaw_tilt_saturation_flags.tilt_yaw_pos = yaw_saturated_positive;
}

void ActuatorEffectivenessFullyActuated::setFlightPhase(const FlightPhase &flight_phase)
{
	if (_flight_phase == flight_phase) {
		return;
	}

	ActuatorEffectiveness::setFlightPhase(flight_phase);

	// For fully actuated vehicles, we always keep all motors running
	_stopped_motors_mask = 0;
}

void ActuatorEffectivenessFullyActuated::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// only handle matrix 0 (motors and tilts)
	if (matrix_index != 0) {
		return;
	}

	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_yaw_tilt_saturation_flags.tilt_yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_yaw_tilt_saturation_flags.tilt_yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}
}
