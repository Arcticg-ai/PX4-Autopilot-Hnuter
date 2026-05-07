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
 * @file HnuterPositionControl.cpp
 *
 * Position controller for Hnuter tiltrotor vehicles
 *
 * @author PX4 Development Team
 */

#include "HnuterPositionControl.hpp"

#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/Limits.hpp>

using namespace matrix;

HnuterPositionControl::HnuterPositionControl()
{
	// Initialize position gains from hnuter_controller_frame
	_Kp = Vector3f(6.0f, 6.0f, 6.0f);     // Position proportional gain
	_Dp = Vector3f(5.0f, 5.0f, 5.0f);     // Position derivative gain

	// Initialize velocity gains
	_gain_vel_p = Vector3f(0.0f, 0.0f, 0.0f);
	_gain_vel_i = Vector3f(0.0f, 0.0f, 0.0f);
	_gain_vel_d = Vector3f(0.0f, 0.0f, 0.0f);

	// Initialize limits
	_lim_vel_horizontal = 5.0f;
	_lim_vel_up = 3.0f;
	_lim_vel_down = 2.0f;
	_lim_thr_min = -0.9f;
	_lim_thr_max = -0.1f;
	_lim_thr_xy_margin = 0.3f;
	_lim_tilt = math::radians(45.0f);

	// Initialize hover thrust
	_hover_thrust = 0.5f;

	// Initialize states and setpoints
	_pos.setZero();
	_vel.setZero();
	_vel_dot.setZero();
	_vel_int.setZero();
	_yaw = 0.0f;

	_pos_sp.setZero();
	_vel_sp.setZero();
	_acc_sp.setZero();
	_thr_sp.setZero();
	_yaw_sp = 0.0f;
	_yawspeed_sp = 0.0f;
}

void HnuterPositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void HnuterPositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void HnuterPositionControl::setThrustLimits(const float min, const float max)
{
	_lim_thr_min = min;
	_lim_thr_max = max;
}

void HnuterPositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void HnuterPositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_vel_dot = states.acceleration;
	_yaw = states.yaw;
}

void HnuterPositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	// Fill setpoints, NAN means no setpoint, keep previous
	if (!PX4_ISFINITE(setpoint.position[0])) {
		_pos_sp(0) = _pos_sp(0);
	} else {
		_pos_sp(0) = setpoint.position[0];
	}

	if (!PX4_ISFINITE(setpoint.position[1])) {
		_pos_sp(1) = _pos_sp(1);
	} else {
		_pos_sp(1) = setpoint.position[1];
	}

	if (!PX4_ISFINITE(setpoint.position[2])) {
		_pos_sp(2) = _pos_sp(2);
	} else {
		_pos_sp(2) = setpoint.position[2];
	}

	// Velocity setpoints
	if (!PX4_ISFINITE(setpoint.velocity[0])) {
		_vel_sp(0) = 0.0f;
	} else {
		_vel_sp(0) = setpoint.velocity[0];
	}

	if (!PX4_ISFINITE(setpoint.velocity[1])) {
		_vel_sp(1) = 0.0f;
	} else {
		_vel_sp(1) = setpoint.velocity[1];
	}

	if (!PX4_ISFINITE(setpoint.velocity[2])) {
		_vel_sp(2) = 0.0f;
	} else {
		_vel_sp(2) = setpoint.velocity[2];
	}

	// Acceleration setpoints
	if (!PX4_ISFINITE(setpoint.acceleration[0])) {
		_acc_sp(0) = 0.0f;
	} else {
		_acc_sp(0) = setpoint.acceleration[0];
	}

	if (!PX4_ISFINITE(setpoint.acceleration[1])) {
		_acc_sp(1) = 0.0f;
	} else {
		_acc_sp(1) = setpoint.acceleration[1];
	}

	if (!PX4_ISFINITE(setpoint.acceleration[2])) {
		_acc_sp(2) = 0.0f;
	} else {
		_acc_sp(2) = setpoint.acceleration[2];
	}



	// Yaw setpoints
	if (!PX4_ISFINITE(setpoint.yaw)) {
		// No yaw setpoint, keep as is
	} else {
		_yaw_sp = setpoint.yaw;
	}

	if (!PX4_ISFINITE(setpoint.yawspeed)) {
		_yawspeed_sp = 0.0f;
	} else {
		_yawspeed_sp = setpoint.yawspeed;
	}
}

bool HnuterPositionControl::update(const float dt)
{
	if (dt <= 0.0f) {
		return false;
	}

	if (!_inputValid()) {
		return false;
	}

	// Position control (PD)
	_positionControl();

	// Velocity control (PID)
	_velocityControl(dt);

	// Acceleration control
	_accelerationControl();

	return true;
}

void HnuterPositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.timestamp = hrt_absolute_time();

	// Position setpoint
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);

	// Velocity setpoint
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);

	// Acceleration setpoint
	local_position_setpoint.acceleration[0] = _acc_sp(0);
	local_position_setpoint.acceleration[1] = _acc_sp(1);
	local_position_setpoint.acceleration[2] = _acc_sp(2);

	// Thrust setpoint
	local_position_setpoint.thrust[0] = _thr_sp(0);
	local_position_setpoint.thrust[1] = _thr_sp(1);
	local_position_setpoint.thrust[2] = _thr_sp(2);

	// Yaw setpoint
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
}

void HnuterPositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	attitude_setpoint.timestamp = hrt_absolute_time();

	// Convert thrust vector to attitude setpoint
	Vector3f thrust_sp = _thr_sp;

	// Normalize thrust vector
	float thr_norm = thrust_sp.norm();

	if (thr_norm > 0.0f) {
		thrust_sp /= thr_norm;
	} else {
		thrust_sp = Vector3f(0.0f, 0.0f, -1.0f);
	}

	// Calculate desired roll and pitch angles
	float roll_setpoint = asinf(thrust_sp(1));
	float pitch_setpoint = atan2f(-thrust_sp(0), -thrust_sp(2));

	// Limit tilt
	roll_setpoint = math::constrain(roll_setpoint, -_lim_tilt, _lim_tilt);
	pitch_setpoint = math::constrain(pitch_setpoint, -_lim_tilt, _lim_tilt);

	// Create quaternion from euler angles
	Quatf q_setpoint(Eulerf(roll_setpoint, pitch_setpoint, _yaw_sp));

	// Set attitude setpoint
	q_setpoint.copyTo(attitude_setpoint.q_d);
	attitude_setpoint.thrust_body[0] = _thr_sp(0);
	attitude_setpoint.thrust_body[1] = _thr_sp(1);
	attitude_setpoint.thrust_body[2] = _thr_sp(2);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}

bool HnuterPositionControl::_inputValid()
{
	// Check if position and velocity are finite
	return PX4_ISFINITE(_pos(0)) && PX4_ISFINITE(_pos(1)) && PX4_ISFINITE(_pos(2)) &&
		PX4_ISFINITE(_vel(0)) && PX4_ISFINITE(_vel(1)) && PX4_ISFINITE(_vel(2)) &&
		PX4_ISFINITE(_yaw) &&
		PX4_ISFINITE(_pos_sp(0)) && PX4_ISFINITE(_pos_sp(1)) && PX4_ISFINITE(_pos_sp(2));
}

void HnuterPositionControl::_positionControl()
{
	// Position error and velocity error
	Vector3f pos_error = _pos_sp - _pos;
	Vector3f vel_error = _vel_sp - _vel;

	// Desired acceleration (PD control from hnuter_controller_frame)
	_acc_sp = _acc_sp + _Kp.emult(pos_error) + _Dp.emult(vel_error);

	// Limit desired acceleration
	float acc_xy_norm = _acc_sp.xy().norm();

	if (acc_xy_norm > _lim_vel_horizontal) {
		_acc_sp.xy() = _acc_sp.xy() * (_lim_vel_horizontal / acc_xy_norm);
	}

	_acc_sp(2) = math::constrain(_acc_sp(2), -_lim_vel_down, _lim_vel_up);
}

void HnuterPositionControl::_velocityControl(const float dt)
{
	// Velocity error
	Vector3f vel_error = _vel_sp - _vel;

	// Proportional term
	Vector3f vel_p = _gain_vel_p.emult(vel_error);

	// Integral term
	_vel_int += _gain_vel_i.emult(vel_error) * dt;

	// Limit integral term
	for (int i = 0; i < 3; i++) {
		_vel_int(i) = math::constrain(_vel_int(i), -1.0f, 1.0f);
	}

	// Derivative term (using acceleration estimate)
	Vector3f vel_d = _gain_vel_d.emult(-_vel_dot);

	// Total velocity control output
	Vector3f vel_control = vel_p + _vel_int + vel_d;

	// Add to acceleration setpoint
	_acc_sp += vel_control;
}

void HnuterPositionControl::_accelerationControl()
{
	// Assume mass and gravity
	const float mass = 4.2f; // From hnuter_controller_frame
	const float gravity = 9.81f;

	// World frame desired acceleration
	Vector3f acc_world = _acc_sp + Vector3f(0.0f, 0.0f, gravity);

	// Convert to body frame using yaw angle (simplified for yaw only)
	Dcmf R = Dcmf(Quatf(Eulerf(0.0f, 0.0f, _yaw)));
	Vector3f acc_body = R.transpose() * acc_world;

	// Calculate thrust vector
	_thr_sp = mass * acc_body;

	// Normalize thrust vector and apply limits
	float thr_norm = _thr_sp.norm();

	if (thr_norm > 0.0f) {
		// Apply thrust limits
		float thr_norm_limited = math::constrain(thr_norm, -_lim_thr_min, -_lim_thr_max);
		_thr_sp = _thr_sp * (thr_norm_limited / thr_norm);
	} else {
		// Default to vertical thrust
		_thr_sp = Vector3f(0.0f, 0.0f, mass * gravity);
	}
}

// Static member initialization
const trajectory_setpoint_s HnuterPositionControl::empty_trajectory_setpoint = {
	.timestamp = 0,
	.position = {NAN, NAN, NAN},
	.velocity = {NAN, NAN, NAN},
	.acceleration = {NAN, NAN, NAN},
	.jerk = {NAN, NAN, NAN},
	.yaw = NAN,
	.yawspeed = NAN
};
