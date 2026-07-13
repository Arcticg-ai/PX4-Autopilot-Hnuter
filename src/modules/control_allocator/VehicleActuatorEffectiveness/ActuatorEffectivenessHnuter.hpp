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
 * @file ActuatorEffectivenessHnuter.hpp
 *
 * Actuator effectiveness for Hnuter tiltrotor vehicles
 *
 * @author PX4 Development Team
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"

#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>

class ActuatorEffectivenessHnuter : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessHnuter(ModuleParams *parent);
	virtual ~ActuatorEffectivenessHnuter() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	int numMatrices() const override { return 1; }

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override;

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override;

	void setFlightPhase(const FlightPhase &flight_phase) override;

	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	const char *name() const override;

private:

	void updateParams() override;

	bool _collective_tilt_updated{true};
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessTilts _tilts;

	uint32_t _motors{};
	uint32_t _untiltable_motors{};
	int _first_tilt_idx{0};
	float _last_collective_tilt_control{NAN};

	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	bool _armed{false};
	bool _offboard_enabled{false};
	bool _prev_armed{false};
	hrt_abstime _armed_time{0};

	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	bool _landed{true};

	param_t _param_sim_gz_ec_min1{PARAM_INVALID};
	param_t _param_sim_gz_ec_max1{PARAM_INVALID};
	param_t _param_hntr_mot_hov{PARAM_INVALID};
	param_t _param_hntr_mot_expo{PARAM_INVALID};
	param_t _param_hntr_mass{PARAM_INVALID};
	param_t _param_hntr_max_arm_t{PARAM_INVALID};
	param_t _param_hntr_max_tail_t{PARAM_INVALID};
	param_t _param_hntr_l1{PARAM_INVALID};
	param_t _param_hntr_l2{PARAM_INVALID};
	param_t _param_hntr_roll_sign{PARAM_INVALID};
	param_t _param_hntr_tail_sign{PARAM_INVALID};
	param_t _param_hntr_tail_comp{PARAM_INVALID};
	param_t _param_hntr_to_sup_t{PARAM_INVALID};
	param_t _param_hntr_to_lock_t{PARAM_INVALID};
	param_t _param_hntr_to_tilt{PARAM_INVALID};
	param_t _param_hntr_lock_tilt{PARAM_INVALID};
	float _sim_min_velocity{10.f};
	float _sim_max_velocity{1000.f};
	float _motor_hover_control{0.4f};
	float _motor_thrust_exponent{0.5f};
	bool _simulation_motor_model{false};
	float _mass{4.5f};
	float _max_thrust_per_arm{85.48f * 2.0f};
	float _max_tail_thrust{85.48f};
	float _l1{0.33f};
	float _l2{0.664f};
	float _roll_torque_sign{1.f};
	float _tail_torque_sign{1.f};
	float _tail_collective_comp{0.f};
	float _takeoff_tilt_suppress_time_s{1.f};
	float _takeoff_xy_lock_time_s{3.f};
	float _takeoff_tilt_limit{0.349066f};
	float _xy_lock_tilt_limit{0.523599f};
	float _motor_constant{8.54858e-05f};

	hrt_abstime _last_servo_update{0};
	matrix::Vector<float, 4> _last_servo_sp{};
};
