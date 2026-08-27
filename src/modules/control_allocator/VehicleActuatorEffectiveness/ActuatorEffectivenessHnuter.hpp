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

#include <cstdint>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/hnuter_allocator_status.h>
#include <uORB/topics/vehicle_control_mode.h>

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
	float applyTailReversalGuard(float desired_tail_force, hrt_abstime now);
	static float slewTowards(float current, float target, float max_delta);

	bool _collective_tilt_updated{true};
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessTilts _tilts;

	uint32_t _motors{};
	uint32_t _untiltable_motors{};
	int _first_tilt_idx{0};
	float _last_collective_tilt_control{NAN};

	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	bool _armed{false};
	uORB::Publication<hnuter_allocator_status_s> _hnuter_allocator_status_pub{ORB_ID(hnuter_allocator_status)};

	param_t _param_sim_gz_ec_min1{PARAM_INVALID};
	param_t _param_sim_gz_ec_max1{PARAM_INVALID};
	param_t _param_hntr_mot_hov{PARAM_INVALID};
	param_t _param_hntr_mot_expo{PARAM_INVALID};
	param_t _param_hntr_mass{PARAM_INVALID};
	param_t _param_hntr_max_arm_t{PARAM_INVALID};
	param_t _param_hntr_tail_t_pos{PARAM_INVALID};
	param_t _param_hntr_tail_t_neg{PARAM_INVALID};
	param_t _param_hntr_tail_exp_p{PARAM_INVALID};
	param_t _param_hntr_tail_exp_n{PARAM_INVALID};
	param_t _param_hntr_l1{PARAM_INVALID};
	param_t _param_hntr_l2{PARAM_INVALID};
	param_t _param_hntr_cg_z{PARAM_INVALID};
	param_t _param_hntr_s2_gear{PARAM_INVALID};
	param_t _param_hntr_roll_sign{PARAM_INVALID};
	param_t _param_hntr_tail_sign{PARAM_INVALID};
	param_t _param_hntr_tail_rev_t{PARAM_INVALID};
	param_t _param_hntr_t_rev_min{PARAM_INVALID};
	param_t _param_hntr_t_force_ref{PARAM_INVALID};
	param_t _param_hntr_t_slew_dn{PARAM_INVALID};
	param_t _param_hntr_t_rev_slew{PARAM_INVALID};
	param_t _param_hntr_t_rpm_max{PARAM_INVALID};
	param_t _param_pwm_main_min5{PARAM_INVALID};
	param_t _param_pwm_main_max5{PARAM_INVALID};
	param_t _param_hntr_s1_rate{PARAM_INVALID};
	param_t _param_hntr_s2_rate{PARAM_INVALID};
	float _sim_min_velocity{10.f};
	float _sim_max_velocity{1000.f};
	float _motor_hover_control{0.4f};
	float _motor_thrust_exponent{0.5f};
	bool _simulation_motor_model{false};
	float _mass{4.5f};
	float _max_thrust_per_arm{85.48f * 2.0f};
	float _max_tail_thrust_positive{12.78f};
	float _max_tail_thrust_negative{6.04f};
	float _tail_force_exponent_positive{0.55f};
	float _tail_force_exponent_negative{0.68f};
	float _l1{0.33f};
	float _l2{0.720f};
	float _cg_z{0.f};
	float _secondary_servo_gear_ratio{2.f};
	float _roll_torque_sign{1.f};
	float _tail_torque_sign{1.f};
	float _tail_reverse_delay_s{0.10f};
	float _tail_reverse_min_delay_s{0.02f};
	float _tail_dynamic_force_ref{12.8f};
	float _tail_slew_down_norm_s{10.f};
	float _tail_reverse_slew_norm_s{4.f};
	float _tail_rpm_max{20650.f};
	float _tail_pwm_min{900.f};
	float _tail_pwm_max{2000.f};
	float _primary_servo_rate_rad_s{4.7f};
	float _secondary_servo_rate_rad_s{4.7f};
	float _motor_constant{8.54858e-05f};

	hrt_abstime _last_servo_update{0};
	matrix::Vector<float, 4> _last_servo_sp{};
	float _tail_force_command{0.f};
	float _tail_force_requested{0.f};
	float _tail_reverse_dwell_required_s{0.f};
	float _tail_reverse_dwell_elapsed_s{0.f};
	hrt_abstime _tail_zero_timestamp{0};
	hrt_abstime _tail_last_update{0};
	int8_t _tail_last_nonzero_direction{0};
	int8_t _tail_pending_direction{0};
	uint8_t _tail_state{hnuter_allocator_status_s::TAIL_STATE_TRACKING};
	uint32_t _tail_reversal_count{0};
	bool _tail_limited{false};
	matrix::Vector<float, NUM_AXES> _unallocated_control{};
};
