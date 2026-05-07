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
 * @file ActuatorEffectivenessFullyActuated.hpp
 *
 * Actuator effectiveness for fully actuated vehicles
 *
 * @author PX4 Development Team
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"

#include <px4_platform_common/module_params.h>

#include <uORB/topics/tiltrotor_extra_controls.h>
#include <uORB/Subscription.hpp>

class ActuatorEffectivenessFullyActuated : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessFullyActuated(ModuleParams *parent);
	virtual ~ActuatorEffectivenessFullyActuated() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	int numMatrices() const override { return 1; }

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		static_assert(MAX_NUM_MATRICES >= 1, "expecting at least 1 matrix");
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	void setFlightPhase(const FlightPhase &flight_phase) override;

	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

	const char *name() const override { return "Fully Actuated"; }

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

protected:
	bool _collective_tilt_updated{true};
	ActuatorEffectivenessRotors _mc_rotors;
	ActuatorEffectivenessTilts _tilts;

	uint32_t _motors{};
	uint32_t _untiltable_motors{};

	int _first_tilt_idx{0}; ///< applies to matrix 0

	float _last_collective_tilt_control{NAN};

	uORB::Subscription _tiltrotor_extra_controls_sub{ORB_ID(tiltrotor_extra_controls)};

	struct YawTiltSaturationFlags {
		bool tilt_yaw_pos;
		bool tilt_yaw_neg;
	};

	YawTiltSaturationFlags _yaw_tilt_saturation_flags{};

private:

	void updateParams() override;
};
