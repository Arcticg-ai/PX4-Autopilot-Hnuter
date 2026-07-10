/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/rate_control/rate_control.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

class HnuterControl : public ModuleParams
{
public:
	struct Output {
		vehicle_thrust_setpoint_s thrust_setpoint{};
		vehicle_torque_setpoint_s torque_setpoint{};
		rate_ctrl_status_s rate_ctrl_status{};
		bool rate_ctrl_status_updated{false};
	};

	HnuterControl(ModuleParams *parent);

	void parametersUpdated() { ModuleParams::updateParams(); }

	bool update(const vehicle_angular_velocity_s &angular_velocity, const vehicle_control_mode_s &vehicle_control_mode,
		    bool landed, bool maybe_landed, RateControl &rate_control, AlphaFilter<float> &yaw_torque_filter,
		    float dt, const matrix::Vector3f &rates, Output &output);

	void reset();

private:
	static float forceToNormalizedThrust(float force, float hover_thrust, float hover_force, float max_force);

	uORB::Subscription _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};

	matrix::Vector3f _integral_pos_error{};
	matrix::Vector3f _integral_e_R{};
	matrix::Vector2f _xy_lock_position{};
	bool _xy_lock_initialized{false};
	bool _prev_armed{false};
	hrt_abstime _armed_time{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::HNTR_CTRL_MODE>) _param_hntr_ctrl_mode,
		(ParamFloat<px4::params::HNTR_MASS>) _param_hntr_mass,
		(ParamFloat<px4::params::HNTR_MAX_ARM_T>) _param_hntr_max_arm_t,
		(ParamFloat<px4::params::HNTR_MAX_TAIL_T>) _param_hntr_max_tail_t,
		(ParamFloat<px4::params::HNTR_L1>) _param_hntr_l1,
		(ParamFloat<px4::params::HNTR_L2>) _param_hntr_l2,
		(ParamFloat<px4::params::HNTR_HOV_THR>) _param_hntr_hov_thr,
		(ParamFloat<px4::params::HNTR_XY_P>) _param_hntr_xy_p,
		(ParamFloat<px4::params::HNTR_Z_P>) _param_hntr_z_p,
		(ParamFloat<px4::params::HNTR_XY_D>) _param_hntr_xy_d,
		(ParamFloat<px4::params::HNTR_Z_D>) _param_hntr_z_d,
		(ParamFloat<px4::params::HNTR_XY_I>) _param_hntr_xy_i,
		(ParamFloat<px4::params::HNTR_Z_I>) _param_hntr_z_i,
		(ParamFloat<px4::params::HNTR_ACC_XY>) _param_hntr_acc_xy,
		(ParamFloat<px4::params::HNTR_ACC_Z>) _param_hntr_acc_z,
		(ParamFloat<px4::params::HNTR_TILT_MAX>) _param_hntr_tilt_max,
		(ParamFloat<px4::params::HNTR_TO_SUP_T>) _param_hntr_to_sup_t,
		(ParamFloat<px4::params::HNTR_TO_LOCK_T>) _param_hntr_to_lock_t,
		(ParamFloat<px4::params::HNTR_TO_TILT>) _param_hntr_to_tilt,
		(ParamFloat<px4::params::HNTR_LOCK_TILT>) _param_hntr_lock_tilt,
		(ParamFloat<px4::params::HNTR_LOCK_ACC>) _param_hntr_lock_acc,
		(ParamFloat<px4::params::HNTR_LOCK_KP>) _param_hntr_lock_kp,
		(ParamFloat<px4::params::HNTR_ATT_KR_R>) _param_hntr_att_kr_r,
		(ParamFloat<px4::params::HNTR_ATT_KR_P>) _param_hntr_att_kr_p,
		(ParamFloat<px4::params::HNTR_ATT_KR_Y>) _param_hntr_att_kr_y,
		(ParamFloat<px4::params::HNTR_ATT_D_R>) _param_hntr_att_d_r,
		(ParamFloat<px4::params::HNTR_ATT_D_P>) _param_hntr_att_d_p,
		(ParamFloat<px4::params::HNTR_ATT_D_Y>) _param_hntr_att_d_y,
		(ParamFloat<px4::params::HNTR_TAU_R>) _param_hntr_tau_r,
		(ParamFloat<px4::params::HNTR_TAU_P>) _param_hntr_tau_p,
		(ParamFloat<px4::params::HNTR_TAU_Y>) _param_hntr_tau_y
	)
};
