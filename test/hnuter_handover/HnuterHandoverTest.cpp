/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/
#include <cassert>
#include <cmath>
#include <cstdio>
#include <mc_rate_control/HnuterControl.hpp>
#include <control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.hpp>
#include <uORB/uORBManager.hpp>
#include <uORB/topics/takeoff_status.h>

static hrt_abstime test_now = 10000000;
extern "C" hrt_abstime __wrap_hrt_absolute_time() { return test_now; }

template<typename T> static void setParam(const char *name, T value)
{
	const param_t handle = param_find(name);
	assert(handle != PARAM_INVALID);
	assert(param_set_no_notification(handle, &value) == 0);
}

static bool close(float a, float b) { return fabsf(a - b) < 1e-4f; }

struct HnuterHandoverTest {
	static void controller()
	{
		setParam<int32_t>("HNTR_RC_ATT_EN", 1);
		HnuterControl c(nullptr);
		RateControl rate;
		AlphaFilter<float> yaw;
		c.setControlActive(true, true, rate, yaw);
		c._armed_time = 123;
		c._prev_armed = true;
		c._velocity_integral = matrix::Vector3f{1.f, 2.f, 3.f};
		c._integral_e_R = matrix::Vector3f{1.f, 1.f, 1.f};
		c._rc_attitude_initialized = true;
		yaw.reset(0.8f);
		c.setControlActive(false, true, rate, yaw);
		assert(c._velocity_integral.norm() < 1e-6f && c._integral_e_R.norm() < 1e-6f);
		assert(!c._rc_attitude_initialized && close(yaw.getState(), 0.f));
		assert(c._armed_time == 123 && c._prev_armed);
		c.setControlActive(false, true, rate, yaw);
		c.setControlActive(true, true, rate, yaw);
		assert(c._resume_attitude_hold && c._reference_resume_timestamp == test_now);
		c._velocity_integral(0) = 0.2f;
		c.setControlActive(true, true, rate, yaw);
		assert(close(c._velocity_integral(0), 0.2f)); // no reset on steady active cycles
		c._velocity_integral.setZero();

		uORB::Publication<vehicle_attitude_s> att_pub{ORB_ID(vehicle_attitude)};
		uORB::Publication<vehicle_odometry_s> odom_pub{ORB_ID(vehicle_odometry)};
		uORB::Publication<trajectory_setpoint_s> traj_pub{ORB_ID(trajectory_setpoint)};
		uORB::Publication<manual_control_setpoint_s> manual_pub{ORB_ID(manual_control_setpoint)};
		uORB::Publication<takeoff_status_s> takeoff_pub{ORB_ID(takeoff_status)};
		vehicle_attitude_s att{};
		matrix::Quatf q{matrix::Eulerf{0.3f, -0.4f, 0.6f}};
		q.copyTo(att.q);
		att.timestamp = test_now;
		att_pub.publish(att);
		vehicle_odometry_s odom{};
		odom.position[0] = 1.f; odom.position[1] = 2.f; odom.position[2] = -3.f;
		odom.timestamp = test_now;
		odom_pub.publish(odom);
		trajectory_setpoint_s traj{};
		traj.timestamp = test_now - 1;
		traj.position[0] = 100.f;
		traj_pub.publish(traj);
		takeoff_status_s takeoff{};
		takeoff.timestamp = test_now;
		takeoff.takeoff_state = takeoff_status_s::TAKEOFF_STATE_FLIGHT;
		takeoff_pub.publish(takeoff);
		vehicle_control_mode_s mode{};
		mode.flag_armed = true;
		mode.flag_control_rates_enabled = true;
		mode.flag_control_attitude_enabled = true;
		mode.flag_control_position_enabled = true;
		mode.flag_control_manual_enabled = true;
		vehicle_angular_velocity_s gyro{};
		gyro.timestamp_sample = test_now;
		HnuterControl::Output out{};
		auto update = [&]() {
#ifdef HNUTER_HAS_GROUND_CONTACT
			assert(c.update(gyro, mode, false, false, false, rate, yaw, 0.004f, {}, out));
#else
			assert(c.update(gyro, mode, false, false, rate, yaw, 0.004f, {}, out));
#endif
		};
		update(); // no RC, no new trajectory: hold measured reference
		assert(c._resume_attitude_hold);
		assert(close(c._resume_position(0), 1.f));
		uORB::Subscription status_sub{ORB_ID(hnuter_control_status)};
		hnuter_control_status_s status{};
		assert(status_sub.copy(&status));
		assert(fabsf(matrix::Quatf{status.attitude_setpoint_q}.dot(q)) > 0.9999f);

		// A fresh Auto reference must release the temporary non-RC attitude hold.
		mode.flag_control_manual_enabled = false;
		traj.timestamp = test_now;
		traj.position[0] = 1.f; traj.position[1] = 2.f; traj.position[2] = -3.f;
		traj.yaw = -0.2f;
		traj_pub.publish(traj);
		update();
		assert(!c._resume_attitude_hold);
		assert(status_sub.copy(&status));
		assert(close(matrix::Eulerf{matrix::Quatf{status.attitude_setpoint_q}}.psi(), -0.2f));

		// A new bypass must reinitialize the RC reference even if aux switches stay high.
		c.setControlActive(false, true, rate, yaw);
		test_now += 4000;
		c.setControlActive(true, true, rate, yaw);
		manual_control_setpoint_s manual{};
		manual.timestamp = test_now; manual.valid = true; manual.aux3 = 1.f;
		manual_pub.publish(manual);
		mode.flag_control_manual_enabled = true;
		update();
		assert(c._rc_attitude_initialized && !c._rc_level_return_active);
		assert(fabsf(c._rc_attitude_q_sp.dot(q)) > 0.9999f);

		// An old post-exit sample must not remain a valid trajectory indefinitely.
		traj.timestamp = test_now;
		traj_pub.publish(traj);
		update();
		test_now += 600000;
		odom.position[0] = 5.f;
		odom_pub.publish(odom);
		update();
		assert(close(c._resume_position(0), 5.f));
		c.setControlActive(false, false, rate, yaw);
		assert(!c._prev_armed && c._armed_time == 0 && !c._control_was_bypassed);
		puts("PASS controller: edges, integrals, lifecycle, stale trajectory, RC and Auto references");
	}

	static void allocator()
	{
		setParam<int32_t>("CA_ROTOR_COUNT", 5);
		setParam<int32_t>("CA_SV_TL_COUNT", 4);
		ActuatorEffectivenessHnuter a(nullptr);
		ActuatorEffectiveness::Configuration config{};
		assert(a.getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
		vehicle_control_mode_s mode{};
		mode.flag_armed = true; mode.flag_control_offboard_enabled = true;
		mode.timestamp = test_now - 20000;
		a.updateControlMode(mode);
		uORB::Publication<actuator_motors_s> motor_pub{ORB_ID(actuator_motors)};
		uORB::Publication<actuator_servos_s> servo_pub{ORB_ID(actuator_servos)};
		actuator_motors_s motors{};
		motors.timestamp = test_now - 4000;
		motors.reversible_flags = 1u << 4;
		for (int i = 0; i < 4; ++i) { motors.control[i] = 0.3f + 0.05f * i; }
		motors.control[4] = 0.2f;
		motor_pub.publish(motors);
		actuator_servos_s servos{};
		servos.timestamp = motors.timestamp;
		for (int i = 0; i < 4; ++i) { servos.control[i] = -0.2f + 0.1f * i; }
		servo_pub.publish(servos);
		mode.flag_control_offboard_enabled = false; mode.flag_control_allocation_enabled = true;
		mode.timestamp = test_now;
		a.updateControlMode(mode);
		assert(a.handoverPending());
		ActuatorEffectiveness::ActuatorVector sp{};
		a.resumeActuatorState(test_now, sp);
		for (int i = 0; i < 4; ++i) {
			assert(close(sp(i), motors.control[i]));
			assert(close(a._last_servo_sp(i), servos.control[i]));
			assert(close(sp(a._first_tilt_idx + i), servos.control[i]));
		}
		assert(a._tail_last_nonzero_direction == 1 && a._tail_force_command > 0.f);
		assert(close(a.handoverProgress(test_now), 0.f));
		assert(close(a.handoverProgress(test_now + 250000), 0.5f));
		assert(close(a.handoverProgress(test_now + 500000), 1.f));
		ActuatorEffectiveness::ActuatorVector lo{}, hi{};
		lo.setAll(-1.f); hi.setAll(1.f);
		matrix::Vector<float, 6> wrench{};
		wrench(5) = -0.3f;
#ifdef HNUTER_HAS_GROUND_CONTACT
		wrench(1) = a._handover_tail_force / math::max(a._max_tail_thrust_positive, a._max_tail_thrust_negative);
#else
		wrench(1) = a._handover_tail_force * (a._cg_x + a._l2) / (a._max_tail_thrust * a._l2);
#endif
		a.updateSetpoint(wrench, 0, sp, lo, hi);
		for (int i = 0; i < 4; ++i) { assert(close(sp(i), motors.control[i])); }
		assert(sp(4) > 0.f); // same model inverse retains signed tail command
		assert(close(sp(4), motors.control[4]));
		for (int i = 0; i < 4; ++i) { assert(fabsf(sp(a._first_tilt_idx + i) - servos.control[i]) < 0.02f); }
		test_now += 250000;
		a.updateSetpoint(wrench, 0, sp, lo, hi);
		const auto midpoint = sp;
		test_now += 250000;
		a.updateSetpoint(wrench, 0, sp, lo, hi);
		for (int i = 0; i < 4; ++i) { assert(close(midpoint(i), 0.5f * (motors.control[i] + sp(i)))); }
		// Zero has no spin direction: neither positive nor negative may bypass dwell.
		a._handover_tail_neutral_until = test_now + 100000;
		assert(close(a.applyTailReversalGuard(-4.f, test_now), 0.f));
		assert(close(a.applyTailReversalGuard(4.f, test_now), 0.f));
		puts("PASS allocator: channel seed, servo first step, front first output, tail inverse, 0.5 s blend, dwell");
	}
};

static void snapshotPolicy()
{
	HnuterActuatorHandover h;
	float values[5]{};
	h.update(true, true, 100000);
	h.update(true, false, 120000);
	assert(!h.validSnapshot(0, 120000, values, 5));
	assert(!h.validSnapshot(99999, 120000, values, 5));
	assert(!h.validSnapshot(100000, 120000, values, 5));
	assert(h.validSnapshot(110000, 120000, values, 5)); // fresh zero is not "missing"
	assert(!h.validSnapshot(120001, 130000, values, 5)); // post-exit writer
	assert(!h.validSnapshot(110000, 109999, values, 5)); // future clock
	assert(!h.validSnapshot(110000, 210001, values, 5)); // >100 ms old
	values[3] = NAN; assert(!h.validSnapshot(110000, 120000, values, 5));
	values[3] = 1.1f; assert(!h.validSnapshot(110000, 120000, values, 5));
	h.update(false, false, 130000); assert(!h.pending());
	h.update(true, true, 140000);
	h.update(true, false, 150000); values[3] = 0.f;
	assert(!h.validSnapshot(110000, 150000, values, 5)); // prior session
	puts("PASS snapshot: pre-entry, future, post-exit, stale, NaN, bounds, disarm, repeated session");
}

int main()
{
	assert(uORB::Manager::initialize());
	snapshotPolicy();
	HnuterHandoverTest::controller();
	HnuterHandoverTest::allocator();
	puts("PASS all Hnuter handover offline regressions (no SITL or hardware started)");
	return 0;
}
