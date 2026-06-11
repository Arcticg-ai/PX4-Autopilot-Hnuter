# Hnuter Flight Log Fix - 2026-06-10

## Log Source

- Online log: `https://logs.px4.io/plot_app?log=3c23b6d0-3f7a-4265-9c37-953f1ca8e101`
- Local log: `log_127_2026-6-9-16-14-46.ulg`
- Vehicle: `CUAV_7_NANO`
- Airframe: `SYS_AUTOSTART=4051`, `CA_AIRFRAME=16`
- Log duration: about 70 s
- Dropouts: none

## Observations

- Attitude response was not a normal PID tuning issue. The log showed axis/sign problems:
  - Around 4 s, pitch setpoint was about `-11 deg`, while actual pitch moved to about `+13 deg`.
  - Around 8 s, pitch setpoint was about `-26 deg`, while actual pitch stayed positive at about `+14 deg`.
  - Maximum pitch reached about `82.8 deg`, and failure detector later reported pitch attitude failure.
- `PWM_MAIN_REV=1664` was active in the log. This means MAIN8, MAIN10 and MAIN11 were reversed:
  - MAIN8 = Servo1
  - MAIN10 = Servo3
  - MAIN11 = Servo4
  - Servo2 was not reversed, so the two-stage tilt geometry was no longer symmetric.
- Motor5 was configured reversible through `CA_R_REV=16`, with `PWM_MAIN_MIN5=900`, `PWM_MAIN_MAX5=2000`, `PWM_MAIN_DIS5=1450`.
- The logged rate D gains were still zero:
  - `MC_ROLLRATE_D=0`
  - `MC_PITCHRATE_D=0`
  - `MC_YAWRATE_D=0`
- Auto-disarm parameters in the log were valid:
  - `COM_DISARM_LAND=2`
  - `COM_DISARM_PRFLT=10`
  The late disarm behavior came mainly from land detection: `landed` became true only near 68.35 s.

## Root Causes

1. Tail pitch actuator sign was wrong.
   Motor5 is behind the CG. A positive vertical tail thrust creates negative body-y pitch torque, so the tail force must have the opposite sign to the pitch torque setpoint.

2. Saved `PWM_MAIN_REV` bits overrode the Hnuter allocator's expected geometry.
   The airframe previously used `param set-default PWM_MAIN_REV 0`, which does not clear saved values. The log still had `1664`, reversing several tilt servos.

3. Land detection was too slow for this vehicle behavior.
   The log only entered `landed` at about 68.35 s. With `COM_DISARM_LAND=2`, auto-disarm would only happen after that, unless the kill switch was used first.

4. Several stabilizing D/velocity terms were still zero on the real vehicle.
   The real airframe had not forced the updated values, so saved/default zeros remained active.

## Code Changes

- `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.cpp`
  - Changed Motor5 pitch allocation from `F3 = W[4] / l2` to `F3 = -W[4] / l2`.
  - Kept Motor5 independent from collective throttle; the four front motors carry vertical thrust.
  - Kept Motor5 signed output through `CA_R_REV=16`.

- `ROMFS/px4fmu_common/init.d/airframes/4051_gz_hnuter`
  - Forced `PWM_MAIN_REV=0` so saved actuator-test reverse bits cannot invert tilt servos.
  - Forced Motor5 reversible PWM center settings.
  - Forced updated D/velocity gains and land-disarm parameters.

- `ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter`
  - Matched land-disarm defaults to the real airframe.
  - Kept Motor5 reversible in SITL.

- Existing ESC calibration support remains:
  - `src/modules/commander/esc_calibration.cpp` sends `-1` as the low calibration point for reversible motors.
  - `src/lib/mixer_module/actuator_test.cpp` falls back to `CA_R_REV` if `actuator_motors.reversible_flags` is not yet available.

## Parameter Changes

Forced on Hnuter real hardware airframe:

```sh
param set PWM_MAIN_REV 0
param set CA_R_REV 16
param set PWM_MAIN_MIN5 900
param set PWM_MAIN_MAX5 2000
param set PWM_MAIN_DIS5 1450
param set PWM_MAIN_FAIL5 1450
param set PWM_MAIN_TIM0 400
param set PWM_MAIN_TIM1 400

param set MPC_THR_HOVER 0.50
param set MPC_THR_MIN 0.12
param set MPC_USE_HTE 0

param set MPC_XY_VEL_I_ACC 0.5
param set MPC_XY_VEL_D_ACC 4.0
param set MPC_Z_VEL_I_ACC 0.5
param set MPC_Z_VEL_D_ACC 6.0

param set MC_ROLLRATE_D 0.001
param set MC_PITCHRATE_D 0.001
param set MC_YAWRATE_D 0.001

param set COM_DISARM_PRFLT 10
param set COM_DISARM_LAND 1
param set LNDMC_TRIG_TIME 0.5
param set LNDMC_ROT_MAX 35
```

## Build Results

Commands run:

```sh
CCACHE_DIR=/tmp/ccache make cuav_7-nano_default
CCACHE_DIR=/tmp/ccache make px4_fmu-v6x_default
```

Results:

- `build/cuav_7-nano_default/cuav_7-nano_default.px4`
  - FLASH: `1845924 B / 1920 KB`, `93.89%`
- `build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`
  - FLASH: `1941988 B / 1920 KB`, `98.77%`

## Post-Flash Checks

After flashing and selecting/rebooting into airframe 4051, verify:

```sh
param show PWM_MAIN_REV
param show CA_R_REV
param show PWM_MAIN_MIN5
param show PWM_MAIN_MAX5
param show PWM_MAIN_DIS5
param show COM_DISARM_LAND
param show LNDMC_TRIG_TIME
```

Expected key values:

```text
PWM_MAIN_REV = 0
CA_R_REV = 16
PWM_MAIN_MIN5 = 900
PWM_MAIN_MAX5 = 2000
PWM_MAIN_DIS5 = 1450
COM_DISARM_LAND = 1
LNDMC_TRIG_TIME = 0.5
```

Bench test with propellers removed:

- Increase throttle: Motor1-4 should respond; Motor5 should stay near 1450 unless pitch torque is commanded.
- Push nose-up/pitch disturbance by hand while armed in a safe test mode: Motor5 should move in the direction that opposes the disturbance.
- Check MAIN8-11 servo directions after `PWM_MAIN_REV=0`. If any physical linkage is reversed, prefer fixing the Hnuter allocator sign/mapping or servo linkage, not reintroducing arbitrary saved reverse bits.
