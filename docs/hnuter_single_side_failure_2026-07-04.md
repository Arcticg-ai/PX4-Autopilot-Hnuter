# Hnuter trajectory 5: single-side actuator failure

Date: 2026-07-04

## Scope

Trajectory 5 was added to:

`~/px4_ws_ros2/hnuter_sphere_surface_controller.py`

Press `5` after takeoff and stable hover. The controller:

1. Ramps the roll reference from 0 to 90 degrees in 10 seconds.
2. Holds the fault attitude for 10 seconds.
3. Fades the selected side to zero over 10 seconds.
4. Keeps the failed side at exactly zero and allocates collective/cyclic
   commands to the surviving side and reversible Motor5.

The default failed side is the left pod (Motor 3/4). It can be changed with
`single_side_fail_left` in:

`~/px4_ws_ros2/hnuter_sphere_tuning.json`

## Control changes

- The fault reference uses a smooth quintic roll profile.
- The surviving body-Y thrust axis is aligned with the requested world-force
  direction after the 90-degree attitude is established.
- Rotation about the surviving collective axis is treated as a free heading
  with angular-rate damping.
- The failed pod is faded to zero instead of being cut instantaneously.
- The surviving allocation includes the `r_z` parasitic roll moment that was
  missing from the first implementation.
- The trajectory, failure blend, target/actual state, and actuator commands
  are recorded in `hnuter_sphere_direct_<timestamp>.csv`.

## Closed-loop SITL result

Test command:

```bash
HEADLESS=1 make px4_sitl gz_hnuter
source /opt/ros/jazzy/setup.bash
python3 ~/px4_ws_ros2/hnuter_sphere_surface_controller.py
```

Then press `o`, wait for hover, and press `5`.

Representative logs:

- `~/px4_ws_ros2/hnuter_sphere_direct_1783133827.csv`
- `~/px4_ws_ros2/hnuter_sphere_direct_1783134872.csv`

Verified behavior:

- The reference transitions smoothly to the fault attitude.
- The failed Motor 3/4 commands reach zero.
- Altitude remains close to 1.30 m through most of the failure transition.
- Roll remains approximately 82 to 86 degrees near full failure.

Observed limitation:

- Horizontal position does not converge after complete pod failure. The
  aircraft accelerates mainly along world X.
- Increasing gains, changing the roll branch, roll precompensation, and using
  same-pod motor differential did not remove this coupling. The differential
  experiment reduced stability and was reverted.

## Controllability conclusion

With one pod removed, the conservative aggregate model has four independent
inputs: the surviving three-axis thrust vector and Motor5. The requested
stationary problem imposes three force constraints plus attitude-moment
constraints. The surviving pod's transverse force also creates a moment
through its lateral arm, so horizontal force and attitude torque cannot be
selected independently.

Consequently, this model can maintain a helicopter-like fault attitude and
altitude, but it cannot simultaneously guarantee fixed three-dimensional
position under complete single-pod failure. A true position hover requires at
least one additional independent control effect, such as a dedicated cyclic
moment actuator, aerodynamic control surface authority, or a validated
differential-thrust effectiveness model.

Trajectory 5 should therefore be treated as an experimental SITL
controllability test, not as a flight-ready failure-recovery mode.
