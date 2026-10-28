# Direct Offboard to Position handover

## Ownership and references

`MulticopterRateControl` now informs `HnuterControl` about ownership on every
gyro cycle, including direct-actuator cycles where `update()` is bypassed.
On return, position/attitude integrals, manual position locks, RC attitude
initialization, and rate-controller/yaw-filter history are reset. Armed-time
history is preserved so an airborne return is not treated as a new takeoff.

The reference gap is bridged using measured position and full attitude. A new
RC attitude session starts from that attitude, without an automatic level-return
request. Auto/non-RC control may take ownership with a fresh reference. Cached
pre-exit trajectory/attitude setpoints and allocator residuals are not reused.

## Actuator history

The allocator observes mode callbacks even while torque control is bypassed.
It waits for both new torque and new thrust after the ownership edge, then seeds
its history from fresh direct motor/servo commands. Samples must be finite,
normalized, inside the direct session, no later than exit, and at most 100 ms
old. These are command-history samples, not measured actuator state.

The first servo slew step uses a normal 4 ms allowance, not the bypass duration.
Front motor commands and tail force demand transition over 0.5 s. Tail demand
is blended before the reversal guard, preserving neutral dwell. Unknown tail
direction requires a full neutral dwell. Allocation residuals account for the
blended front commands. A missing valid snapshot is reported; seamless recovery
cannot be guaranteed without recent command history.

## Validation and deployment boundaries

Run `bash test/hnuter_handover/run.sh` for the offline regression suite. It checks
invalid/stale snapshots, repeated ownership edges, integrator/reference reset,
RC/Auto ownership, servo slew, motor mapping inversion, blending and tail dwell.
These tests compile selected production methods against small dependency stubs;
they do not replace full PX4 builds or closed-loop simulation.

The matching no-delay Tail-SITL changes are tested with the hardware-v2 Python
control path through a documented Gazebo calibration adapter. The controller
repository contains the experiment harness and report. No aircraft parameters
are changed by these source edits. The aircraft must actually be flashed with
the matching CUAV 7 Nano build for this firmware-side handover fix to take effect;
updating the Python script alone is insufficient. No automatic flashing occurs.
