#!/usr/bin/env bash

set -euo pipefail

px4_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
external_repo=${HNUTER_EXTERNAL_CONTROL_REPO:-/home/hnuter/px4_ws_ros2}
experiment_variant=${HNUTER_IEBC_EXPERIMENT_VARIANT:-closed_loop}
variant_iebc_enable=""
variant_e_max=""
variant_require_barrier=""
variant_energy_reserve=""
variant_stop_distance=""
variant_brake_force=""
variant_ref_speed=""
variant_ref_accel=""
variant_ref_jerk=""
variant_observe_s=""
gz_headless=${HNUTER_GZ_HEADLESS:-1}
gz_keep_open=${HNUTER_GZ_KEEP_OPEN:-0}
run_dir=$(mktemp -d /tmp/hnuter_iebc_cube_contact.XXXXXX)
px4_pid=""
agent_pid=""

cleanup()
{
    trap - EXIT INT TERM

    if [[ -n "${px4_pid}" ]]; then
        # PX4 starts Gazebo as descendants of make.  The dedicated session
        # lets cleanup stop the complete simulator process group, including
        # error paths where make has already exited.
        kill -INT -- "-${px4_pid}" 2>/dev/null || true
        for _ in $(seq 1 20); do
            kill -0 "${px4_pid}" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM -- "-${px4_pid}" 2>/dev/null || true
        wait "${px4_pid}" 2>/dev/null || true
    fi

    if [[ -n "${agent_pid}" ]]; then
        kill -INT "${agent_pid}" 2>/dev/null || true
        for _ in $(seq 1 20); do
            kill -0 "${agent_pid}" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM "${agent_pid}" 2>/dev/null || true
        wait "${agent_pid}" 2>/dev/null || true
    fi

    echo "Run logs: ${run_dir}"
}
trap cleanup EXIT INT TERM

case "${experiment_variant}" in
    closed_loop)
        experiment_script=hnuter_external_controller_px4_position_iebc_simulation.py
        ;;
    nominal)
        experiment_script=hnuter_external_controller_px4_position_iebc_simulation.py
        variant_iebc_enable=0
        variant_require_barrier=0
        variant_observe_s=12.0
        ;;
    iebc_high)
        experiment_script=hnuter_external_controller_px4_position_iebc_simulation.py
        variant_iebc_enable=1
        variant_e_max=200.0
        variant_require_barrier=0
        variant_stop_distance=2.80
        variant_brake_force=40.0
        variant_ref_speed=2.00
        variant_ref_accel=12.0
        variant_ref_jerk=50.0
        variant_observe_s=12.0
        ;;
    iebc_active)
        experiment_script=hnuter_external_controller_px4_position_iebc_simulation.py
        variant_iebc_enable=1
        variant_e_max=80.0
        # A safe sub-limit run need not touch the energy boundary. Success is
        # based on non-negative barriers, zero QP slack and reaching HOLD.
        variant_require_barrier=0
        variant_energy_reserve=0.50
        variant_stop_distance=2.80
        variant_brake_force=40.0
        variant_ref_speed=2.00
        variant_ref_accel=12.0
        variant_ref_jerk=50.0
        # The recovery HOLD latch can occur near 10 s after release. Allow a
        # full one-second position-stability window after that latch instead
        # of timing out a physically settled run at the 12 s boundary.
        variant_observe_s=18.0
        ;;
    *)
        echo "Unknown HNUTER_IEBC_EXPERIMENT_VARIANT=${experiment_variant}; use closed_loop, nominal, iebc_high, or iebc_active" >&2
        exit 2
        ;;
esac

if [[ ! -f "${external_repo}/${experiment_script}" ]]; then
    echo "Missing ${experiment_variant} IEBC experiment controller: ${external_repo}/${experiment_script}" >&2
    exit 2
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy setup not found at /opt/ros/jazzy/setup.bash" >&2
    exit 2
fi

if [[ ! -f "${external_repo}/install/local_setup.bash" ]]; then
    echo "External-control ROS workspace has not been built: ${external_repo}/install" >&2
    exit 2
fi

cd "${px4_root}"
make px4_sitl_default

if [[ "${gz_headless}" == "1" ]]; then
    setsid env HEADLESS=1 make px4_sitl_default gz_hnuter_contact_hnuter_cube_contact \
        >"${run_dir}/px4.log" 2>&1 &
else
    # px4-rc.gzsim tests whether HEADLESS is unset, rather than whether its
    # value is zero.  Explicitly remove it so that `gz sim -g` is launched.
    setsid env -u HEADLESS make px4_sitl_default gz_hnuter_contact_hnuter_cube_contact \
        >"${run_dir}/px4.log" 2>&1 &
fi
px4_pid=$!

for _ in $(seq 1 60); do
    if grep -q "Startup script returned successfully" "${run_dir}/px4.log"; then
        break
    fi
    if ! kill -0 "${px4_pid}" 2>/dev/null; then
        echo "PX4/Gazebo exited during startup; see ${run_dir}/px4.log" >&2
        exit 3
    fi
    sleep 0.5
done

if ! grep -q "Startup script returned successfully" "${run_dir}/px4.log"; then
    echo "Timed out waiting for PX4/Gazebo; see ${run_dir}/px4.log" >&2
    exit 3
fi

MicroXRCEAgent udp4 -p 8888 >"${run_dir}/dds_agent.log" 2>&1 &
agent_pid=$!
sleep 1

set +e
(
    cd "${external_repo}"
    set +u
    source /opt/ros/jazzy/setup.bash
    source install/local_setup.bash
    set -u
    if [[ -n "${variant_iebc_enable}" ]]; then
        export HNUTER_IEBC_ENABLE="${variant_iebc_enable}"
    fi
    if [[ -n "${variant_e_max}" ]]; then
        export HNUTER_IEBC_E_MAX_J="${variant_e_max}"
    fi
    if [[ -n "${variant_require_barrier}" ]]; then
        export HNUTER_CUBE_REQUIRE_BARRIER_ACTIVE="${variant_require_barrier}"
    fi
    if [[ -n "${variant_energy_reserve}" ]]; then
        export HNUTER_IEBC_ENERGY_RESERVE_J="${variant_energy_reserve}"
    fi
    if [[ -n "${variant_stop_distance}" && -z "${HNUTER_IEBC_STOP_DISTANCE_M:-}" ]]; then
        export HNUTER_IEBC_STOP_DISTANCE_M="${variant_stop_distance}"
    fi
    if [[ -n "${variant_brake_force}" && -z "${HNUTER_IEBC_BRAKE_FORCE_CERT_N:-}" ]]; then
        export HNUTER_IEBC_BRAKE_FORCE_CERT_N="${variant_brake_force}"
    fi
    if [[ -n "${variant_ref_speed}" && -z "${HNUTER_IEBC_MAX_REF_SPEED_MPS:-}" ]]; then
        export HNUTER_IEBC_MAX_REF_SPEED_MPS="${variant_ref_speed}"
    fi
    if [[ -n "${variant_ref_accel}" && -z "${HNUTER_IEBC_MAX_REF_ACCEL_MPS2:-}" ]]; then
        export HNUTER_IEBC_MAX_REF_ACCEL_MPS2="${variant_ref_accel}"
    fi
    if [[ -n "${variant_ref_jerk}" && -z "${HNUTER_IEBC_MAX_REF_JERK_MPS3:-}" ]]; then
        export HNUTER_IEBC_MAX_REF_JERK_MPS3="${variant_ref_jerk}"
    fi
    if [[ -n "${variant_observe_s}" && -z "${HNUTER_CUBE_OBSERVE_S:-}" ]]; then
        export HNUTER_CUBE_OBSERVE_S="${variant_observe_s}"
    fi
    if [[ "${experiment_variant}" == "nominal" || "${experiment_variant}" == "iebc_high" || "${experiment_variant}" == "iebc_active" ]]; then
        export HNUTER_CUBE_FORCE_N="${HNUTER_CUBE_FORCE_N:-54.0}"
        export HNUTER_CUBE_PUSH_MPS="${HNUTER_CUBE_PUSH_MPS:-0.05}"
        export HNUTER_CUBE_RELEASE_MODE="${HNUTER_CUBE_RELEASE_MODE:-time}"
        export HNUTER_CUBE_RELEASE_TIME_S="${HNUTER_CUBE_RELEASE_TIME_S:-85.0}"
        export HNUTER_CUBE_MAX_PUSH_M="${HNUTER_CUBE_MAX_PUSH_M:-4.5}"
        export HNUTER_CUBE_MAX_PUSH_TIME_S="${HNUTER_CUBE_MAX_PUSH_TIME_S:-95.0}"
    fi
    HNUTER_IEBC_CUBE_SIM=1 \
    HNUTER_GZ_WORLD=hnuter_cube_contact \
        python3 "${experiment_script}"
) 2>&1 | tee "${run_dir}/controller.log"
controller_status=${PIPESTATUS[0]}
set -e

if [[ ${controller_status} -ne 0 ]]; then
    echo "Cube-contact experiment failed with status ${controller_status}" >&2
    exit "${controller_status}"
fi

echo "Cube-contact experiment completed successfully."

if [[ "${gz_keep_open}" == "1" && "${gz_headless}" == "0" ]]; then
    echo "Gazebo remains open for inspection; close the window or press Ctrl-C to stop."
    wait "${px4_pid}"
fi
