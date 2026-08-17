#!/usr/bin/env bash

set -euo pipefail

px4_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
external_repo=${HNUTER_EXTERNAL_CONTROL_REPO:-/home/hnuter/px4_ws_ros2}
run_dir=$(mktemp -d /tmp/hnuter_iebc_cube_contact.XXXXXX)
px4_pid=""
agent_pid=""

cleanup()
{
    if [[ -n "${px4_pid}" ]]; then
        # PX4 starts Gazebo as descendants of make.  The dedicated session
        # lets cleanup stop the complete simulator process group, including
        # error paths where make has already exited.
        kill -INT -- "-${px4_pid}" 2>/dev/null || true
        wait "${px4_pid}" 2>/dev/null || true
    fi

    if [[ -n "${agent_pid}" ]]; then
        kill -INT "${agent_pid}" 2>/dev/null || true
        wait "${agent_pid}" 2>/dev/null || true
    fi

    echo "Run logs: ${run_dir}"
}
trap cleanup EXIT INT TERM

if [[ ! -f "${external_repo}/hnuter_iebc_cube_contact_experiment.py" ]]; then
    echo "Missing IEBC experiment controller in ${external_repo}" >&2
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

setsid env HEADLESS=1 make px4_sitl_default gz_hnuter_contact_hnuter_cube_contact \
    >"${run_dir}/px4.log" 2>&1 &
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
    HNUTER_IEBC_CUBE_SIM=1 \
    HNUTER_GZ_WORLD=hnuter_cube_contact \
        python3 hnuter_iebc_cube_contact_experiment.py
) 2>&1 | tee "${run_dir}/controller.log"
controller_status=${PIPESTATUS[0]}
set -e

if [[ ${controller_status} -ne 0 ]]; then
    echo "Cube-contact experiment failed with status ${controller_status}" >&2
    exit "${controller_status}"
fi

echo "Cube-contact experiment completed successfully."
