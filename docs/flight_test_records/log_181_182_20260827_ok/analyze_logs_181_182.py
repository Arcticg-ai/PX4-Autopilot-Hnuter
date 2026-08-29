#!/usr/bin/env python3
"""Reproduce the log_181/log_182 OK-version archive and flight plots."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog
from scipy.signal import detrend, welch


DEFAULT_LOGS = (
    Path(
        "/home/hnuter/文档/QGroundControl Daily/Logs/"
        "程序开门，成功，两次/log_181_2026-8-27-17-28-02.ulg"
    ),
    Path(
        "/home/hnuter/文档/QGroundControl Daily/Logs/"
        "程序开门，成功，两次/log_182_2026-8-27-19-10-30.ulg"
    ),
)


def topic(ulog: ULog, name: str, multi_id: int = 0):
    return next(d for d in ulog.data_list if d.name == name and d.multi_id == multi_id)


def interp(dataset, field: str, timestamp_us: np.ndarray) -> np.ndarray:
    return np.interp(timestamp_us, dataset.data["timestamp"], dataset.data[field])


def previous(dataset, field: str, timestamp_us: np.ndarray) -> np.ndarray:
    source_time = np.asarray(dataset.data["timestamp"])
    index = np.searchsorted(source_time, timestamp_us, side="right") - 1
    index = np.clip(index, 0, len(source_time) - 1)
    return np.asarray(dataset.data[field])[index]


def quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = np.moveaxis(left, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(right, -1, 0)
    return np.stack(
        (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ),
        axis=-1,
    )


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    return q / np.maximum(np.linalg.norm(q, axis=1)[:, None], 1e-12)


def conjugate(q: np.ndarray) -> np.ndarray:
    result = q.copy()
    result[:, 1:] *= -1.0
    return result


def fixed_heading_pitch_deg(q: np.ndarray, heading: np.ndarray) -> np.ndarray:
    """Match HnuterControl::tiltForFixedHeading and return swing-vector Y."""
    q = normalize_quaternion(q)
    q_heading = np.column_stack(
        (np.cos(heading / 2.0), np.zeros_like(heading), np.zeros_like(heading), np.sin(heading / 2.0))
    )
    q_relative = quat_multiply(conjugate(q_heading), q)
    q_twist = np.column_stack(
        (q_relative[:, 0], np.zeros_like(heading), np.zeros_like(heading), q_relative[:, 3])
    )
    valid = np.linalg.norm(q_twist, axis=1) >= 1e-5
    q_twist[valid] = normalize_quaternion(q_twist[valid])
    q_twist[~valid] = np.array([1.0, 0.0, 0.0, 0.0])
    q_swing = normalize_quaternion(quat_multiply(q_relative, conjugate(q_twist)))
    q_swing[q_swing[:, 0] < 0.0] *= -1.0
    vector_norm = np.linalg.norm(q_swing[:, 1:], axis=1)
    angle = 2.0 * np.arctan2(vector_norm, q_swing[:, 0])
    pitch = angle * q_swing[:, 2] / np.maximum(vector_norm, 1e-12)
    return np.degrees(pitch)


def build_data(ulog: ULog) -> dict[str, np.ndarray]:
    control = topic(ulog, "hnuter_control_status")
    allocator = topic(ulog, "hnuter_allocator_status")
    attitude = topic(ulog, "vehicle_attitude")
    angular_velocity = topic(ulog, "vehicle_angular_velocity")
    manual = topic(ulog, "manual_control_setpoint")
    status = topic(ulog, "vehicle_status")
    outputs = topic(ulog, "actuator_outputs")
    local_position = topic(ulog, "vehicle_local_position")
    trajectory = topic(ulog, "trajectory_setpoint")

    timestamp_us = np.asarray(control.data["timestamp"], dtype=np.int64)
    time_s = (timestamp_us - timestamp_us[0]) / 1e6
    heading = np.asarray(control.data["rc_yaw_setpoint"], dtype=float)
    q_actual = np.column_stack([interp(attitude, f"q[{i}]", timestamp_us) for i in range(4)])
    q_target = np.column_stack([control.data[f"attitude_setpoint_q[{i}]"] for i in range(4)])
    force_x = np.asarray(control.data["force_body[0]"], dtype=float)
    force_y = np.asarray(control.data["force_body[1]"], dtype=float)
    data = {
        "timestamp_us": timestamp_us,
        "time_s": time_s,
        "nav_state": previous(status, "nav_state", timestamp_us).astype(int),
        "aux2": interp(manual, "aux2", timestamp_us),
        "aux4": interp(manual, "aux4", timestamp_us),
        "x_m": interp(local_position, "x", timestamp_us),
        "y_m": interp(local_position, "y", timestamp_us),
        "z_m": interp(local_position, "z", timestamp_us),
        "x_target_m": previous(trajectory, "position[0]", timestamp_us),
        "y_target_m": previous(trajectory, "position[1]", timestamp_us),
        "z_target_m": previous(trajectory, "position[2]", timestamp_us),
        "vx_target_mps": previous(trajectory, "velocity[0]", timestamp_us),
        "pitch_actual_deg": fixed_heading_pitch_deg(q_actual, heading),
        "pitch_target_deg": fixed_heading_pitch_deg(q_target, heading),
        "pitch_rate_dps": np.degrees(interp(angular_velocity, "xyz[1]", timestamp_us)),
        "roll_error_deg": np.degrees(np.asarray(control.data["attitude_error[0]"], dtype=float)),
        "pitch_error_so3_deg": np.degrees(np.asarray(control.data["attitude_error[1]"], dtype=float)),
        "yaw_error_deg": np.degrees(np.asarray(control.data["attitude_error[2]"], dtype=float)),
        "pitch_torque_nm": np.asarray(control.data["torque_command[1]"], dtype=float),
        "pitch_torque_p_nm": np.asarray(control.data["torque_p[1]"], dtype=float),
        "pitch_torque_d_nm": np.asarray(control.data["torque_d[1]"], dtype=float),
        "pitch_gravity_nm": np.asarray(control.data["torque_gravity[1]"], dtype=float),
        "pitch_bias_nm": np.asarray(control.data["torque_bias[1]"], dtype=float),
        "pitch_residual_nm": np.asarray(control.data["allocator_pitch_residual_nm"], dtype=float),
        "force_horizontal_n": np.hypot(force_x, force_y),
        "tail_pwm_us": interp(outputs, "output[4]", timestamp_us),
        "tail_force_requested_n": interp(allocator, "tail_force_requested", timestamp_us),
        "tail_force_commanded_n": interp(allocator, "tail_force_commanded", timestamp_us),
        "tail_force_error_n": interp(allocator, "tail_force_error", timestamp_us),
        "tail_limited": previous(allocator, "tail_limited", timestamp_us).astype(int),
        "tail_reversal_count": previous(allocator, "reversal_count", timestamp_us).astype(int),
    }
    data["pitch_error_deg"] = data["pitch_target_deg"] - data["pitch_actual_deg"]
    return data


def contiguous_windows(mask: np.ndarray, time_s: np.ndarray, minimum_duration_s: float) -> list[tuple[float, float]]:
    changes = np.diff(np.r_[False, mask, False].astype(np.int8))
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1) - 1
    return [
        (float(time_s[start]), float(time_s[end]))
        for start, end in zip(starts, ends)
        if time_s[end] - time_s[start] >= minimum_duration_s
    ]


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def finite_rms(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return rms(finite) if len(finite) else float("nan")


def finite_abs_max(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.max(np.abs(finite))) if len(finite) else float("nan")


def event_rows(log_id: str, data: dict[str, np.ndarray]) -> tuple[list[dict], list[tuple[float, float]]]:
    windows = contiguous_windows(data["aux4"] > 0.5, data["time_s"], 1.0)
    rows = []
    for event_number, (start, end) in enumerate(windows, 1):
        mask = (data["time_s"] >= start) & (data["time_s"] <= end)
        x_error = data["x_target_m"][mask] - data["x_m"][mask]
        y_error = data["y_target_m"][mask] - data["y_m"][mask]
        z_error = data["z_target_m"][mask] - data["z_m"][mask]
        reversal_delta = int(data["tail_reversal_count"][mask][-1] - data["tail_reversal_count"][mask][0])
        rows.append(
            {
                "log": log_id,
                "event": event_number,
                "start_s": round(start, 3),
                "end_s": round(end, 3),
                "duration_s": round(end - start, 3),
                "x_error_rms_m": round(finite_rms(x_error), 4),
                "x_error_abs_max_m": round(finite_abs_max(x_error), 4),
                "y_error_rms_m": round(finite_rms(y_error), 4),
                "z_error_rms_m": round(finite_rms(z_error), 4),
                "z_error_abs_max_m": round(finite_abs_max(z_error), 4),
                "horizontal_force_max_n": round(float(np.max(data["force_horizontal_n"][mask])), 3),
                "pitch_target_median_deg": round(float(np.median(data["pitch_target_deg"][mask])), 3),
                "pitch_actual_median_deg": round(float(np.median(data["pitch_actual_deg"][mask])), 3),
                "pitch_error_rms_deg": round(rms(data["pitch_error_so3_deg"][mask]), 3),
                "roll_error_rms_deg": round(rms(data["roll_error_deg"][mask]), 3),
                "yaw_error_rms_deg": round(rms(data["yaw_error_deg"][mask]), 3),
                "pitch_torque_median_nm": round(float(np.median(data["pitch_torque_nm"][mask])), 3),
                "tail_force_median_n": round(float(np.median(data["tail_force_commanded_n"][mask])), 3),
                "tail_pwm_min_us": round(float(np.min(data["tail_pwm_us"][mask])), 1),
                "tail_pwm_max_us": round(float(np.max(data["tail_pwm_us"][mask])), 1),
                "tail_limited_fraction": round(float(np.mean(data["tail_limited"][mask] != 0)), 5),
                "tail_reversal_count_delta": reversal_delta,
            }
        )
    return rows, windows


def position_release_rows(log_id: str, data: dict[str, np.ndarray], deadband: float = 0.08) -> list[dict]:
    active = np.abs(data["aux2"]) > deadband
    transitions = np.flatnonzero(active[1:] != active[:-1]) + 1
    releases = transitions[(~active[transitions]) & (data["nav_state"][transitions] == 2)]
    result = []
    for index in releases:
        before = max(index - 2, 0)
        after = min(index + 2, len(active) - 1)
        nearby = slice(max(0, index - 10), min(len(active), index + 11))
        adjacent_step = np.max(np.abs(np.diff(data["pitch_target_deg"][nearby])))
        result.append(
            {
                "log": log_id,
                "time_s": round(float(data["time_s"][index]), 3),
                "actual_pitch_deg": round(float(data["pitch_actual_deg"][index]), 3),
                "target_before_deg": round(float(data["pitch_target_deg"][before]), 3),
                "target_after_deg": round(float(data["pitch_target_deg"][after]), 3),
                "four_sample_target_change_deg": round(float(data["pitch_target_deg"][after] - data["pitch_target_deg"][before]), 4),
                "max_adjacent_target_step_deg": round(float(adjacent_step), 4),
            }
        )
    return result


def transition_rows(log_id: str, data: dict[str, np.ndarray]) -> list[dict]:
    nav = data["nav_state"]
    indices = np.flatnonzero(nav[1:] != nav[:-1]) + 1
    rows = []
    for index in indices:
        old_state = int(nav[index - 1])
        new_state = int(nav[index])
        if old_state not in (2, 14) or new_state not in (2, 14):
            continue
        time = float(data["time_s"][index])
        before = max(index - 2, 0)
        after = min(index + 2, len(nav) - 1)
        follow = (data["time_s"] >= time) & (data["time_s"] <= time + 5.0)
        rows.append(
            {
                "log": log_id,
                "time_s": round(time, 3),
                "from_mode": {2: "Position", 14: "Offboard"}[old_state],
                "to_mode": {2: "Position", 14: "Offboard"}[new_state],
                "target_change_deg": round(float(data["pitch_target_deg"][after] - data["pitch_target_deg"][before]), 3),
                "pitch_rate_abs_max_next_5s_dps": round(float(np.max(np.abs(data["pitch_rate_dps"][follow]))), 3),
                "tail_pwm_min_next_5s_us": round(float(np.min(data["tail_pwm_us"][follow])), 1),
                "tail_pwm_max_next_5s_us": round(float(np.max(data["tail_pwm_us"][follow])), 1),
            }
        )
    return rows


def oscillation_row(log_id: str, data: dict[str, np.ndarray]) -> dict:
    time_s = data["time_s"]
    stable = (
        (time_s >= 20.0)
        & (time_s <= time_s[-1] - 20.0)
        & (np.abs(data["aux2"]) <= 0.08)
        & (data["tail_limited"] == 0)
        & (np.abs(data["pitch_rate_dps"]) < 40.0)
    )
    changes = np.diff(np.r_[False, stable, False].astype(np.int8))
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1)
    longest = int(np.argmax(ends - starts))
    segment = slice(starts[longest], ends[longest])
    time = time_s[segment]
    rate = data["pitch_rate_dps"][segment]
    pitch = data["pitch_actual_deg"][segment]
    target = data["pitch_target_deg"][segment]
    fs = 1.0 / np.median(np.diff(time_s))
    frequency, power = welch(detrend(rate), fs=fs, nperseg=min(len(rate), 4096))
    band = (frequency >= 0.8) & (frequency <= 2.2)
    peak = np.flatnonzero(band)[np.argmax(power[band])]
    f_angle, p_angle = welch(detrend(pitch), fs=fs, nperseg=min(len(pitch), 4096))
    angle_band = (f_angle >= 1.0) & (f_angle <= 2.2)
    angle_band_rms = np.sqrt(np.trapz(p_angle[angle_band], f_angle[angle_band]))
    f_target, p_target = welch(detrend(target), fs=fs, nperseg=min(len(target), 4096))
    target_band = (f_target >= 1.0) & (f_target <= 2.2)
    target_band_rms = np.sqrt(np.trapz(p_target[target_band], f_target[target_band]))
    return {
        "log": log_id,
        "sample_duration_s": round(float(time[-1] - time[0]), 2),
        "dominant_pitch_rate_0p8_to_2p2_hz": round(float(frequency[peak]), 3),
        "pitch_angle_1_to_2p2_hz_rms_deg": round(float(angle_band_rms), 3),
        "pitch_target_1_to_2p2_hz_rms_deg": round(float(target_band_rms), 3),
        "pitch_rate_rms_dps": round(rms(rate), 3),
        "tail_pwm_std_us": round(float(np.std(data["tail_pwm_us"][segment])), 3),
    }


def safety_row(log_id: str, ulog: ULog, data: dict[str, np.ndarray]) -> dict:
    status = topic(ulog, "vehicle_status")
    battery = topic(ulog, "battery_status")
    failure = np.asarray(status.data["failure_detector_status"])
    failsafe = np.asarray(status.data["failsafe"])
    voltage = np.asarray(battery.data["voltage_v"], dtype=float)
    current = np.asarray(battery.data["current_a"], dtype=float)
    return {
        "log": log_id,
        "duration_s": round(float(data["time_s"][-1]), 3),
        "ulog_dropouts": len(ulog.dropouts),
        "in_flight_parameter_changes": len(ulog.changed_parameters),
        "failsafe_seen": int(np.any(failsafe != 0)),
        "failure_detector_seen": int(np.any(failure != 0)),
        "battery_voltage_min_v": round(float(np.nanmin(voltage)), 3),
        "battery_current_max_a": round(float(np.nanmax(current)), 3),
        "horizontal_force_max_n": round(float(np.max(data["force_horizontal_n"])), 3),
        "pitch_torque_abs_max_nm": round(float(np.max(np.abs(data["pitch_torque_nm"]))), 3),
        "tail_pwm_min_us": round(float(np.min(data["tail_pwm_us"])), 1),
        "tail_pwm_median_us": round(float(np.median(data["tail_pwm_us"])), 1),
        "tail_pwm_max_us": round(float(np.max(data["tail_pwm_us"])), 1),
        "tail_limited_fraction": round(float(np.mean(data["tail_limited"] != 0)), 6),
        "tail_reversal_count_final": int(data["tail_reversal_count"][-1]),
        "tail_force_error_rms_n": round(rms(data["tail_force_error_n"]), 4),
        "pitch_residual_rms_nm": round(rms(data["pitch_residual_nm"]), 4),
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_window(data: dict[str, np.ndarray], start: float, end: float, path: Path, title: str) -> None:
    mask = (data["time_s"] >= start) & (data["time_s"] <= end)
    time = data["time_s"][mask]
    fig, axes = plt.subplots(5, 1, figsize=(17, 14), sharex=True, constrained_layout=True)
    axes[0].plot(time, data["pitch_target_deg"][mask], label="Pitch target", linewidth=1.6)
    axes[0].plot(time, data["pitch_actual_deg"][mask], label="Pitch actual", linewidth=1.2)
    axes[0].set_ylabel("Pitch (deg)")
    axes[0].legend(loc="best")
    axes[1].plot(time, data["x_target_m"][mask], label="X target")
    axes[1].plot(time, data["x_m"][mask], label="X actual")
    axes[1].plot(time, data["vx_target_mps"][mask], label="X velocity target")
    axes[1].set_ylabel("X / Vx")
    axes[1].legend(loc="best")
    axes[2].plot(time, data["force_horizontal_n"][mask], label="Horizontal force")
    axes[2].plot(time, data["pitch_torque_nm"][mask], label="Pitch torque")
    axes[2].set_ylabel("N / N m")
    axes[2].legend(loc="best")
    axes[3].plot(time, data["tail_force_requested_n"][mask], label="Tail requested")
    axes[3].plot(time, data["tail_force_commanded_n"][mask], label="Tail commanded")
    axes[3].plot(time, (data["tail_pwm_us"][mask] - 1500.0) / 100.0, label="(PWM-1500)/100")
    axes[3].set_ylabel("Tail N / PWM")
    axes[3].legend(loc="best")
    axes[4].plot(time, data["aux4"][mask], label="AUX4 task")
    axes[4].step(time, data["nav_state"][mask], where="post", label="nav_state")
    axes[4].set_ylabel("Command / mode")
    axes[4].set_xlabel("Time from first Hnuter control sample (s)")
    axes[4].legend(loc="best")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    fig.suptitle(title)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="*", type=Path, default=list(DEFAULT_LOGS))
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    if len(args.logs) != 2:
        parser.error("exactly two ULogs are required")
    args.output.mkdir(parents=True, exist_ok=True)

    manifests = []
    all_events = []
    all_releases = []
    all_transitions = []
    oscillations = []
    safety = []

    for index, path in enumerate(args.logs, 181):
        log_id = f"log_{index}"
        ulog = ULog(str(path))
        data = build_data(ulog)
        parameters = [{"parameter": key, "value": value} for key, value in sorted(ulog.initial_parameters.items())]
        write_csv(args.output / f"{log_id}_parameters.csv", parameters)
        events, windows = event_rows(log_id, data)
        all_events.extend(events)
        all_releases.extend(position_release_rows(log_id, data))
        all_transitions.extend(transition_rows(log_id, data))
        oscillations.append(oscillation_row(log_id, data))
        safety.append(safety_row(log_id, ulog, data))

        plot_window(data, 0.0, float(data["time_s"][-1]), args.output / f"{log_id}_full_flight.png", f"{log_id} full flight")
        for event_number, (start, end) in enumerate(windows, 1):
            plot_window(
                data,
                max(0.0, start - 5.0),
                min(float(data["time_s"][-1]), end + 8.0),
                args.output / f"{log_id}_event_{event_number:02d}.png",
                f"{log_id} AUX4 event {event_number}",
            )

        manifests.append(
            {
                "log": log_id,
                "source_ulog": str(path),
                "source_ulog_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "firmware_git_hash": ulog.msg_info_dict.get("ver_sw"),
                "firmware_branch": ulog.msg_info_dict.get("ver_sw_branch"),
                "hardware": ulog.msg_info_dict.get("ver_hw"),
                "duration_s": round((ulog.last_timestamp - ulog.start_timestamp) / 1e6, 3),
                "dropouts": len(ulog.dropouts),
                "initial_parameter_count": len(ulog.initial_parameters),
                "in_flight_parameter_change_count": len(ulog.changed_parameters),
                "aux4_event_windows_s": [[round(a, 3), round(b, 3)] for a, b in windows],
            }
        )

    write_csv(args.output / "event_summary.csv", all_events)
    write_csv(args.output / "position_release_summary.csv", all_releases)
    write_csv(args.output / "mode_transition_summary.csv", all_transitions)
    write_csv(args.output / "oscillation_summary.csv", oscillations)
    write_csv(args.output / "safety_summary.csv", safety)
    (args.output / "analysis_manifest.json").write_text(
        json.dumps({"logs": manifests}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
