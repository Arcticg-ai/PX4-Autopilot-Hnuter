#!/usr/bin/env python3
"""Compare the three 6S Hnuter battery configurations on airborne samples."""

from __future__ import annotations

import csv
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog


@dataclass(frozen=True)
class BatteryConfig:
    key: str
    label: str
    battery_mass_kg: float
    nominal_capacity_mah: float
    usable_capacity_mah: float
    total_mass_kg: float
    recommended_hover: float
    charged_cell_v: float
    pitch_p: float
    pitch_d: float
    log_paths: tuple[Path, ...]


BASE = Path("/home/hnuter/文档/QGroundControl Daily/Logs")
CONFIGS = (
    BatteryConfig(
        key="semisolid_22000",
        label="22 Ah semi-solid, 1.4 kg",
        battery_mass_kg=1.4,
        nominal_capacity_mah=22000.0,
        usable_capacity_mah=10000.0,
        total_mass_kg=6.2,
        recommended_hover=0.58,
        charged_cell_v=4.12,
        pitch_p=4.2,
        pitch_d=1.7,
        log_paths=(
            BASE / "固态电池飞行/log_184_2026-8-28-17-13-36.ulg",
            BASE / "固态电池飞行/log_185_2026-8-28-17-26-10.ulg",
        ),
    ),
    BatteryConfig(
        key="lipo_8000",
        label="Gens Ace LiPo 8 Ah, 1.0 kg",
        battery_mass_kg=1.0,
        nominal_capacity_mah=8000.0,
        usable_capacity_mah=7200.0,
        total_mass_kg=5.8,
        recommended_hover=0.58,
        charged_cell_v=4.18,
        pitch_p=4.2,
        pitch_d=1.5,
        log_paths=(BASE / "格式8000mAh电池，10分钟/log_186_2026-8-28-20-37-48.ulg",),
    ),
    BatteryConfig(
        key="liion_21700_8400",
        label="21700 Li-ion 8.4 Ah, 0.9 kg",
        battery_mass_kg=0.9,
        nominal_capacity_mah=8400.0,
        usable_capacity_mah=5600.0,
        total_mass_kg=5.7,
        recommended_hover=0.57,
        charged_cell_v=4.15,
        pitch_p=4.5,
        pitch_d=1.5,
        log_paths=(BASE / "飞起来8400mAh电池，7分钟/log_187_2026-8-28-21-04-56.ulg",),
    ),
)


def topic(ulog: ULog, name: str, multi_id: int = 0):
    return next(d for d in ulog.data_list if d.name == name and d.multi_id == multi_id)


def previous(dataset, field: str, timestamp_us: np.ndarray) -> np.ndarray:
    source_time = np.asarray(dataset.data["timestamp"])
    index = np.searchsorted(source_time, timestamp_us, side="right") - 1
    index = np.clip(index, 0, len(source_time) - 1)
    return np.asarray(dataset.data[field])[index]


def airborne_segments(mask: np.ndarray) -> list[tuple[int, int]]:
    changes = np.diff(np.r_[False, mask, False].astype(np.int8))
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1)
    return [(int(start), int(end)) for start, end in zip(starts, ends) if end - start > 2]


def moving_average(values: np.ndarray, samples: int) -> np.ndarray:
    if samples <= 1:
        return values.copy()
    kernel = np.ones(samples, dtype=float) / samples
    padded = np.pad(values, (samples // 2, samples - 1 - samples // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def extract_profile(config: BatteryConfig) -> tuple[dict[str, np.ndarray], list[dict], list[float]]:
    chunks = []
    source_logs = []
    internal_resistance = []
    elapsed_offset = 0.0
    capacity_offset = 0.0
    energy_offset = 0.0

    for path in config.log_paths:
        ulog = ULog(str(path))
        battery = topic(ulog, "battery_status")
        land = topic(ulog, "vehicle_land_detected")
        timestamp_us = np.asarray(battery.data["timestamp"], dtype=np.int64)
        landed = previous(land, "landed", timestamp_us) != 0
        voltage = np.asarray(battery.data["voltage_v"], dtype=float)
        current = np.maximum(np.asarray(battery.data["current_a"], dtype=float), 0.0)
        resistance = np.asarray(battery.data["internal_resistance_estimate"], dtype=float)
        valid_resistance = resistance[np.isfinite(resistance) & (resistance > 0.0)]
        internal_resistance.extend(valid_resistance.tolist())

        source_logs.append(
            {
                "path": str(path),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "firmware_git_hash": ulog.msg_info_dict.get("ver_sw"),
                "firmware_branch": ulog.msg_info_dict.get("ver_sw_branch"),
                "ulog_duration_s": round((ulog.last_timestamp - ulog.start_timestamp) / 1e6, 3),
                "dropouts": len(ulog.dropouts),
                "parameter_changes": len(ulog.changed_parameters),
                "discharged_mah_start": round(float(battery.data["discharged_mah"][0]), 3),
                "discharged_mah_end": round(float(battery.data["discharged_mah"][-1]), 3),
            }
        )

        for start, end in airborne_segments(~landed):
            segment_time = timestamp_us[start:end].astype(float) / 1e6
            segment_voltage = voltage[start:end]
            segment_current = current[start:end]
            if len(segment_time) < 3:
                continue
            dt = np.diff(segment_time, prepend=segment_time[0])
            dt[0] = float(np.median(np.diff(segment_time)))
            dt = np.clip(dt, 0.0, 2.0)
            elapsed = elapsed_offset + np.cumsum(dt)
            capacity = capacity_offset + np.cumsum(segment_current * dt / 3.6)
            power = segment_voltage * segment_current
            energy = energy_offset + np.cumsum(power * dt / 3600.0)
            chunks.append(
                {
                    "elapsed_s": elapsed,
                    "voltage_v": segment_voltage,
                    "current_a": segment_current,
                    "power_w": power,
                    "capacity_mah": capacity,
                    "energy_wh": energy,
                }
            )
            elapsed_offset = float(elapsed[-1])
            capacity_offset = float(capacity[-1])
            energy_offset = float(energy[-1])

    profile = {key: np.concatenate([chunk[key] for chunk in chunks]) for key in chunks[0]}
    return profile, source_logs, internal_resistance


def resample_profile(profile: dict[str, np.ndarray], interval_s: float = 1.0) -> dict[str, np.ndarray]:
    output_time = np.arange(0.0, profile["elapsed_s"][-1] + 1e-6, interval_s)
    output = {"elapsed_s": output_time}
    for key, values in profile.items():
        if key != "elapsed_s":
            output[key] = np.interp(output_time, profile["elapsed_s"], values)
    return output


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_single_metric(
    profiles: dict[str, dict[str, np.ndarray]],
    field: str,
    ylabel: str,
    title: str,
    path: Path,
    smooth_seconds: int = 1,
) -> None:
    fig, axis = plt.subplots(figsize=(16, 8), constrained_layout=True)
    for config in CONFIGS:
        profile = profiles[config.key]
        values = moving_average(profile[field], smooth_seconds)
        axis.plot(profile["elapsed_s"] / 60.0, values, linewidth=1.8, label=config.label)
    axis.set_xlabel("Cumulative airborne time (min)")
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    output_dir = Path(__file__).resolve().parent
    output_dir.mkdir(parents=True, exist_ok=True)
    profiles = {}
    manifest_logs = {}
    summary_rows = []
    parameter_rows = []

    for config in CONFIGS:
        raw_profile, source_logs, resistance = extract_profile(config)
        profile = resample_profile(raw_profile)
        profiles[config.key] = profile
        manifest_logs[config.key] = source_logs
        # Keep the plots and exported comparison profile uniformly sampled at 1 Hz,
        # but calculate extrema and totals from every original airborne sample.
        current = raw_profile["current_a"]
        voltage = raw_profile["voltage_v"]
        power = raw_profile["power_w"]
        used_mah = float(raw_profile["capacity_mah"][-1])
        used_wh = float(raw_profile["energy_wh"][-1])
        summary_rows.append(
            {
                "battery": config.key,
                "label": config.label,
                "battery_mass_kg": config.battery_mass_kg,
                "flight_time_min": round(float(raw_profile["elapsed_s"][-1] / 60.0), 3),
                "voltage_start_v": round(float(voltage[0]), 3),
                "voltage_median_v": round(float(np.median(voltage)), 3),
                "voltage_min_v": round(float(np.min(voltage)), 3),
                "current_mean_a": round(float(np.mean(current)), 3),
                "current_p95_a": round(float(np.percentile(current, 95)), 3),
                "current_max_a": round(float(np.max(current)), 3),
                "power_mean_w": round(float(np.mean(power)), 2),
                "power_max_w": round(float(np.max(power)), 2),
                "used_capacity_mah": round(used_mah, 1),
                "used_energy_wh": round(used_wh, 2),
                "nominal_capacity_used_fraction": round(used_mah / config.nominal_capacity_mah, 4),
                "usable_specific_energy_wh_per_kg": round(used_wh / config.battery_mass_kg, 2),
                "estimated_cell_resistance_mohm_median": round(float(np.median(resistance)) * 1000.0, 3),
                "estimated_pack_resistance_mohm_median": round(float(np.median(resistance)) * 6000.0, 3),
            }
        )
        parameter_rows.append(
            {
                "battery": config.key,
                "HNTR_MASS": config.total_mass_kg,
                "HNTR_MOT_HOV": config.recommended_hover,
                "HNTR_ATT_KR_P": config.pitch_p,
                "HNTR_ATT_D_P": config.pitch_d,
                "HNTR_ATT_I_P": 0.05,
                "HNTR_ATT_ILIM_P": 1.0,
                "HNTR_PITCH_BIAS": 0.10,
                "HNTR_TAU_P": 8.0,
                "BAT1_N_CELLS": 6,
                "BAT1_CAPACITY": int(config.usable_capacity_mah),
                "BAT1_V_CHARGED": config.charged_cell_v,
                "BAT1_V_EMPTY": 3.60,
                "BAT1_R_INTERNAL": -1,
                "safe_planned_flight_min": {
                    "semisolid_22000": "9-10",
                    "lipo_8000": "8.5-9",
                    "liion_21700_8400": "6-6.5",
                }[config.key],
            }
        )

    profile_rows = []
    for config in CONFIGS:
        profile = profiles[config.key]
        for index in range(len(profile["elapsed_s"])):
            profile_rows.append(
                {
                    "battery": config.key,
                    "elapsed_s": round(float(profile["elapsed_s"][index]), 3),
                    "voltage_v": round(float(profile["voltage_v"][index]), 4),
                    "current_a": round(float(profile["current_a"][index]), 4),
                    "power_w": round(float(profile["power_w"][index]), 3),
                    "capacity_used_mah": round(float(profile["capacity_mah"][index]), 3),
                    "energy_used_wh": round(float(profile["energy_wh"][index]), 4),
                }
            )

    write_csv(output_dir / "battery_comparison_summary.csv", summary_rows)
    write_csv(output_dir / "recommended_parameters.csv", parameter_rows)
    write_csv(output_dir / "battery_profiles_1hz.csv", profile_rows)
    (output_dir / "analysis_manifest.json").write_text(
        json.dumps(
            {
                "method": "Airborne samples only; semi-solid profile concatenates log184 then log185; capacity and energy are integrated from current and voltage.",
                "base_aircraft_mass_without_battery_kg": 4.8,
                "source_logs": manifest_logs,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    plot_single_metric(profiles, "voltage_v", "Pack voltage (V)", "Battery voltage comparison", output_dir / "01_voltage_comparison.png", 5)
    plot_single_metric(profiles, "current_a", "Current (A)", "Battery current comparison (5 s moving average)", output_dir / "02_current_comparison.png", 5)
    plot_single_metric(profiles, "capacity_mah", "Integrated capacity used (mAh)", "Airborne capacity consumption", output_dir / "03_capacity_comparison.png")
    plot_single_metric(profiles, "energy_wh", "Integrated energy used (Wh)", "Airborne energy consumption", output_dir / "04_energy_comparison.png")


if __name__ == "__main__":
    main()
