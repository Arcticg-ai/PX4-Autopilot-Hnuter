#!/usr/bin/env python3
"""Fit the 2026-08-11 HNUTER reversible tail-motor bench data."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import least_squares


GRAVITY = 9.80665


def directional_curve(pwm: np.ndarray, maximum: float, exponent: float, direction: int) -> np.ndarray:
    if direction > 0:
        ratio = np.clip((pwm - 1500.0) / 500.0, 0.0, 1.0)
    else:
        ratio = np.clip((1500.0 - pwm) / 500.0, 0.0, 1.0)
    return maximum * ratio**exponent


def fit_direction(pwm: np.ndarray, value: np.ndarray, direction: int, maximum_bounds: tuple[float, float]) -> dict:
    initial = np.array([float(value.max()), 1.8])
    lower = np.array([maximum_bounds[0], 0.4])
    upper = np.array([maximum_bounds[1], 4.0])
    result = least_squares(
        lambda parameters: directional_curve(pwm, *parameters, direction) - value,
        initial,
        bounds=(lower, upper),
    )
    prediction = directional_curve(pwm, *result.x, direction)
    rmse = float(np.sqrt(np.mean((prediction - value) ** 2)))
    return {
        "maximum": float(result.x[0]),
        "exponent": float(result.x[1]),
        "neutral_pwm_us": 1500.0,
        "rmse": rmse,
    }


def load_data(path: Path) -> pd.DataFrame:
    data = pd.read_csv(path, sep="\t", encoding="gb18030", skiprows=11)
    numeric = ["TIME(ms)", "PWM(us)", "U(V)", "I(A)", "N(RPM)", "F(KG)", "T(N.M)"]
    for column in numeric:
        data[column] = pd.to_numeric(data[column], errors="coerce")
    data = data.dropna(subset=["TIME(ms)", "PWM(us)", "N(RPM)", "F(KG)"]).copy()
    data["time_s"] = (data["TIME(ms)"] - data["TIME(ms)"].iloc[0]) * 1e-3
    data["pwm_level_us"] = (data["PWM(us)"] / 50.0).round() * 50.0
    data["is_plateau"] = (data["PWM(us)"] - data["pwm_level_us"]).abs() <= 2.0
    data["signed_rpm"] = np.where(data["PWM(us)"] < 1500.0, -data["N(RPM)"], data["N(RPM)"])
    data.loc[(data["PWM(us)"] >= 1490.0) & (data["PWM(us)"] <= 1510.0), "signed_rpm"] = 0.0
    data["force_n"] = data["F(KG)"] * GRAVITY
    return data


def plateau_summary(data: pd.DataFrame) -> pd.DataFrame:
    stable = data[data["is_plateau"]].copy()
    stable["block"] = (
        (stable.index.to_series().diff().fillna(1) != 1)
        | (stable["pwm_level_us"] != stable["pwm_level_us"].shift())
    ).cumsum()
    blocks = []
    for _, block in stable.groupby("block"):
        if len(block) < 5:
            continue
        settled = block.iloc[min(2, len(block) - 1) :]
        blocks.append(
            {
                "pwm_us": float(block["pwm_level_us"].median()),
                "start_s": float(block["time_s"].iloc[0]),
                "end_s": float(block["time_s"].iloc[-1]),
                "samples": int(len(settled)),
                "voltage_v": float(settled["U(V)"].median()),
                "current_a": float(settled["I(A)"].median()),
                "rpm": float(settled["N(RPM)"].median()),
                "signed_rpm": float(settled["signed_rpm"].median()),
                "force_kg": float(settled["F(KG)"].median()),
                "force_n": float(settled["force_n"].median()),
                "torque_nm": float(settled["T(N.M)"].median()),
            }
        )
    summary = pd.DataFrame(blocks)
    # Multiple long neutral blocks are expected; combine all fully settled neutral samples.
    return summary.groupby("pwm_us", as_index=False).median(numeric_only=True).sort_values("pwm_us")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    data = load_data(args.input)
    summary = plateau_summary(data)
    reverse = summary[summary["pwm_us"] <= 1450.0].copy()
    forward = summary[summary["pwm_us"] >= 1600.0].copy()

    force_forward = fit_direction(
        forward["pwm_us"].to_numpy(), forward["force_n"].to_numpy(), 1, (5.0, 25.0)
    )
    force_reverse = fit_direction(
        reverse["pwm_us"].to_numpy(), -reverse["force_n"].to_numpy(), -1, (3.0, 15.0)
    )
    rpm_forward = fit_direction(
        forward["pwm_us"].to_numpy(), forward["rpm"].to_numpy(), 1, (10_000.0, 40_000.0)
    )
    rpm_reverse = fit_direction(
        reverse["pwm_us"].to_numpy(), reverse["rpm"].to_numpy(), -1, (10_000.0, 40_000.0)
    )

    omega_forward = forward["rpm"].to_numpy() * 2.0 * np.pi / 60.0
    omega_reverse = reverse["rpm"].to_numpy() * 2.0 * np.pi / 60.0
    force_forward_values = forward["force_n"].to_numpy()
    force_reverse_values = -reverse["force_n"].to_numpy()
    k_forward = float(np.dot(omega_forward**2, force_forward_values) / np.dot(omega_forward**2, omega_forward**2))
    k_reverse = float(np.dot(omega_reverse**2, force_reverse_values) / np.dot(omega_reverse**2, omega_reverse**2))

    torque_ratio_forward = float(
        np.median(np.abs(forward["torque_nm"].to_numpy()) / np.maximum(forward["force_n"].to_numpy(), 0.05))
    )
    torque_ratio_reverse = float(
        np.median(np.abs(reverse["torque_nm"].to_numpy()) / np.maximum(-reverse["force_n"].to_numpy(), 0.05))
    )

    sample_period = float(np.median(np.diff(data["time_s"])))
    results = {
        "source": str(args.input.resolve()),
        "sample_period_s": sample_period,
        "pwm_plateau_duration_s": 2.0,
        "neutral_pwm_us": 1500.0,
        "force_forward": force_forward,
        "force_reverse": force_reverse,
        "rpm_forward": rpm_forward,
        "rpm_reverse": rpm_reverse,
        "force_constant_forward_n_per_rad_s2": k_forward,
        "force_constant_reverse_n_per_rad_s2": k_reverse,
        "moment_constant_forward_m": torque_ratio_forward,
        "moment_constant_reverse_m": torque_ratio_reverse,
        "measured_forward_max_force_n": float(forward.loc[forward["pwm_us"].idxmax(), "force_n"]),
        "measured_reverse_max_force_n": float(-reverse.loc[reverse["pwm_us"].idxmin(), "force_n"]),
        "measured_forward_max_rpm": float(forward.loc[forward["pwm_us"].idxmax(), "rpm"]),
        "measured_reverse_max_rpm": float(reverse.loc[reverse["pwm_us"].idxmin(), "rpm"]),
        "dynamic_identification": (
            "The 5 Hz log cannot identify a 0.1-0.2 s reversal transient. "
            "Use 0.05 s as a conservative simulation time constant and retain the PX4 reversal guard."
        ),
    }

    summary.to_csv(args.output / "tail_motor_plateaus.csv", index=False)
    (args.output / "tail_motor_fit.json").write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")

    pwm_grid_forward = np.linspace(1500, 2000, 251)
    pwm_grid_reverse = np.linspace(1000, 1500, 251)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(data["time_s"], data["PWM(us)"], color="#444444")
    axes[0].set_ylabel("PWM (us)")
    axes[1].plot(data["time_s"], data["signed_rpm"], color="#1f77b4")
    axes[1].set_ylabel("Signed RPM")
    axes[2].plot(data["time_s"], data["force_n"], color="#d62728")
    axes[2].set_ylabel("Thrust (N)")
    axes[2].set_xlabel("Time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    fig.suptitle("HNUTER 1405 / 3-inch reversible tail motor bench data")
    fig.tight_layout()
    fig.savefig(args.output / "tail_motor_time_series.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 9), sharex=True)
    axes[0].scatter(summary["pwm_us"], summary["force_n"], color="#111111", label="steady median")
    axes[0].plot(
        pwm_grid_forward,
        directional_curve(pwm_grid_forward, force_forward["maximum"], force_forward["exponent"], 1),
        color="#d62728",
        label="forward fit",
    )
    axes[0].plot(
        pwm_grid_reverse,
        -directional_curve(pwm_grid_reverse, force_reverse["maximum"], force_reverse["exponent"], -1),
        color="#1f77b4",
        label="reverse fit",
    )
    axes[0].set_ylabel("Thrust (N)")
    axes[0].legend()
    axes[1].scatter(summary["pwm_us"], summary["signed_rpm"], color="#111111", label="steady median")
    axes[1].plot(
        pwm_grid_forward,
        directional_curve(pwm_grid_forward, rpm_forward["maximum"], rpm_forward["exponent"], 1),
        color="#d62728",
        label="forward fit",
    )
    axes[1].plot(
        pwm_grid_reverse,
        -directional_curve(pwm_grid_reverse, rpm_reverse["maximum"], rpm_reverse["exponent"], -1),
        color="#1f77b4",
        label="reverse fit",
    )
    axes[1].set_ylabel("Signed RPM")
    axes[1].set_xlabel("PWM (us)")
    axes[1].legend()
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.axvline(1500, color="#777777", linestyle="--", linewidth=1)
    fig.suptitle("Static PWM curves and directional fits")
    fig.tight_layout()
    fig.savefig(args.output / "tail_motor_static_fit.png", dpi=180)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(9, 6))
    axis.scatter(omega_forward**2, force_forward_values, label="forward", color="#d62728")
    axis.scatter(omega_reverse**2, force_reverse_values, label="reverse", color="#1f77b4")
    omega_sq_grid = np.linspace(0, max((omega_forward**2).max(), (omega_reverse**2).max()), 200)
    axis.plot(omega_sq_grid, k_forward * omega_sq_grid, color="#d62728", linestyle="--")
    axis.plot(omega_sq_grid, k_reverse * omega_sq_grid, color="#1f77b4", linestyle="--")
    axis.set_xlabel("Angular velocity squared ((rad/s)^2)")
    axis.set_ylabel("Thrust magnitude (N)")
    axis.grid(True, alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(args.output / "tail_motor_thrust_vs_omega2.png", dpi=180)
    plt.close(fig)

    print(json.dumps(results, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
