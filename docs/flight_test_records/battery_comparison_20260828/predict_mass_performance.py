#!/usr/bin/env python3
"""Predict Hnuter hover current and endurance as total mass changes."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


OUTPUT_DIR = Path(__file__).resolve().parent
SUMMARY_PATH = OUTPUT_DIR / "battery_comparison_summary.csv"
AIRFRAME_MASS_KG = 4.8
MASS_MIN_KG = 5.3
MASS_MAX_KG = 7.0
MASS_STEP_KG = 0.1
SCALING_BATTERY_MASS_MIN_KG = 0.5
SCALING_BATTERY_MASS_MAX_KG = 2.2
INDUCED_POWER_EXPONENT = 1.5

LABELS = {
    "semisolid_22000": "22 Ah semi-solid",
    "lipo_8000": "Gens Ace LiPo 8 Ah",
    "liion_21700_8400": "21700 Li-ion 8.4 Ah",
}

NOMINAL_CAPACITY_MAH = {
    "semisolid_22000": 22000.0,
    "lipo_8000": 8000.0,
    "liion_21700_8400": 8400.0,
}

REFERENCE_MASS_KG = {
    "semisolid_22000": 6.2,
    "lipo_8000": 5.8,
    "liion_21700_8400": 5.7,
}


def load_summary() -> list[dict[str, str]]:
    with SUMMARY_PATH.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def fit_current_coefficient(rows: list[dict[str, str]]) -> tuple[float, float]:
    masses = np.array([REFERENCE_MASS_KG[row["battery"]] for row in rows])
    currents = np.array([float(row["current_mean_a"]) for row in rows])
    basis = masses**INDUCED_POWER_EXPONENT
    coefficient = float(np.dot(basis, currents) / np.dot(basis, basis))
    predicted = coefficient * basis
    residual_sum = float(np.sum((currents - predicted) ** 2))
    total_sum = float(np.sum((currents - np.mean(currents)) ** 2))
    r_squared = 1.0 - residual_sum / total_sum
    return coefficient, r_squared


def plot_current(
    rows: list[dict[str, str]], masses: np.ndarray, coefficient: float, r_squared: float
) -> None:
    prediction = coefficient * masses**INDUCED_POWER_EXPONENT
    fig, axis = plt.subplots(figsize=(16, 8), constrained_layout=True)
    axis.fill_between(
        masses,
        prediction * 0.90,
        prediction * 1.10,
        color="tab:blue",
        alpha=0.15,
        label="Engineering uncertainty band (+/-10%)",
    )
    axis.plot(
        masses,
        prediction,
        color="tab:blue",
        linewidth=2.4,
        label=f"I = {coefficient:.3f} m^1.5, R^2 = {r_squared:.3f}",
    )
    for row in rows:
        key = row["battery"]
        axis.scatter(
            REFERENCE_MASS_KG[key],
            float(row["current_mean_a"]),
            s=95,
            label=f"Measured: {LABELS[key]}",
            zorder=4,
        )
    axis.axvspan(5.7, 6.2, color="grey", alpha=0.08, label="Measured mass range")
    axis.set_xlabel("Total takeoff mass (kg)")
    axis.set_ylabel("Predicted average airborne current (A)")
    axis.set_title("Predicted mass-current relationship")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper left")
    fig.savefig(OUTPUT_DIR / "05_mass_current_prediction.png", dpi=180)
    plt.close(fig)


def plot_endurance(rows: list[dict[str, str]], masses: np.ndarray) -> None:
    fig, axis = plt.subplots(figsize=(16, 8), constrained_layout=True)
    colors = {
        "semisolid_22000": "tab:blue",
        "lipo_8000": "tab:orange",
        "liion_21700_8400": "tab:green",
    }
    for row in rows:
        key = row["battery"]
        reference_mass = REFERENCE_MASS_KG[key]
        reference_time = float(row["flight_time_min"])
        prediction = reference_time * (reference_mass / masses) ** INDUCED_POWER_EXPONENT
        axis.fill_between(
            masses,
            prediction * 0.85,
            prediction * 1.15,
            color=colors[key],
            alpha=0.09,
        )
        axis.plot(masses, prediction, color=colors[key], linewidth=2.4, label=LABELS[key])
        axis.scatter(reference_mass, reference_time, color=colors[key], s=95, zorder=4)
    axis.axvspan(5.7, 6.2, color="grey", alpha=0.08, label="Measured mass range")
    axis.set_xlabel("Total takeoff mass (kg)")
    axis.set_ylabel("Predicted airborne duration (min)")
    axis.set_title("Predicted mass-endurance relationship at the observed usable energy")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper right")
    fig.savefig(OUTPUT_DIR / "06_mass_endurance_prediction.png", dpi=180)
    plt.close(fig)


def plot_capacity_scaling_endurance(rows: list[dict[str, str]], battery_masses: np.ndarray) -> None:
    total_masses = AIRFRAME_MASS_KG + battery_masses
    fig, axis = plt.subplots(figsize=(16, 8), constrained_layout=True)
    colors = {
        "semisolid_22000": "tab:blue",
        "lipo_8000": "tab:orange",
        "liion_21700_8400": "tab:green",
    }
    for row in rows:
        key = row["battery"]
        reference_battery_mass = float(row["battery_mass_kg"])
        reference_total_mass = REFERENCE_MASS_KG[key]
        reference_time = float(row["flight_time_min"])
        capacity_scale = battery_masses / reference_battery_mass
        power_scale = (total_masses / reference_total_mass) ** INDUCED_POWER_EXPONENT
        prediction = reference_time * capacity_scale / power_scale
        axis.fill_between(
            total_masses,
            prediction * 0.85,
            prediction * 1.15,
            color=colors[key],
            alpha=0.09,
        )
        axis.plot(total_masses, prediction, color=colors[key], linewidth=2.4, label=LABELS[key])
        axis.scatter(reference_total_mass, reference_time, color=colors[key], s=95, zorder=4)

    axis.axvspan(5.7, 6.2, color="grey", alpha=0.08, label="Measured total-mass range")
    axis.set_xlabel("Total takeoff mass (kg), including a 4.8 kg airframe")
    axis.set_ylabel("Predicted airborne duration (min)")
    axis.set_title("Predicted endurance when battery mass and capacity increase together")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper left")
    secondary = axis.secondary_xaxis(
        "top",
        functions=(
            lambda total_mass: total_mass - AIRFRAME_MASS_KG,
            lambda battery_mass: battery_mass + AIRFRAME_MASS_KG,
        ),
    )
    secondary.set_xlabel("Battery mass (kg)")
    fig.savefig(OUTPUT_DIR / "07_mass_endurance_capacity_scaling_prediction.png", dpi=180)
    plt.close(fig)


def plot_nominal_capacity_scaling_endurance(
    rows: list[dict[str, str]], battery_masses: np.ndarray
) -> None:
    total_masses = AIRFRAME_MASS_KG + battery_masses
    fig, axis = plt.subplots(figsize=(16, 8), constrained_layout=True)
    colors = {
        "semisolid_22000": "tab:blue",
        "lipo_8000": "tab:orange",
        "liion_21700_8400": "tab:green",
    }
    for row in rows:
        key = row["battery"]
        reference_battery_mass = float(row["battery_mass_kg"])
        reference_total_mass = REFERENCE_MASS_KG[key]
        reference_time = float(row["flight_time_min"])
        observed_capacity = float(row["used_capacity_mah"])
        nominal_usable_capacity = NOMINAL_CAPACITY_MAH[key] * 0.90
        capacity_scale = battery_masses / reference_battery_mass
        power_scale = (total_masses / reference_total_mass) ** INDUCED_POWER_EXPONENT
        prediction = (
            reference_time
            * (nominal_usable_capacity / observed_capacity)
            * capacity_scale
            / power_scale
        )
        axis.plot(
            total_masses,
            prediction,
            color=colors[key],
            linewidth=2.4,
            linestyle="--",
            label=f"{LABELS[key]} (90% nameplate)",
        )

    axis.axvspan(5.7, 6.2, color="grey", alpha=0.08, label="Measured total-mass range")
    axis.set_xlabel("Total takeoff mass (kg), including a 4.8 kg airframe")
    axis.set_ylabel("Theoretical airborne duration (min)")
    axis.set_title("Nameplate-capacity upper bound when battery mass and capacity increase together")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper left")
    secondary = axis.secondary_xaxis(
        "top",
        functions=(
            lambda total_mass: total_mass - AIRFRAME_MASS_KG,
            lambda battery_mass: battery_mass + AIRFRAME_MASS_KG,
        ),
    )
    secondary.set_xlabel("Battery mass (kg)")
    fig.savefig(OUTPUT_DIR / "08_mass_endurance_nameplate_upper_bound.png", dpi=180)
    plt.close(fig)


def write_predictions(
    rows: list[dict[str, str]], masses: np.ndarray, coefficient: float, r_squared: float
) -> None:
    source_by_key = {row["battery"]: row for row in rows}
    records = []
    for mass in masses:
        current = coefficient * mass**INDUCED_POWER_EXPONENT
        record = {
            "total_mass_kg": round(float(mass), 2),
            "predicted_average_current_a": round(float(current), 2),
            "current_lower_10pct_a": round(float(current * 0.90), 2),
            "current_upper_10pct_a": round(float(current * 1.10), 2),
        }
        for key in LABELS:
            reference_time = float(source_by_key[key]["flight_time_min"])
            reference_mass = REFERENCE_MASS_KG[key]
            duration = reference_time * (reference_mass / mass) ** INDUCED_POWER_EXPONENT
            record[f"{key}_duration_min"] = round(float(duration), 2)
        records.append(record)

    path = OUTPUT_DIR / "mass_performance_prediction.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)

    (OUTPUT_DIR / "mass_prediction_model.txt").write_text(
        "\n".join(
            (
                "Local Hnuter mass-performance model",
                f"average_current_a = {coefficient:.9f} * total_mass_kg^1.5",
                f"current_fit_r_squared = {r_squared:.6f}",
                "duration_i(m) = measured_duration_i * (measured_mass_i / m)^1.5",
                "capacity-scaling duration_i = measured_duration_i * (battery_mass / measured_battery_mass_i) * (measured_total_mass_i / total_mass)^1.5",
                f"airframe mass without battery = {AIRFRAME_MASS_KG:.1f} kg",
                "current uncertainty shown = +/-10% (engineering band)",
                "duration uncertainty shown = +/-15% (engineering band)",
                "calibration mass range = 5.7 to 6.2 kg",
                f"fixed-capacity prediction range = {MASS_MIN_KG:.1f} to {MASS_MAX_KG:.1f} kg",
                f"capacity-scaling battery-mass range = {SCALING_BATTERY_MASS_MIN_KG:.1f} to {SCALING_BATTERY_MASS_MAX_KG:.1f} kg",
                "valid only for the current Hnuter propulsion, hover-like flight profile, and 6S voltage range",
            )
        )
        + "\n",
        encoding="utf-8",
    )


def write_capacity_scaling_predictions(
    rows: list[dict[str, str]], battery_masses: np.ndarray, coefficient: float
) -> None:
    total_masses = AIRFRAME_MASS_KG + battery_masses
    records = []
    for battery_mass, total_mass in zip(battery_masses, total_masses):
        current = coefficient * total_mass**INDUCED_POWER_EXPONENT
        record = {
            "battery_mass_kg": round(float(battery_mass), 2),
            "total_mass_kg": round(float(total_mass), 2),
            "predicted_average_current_a": round(float(current), 2),
        }
        for row in rows:
            key = row["battery"]
            reference_battery_mass = float(row["battery_mass_kg"])
            reference_total_mass = REFERENCE_MASS_KG[key]
            capacity_scale = battery_mass / reference_battery_mass
            power_scale = (total_mass / reference_total_mass) ** INDUCED_POWER_EXPONENT
            duration = float(row["flight_time_min"]) * capacity_scale / power_scale
            usable_capacity = float(row["used_capacity_mah"]) * capacity_scale
            usable_energy = float(row["used_energy_wh"]) * capacity_scale
            nominal_90_capacity = NOMINAL_CAPACITY_MAH[key] * 0.90 * capacity_scale
            nominal_duration = duration * nominal_90_capacity / usable_capacity
            record[f"{key}_empirical_usable_capacity_mah"] = round(float(usable_capacity), 1)
            record[f"{key}_empirical_usable_energy_wh"] = round(float(usable_energy), 2)
            record[f"{key}_duration_min"] = round(float(duration), 2)
            record[f"{key}_nameplate_90pct_capacity_mah"] = round(float(nominal_90_capacity), 1)
            record[f"{key}_nameplate_upper_duration_min"] = round(float(nominal_duration), 2)
        records.append(record)

    path = OUTPUT_DIR / "capacity_scaling_prediction.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)


def main() -> None:
    rows = load_summary()
    masses = np.round(np.arange(MASS_MIN_KG, MASS_MAX_KG + MASS_STEP_KG / 2, MASS_STEP_KG), 2)
    battery_masses = np.round(
        np.arange(
            SCALING_BATTERY_MASS_MIN_KG,
            SCALING_BATTERY_MASS_MAX_KG + MASS_STEP_KG / 2,
            MASS_STEP_KG,
        ),
        2,
    )
    coefficient, r_squared = fit_current_coefficient(rows)
    plot_current(rows, masses, coefficient, r_squared)
    plot_endurance(rows, masses)
    plot_capacity_scaling_endurance(rows, battery_masses)
    plot_nominal_capacity_scaling_endurance(rows, battery_masses)
    write_predictions(rows, masses, coefficient, r_squared)
    write_capacity_scaling_predictions(rows, battery_masses, coefficient)


if __name__ == "__main__":
    main()
