#!/usr/bin/env python3
"""Create publication-quality plots for the Hnuter sphere experiment."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


COLORS = {
    "blue": "#0072B2",
    "orange": "#D55E00",
    "green": "#009E73",
    "purple": "#8E5AA9",
    "black": "#202124",
    "gray": "#7A7A7A",
    "light": "#D9D9D9",
}


def configure_style() -> None:
    mpl.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Nimbus Roman", "Liberation Serif", "DejaVu Serif"],
            "font.size": 8.5,
            "axes.labelsize": 9,
            "axes.titlesize": 9,
            "axes.linewidth": 0.8,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.fontsize": 8,
            "legend.frameon": False,
            "xtick.direction": "in",
            "ytick.direction": "in",
            "xtick.major.size": 3.5,
            "ytick.major.size": 3.5,
            "xtick.minor.size": 2.0,
            "ytick.minor.size": 2.0,
            "xtick.major.width": 0.7,
            "ytick.major.width": 0.7,
            "lines.linewidth": 1.25,
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.03,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def rotation_zyx_deg(euler_deg: np.ndarray) -> np.ndarray:
    """Return R_world_body for ZYX roll-pitch-yaw angles."""
    roll, pitch, yaw = np.deg2rad(euler_deg).T
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rotation = np.empty((len(euler_deg), 3, 3), dtype=float)
    rotation[:, 0, 0] = cy * cp
    rotation[:, 0, 1] = cy * sp * sr - sy * cr
    rotation[:, 0, 2] = cy * sp * cr + sy * sr
    rotation[:, 1, 0] = sy * cp
    rotation[:, 1, 1] = sy * sp * sr + cy * cr
    rotation[:, 1, 2] = sy * sp * cr - cy * sr
    rotation[:, 2, 0] = -sp
    rotation[:, 2, 1] = cp * sr
    rotation[:, 2, 2] = cp * cr
    return rotation


def so3_error(target: np.ndarray, actual: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return the rotation-vector error and geodesic angle in degrees."""
    error_rotation = np.einsum("nji,njk->nik", target, actual)
    trace = np.trace(error_rotation, axis1=1, axis2=2)
    angle = np.arccos(np.clip(0.5 * (trace - 1.0), -1.0, 1.0))
    vee = np.column_stack(
        (
            error_rotation[:, 2, 1] - error_rotation[:, 1, 2],
            error_rotation[:, 0, 2] - error_rotation[:, 2, 0],
            error_rotation[:, 1, 0] - error_rotation[:, 0, 1],
        )
    )
    scale = np.empty_like(angle)
    small = angle < 1e-6
    scale[small] = 0.5
    scale[~small] = angle[~small] / (2.0 * np.sin(angle[~small]))
    return np.rad2deg(vee * scale[:, None]), np.rad2deg(angle)


def uniform_derivative(time: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Differentiate noisy, nearly uniform telemetry after resampling."""
    dt = float(np.median(np.diff(time)))
    uniform_time = np.arange(time[0], time[-1] + 0.5 * dt, dt)
    uniform_values = np.column_stack(
        [np.interp(uniform_time, time, values[:, i]) for i in range(values.shape[1])]
    )
    window = min(21, len(uniform_time) - (1 - len(uniform_time) % 2))
    window = max(window, 5)
    derivative = np.column_stack(
        [
            savgol_filter(
                uniform_values[:, i],
                window_length=window,
                polyorder=3,
                deriv=1,
                delta=dt,
                mode="interp",
            )
            for i in range(values.shape[1])
        ]
    )
    return np.column_stack(
        [np.interp(time, uniform_time, derivative[:, i]) for i in range(values.shape[1])]
    )


def smooth(values: np.ndarray, window: int = 11) -> np.ndarray:
    usable = min(window, len(values) - (1 - len(values) % 2))
    if usable < 5:
        return values.copy()
    return savgol_filter(values, usable, 3, mode="interp")


def add_panel_label(axis: plt.Axes, label: str) -> None:
    axis.text(
        -0.105,
        1.02,
        label,
        transform=axis.transAxes,
        fontsize=10,
        fontweight="bold",
        va="bottom",
    )


def shade_surface(axis: plt.Axes, start: float, end: float) -> None:
    axis.axvspan(start, end, color=COLORS["blue"], alpha=0.055, linewidth=0)


def save_figure(figure: plt.Figure, output_dir: Path, stem: str) -> None:
    figure.savefig(output_dir / f"{stem}.pdf")
    figure.savefig(output_dir / f"{stem}.png", dpi=300)
    plt.close(figure)


def analyze(
    csv_path: Path,
    output_dir: Path,
    sphere_center: np.ndarray,
    sphere_radius: float,
    shell_radius: float,
    mass: float,
    gravity: float,
) -> dict[str, float]:
    data = pd.read_csv(csv_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    time = data["time_s"].to_numpy(dtype=float)
    time -= time[0]
    position = data[
        ["position_x_enu_m", "position_y_enu_m", "position_z_rel_m"]
    ].to_numpy(dtype=float)
    target_position = data[
        ["target_x_enu_m", "target_y_enu_m", "target_z_rel_m"]
    ].to_numpy(dtype=float)
    velocity = data[
        ["velocity_x_enu_mps", "velocity_y_enu_mps", "velocity_z_enu_mps"]
    ].to_numpy(dtype=float)

    actual_rotation = rotation_zyx_deg(
        data[["roll_deg", "pitch_deg", "yaw_deg"]].to_numpy(dtype=float)
    )
    target_rotation = rotation_zyx_deg(
        data[["target_roll_deg", "target_pitch_deg", "target_yaw_deg"]].to_numpy(
            dtype=float
        )
    )
    attitude_error_vector, attitude_error_angle = so3_error(
        target_rotation, actual_rotation
    )

    actual_radius = np.linalg.norm(position - sphere_center, axis=1)
    target_radius = np.linalg.norm(target_position - sphere_center, axis=1)
    inward_normal = sphere_center - position
    inward_normal /= np.linalg.norm(inward_normal, axis=1)[:, None]

    # For the current allocator signs (+X, -Y), logged W[0:3] is the force
    # vector expressed in body FLU. Rotate it to ENU before radial projection.
    body_force_flu = data[
        ["wrench_fx_body_n", "wrench_fy_body_n", "wrench_fz_body_n"]
    ].to_numpy(dtype=float)
    thrust_force_enu = np.einsum(
        "nij,nj->ni", actual_rotation, body_force_flu
    )
    noncontact_force_enu = thrust_force_enu + np.array(
        [0.0, 0.0, -mass * gravity]
    )
    commanded_normal_load = np.einsum(
        "ij,ij->i", noncontact_force_enu, inward_normal
    )

    acceleration = uniform_derivative(time, velocity)
    inertial_normal_force = mass * np.einsum(
        "ij,ij->i", acceleration, inward_normal
    )
    contact_force_estimate = commanded_normal_load - inertial_normal_force
    contact_force_smooth = smooth(contact_force_estimate)

    sphere_mode = data["auto_traj_mode"].eq("sphere").to_numpy()
    surface_mask = (
        sphere_mode
        & (target_radius <= shell_radius + 0.02)
        & (actual_radius <= sphere_radius + 0.45)
    )
    if not np.any(surface_mask):
        raise RuntimeError("No sphere-surface segment was detected in the log")

    surface_start = float(time[np.flatnonzero(surface_mask)[0]])
    surface_end = float(time[np.flatnonzero(surface_mask)[-1]])
    moving = surface_mask & (np.abs(data["target_pitch_deg"].to_numpy()) > 0.1)
    traverse_start = (
        float(time[np.flatnonzero(moving)[0]]) if np.any(moving) else surface_start
    )

    position_error = target_position - position
    position_error_norm = np.linalg.norm(position_error, axis=1)
    target_pitch = data["target_pitch_deg"].to_numpy(dtype=float)
    actual_pitch = data["continuous_test_pitch_deg"].to_numpy(dtype=float)
    pitch_error = target_pitch - actual_pitch

    configure_style()

    # Figure 1: estimated attachment/contact force and radial geometry.
    mask_plot = time >= surface_start
    fig, axes = plt.subplots(
        2, 1, figsize=(7.15, 4.7), sharex=True, constrained_layout=True
    )
    axes[0].plot(
        time[mask_plot],
        contact_force_estimate[mask_plot],
        color=COLORS["gray"],
        alpha=0.28,
        linewidth=0.65,
        label="Estimated contact force (raw)",
    )
    axes[0].plot(
        time[mask_plot],
        contact_force_smooth[mask_plot],
        color=COLORS["blue"],
        linewidth=1.5,
        label="Estimated contact force (smoothed)",
    )
    axes[0].plot(
        time[mask_plot],
        commanded_normal_load[mask_plot],
        color=COLORS["orange"],
        linestyle="--",
        linewidth=1.05,
        label="Commanded inward normal load",
    )
    axes[0].axhline(0.0, color=COLORS["black"], linewidth=0.6)
    axes[0].set_ylabel("Normal force [N]")
    axes[0].legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.18))
    axes[0].grid(True, color=COLORS["light"], linewidth=0.45, alpha=0.7)
    add_panel_label(axes[0], "(a)")

    axes[1].plot(
        time[mask_plot],
        actual_radius[mask_plot] - sphere_radius,
        color=COLORS["blue"],
        label="Actual vehicle-origin offset",
    )
    axes[1].plot(
        time[mask_plot],
        target_radius[mask_plot] - sphere_radius,
        color=COLORS["orange"],
        linestyle="--",
        label="Commanded offset",
    )
    axes[1].axhline(0.0, color=COLORS["black"], linewidth=0.6)
    axes[1].set_ylabel("Radial offset [m]")
    axes[1].set_xlabel("Mission time [s]")
    axes[1].legend(ncol=2, loc="upper right")
    axes[1].grid(True, color=COLORS["light"], linewidth=0.45, alpha=0.7)
    add_panel_label(axes[1], "(b)")
    for axis in axes:
        shade_surface(axis, surface_start, surface_end)
        axis.axvline(
            traverse_start,
            color=COLORS["purple"],
            linestyle=":",
            linewidth=0.9,
        )
        axis.minorticks_on()
        axis.set_xlim(surface_start, surface_end)
    axes[0].text(
        0.012,
        0.94,
        "surface operation",
        transform=axes[0].transAxes,
        color=COLORS["blue"],
        fontsize=7.5,
        va="top",
    )
    axes[0].text(
        traverse_start + 1.5,
        0.84,
        "traversal begins",
        transform=axes[0].get_xaxis_transform(),
        color=COLORS["purple"],
        fontsize=7.5,
        rotation=90,
        va="top",
    )
    save_figure(fig, output_dir, "figure_1_attachment_force")

    # Figure 2: Cartesian position tracking and component/norm errors.
    axis_names = ("$x_{ENU}$", "$y_{ENU}$", "$z_{rel}$")
    axis_colors = (COLORS["blue"], COLORS["orange"], COLORS["green"])
    fig, axes = plt.subplots(
        4, 1, figsize=(7.15, 7.0), sharex=True, constrained_layout=True
    )
    for i in range(3):
        axes[i].plot(
            time,
            target_position[:, i],
            color=COLORS["black"],
            linestyle="--",
            linewidth=1.0,
            label="Desired",
        )
        axes[i].plot(
            time,
            position[:, i],
            color=axis_colors[i],
            linewidth=1.25,
            label="Measured",
        )
        axes[i].set_ylabel(f"{axis_names[i]} [m]")
        axes[i].legend(ncol=2, loc="upper right")
        add_panel_label(axes[i], f"({chr(ord('a') + i)})")

    for i, label in enumerate(("$e_x$", "$e_y$", "$e_z$")):
        axes[3].plot(
            time,
            position_error[:, i],
            color=axis_colors[i],
            linewidth=1.0,
            label=label,
        )
    axes[3].plot(
        time,
        position_error_norm,
        color=COLORS["black"],
        linewidth=1.35,
        label=r"$\|\mathbf{e}_p\|_2$",
    )
    axes[3].set_ylabel("Position error [m]")
    axes[3].set_xlabel("Mission time [s]")
    axes[3].legend(ncol=4, loc="upper right")
    add_panel_label(axes[3], "(d)")
    for axis in axes:
        shade_surface(axis, surface_start, surface_end)
        axis.grid(True, color=COLORS["light"], linewidth=0.45, alpha=0.7)
        axis.minorticks_on()
    save_figure(fig, output_dir, "figure_2_position_tracking")

    # Figure 3: continuous pitch tracking and representation-free SO(3) error.
    fig, axes = plt.subplots(
        3, 1, figsize=(7.15, 5.7), sharex=True, constrained_layout=True
    )
    axes[0].plot(
        time,
        target_pitch,
        color=COLORS["black"],
        linestyle="--",
        linewidth=1.05,
        label="Desired pitch",
    )
    axes[0].plot(
        time,
        actual_pitch,
        color=COLORS["blue"],
        linewidth=1.3,
        label="Measured continuous pitch",
    )
    axes[0].set_ylabel("Pitch [deg]")
    axes[0].legend(ncol=2, loc="upper right")
    add_panel_label(axes[0], "(a)")

    for i, label in enumerate((r"$e_{R,x}$", r"$e_{R,y}$", r"$e_{R,z}$")):
        axes[1].plot(
            time,
            attitude_error_vector[:, i],
            color=axis_colors[i],
            linewidth=1.0,
            label=label,
        )
    axes[1].set_ylabel("Rotation-vector error [deg]")
    axes[1].legend(ncol=3, loc="upper right")
    add_panel_label(axes[1], "(b)")

    axes[2].plot(
        time,
        np.abs(pitch_error),
        color=COLORS["orange"],
        linewidth=1.0,
        label="Absolute continuous-pitch error",
    )
    axes[2].plot(
        time,
        attitude_error_angle,
        color=COLORS["black"],
        linewidth=1.35,
        label=r"SO(3) geodesic error $\theta_R$",
    )
    axes[2].set_ylabel("Attitude error [deg]")
    axes[2].set_xlabel("Mission time [s]")
    axes[2].legend(ncol=2, loc="upper right")
    add_panel_label(axes[2], "(c)")
    for axis in axes:
        shade_surface(axis, surface_start, surface_end)
        axis.grid(True, color=COLORS["light"], linewidth=0.45, alpha=0.7)
        axis.minorticks_on()
    save_figure(fig, output_dir, "figure_3_attitude_tracking")

    surface_force = contact_force_smooth[surface_mask]
    surface_pos_error = position_error[surface_mask]
    surface_pos_norm = position_error_norm[surface_mask]
    surface_attitude_error = attitude_error_angle[surface_mask]
    surface_pitch_error = np.abs(pitch_error[surface_mask])
    metrics = {
        "surface_start_s": surface_start,
        "surface_end_s": surface_end,
        "contact_force_mean_n": float(np.mean(surface_force)),
        "contact_force_std_n": float(np.std(surface_force)),
        "contact_force_p05_n": float(np.quantile(surface_force, 0.05)),
        "contact_force_p95_n": float(np.quantile(surface_force, 0.95)),
        "position_rmse_x_m": float(np.sqrt(np.mean(surface_pos_error[:, 0] ** 2))),
        "position_rmse_y_m": float(np.sqrt(np.mean(surface_pos_error[:, 1] ** 2))),
        "position_rmse_z_m": float(np.sqrt(np.mean(surface_pos_error[:, 2] ** 2))),
        "position_error_norm_rmse_m": float(
            np.sqrt(np.mean(surface_pos_norm**2))
        ),
        "position_error_norm_p95_m": float(np.quantile(surface_pos_norm, 0.95)),
        "attitude_geodesic_rmse_deg": float(
            np.sqrt(np.mean(surface_attitude_error**2))
        ),
        "attitude_geodesic_p95_deg": float(
            np.quantile(surface_attitude_error, 0.95)
        ),
        "pitch_error_mae_deg": float(np.mean(surface_pitch_error)),
        "pitch_error_p95_deg": float(np.quantile(surface_pitch_error, 0.95)),
    }
    pd.DataFrame([metrics]).to_csv(output_dir / "derived_metrics.csv", index=False)

    summary = f"""# Hnuter Sphere Run Analysis

- Input: `{csv_path}`
- Surface-analysis interval: `{surface_start:.2f}-{surface_end:.2f} s`
- Estimated contact force: `{metrics['contact_force_mean_n']:.2f} +/- {metrics['contact_force_std_n']:.2f} N`
- Estimated contact-force 5-95 percentile: `{metrics['contact_force_p05_n']:.2f}-{metrics['contact_force_p95_n']:.2f} N`
- Position RMSE `[x, y, z]`: `[{metrics['position_rmse_x_m']:.3f}, {metrics['position_rmse_y_m']:.3f}, {metrics['position_rmse_z_m']:.3f}] m`
- Position-error norm RMSE / P95: `{metrics['position_error_norm_rmse_m']:.3f} / {metrics['position_error_norm_p95_m']:.3f} m`
- SO(3) attitude-error RMSE / P95: `{metrics['attitude_geodesic_rmse_deg']:.2f} / {metrics['attitude_geodesic_p95_deg']:.2f} deg`
- Continuous-pitch MAE / P95: `{metrics['pitch_error_mae_deg']:.2f} / {metrics['pitch_error_p95_deg']:.2f} deg`

The log does not contain a Gazebo contact-wrench sensor. Contact force is
estimated from rigid-body normal dynamics:

`N_est = (F_thrust + F_gravity) dot n_in - m * a dot n_in`.

Acceleration is obtained from velocity after uniform resampling and a
Savitzky-Golay derivative. The radial offset is measured at the vehicle origin,
not at the collision-mesh contact point.
"""
    (output_dir / "analysis_summary.md").write_text(summary, encoding="utf-8")
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sphere-center", nargs=3, type=float, default=(14.0, 0.0, 11.5))
    parser.add_argument("--sphere-radius", type=float, default=10.0)
    parser.add_argument("--shell-radius", type=float, default=9.9)
    parser.add_argument("--mass", type=float, default=4.5)
    parser.add_argument("--gravity", type=float, default=9.81)
    args = parser.parse_args()

    metrics = analyze(
        csv_path=args.csv.resolve(),
        output_dir=args.output_dir.resolve(),
        sphere_center=np.asarray(args.sphere_center, dtype=float),
        sphere_radius=args.sphere_radius,
        shell_radius=args.shell_radius,
        mass=args.mass,
        gravity=args.gravity,
    )
    for key, value in metrics.items():
        print(f"{key}: {value:.6g}")


if __name__ == "__main__":
    main()
