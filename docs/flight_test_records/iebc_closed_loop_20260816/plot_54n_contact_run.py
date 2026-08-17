#!/usr/bin/env python3
"""Create a TRO-style summary figure for one closed-loop contact run."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


COLORS = {
    'blue': '#0072B2',
    'orange': '#D55E00',
    'green': '#009E73',
    'purple': '#CC79A7',
    'sky': '#56B4E9',
    'yellow': '#E69F00',
    'gray': '#6B6B6B',
    'black': '#111111',
}


def load_csv(path: Path):
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f'No samples in {path}')
    return rows


def values(rows, key):
    result = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            value = math.nan
        result.append(value)
    return np.asarray(result, dtype=float)


def first_index(rows, predicate):
    for index, row in enumerate(rows):
        if predicate(row):
            return index
    return None


def style_axes(axis):
    axis.grid(True, color='#D9D9D9', linewidth=0.45, alpha=0.70)
    axis.tick_params(direction='in', width=0.65, length=2.7, pad=2)
    for spine in axis.spines.values():
        spine.set_linewidth(0.65)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_path', type=Path)
    parser.add_argument('--output-prefix', type=Path, required=True)
    args = parser.parse_args()

    rows = load_csv(args.csv_path)
    push_index = first_index(rows, lambda row: row['stage'] == 'PUSH_RAMP')
    settle_index = first_index(rows, lambda row: row['stage'] == 'LOAD_SETTLE')
    release_index = first_index(rows, lambda row: row['release_event_seen'] == '1')
    if push_index is None or settle_index is None or release_index is None:
        raise RuntimeError('Run does not contain LOAD_SETTLE, PUSH_RAMP, and release event')

    sample = rows[settle_index:]
    push_offset = push_index - settle_index
    release_offset = release_index - settle_index
    t_absolute = values(sample, 'px4_time_s')
    t = t_absolute - t_absolute[push_offset]
    release_time = t[release_offset]

    force_raw = values(sample, 'contact_force_raw_n')
    force_filtered = values(sample, 'contact_force_filtered_n')
    force_threshold = values(sample, 'release_force_threshold_n')
    commanded_force = float(force_threshold[release_offset])
    vehicle_x = values(sample, 'vehicle_enu_x_m')
    target_x = values(sample, 'target_enu_x_m')
    cube_x = values(sample, 'cube_world_x_m')
    vehicle_vx = values(sample, 'vehicle_enu_vx_mps')
    vehicle_vy = values(sample, 'vehicle_enu_vy_mps')
    vehicle_vz = values(sample, 'vehicle_enu_vz_mps')
    vehicle_speed = np.sqrt(vehicle_vx ** 2 + vehicle_vy ** 2 + vehicle_vz ** 2)
    yaw_error = values(sample, 'yaw_error_deg')
    energy = values(sample, 'iebc_energy_j')
    barrier = values(sample, 'iebc_barrier_j')
    storage = values(sample, 'iebc_storage_j')
    controller_storage = values(sample, 'iebc_controller_storage_j')
    emax = float(rows[0]['iebc_energy_j']) + float(rows[0]['iebc_barrier_j'])
    v_nominal = values(sample, 'iebc_nominal_reference_velocity_mps')
    v_safe = values(sample, 'iebc_safe_reference_velocity_mps')
    reference_error = values(sample, 'iebc_reference_error_m')
    raw_peak_before_release = float(np.nanmax(force_raw[:release_offset + 1]))
    raw_peak_all = float(np.nanmax(force_raw))

    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['Nimbus Roman', 'Times New Roman', 'Times', 'DejaVu Serif'],
        'font.size': 8.0,
        'axes.labelsize': 8.0,
        'axes.titlesize': 8.0,
        'xtick.labelsize': 7.0,
        'ytick.labelsize': 7.0,
        'legend.fontsize': 6.8,
        'axes.linewidth': 0.65,
        'lines.linewidth': 1.6,
        'pdf.fonttype': 42,
        'ps.fonttype': 42,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.025,
    })

    figure, axes = plt.subplots(3, 2, figsize=(7.16, 6.15), sharex=True)
    axes = axes.ravel()

    # (a) Contact force and release threshold.
    axes[0].plot(t, force_raw, color=COLORS['gray'], linewidth=0.75,
                 alpha=0.58, label=r'$F_{x,\mathrm{raw}}$')
    axes[0].plot(t, force_filtered, color=COLORS['blue'], linewidth=1.65,
                 label=r'$F_{x,\mathrm{LPF}}$')
    axes[0].plot(t, force_threshold, color=COLORS['orange'], linewidth=1.25,
                 linestyle='--', label=f'{commanded_force:.0f} N threshold')
    axes[0].set_ylim(-4.0, 72.0)
    axes[0].text(
        0.985, 0.82,
        f'raw peaks clipped: {raw_peak_before_release:.1f} N pre-release, '
        f'{raw_peak_all:.1f} N after release',
        transform=axes[0].transAxes, ha='right', va='top', fontsize=6.4,
        color=COLORS['gray'])
    axes[0].set_ylabel('Contact force (N)')
    axes[0].legend(loc='upper center', frameon=False, ncol=3, columnspacing=0.8)

    # (b) Vehicle, commanded, and door-proxy positions.
    axes[1].plot(t, target_x, color=COLORS['orange'], linestyle='--',
                 linewidth=1.35, label=r'$x_d$')
    axes[1].plot(t, vehicle_x, color=COLORS['blue'], linewidth=1.65,
                 label=r'$x$')
    axes[1].plot(t, cube_x, color=COLORS['green'], linewidth=1.45,
                 label=r'$x_{door}$')
    axes[1].set_ylabel('World-X position (m)')
    axes[1].legend(loc='upper center', frameon=False, ncol=3, columnspacing=0.9)

    # (c) Vehicle speed response.
    axes[2].plot(t, vehicle_vx, color=COLORS['blue'], linewidth=1.55,
                 label=r'$v_x$')
    axes[2].plot(t, vehicle_speed, color=COLORS['orange'], linewidth=1.35,
                 label=r'$\|v\|$')
    axes[2].axhline(0.0, color=COLORS['black'], linewidth=0.55)
    axes[2].set_ylabel('Vehicle velocity (m/s)')
    axes[2].legend(loc='upper center', frameon=False, ncol=2)

    # (d) Head-on geometry gate.
    axes[3].plot(t, yaw_error, color=COLORS['purple'], linewidth=1.55,
                 label='Yaw error')
    axes[3].axhline(5.0, color=COLORS['orange'], linewidth=1.1, linestyle='--')
    axes[3].axhline(-5.0, color=COLORS['orange'], linewidth=1.1, linestyle='--',
                    label=r'$\pm5^{\circ}$ gate')
    axes[3].set_ylabel('Yaw error (deg)')
    axes[3].legend(loc='upper center', frameon=False, ncol=2)

    # (e) IEBC energy bookkeeping before release/reset.
    axes[4].plot(t, energy, color=COLORS['blue'], linewidth=1.55,
                 label=r'$E_I$')
    axes[4].plot(t, barrier, color=COLORS['green'], linewidth=1.45,
                 label=r'$h_I=E_{max}-E_I$')
    axes[4].plot(t, storage, color=COLORS['yellow'], linewidth=1.15,
                 label=r'$\bar S$')
    axes[4].plot(t, controller_storage, color=COLORS['purple'], linewidth=1.15,
                 label=r'$V_c$')
    axes[4].axhline(emax, color=COLORS['orange'], linewidth=1.1, linestyle='--',
                    label=r'$E_{max}$')
    axes[4].set_ylabel('IEBC energy (J)')
    axes[4].set_xlabel('Time from push onset (s)')
    axes[4].legend(loc='lower left', frameon=False, ncol=2, columnspacing=0.8)

    # (f) Reference filter behavior.
    axes[5].plot(t, v_nominal, color=COLORS['gray'], linewidth=1.15,
                 linestyle='--', label=r'$v_{nom}$')
    axes[5].plot(t, v_safe, color=COLORS['blue'], linewidth=1.55,
                 label=r'$v_{safe}$')
    error_axis = axes[5].twinx()
    error_axis.plot(t, reference_error, color=COLORS['orange'], linewidth=1.3,
                    label=r'$e_{ref}$')
    error_axis.set_ylabel('Reference error (m)', color=COLORS['orange'])
    error_axis.tick_params(axis='y', colors=COLORS['orange'], direction='in',
                           width=0.65, length=2.7, pad=2)
    axes[5].set_ylabel('Reference velocity (m/s)')
    axes[5].set_xlabel('Time from push onset (s)')
    handles_a, labels_a = axes[5].get_legend_handles_labels()
    handles_b, labels_b = error_axis.get_legend_handles_labels()
    axes[5].legend(handles_a + handles_b, labels_a + labels_b,
                   loc='upper center', frameon=False, ncol=3, columnspacing=0.7)

    labels = ('(a)', '(b)', '(c)', '(d)', '(e)', '(f)')
    for axis, label in zip(axes, labels):
        style_axes(axis)
        axis.axvline(release_time, color=COLORS['black'], linewidth=0.9,
                     linestyle=':', zorder=8)
        axis.text(0.015, 0.955, label, transform=axis.transAxes,
                  ha='left', va='top', fontweight='bold')
    style_axes(error_axis)

    figure.text(0.5, 0.995,
                'HNUTER closed-loop IEBC contact experiment: '
                f'{commanded_force:.0f} N virtual resistance',
                ha='center', va='top', fontsize=8.6)
    figure.text(0.5, 0.973,
                f'Dotted line: release at t={release_time:.2f} s; '
                f'filtered force={force_filtered[release_offset]:.2f} N',
                ha='center', va='top', fontsize=7.2)
    figure.tight_layout(rect=(0.01, 0.01, 0.995, 0.955), h_pad=0.8, w_pad=0.9)

    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output_prefix.with_suffix('.pdf'))
    figure.savefig(args.output_prefix.with_suffix('.png'), dpi=600)
    plt.close(figure)


if __name__ == '__main__':
    main()
