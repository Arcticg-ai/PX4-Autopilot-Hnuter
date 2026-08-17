#!/usr/bin/env python3
"""Plot the 2026-08-16 timed-release IEBC comparison in TRO style."""

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


def load_csv(path):
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f'No samples in {path}')
    return rows


def values(rows, key):
    result = []
    for row in rows:
        try:
            result.append(float(row[key]))
        except (KeyError, TypeError, ValueError):
            result.append(math.nan)
    return np.asarray(result, dtype=float)


def first_index(rows, stage):
    for index, row in enumerate(rows):
        if row['stage'] == stage:
            return index
    return None


def prepare(path):
    rows = load_csv(path)
    push_index = first_index(rows, 'PUSH_RAMP')
    if push_index is None:
        raise RuntimeError(f'{path} has no PUSH_RAMP stage')
    push_time = float(rows[push_index]['px4_time_s'])
    release_index = first_index(rows, 'RELEASE_OBSERVE')
    return {
        'path': path,
        'rows': rows,
        't': values(rows, 'px4_time_s') - push_time,
        'release_index': release_index,
        'release_time': (
            float(rows[release_index]['px4_time_s']) - push_time
            if release_index is not None else math.nan),
    }


def style_axes(axis):
    axis.grid(True, color='#D9D9D9', linewidth=0.45, alpha=0.70)
    axis.tick_params(direction='in', width=0.65, length=2.7, pad=2)
    for spine in axis.spines.values():
        spine.set_linewidth(0.65)


def shade_active(axis, active):
    barrier = values(active['rows'], 'iebc_barrier_active') > 0.5
    t = active['t']
    # Ignore the short initialization pulse and shade sustained intervention.
    indices = np.flatnonzero(barrier & (t > 1.0))
    if len(indices):
        axis.axvspan(t[indices[0]], t[indices[-1]], color=COLORS['yellow'],
                     alpha=0.13, linewidth=0.0, zorder=0)


def release_response(run):
    index = run['release_index']
    if index is None:
        return None
    rows = run['rows'][index:]
    t = values(rows, 'px4_time_s') - float(rows[0]['px4_time_s'])
    vx = values(rows, 'vehicle_enu_vx_mps')
    vy = values(rows, 'vehicle_enu_vy_mps')
    vz = values(rows, 'vehicle_enu_vz_mps')
    speed = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
    x = values(rows, 'vehicle_enu_x_m')
    y = values(rows, 'vehicle_enu_y_m')
    z = values(rows, 'vehicle_enu_z_m')
    displacement = np.sqrt((x - x[0]) ** 2 + (y - y[0]) ** 2 + (z - z[0]) ** 2)
    return t, speed, displacement


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--nominal', type=Path, required=True)
    parser.add_argument('--high', type=Path, required=True)
    parser.add_argument('--active', type=Path, required=True)
    parser.add_argument('--output-prefix', type=Path, required=True)
    args = parser.parse_args()

    nominal = prepare(args.nominal)
    high = prepare(args.high)
    active = prepare(args.active)
    release_time = active['release_time']

    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['Nimbus Roman', 'Times New Roman', 'Times', 'DejaVu Serif'],
        'font.size': 8.0,
        'axes.labelsize': 8.0,
        'axes.titlesize': 8.0,
        'xtick.labelsize': 7.0,
        'ytick.labelsize': 7.0,
        'legend.fontsize': 6.7,
        'axes.linewidth': 0.65,
        'lines.linewidth': 1.6,
        'pdf.fonttype': 42,
        'ps.fonttype': 42,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.025,
    })

    figure, axes = plt.subplots(3, 2, figsize=(7.16, 6.25))
    axes = axes.ravel()

    # (a) Same environment schedule, with nominal retained as a failed run.
    for run, color, label, style in (
            (nominal, COLORS['gray'], 'Nominal (yaw abort)', '--'),
            (high, COLORS['green'], 'IEBC 200 J', '-.'),
            (active, COLORS['blue'], 'IEBC 80 J', '-')):
        axes[0].plot(run['t'], values(run['rows'], 'contact_force_filtered_n'),
                     color=color, linestyle=style, linewidth=1.45, label=label)
    axes[0].axvline(nominal['t'][-1], color=COLORS['gray'], linewidth=0.9,
                    linestyle=':', label='Nominal safety abort')
    axes[0].set_ylabel('Filtered contact force (N)')
    axes[0].legend(loc='upper left', frameon=False, ncol=2, columnspacing=0.8)

    # (b) Active-run nominal and safe reference positions.
    axes[1].plot(active['t'], values(active['rows'], 'iebc_nominal_reference_position_m'),
                 color=COLORS['gray'], linestyle='--', linewidth=1.35,
                 label=r'$s_{nom}$')
    axes[1].plot(active['t'], values(active['rows'], 'iebc_safe_reference_position_m'),
                 color=COLORS['blue'], linewidth=1.65, label=r'$s_{safe}$')
    axes[1].set_ylabel('Interaction reference (m)')
    axes[1].legend(loc='upper left', frameon=False, ncol=2)

    # (c) Reference power limiting.
    axes[2].plot(active['t'], values(active['rows'], 'iebc_reference_power_nominal_w'),
                 color=COLORS['gray'], linestyle='--', linewidth=1.25,
                 label=r'$P_{ref,nom}$')
    axes[2].plot(active['t'], values(active['rows'], 'iebc_reference_power_safe_w'),
                 color=COLORS['blue'], linewidth=1.6, label=r'$P_{ref,safe}$')
    axes[2].plot(active['t'], values(active['rows'], 'iebc_allowed_power_w'),
                 color=COLORS['orange'], linestyle='-.', linewidth=1.25,
                 label=r'$P_{allow}$')
    axes[2].axhline(0.0, color=COLORS['black'], linewidth=0.55)
    axes[2].set_ylim(-1.6, 3.2)
    axes[2].set_ylabel('Reference power (W)')
    axes[2].legend(loc='upper left', frameon=False, ncol=3, columnspacing=0.7)

    # (d) Energy decomposition and the physical 80 J bound.
    axes[3].plot(active['t'], values(active['rows'], 'iebc_energy_j'),
                 color=COLORS['blue'], linewidth=1.65, label=r'$E_I$')
    axes[3].plot(active['t'], values(active['rows'], 'iebc_controller_storage_j'),
                 color=COLORS['purple'], linewidth=1.25, label=r'$V_c$')
    axes[3].plot(active['t'], values(active['rows'], 'iebc_storage_j'),
                 color=COLORS['yellow'], linewidth=1.25, label=r'$\bar S$')
    axes[3].plot(active['t'], values(active['rows'], 'iebc_kinetic_j'),
                 color=COLORS['green'], linewidth=1.05, label=r'$K_I$')
    axes[3].axhline(80.0, color=COLORS['orange'], linestyle='--',
                    linewidth=1.15, label=r'$E_{max}=80$ J')
    axes[3].set_ylabel('IEBC energy (J)')
    axes[3].legend(loc='center left', frameon=False, ncol=2, columnspacing=0.8)

    # (e) Sustained active reference velocity reduction.
    axes[4].plot(active['t'], values(active['rows'], 'iebc_nominal_reference_velocity_mps'),
                 color=COLORS['gray'], linestyle='--', linewidth=1.3,
                 label=r'$v_{nom}$')
    axes[4].plot(active['t'], values(active['rows'], 'iebc_safe_reference_velocity_mps'),
                 color=COLORS['blue'], linewidth=1.65, label=r'$v_{safe}$')
    axes[4].set_ylabel('Reference velocity (m/s)')
    axes[4].set_xlabel('Time from push onset (s)')
    axes[4].legend(loc='upper left', frameon=False, ncol=2)

    # (f) Release transient: speed and displacement use separate axes.
    displacement_axis = axes[5].twinx()
    for run, color, label in (
            (high, COLORS['green'], '200 J'),
            (active, COLORS['blue'], '80 J')):
        t, speed, displacement = release_response(run)
        axes[5].plot(t, speed, color=color, linewidth=1.6,
                     label=label + r' $\|v\|$')
        displacement_axis.plot(t, displacement, color=color, linewidth=1.15,
                               linestyle='--', label=label + r' $\|\Delta p\|$')
    axes[5].set_ylabel('Vehicle speed (m/s)')
    axes[5].set_xlabel('Time from scheduled release (s)')
    displacement_axis.set_ylabel('Displacement (m)')
    handles_a, labels_a = axes[5].get_legend_handles_labels()
    handles_b, labels_b = displacement_axis.get_legend_handles_labels()
    axes[5].legend(handles_a + handles_b, labels_a + labels_b,
                   loc='upper right', frameon=False, ncol=2, columnspacing=0.7)

    labels = ('(a)', '(b)', '(c)', '(d)', '(e)', '(f)')
    for index, (axis, label) in enumerate(zip(axes, labels)):
        style_axes(axis)
        axis.text(0.015, 0.955, label, transform=axis.transAxes,
                  ha='left', va='top', fontweight='bold')
        if index < 5:
            axis.axvline(release_time, color=COLORS['black'], linewidth=0.9,
                         linestyle=':', zorder=8)
            shade_active(axis, active)
            axis.set_xlim(-2.0, 92.5)
    style_axes(displacement_axis)

    figure.text(0.5, 0.995,
                'HNUTER timed-release IEBC validation under 54 N virtual resistance',
                ha='center', va='top', fontsize=8.6)
    figure.text(0.5, 0.973,
                'Yellow: sustained 80 J intervention; dotted line: scheduled release at 85 s',
                ha='center', va='top', fontsize=7.2)
    figure.tight_layout(rect=(0.01, 0.01, 0.995, 0.955), h_pad=0.8, w_pad=1.0)

    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output_prefix.with_suffix('.pdf'))
    figure.savefig(args.output_prefix.with_suffix('.png'), dpi=600)
    plt.close(figure)


if __name__ == '__main__':
    main()
