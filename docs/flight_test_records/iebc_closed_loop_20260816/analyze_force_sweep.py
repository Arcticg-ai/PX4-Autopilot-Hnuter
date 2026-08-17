#!/usr/bin/env python3
"""Create the compact IEBC closed-loop force-sweep evidence table and plot."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


LOG_ROOT = Path('/home/hnuter/px4_ws_ros2/hnuter_logs/external_control')
OUT = Path(__file__).resolve().parent / 'evidence'

# These are deliberately selected A/B and final-boundary trials.  The full
# working directory contains exploratory runs with invalid yaw geometry or IEBC
# disabled and must not be mixed into the final force envelope.
RUNS = [
    ('1786811766', '6 N before pose fix', 'failed', 'wrong scoped-link yaw / proxy 3-D power'),
    ('1786812197', '6 N pose fix', 'complete', 'model pose selected correctly'),
    ('1786812627', '10 N at 35 mm/s', 'failed', 'dynamic contact yaw loss'),
    ('1786812802', '10 N at 20 mm/s', 'complete', 'quasi-static A/B'),
    ('1786812896', '12 N quasi-static', 'complete', 'pre world-yaw outer-loop'),
    ('1786814460', '7 N final default', 'complete', 'Emax 2.5 J final-code regression'),
    ('1786813791', '14 N final control', 'complete', 'continuous world-yaw outer-loop'),
    ('1786813890', '16 N final control', 'complete', 'continuous world-yaw outer-loop'),
    ('1786813992', '18 N boundary', 'failed', '0.8 m push limit; cube travel 17 mm'),
    ('1786814091', '17 N final control', 'complete', 'maximum verified load'),
]


def summarize(run_id: str, label: str, result: str, note: str) -> dict:
    path = LOG_ROOT / f'hnuter_iebc_cube_contact_closed_loop_{run_id}.csv'
    data = pd.read_csv(path)
    interaction = data[data.stage.isin(('LOAD_SETTLE', 'PUSH_RAMP'))]
    release = data[data.stage == 'RELEASE_OBSERVE']
    release_speed = float(np.linalg.norm(
        release[['vehicle_enu_vx_mps', 'vehicle_enu_vy_mps', 'vehicle_enu_vz_mps']].to_numpy(),
        axis=1).max()) if len(release) else np.nan
    release_displacement = np.nan
    if len(release):
        position = release[['vehicle_enu_x_m', 'vehicle_enu_y_m', 'vehicle_enu_z_m']].to_numpy()
        release_displacement = float(np.linalg.norm(position - position[0], axis=1).max())
    emax = float((interaction.iebc_energy_j + interaction.iebc_barrier_j).median())
    return {
        'run_id': run_id,
        'label': label,
        'result': result,
        'virtual_load_n': float(data.virtual_force_n.max()),
        'emax_j': emax,
        'max_filtered_contact_n': float(interaction.contact_force_filtered_n.max()),
        'max_abs_yaw_error_deg': float(interaction.yaw_error_deg.abs().max()),
        'max_interaction_energy_j': float(interaction.iebc_energy_j.max()),
        'min_barrier_j': float(interaction.iebc_barrier_j.min()),
        'max_qp_slack_w': float(interaction.iebc_qp_slack_w.max()),
        'max_cube_breakaway_m': float(interaction.cube_breakaway_m.max()),
        'peak_release_speed_mps': release_speed,
        'peak_release_displacement_m': release_displacement,
        'note': note,
        'source_csv': str(path),
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    table = pd.DataFrame(summarize(*run) for run in RUNS)
    table.to_csv(OUT / 'force_sweep_summary.csv', index=False, float_format='%.6f')

    final = table[table.run_id.isin(
        ('1786813791', '1786813890', '1786813992', '1786814091'))].sort_values('virtual_load_n')
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), constrained_layout=True)
    colors = ['#2a9d8f' if result == 'complete' else '#e76f51' for result in final.result]
    axes[0].bar(final.virtual_load_n.astype(str), final.max_filtered_contact_n, color=colors)
    axes[0].plot(final.virtual_load_n.astype(str), final.virtual_load_n, 'k--', label='virtual load')
    axes[0].set_xlabel('Virtual load [N]')
    axes[0].set_ylabel('Peak filtered contact force [N]')
    axes[0].set_title('Final head-on capacity sweep')
    axes[0].legend()

    completed = final[final.result == 'complete']
    axes[1].plot(completed.virtual_load_n, completed.peak_release_speed_mps, 'o-', label='speed [m/s]')
    axes[1].plot(completed.virtual_load_n, completed.peak_release_displacement_m, 's-', label='displacement [m]')
    axes[1].set_xlabel('Virtual load [N]')
    axes[1].set_title('Post-release recovery severity')
    axes[1].grid(alpha=0.3)
    axes[1].legend()
    fig.savefig(OUT / 'force_sweep_summary.png', dpi=180)


if __name__ == '__main__':
    main()
