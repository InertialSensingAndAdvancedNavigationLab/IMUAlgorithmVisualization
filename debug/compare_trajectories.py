# -*- coding: utf-8 -*-
"""
Multi-Algorithm Trajectory Comparator (v1.0)

This script reads multiple trajectory files (TUM format) from a results
directory and plots their attitude (Roll, Pitch, Yaw) against the ground
truth for direct comparison.

Author: Winyunq & AI Collaborator
Date: [Current Date]
Version: 1.0
"""

import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.spatial.transform import Rotation as R

# --- 1. CORE CONFIGURATION ---

# The sequence to analyze
SEQUENCE_TO_ANALYZE = 'V2_03_difficult'

# Base path for the results directory
RESULTS_BASE_PATH = r'D:\中北大学惯性传感与先进导航实验室\IMU-GNSS\mahony_madwick\results'

# Algorithm configuration, mapping file prefixes to display names for the plot legend.
# This mirrors the structure from your MATLAB configuration.
ALGORITHMS_TO_COMPARE = {
    # File Prefix           # Display Name
    'Groundtruth-':         'Ground Truth',
    'ESEKF-':               'ESEKF (Ours)',
    'Gyro-Integration-':    'Gyro Integration',
    'Mahony-AHRS-':         'Mahony AHRS',
    'Madgwick-AHRS-':       'Madgwick AHRS',
    # 'ACF-AHRS-':            'Ours (ACF-AHRS)', # Example of another algorithm
}

# --- 2. HELPER FUNCTIONS ---

def read_tum_trajectory(filepath):
    """Reads a TUM format trajectory file (timestamp, tx, ty, tz, qx, qy, qz, qw)."""
    if not os.path.exists(filepath):
        print(f"Warning: Trajectory file not found, skipping: '{filepath}'")
        return None, None
    try:
        data = np.loadtxt(filepath, comments='#')
        if data.ndim == 1: data = data.reshape(1, -1)
        # TUM format is qx, qy, qz, qw. We need to reorder for SciPy (x,y,z,w)
        return data[:, 0], data[:, 4:8]
    except Exception as e:
        print(f"Error reading TUM file '{filepath}': {e}")
        return None, None

def custom_quat2eul_matlab(q_tum_xyzw):
    """Replicates MATLAB's quat2eul with 'ZYX' sequence, returning degrees."""
    r = R.from_quat(q_tum_xyzw)
    # Use 'zyx' to get Yaw, Pitch, Roll, then reverse to get Roll, Pitch, Yaw
    return r.as_euler('zyx', degrees=True)[:, ::-1]

# --- 3. MAIN ANALYSIS SCRIPT ---

def main():
    """Main comparison script."""
    print(f"--- Comparing Algorithm Trajectories for Sequence: {SEQUENCE_TO_ANALYZE} ---")

    # --- 1. Load All Trajectories ---
    all_trajectories = {}
    for prefix, name in ALGORITHMS_TO_COMPARE.items():
        filepath = os.path.join(RESULTS_BASE_PATH, f"{prefix}{SEQUENCE_TO_ANALYZE}.txt")
        timestamps, quats_xyzw = read_tum_trajectory(filepath)
        if timestamps is not None:
            all_trajectories[name] = {'ts': timestamps, 'quat': quats_xyzw}

    if 'Ground Truth' not in all_trajectories:
        print("Error: Ground Truth trajectory is required for comparison. Aborting.")
        return

    # --- 2. Synchronize and Align Data ---
    print("Synchronizing and aligning all trajectories to Ground Truth...")
    gt_data = all_trajectories.pop('Ground Truth')
    gt_ts = gt_data['ts']
    gt_eul_deg = custom_quat2eul_matlab(gt_data['quat'])

    aligned_eul_data = {}
    for name, data in all_trajectories.items():
        # Interpolate to ground truth timestamps
        interp_quats = np.array([np.interp(gt_ts, data['ts'], data['quat'][:, i]) for i in range(4)]).T
        interp_quats /= np.linalg.norm(interp_quats, axis=1, keepdims=True) # Re-normalize

        # Align to the first ground truth orientation
        r_gt = R.from_quat(gt_data['quat'])
        r_est = R.from_quat(interp_quats)
        r_correction = r_gt[0] * r_est[0].inv()
        r_aligned = r_correction * r_est

        aligned_eul_data[name] = custom_quat2eul_matlab(r_aligned.as_quat())

    # --- 3. Generate Comparison Plots ---
    print("Generating comparison plots...")
    fig, axes = plt.subplots(3, 1, figsize=(20, 15), sharex=True)
    fig.suptitle(f'Algorithm Attitude Comparison on Sequence: {SEQUENCE_TO_ANALYZE}', fontsize=16)

    angle_names = ['Roll (X)', 'Pitch (Y)', 'Yaw (Z)']
    time_axis = gt_ts - gt_ts[0]

    # Define a color and style cycle for plots
    colors = plt.cm.viridis(np.linspace(0, 1, len(aligned_eul_data)))
    styles = ['-', '--', '-.', ':']

    for i in range(3):
        ax = axes[i]
        # Plot Ground Truth first
        ax.plot(time_axis, gt_eul_deg[:, i], color='black', linewidth=2.5, label='Ground Truth')

        # Plot each algorithm's trajectory
        for j, (name, eul_data) in enumerate(aligned_eul_data.items()):
            ax.plot(time_axis, eul_data[:, i],
                    label=name,
                    color=colors[j],
                    linestyle=styles[j % len(styles)],
                    linewidth=1.5)

        ax.set_ylabel(f'{angle_names[i]} [deg]')
        ax.legend(loc='upper right')
        ax.grid(True, linestyle=':')

    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    output_path = os.path.join(RESULTS_BASE_PATH, f"comparison_{SEQUENCE_TO_ANALYZE}.png")
    plt.savefig(output_path, dpi=200)
    print(f"\nComparison plot saved to: '{output_path}'")
    plt.show()

if __name__ == '__main__':
    main()