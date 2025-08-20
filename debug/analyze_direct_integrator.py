# -*- coding: utf-8 -*-
"""
Universal Telemetry Analyzer for AHRS Algorithms (v3.2)

This version re-introduces a top-level configuration for the sequence to
analyze for maximum flexibility during development.

Author: Winyunq & AI Collaborator
Date: [Current Date]
Version: 3.2
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os

# --- 1. CORE CONFIGURATION ---

# *** KEY SETTING: Specify which ALGORITHM and SEQUENCE to analyze here! ***
ALGORITHM_TO_ANALYZE = 'CalibratedIntegrator' 
SEQUENCE_TO_ANALYZE = 'V2_03_difficult'      # <--- 您现在可以自由修改这里！

# *** KEY SETTING: Hardcoded absolute paths for data directories ***
RESULTS_BASE_PATH = r'D:\中北大学惯性传感与先进导航实验室\IMU-GNSS\mahony_madwick\results'
DEBUG_BASE_PATH = r'D:\Ubuntu20ROS1\UKF\debug' 

# --- 2. HELPER FUNCTIONS (No changes needed) ---
# ... (read_tum_trajectory, read_telemetry_csv, etc. are correct) ...
def read_tum_trajectory(filepath):
    if not os.path.exists(filepath): print(f"Error: TUM file not found at '{filepath}'"); return None, None
    try:
        data = np.loadtxt(filepath, comments='#');
        if data.ndim == 1: data = data.reshape(1, -1)
        return data[:, 0], data[:, 4:8]
    except Exception as e: print(f"Error reading TUM file '{filepath}': {e}"); return None, None
def read_telemetry_csv(filepath):
    if not os.path.exists(filepath): print(f"Error: Telemetry file not found at '{filepath}'"); return None
    try:
        with open(filepath, 'r') as f: header = f.readline().strip().split(',')
        data = np.loadtxt(filepath, delimiter=',', skiprows=1);
        if data.ndim == 1: data = data.reshape(1, -1)
        telemetry = {'timestamps': data[:, 0], 'quats_wxyz': data[:, 1:5]}
        if 'P_att_xx' in header: telemetry['P_diag_att'] = data[:, header.index('P_att_xx'):header.index('P_att_xx')+3]
        if 'P_bias_xx' in header: telemetry['P_diag_bias'] = data[:, header.index('P_bias_xx'):header.index('P_bias_xx')+3]
        return telemetry
    except Exception as e: print(f"Error reading telemetry file '{filepath}': {e}"); return None
def align_full_orientation(q_gt_xyzw, q_est_xyzw):
    r_gt = R.from_quat(q_gt_xyzw); r_est = R.from_quat(q_est_xyzw)
    r_correction = r_gt[0] * r_est[0].inv()
    r_aligned = r_correction * r_est
    return r_aligned.as_quat()
def custom_quat2eul_matlab(q_tum_xyzw):
    qw = q_tum_xyzw[:, 3]; qx = q_tum_xyzw[:, 0]; qy = q_tum_xyzw[:, 1]; qz = q_tum_xyzw[:, 2]
    roll = np.rad2deg(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2)))
    pitch_arg = 2 * (qw * qy - qz * qx)
    pitch = np.rad2deg(np.arcsin(np.clip(pitch_arg, -1.0, 1.0)))
    yaw = np.rad2deg(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2)))
    return roll, pitch, yaw

# --- 3. MAIN ANALYSIS SCRIPT ---

def main():
    print(f"--- Analyzing '{ALGORITHM_TO_ANALYZE}' for sequence '{SEQUENCE_TO_ANALYZE}' ---")

    # --- Construct file paths using top-level config ---
    gt_filename = f"Groundtruth-{SEQUENCE_TO_ANALYZE}.txt"
    est_telemetry_filename = f"{ALGORITHM_TO_ANALYZE}-{SEQUENCE_TO_ANALYZE}_telemetry.csv"
    gt_filepath = os.path.join(RESULTS_BASE_PATH, gt_filename)
    telemetry_filepath = os.path.join(DEBUG_BASE_PATH, est_telemetry_filename)

    # --- (The rest of the main function remains identical to the last correct version) ---
    print("Loading data...")
    gt_timestamps, gt_quats_xyzw = read_tum_trajectory(gt_filepath)
    if gt_timestamps is None: return
    telemetry_data = read_telemetry_csv(telemetry_filepath)
    if telemetry_data is None: return
    
    est_timestamps = telemetry_data['timestamps']
    est_quats_xyzw = telemetry_data['quats_wxyz'][:, [1, 2, 3, 0]]
    
    print("Synchronizing and aligning data...")
    start_time = max(gt_timestamps[0], est_timestamps[0])
    end_time = min(gt_timestamps[-1], est_timestamps[-1])
    time_mask = (est_timestamps >= start_time) & (est_timestamps <= end_time)
    timestamps_synced = est_timestamps[time_mask]
    est_quats_synced = est_quats_xyzw[time_mask]
    p_diag_synced = telemetry_data.get('P_diag_att', np.zeros((len(timestamps_synced), 3)))[time_mask]

    gt_quats_synced = np.array([np.interp(timestamps_synced, gt_timestamps, gt_quats_xyzw[:, i]) for i in range(4)]).T
    gt_quats_synced /= np.linalg.norm(gt_quats_synced, axis=1)[:, np.newaxis]
    est_quats_aligned_xyzw = align_full_orientation(gt_quats_synced, est_quats_synced)

    print("Converting to Euler angles using MATLAB replica...")
    time_axis = timestamps_synced - timestamps_synced[0]
    roll_gt, pitch_gt, yaw_gt = custom_quat2eul_matlab(gt_quats_synced)
    roll_est, pitch_est, yaw_est = custom_quat2eul_matlab(est_quats_aligned_xyzw)
    
    print("Generating plots...")
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    fig.suptitle(f'[{ALGORITHM_TO_ANALYZE}] vs. Ground Truth (MATLAB Style)\nSequence: {SEQUENCE_TO_ANALYZE}', fontsize=16)

    plot_titles = ['Roll (X) Angle', 'Pitch (Y) Angle', 'Yaw (Z) Angle']
    gt_euler_data = [roll_gt, pitch_gt, yaw_gt]
    est_euler_data = [roll_est, pitch_est, yaw_est]
    p_std_dev_deg = np.rad2deg(np.sqrt(p_diag_synced))
    p_diag_map = [p_std_dev_deg[:, 0], p_std_dev_deg[:, 1], p_std_dev_deg[:, 2]]

    for i in range(3):
        ax = axes[i]
        ax.plot(time_axis, gt_euler_data[i], 'k-', label='Ground Truth', linewidth=2)
        ax.plot(time_axis, est_euler_data[i], 'b--', label=f'{ALGORITHM_TO_ANALYZE} (Est.)', linewidth=1.5)
        three_sigma = 3 * p_diag_map[i]
        upper_bound = est_euler_data[i] + three_sigma
        lower_bound = est_euler_data[i] - three_sigma
        ax.fill_between(time_axis, lower_bound, upper_bound, color='blue', alpha=0.2, label=r'Estimate $\pm 3\sigma$ (from model)')
        ax.set_ylabel(f'{plot_titles[i]} [deg]')
        ax.legend(loc='best')
        ax.grid(True, linestyle=':')
    
    axes[1].set_title("Note: The confidence interval does not cover the true value due to unmodeled bias drift.", fontsize=10, style='italic')
    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    output_plot_path = os.path.join(DEBUG_BASE_PATH, f"{ALGORITHM_TO_ANALYZE}-{SEQUENCE_TO_ANALYZE}_telemetry_plot_final.png")
    plt.savefig(output_plot_path, dpi=150)
    print(f"Plot saved to {output_plot_path}")
    
    plt.show()


if __name__ == '__main__':
    main()