# -*- coding: utf-8 -*-
"""
Error and Uncertainty Analysis Script for AHRS Algorithms (v4.0)

This script provides a critical analysis by plotting the actual estimation
error against the algorithm's own predicted uncertainty bounds (3-sigma).
It directly answers the question: "Does the algorithm's confidence match its
actual performance?"

Author: Winyunq & AI Collaborator
Date: [Current Date]
Version: 4.0
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os

# --- 1. CORE CONFIGURATION ---
ALGORITHM_TO_ANALYZE = 'AccelOnlyAhrs' 
SEQUENCE_TO_ANALYZE = 'V2_03_difficult'

RESULTS_BASE_PATH = r'D:\中北大学惯性传感与先进导航实验室\IMU-GNSS\mahony_madwick\results'
DEBUG_BASE_PATH = r'D:\Ubuntu20ROS1\UKF\debug'

# --- 2. HELPER FUNCTIONS ---
def read_tum_trajectory(filepath):
    """Reads a TUM format trajectory file."""
    if not os.path.exists(filepath): print(f"Error: TUM file not found at '{filepath}'"); return None, None
    try:
        data = np.loadtxt(filepath, comments='#');
        if data.ndim == 1: data = data.reshape(1, -1)
        return data[:, 0], data[:, 4:8]
    except Exception as e: print(f"Error reading TUM file '{filepath}': {e}"); return None, None

def read_telemetry_csv(filepath):
    """Reads the custom telemetry CSV."""
    if not os.path.exists(filepath): print(f"Error: Telemetry file not found at '{filepath}'"); return None
    try:
        with open(filepath, 'r') as f: header = f.readline().strip().split(',')
        data = np.loadtxt(filepath, delimiter=',', skiprows=1);
        if data.ndim == 1: data = data.reshape(1, -1)
        telemetry = {'timestamps': data[:, 0], 'quats_wxyz': data[:, 1:5]}
        if 'P_att_xx' in header: telemetry['P_diag_att'] = data[:, header.index('P_att_xx'):header.index('P_att_xx')+3]
        return telemetry
    except Exception as e: print(f"Error reading telemetry file '{filepath}': {e}"); return None

def align_full_orientation(q_gt_xyzw, q_est_xyzw):
    """Aligns the estimated trajectory to the initial ground truth orientation."""
    r_gt = R.from_quat(q_gt_xyzw); r_est = R.from_quat(q_est_xyzw)
    r_correction = r_gt[0] * r_est[0].inv()
    r_aligned = r_correction * r_est
    return r_aligned.as_quat()

def custom_quat2eul_matlab(q_tum_xyzw):
    """Precisely replicates the MATLAB custom_quat2eul function."""
    qw = q_tum_xyzw[:, 3]; qx = q_tum_xyzw[:, 0]; qy = q_tum_xyzw[:, 1]; qz = q_tum_xyzw[:, 2]
    roll = np.rad2deg(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2)))
    pitch_arg = 2 * (qw * qy - qz * qx)
    pitch = np.rad2deg(np.arcsin(np.clip(pitch_arg, -1.0, 1.0)))
    yaw = np.rad2deg(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2)))
    return roll, pitch, yaw

def wrap_to_180(angle_deg):
    """Wraps angles to the [-180, 180] interval."""
    return (angle_deg + 180) % 360 - 180

# --- 3. MAIN ANALYSIS SCRIPT ---

def main():
    print(f"--- Analyzing Error vs. Uncertainty for '{ALGORITHM_TO_ANALYZE}' on sequence '{SEQUENCE_TO_ANALYZE}' ---")

    # --- (Data Loading, Sync, and Alignment are the same) ---
    gt_filepath = os.path.join(RESULTS_BASE_PATH, f"Groundtruth-{SEQUENCE_TO_ANALYZE}.txt")
    telemetry_filepath = os.path.join(DEBUG_BASE_PATH, f"{ALGORITHM_TO_ANALYZE}-{SEQUENCE_TO_ANALYZE}_telemetry.csv")
    gt_timestamps, gt_quats_xyzw = read_tum_trajectory(gt_filepath)
    telemetry_data = read_telemetry_csv(telemetry_filepath)
    if gt_timestamps is None or telemetry_data is None: return
    
    est_timestamps = telemetry_data['timestamps']
    est_quats_xyzw = telemetry_data['quats_wxyz'][:, [1, 2, 3, 0]]
    start_time = max(gt_timestamps[0], est_timestamps[0]); end_time = min(gt_timestamps[-1], est_timestamps[-1])
    time_mask = (est_timestamps >= start_time) & (est_timestamps <= end_time)
    timestamps_synced = est_timestamps[time_mask]
    est_quats_synced = est_quats_xyzw[time_mask]
    p_diag_synced = telemetry_data.get('P_diag_att', np.zeros((len(timestamps_synced), 3)))[time_mask]

    gt_quats_synced = np.array([np.interp(timestamps_synced, gt_timestamps, gt_quats_xyzw[:, i]) for i in range(4)]).T
    gt_quats_synced /= np.linalg.norm(gt_quats_synced, axis=1)[:, np.newaxis]
    est_quats_aligned_xyzw = align_full_orientation(gt_quats_synced, est_quats_synced)

    # --- Data Processing: Calculate Error and Uncertainty ---
    print("Calculating estimation error and uncertainty bounds...")
    time_axis = timestamps_synced - timestamps_synced[0]
    
    roll_gt, pitch_gt, yaw_gt = custom_quat2eul_matlab(gt_quats_synced)
    roll_est, pitch_est, yaw_est = custom_quat2eul_matlab(est_quats_aligned_xyzw)

    error_roll = wrap_to_180(roll_est - roll_gt)
    error_pitch = wrap_to_180(pitch_est - pitch_gt)
    error_yaw = wrap_to_180(yaw_est - yaw_gt)
    
    p_std_dev_deg = np.rad2deg(np.sqrt(p_diag_synced))
    three_sigma_roll = 3 * p_std_dev_deg[:, 0]
    three_sigma_pitch = 3 * p_std_dev_deg[:, 1]
    three_sigma_yaw = 3 * p_std_dev_deg[:, 2]

    # --- 4. VISUALIZATION: Error vs. Uncertainty Bounds (with Highlighting) ---
    print("Generating plots with out-of-bounds error highlighting...")
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    fig.suptitle(f'Error vs. Predicted Uncertainty (3σ) for [{ALGORITHM_TO_ANALYZE}]\nSequence: {SEQUENCE_TO_ANALYZE}', fontsize=16)

    plot_titles = ['Roll (X) Error', 'Pitch (Y) Error', 'Yaw (Z) Error']
    error_data = [error_roll, error_pitch, error_yaw]
    three_sigma_data = [three_sigma_roll, three_sigma_pitch, three_sigma_yaw]

    for i in range(3):
        ax = axes[i]
        
        # Plot the theoretical uncertainty bounds (the "GPS circle")
        ax.fill_between(time_axis, -three_sigma_data[i], three_sigma_data[i], 
                        color='skyblue', alpha=0.5, label=r'Predicted Uncertainty Bounds ($\pm 3\sigma$)')
        
        # Plot the actual error that is WITHIN the bounds
        error_in_bounds = np.ma.masked_where(np.abs(error_data[i]) > three_sigma_data[i], error_data[i])
        ax.plot(time_axis, error_in_bounds, color='royalblue', linewidth=1.5, label='Actual Error (In Bounds)')

        # *** KEY NEW FEATURE: Highlight the error that is OUTSIDE the bounds in red ***
        error_out_of_bounds = np.ma.masked_where(np.abs(error_data[i]) <= three_sigma_data[i], error_data[i])
        ax.plot(time_axis, error_out_of_bounds, color='red', linewidth=1.5, label='Actual Error (Out of Bounds)')

        ax.axhline(y=0, color='k', linestyle='--', linewidth=1)
        ax.set_ylabel(f'{plot_titles[i]} [deg]')
        ax.legend(loc='upper right')
        ax.grid(True, linestyle=':')

    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    output_plot_path = os.path.join(DEBUG_BASE_PATH, f"{ALGORITHM_TO_ANALYZE}-{SEQUENCE_TO_ANALYZE}_error_highlighted.png")
    plt.savefig(output_plot_path, dpi=150)
    print(f"Error analysis plot saved to {output_plot_path}")
    
    plt.show()

if __name__ == '__main__':
    main()