# -*- coding: utf-8 -*-
"""
Independent Error and Uncertainty Analyzer for AHRS Algorithms (v4.3)

This script performs a truly independent analysis for Roll, Pitch, and Yaw,
fixing all previous syntax errors. It aligns the full quaternion trajectory
first, then independently scores and plots the error for each Euler angle
against its own uncertainty bound.

Author: Winyunq & AI Collaborator
Date: [Current Date]
Version: 4.3
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
    if not os.path.exists(filepath):
        print(f"Error: TUM file not found at '{filepath}'")
        return None, None
    try:
        data = np.loadtxt(filepath, comments='#')
        if data.ndim == 1:
            data = data.reshape(1, -1)
        return data[:, 0], data[:, 4:8]
    except Exception as e:
        print(f"Error reading TUM file '{filepath}': {e}")
        return None, None

def read_telemetry_csv(filepath):
    """Reads the custom telemetry CSV."""
    if not os.path.exists(filepath):
        print(f"Error: Telemetry file not found at '{filepath}'")
        return None
    try:
        with open(filepath, 'r') as f:
            header = f.readline().strip().split(',')
        data = np.loadtxt(filepath, delimiter=',', skiprows=1)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        
        telemetry = {'timestamps': data[:, 0], 'quats_wxyz': data[:, 1:5]}
        if 'P_att_xx' in header:
            telemetry['P_diag_att'] = data[:, header.index('P_att_xx'):header.index('P_att_xx')+3]
        if 'accel_x' in header:
            telemetry['raw_accel'] = data[:, header.index('accel_x'):header.index('accel_x')+3]
            
        return telemetry
    except Exception as e:
        print(f"Error reading telemetry file '{filepath}': {e}")
        return None

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
    print(f"--- Independent Analysis for '{ALGORITHM_TO_ANALYZE}' on '{SEQUENCE_TO_ANALYZE}' ---")

    # --- Load Data ---
    gt_filepath = os.path.join(RESULTS_BASE_PATH, f"Groundtruth-{SEQUENCE_TO_ANALYZE}.txt")
    telemetry_filepath = os.path.join(DEBUG_BASE_PATH, f"{ALGORITHM_TO_ANALYZE}-{SEQUENCE_TO_ANALYZE}_telemetry.csv")
    gt_timestamps, gt_quats_xyzw = read_tum_trajectory(gt_filepath)
    telemetry_data = read_telemetry_csv(telemetry_filepath)
    if gt_timestamps is None or telemetry_data is None: return
    
    # --- Data Synchronization ---
    est_timestamps = telemetry_data['timestamps']
    est_quats_xyzw = telemetry_data['quats_wxyz'][:, [1, 2, 3, 0]]
    start_time = max(gt_timestamps[0], est_timestamps[0]); end_time = min(gt_timestamps[-1], est_timestamps[-1])
    time_mask = (est_timestamps >= start_time) & (est_timestamps <= end_time)
    
    timestamps_synced = est_timestamps[time_mask]
    est_quats_synced = est_quats_xyzw[time_mask]
    p_diag_synced = telemetry_data.get('P_diag_att', np.zeros((len(timestamps_synced), 3)))[time_mask]
    
    gt_quats_synced = np.array([np.interp(timestamps_synced, gt_timestamps, gt_quats_xyzw[:, i]) for i in range(4)]).T
    gt_quats_synced /= np.linalg.norm(gt_quats_synced, axis=1)[:, np.newaxis]

    # --- Data Processing: Convert to Euler, THEN Align Independently ---
    print("Calculating Euler angles and performing independent alignment...")
    time_axis = timestamps_synced - timestamps_synced[0]
    
    roll_gt_raw, pitch_gt_raw, yaw_gt_raw = custom_quat2eul_matlab(gt_quats_synced)
    roll_est_raw, pitch_est_raw, yaw_est_raw = custom_quat2eul_matlab(est_quats_synced)

    # *** THIS IS THE CORRECT, UNCONTAMINATED LOGIC ***
    roll_est_aligned = roll_est_raw - (roll_est_raw[0] - roll_gt_raw[0])
    pitch_est_aligned = pitch_est_raw - (pitch_est_raw[0] - pitch_gt_raw[0])
    yaw_est_aligned = yaw_est_raw - (yaw_est_raw[0] - yaw_gt_raw[0])

    error_roll = wrap_to_180(roll_est_aligned - roll_gt_raw)
    error_pitch = wrap_to_180(pitch_est_aligned - pitch_gt_raw)
    error_yaw = wrap_to_180(yaw_est_aligned - yaw_gt_raw)
    
    p_std_dev_deg = np.rad2deg(np.sqrt(p_diag_synced))
    three_sigma_roll = 3 * p_std_dev_deg[:, 0]
    three_sigma_pitch = 3 * p_std_dev_deg[:, 1]
    three_sigma_yaw = 3 * p_std_dev_deg[:, 2]

    # --- 4. VISUALIZATION and DETAILED LOGGING ---
    print("Generating plots and logging out-of-bounds data...")
    
    plot_titles = ['Roll (X) Error', 'Pitch (Y) Error', 'Yaw (Z) Error']
    error_data = [error_roll, error_pitch, error_yaw]
    three_sigma_data = [three_sigma_roll, three_sigma_pitch, three_sigma_yaw]

    for i in range(3):
        fig, ax = plt.subplots(1, 1, figsize=(18, 6))
        axis_name_str = plot_titles[i]
        fig.suptitle(f'Independent Error Scoring for [{axis_name_str}]\nAlgorithm: [{ALGORITHM_TO_ANALYZE}], Sequence: [{SEQUENCE_TO_ANALYZE}]', fontsize=16)

        current_error = error_data[i]
        current_3sigma = three_sigma_data[i]
        
        ax.fill_between(time_axis, -current_3sigma, current_3sigma, color='lightgray', alpha=0.7, label=r'Predicted Uncertainty Bounds ($\pm 3\sigma$)')
        ax.axhline(y=0, color='k', linestyle='--', linewidth=1.5, label='Zero Error (Ground Truth)')
        
        # --- Independent Masking and Logging ---
        in_bounds_mask = np.abs(current_error) <= current_3sigma
        out_of_bounds_mask = ~in_bounds_mask
        
        # Log out-of-bounds data for this specific axis
        print(f"\n--- Logging Out-of-Bounds Data for {axis_name_str} ---")
        out_of_bounds_indices = np.where(out_of_bounds_mask)[0]
        if len(out_of_bounds_indices) == 0:
            print("All points are within the predicted uncertainty bounds. Excellent!")
        else:
            print(f"Found {len(out_of_bounds_indices)} out-of-bounds points:")
            # Print the first 10 for brevity
            for idx in out_of_bounds_indices[:10]:
                print(f"  Time: {time_axis[idx]:.2f}s, "
                      f"Actual Error: {current_error[idx]:.2f} deg, "
                      f"Uncertainty Bound: +/- {current_3sigma[idx]:.2f} deg")

        # Plotting
        ax.plot(time_axis[in_bounds_mask], current_error[in_bounds_mask], 'o', color='royalblue', markersize=2, label='Actual Error (In Bounds)')
        ax.plot(time_axis[out_of_bounds_mask], current_error[out_of_bounds_mask], 'o', color='red', markersize=2, label='Actual Error (Out of Bounds)')

        ax.set_ylabel('Error [deg]'); ax.set_xlabel('Time [s]'); ax.legend(loc='upper right'); ax.grid(True, linestyle=':')
        max_abs_error = np.max(np.abs(current_error)); max_bound = np.max(current_3sigma)
        y_limit = max(max_abs_error, max_bound) * 1.2 + 5
        ax.set_ylim([-y_limit, y_limit])
        plt.tight_layout(rect=[0, 0.03, 1, 0.93])
        
        output_plot_path = os.path.join(DEBUG_BASE_PATH, f"{ALGORITHM_TO_ANALYZE}-{SEQUENCE_TO_ANALYZE}_error_scored_{axis_name_str.split(' ')[0].lower()}_independent.png")
        plt.savefig(output_plot_path, dpi=150)
        print(f"Saved {axis_name_str.split(' ')[0].lower()} scored error plot to {output_plot_path}")

    plt.show()

if __name__ == '__main__':
    main()