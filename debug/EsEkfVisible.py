# -*- coding: utf-8 -*-
"""
ES-EKF Internal State Visualizer (v1.1)

This script is specifically designed to analyze and visualize the internal
workings of the EsEkfAhrs algorithm.

It generates three key plots:
1. Scored Error Plot: Compares the actual attitude error against the
   algorithm's predicted uncertainty bounds (3-sigma).
2. Gyro Bias Plot: Shows the estimated gyroscope bias and its uncertainty.
3. Internal Diagnostics Plot: Visualizes the algorithm's decision-making
   process, showing when corrections are applied based on accelerometer
   magnitude, and displaying the corresponding error signal (innovation)
   and correction strength (Kalman gain).

Author: Winyunq & AI Collaborator
Date: [Current Date]
Version: 1.1
"""

import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.spatial.transform import Rotation as R

# --- 1. CORE CONFIGURATION ---
# IMPORTANT: Change these values to analyze different runs
ALGORITHM_TO_ANALYZE = 'ESEKF'
SEQUENCE_TO_ANALYZE = 'V2_03_difficult' # 'V1_01_easy' is also a good test case
RESULTS_BASE_PATH = r'D:\中北大学惯性传感与先进导航实验室\IMU-GNSS\mahony_madwick\results' # Relative path to results
DEBUG_BASE_PATH = r'../debug'   # Relative path to debug
GROUND_TRUTH_GRAVITY = 9.81

# --- 2. HELPER FUNCTIONS ---
def read_tum_trajectory(filepath):
    """Reads a TUM format trajectory file (timestamp, tx, ty, tz, qx, qy, qz, qw)."""
    try:
        data = np.loadtxt(filepath, comments='#')
        if data.ndim == 1: data = data.reshape(1, -1)
        # TUM format is qx, qy, qz, qw. We need to reorder for SciPy (x,y,z,w)
        return data[:, 0], data[:, 4:8]
    except Exception as e:
        print(f"Error reading TUM file '{filepath}': {e}")
        return None, None

def read_ahrs_telemetry_csv(filepath):
    """Reads the detailed AHRS telemetry CSV file."""
    if not os.path.exists(filepath):
        print(f"Error: Telemetry file not found at '{filepath}'")
        return None
    try:
        with open(filepath, 'r') as f:
            header = [h.strip() for h in f.readline().strip().split(',')]
        data = np.loadtxt(filepath, delimiter=',', skiprows=1)
        if data.ndim == 1: data = data.reshape(1, -1)

        telemetry = {'timestamps': data[:, header.index('timestamp')]}

        # Standard data
        telemetry['quats_wxyz'] = data[:, header.index('q_w'):header.index('q_z')+1]
        telemetry['P_diag_att'] = data[:, header.index('P_att_xx'):header.index('P_att_zz')+1]
        if 'q_accel_w' in header:
            telemetry['q_accel_wxyz'] = data[:, header.index('q_accel_w'):header.index('q_accel_z')+1]
        if 'bias_x' in header:
            telemetry['gyro_bias'] = data[:, header.index('bias_x'):header.index('bias_z')+1]

        # Observable-specific data
        if 'innovation_roll' in header:
            telemetry['innovation'] = data[:, header.index('innovation_roll'):header.index('innovation_pitch')+1]
        if 'gain_roll' in header:
            telemetry['gain'] = data[:, header.index('gain_roll'):header.index('gain_pitch')+1]
        if 'correction_active' in header:
            telemetry['correction_active'] = data[:, header.index('correction_active')]
        if 'raw_accel_x' in header:
             telemetry['raw_accel'] = data[:, header.index('raw_accel_x'):header.index('raw_accel_z')+1]

        return telemetry
    except Exception as e:
        print(f"Error reading telemetry file '{filepath}': {e}")
        return None

def read_euroc_imu_data(filepath):
    """Reads EuRoC IMU data to get raw accelerometer readings."""
    try:
        # timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z
        data = np.loadtxt(filepath, delimiter=',', skiprows=1)
        return data[:, 0] * 1e-9, data[:, 4:7] # Timestamps (ns to s) and linear acceleration
    except Exception as e:
        print(f"Error reading EuRoC IMU file '{filepath}': {e}")
        return None, None

def custom_quat2eul_matlab(q_tum_xyzw):
    """Replicates MATLAB's quat2eul with 'ZYX' sequence, returning degrees."""
    r = R.from_quat(q_tum_xyzw)
    return r.as_euler('zyx', degrees=True)[:, ::-1] # a,b,c -> c,b,a to get roll,pitch,yaw

def wrap_to_180(angle_deg):
    """Wraps angles to the [-180, 180] interval."""
    return (angle_deg + 180) % 360 - 180

# --- 3. ANALYSIS AND PLOTTING ---
def plot_scored_error(time_axis, errors_deg, p_std_dev_deg, title_info, accel_errors_deg=None):
    """Generates the error vs. uncertainty bounds plot."""
    print("Generating scored error plot...")
    fig, axes = plt.subplots(3, 1, figsize=(18, 12), sharex=True)
    fig.suptitle(f'Attitude Error vs. Predicted Uncertainty\n{title_info}', fontsize=16)

    angle_names = ['Roll (X)', 'Pitch (Y)', 'Yaw (Z)']

    for i in range(3):
        ax = axes[i]
        error = errors_deg[:, i]
        three_sigma = 3 * p_std_dev_deg[:, i]

        in_bounds = np.abs(error) <= three_sigma

        ax.fill_between(time_axis, -three_sigma, three_sigma, color='gray', alpha=0.3, label=r'Predicted Uncertainty ($\pm 3\sigma$)')
        ax.plot(time_axis[in_bounds], error[in_bounds], '.', color='royalblue', markersize=2, label='Actual Error (In Bounds)')
        if accel_errors_deg is not None and i < 2: # Only plot Roll and Pitch for accel
            ax.plot(time_axis, accel_errors_deg[:, i], color='orange', linestyle='--', linewidth=1.5,
                    label='Accel-Only Error')
        ax.plot(time_axis[~in_bounds], error[~in_bounds], '.', color='red', markersize=3, label='Actual Error (Out of Bounds)')
        ax.set_ylabel(f'{angle_names[i]} Error [deg]')
        ax.legend()
        ax.grid(True, linestyle=':')

    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    output_path = os.path.join(DEBUG_BASE_PATH, f"{ALGORITHM_TO_ANALYZE}-{SEQUENCE_TO_ANALYZE}_error_scored.png")
    plt.savefig(output_path, dpi=150)
    print(f"Saved scored error plot to '{output_path}'")

def plot_bias_estimation(time_axis, telemetry, title_info):
    """Generates the gyroscope bias estimation plot."""
    if 'gyro_bias' not in telemetry:
        print("Gyro bias data not found in telemetry. Skipping bias plot.")
        return

    print("Generating gyro bias plot...")
    fig, axes = plt.subplots(3, 1, figsize=(18, 10), sharex=True)
    fig.suptitle(f'Gyroscope Bias Estimation\n{title_info}', fontsize=16)

    bias_names = ['Bias X', 'Bias Y', 'Bias Z']
    bias_dps = np.rad2deg(telemetry['gyro_bias'])

    for i in range(3):
        ax = axes[i]
        ax.plot(time_axis, bias_dps[:, i], label=f'Estimated {bias_names[i]}')
        ax.set_ylabel(f'{bias_names[i]} [deg/s]')
        ax.legend()
        ax.grid(True, linestyle=':')

    axes[-1].set_xlabel('Time [s]')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    output_path = os.path.join(DEBUG_BASE_PATH, f"{ALGORITHM_TO_ANALYZE}-{SEQUENCE_TO_ANALYZE}_bias_estimation.png")
    plt.savefig(output_path, dpi=150)
    print(f"Saved bias estimation plot to '{output_path}'")

def plot_internal_diagnostics(time_axis, telemetry, raw_accel, title_info):
    """Generates the internal diagnostics plot for observable AHRS."""
    print("Generating internal diagnostics plot...")
    fig, axes = plt.subplots(3, 1, figsize=(18, 12), sharex=True)
    fig.suptitle(f'ES-EKF Internal Diagnostics\n{title_info}', fontsize=16)

    # Subplot 1: Accelerometer Magnitude and Correction Trigger
    accel_mag = np.linalg.norm(raw_accel, axis=1)
    threshold_val = 0.2 # This should ideally be read from config, hardcoded for now

    ax1 = axes[0]
    ax1.plot(time_axis, accel_mag, label='Raw Accel. Magnitude', color='black', linewidth=1)
    ax1.axhline(GROUND_TRUTH_GRAVITY, color='green', linestyle='--', label=f'Gravity ({GROUND_TRUTH_GRAVITY:.2f} m/s^2)')
    ax1.axhline(GROUND_TRUTH_GRAVITY + threshold_val, color='red', linestyle=':', label=f'Correction Threshold (Upper)')
    ax1.axhline(GROUND_TRUTH_GRAVITY - threshold_val, color='red', linestyle=':', label=f'Correction Threshold (Lower)')

    ax1_twin = ax1.twinx()
    ax1_twin.fill_between(time_axis, 0, telemetry['correction_active'], color='cyan', alpha=0.4, label='Correction Active', step='post')
    ax1_twin.set_yticks([0, 1])
    ax1_twin.set_yticklabels(['OFF', 'ON'])

    ax1.set_ylabel('Accel Mag [m/s^2]')
    ax1_twin.set_ylabel('EKF Correction Step')
    ax1.legend(loc='upper left')
    ax1_twin.legend(loc='upper right')
    ax1.grid(True, linestyle=':')

    # Subplot 2: Innovation (Error Signal)
    ax2 = axes[1]
    ax2.plot(time_axis, np.rad2deg(telemetry['innovation'][:, 0]), label='Innovation (Roll Error)', color='coral')
    ax2.plot(time_axis, np.rad2deg(telemetry['innovation'][:, 1]), label='Innovation (Pitch Error)', color='mediumseagreen')
    ax2.set_ylabel('Innovation Signal [deg]')
    ax2.legend()
    ax2.grid(True, linestyle=':')

    # Subplot 3: Kalman Gain (Correction Strength)
    ax3 = axes[2]
    ax3.plot(time_axis, telemetry['gain'][:, 0], label='Gain (Roll)', color='coral')
    ax3.plot(time_axis, telemetry['gain'][:, 1], label='Gain (Pitch)', color='mediumseagreen')
    ax3.set_ylabel('Kalman Gain')
    ax3.set_xlabel('Time [s]')
    ax3.legend()
    ax3.grid(True, linestyle=':')

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    output_path = os.path.join(DEBUG_BASE_PATH, f"{ALGORITHM_TO_ANALYZE}-{SEQUENCE_TO_ANALYZE}_diagnostics.png")
    plt.savefig(output_path, dpi=150)
    print(f"Saved diagnostics plot to '{output_path}'")


def main():
    """Main analysis script."""
    title_info = f"Algorithm: [{ALGORITHM_TO_ANALYZE}], Sequence: [{SEQUENCE_TO_ANALYZE}]"
    print(f"--- Running Analysis for {title_info} ---")

    # --- 1. Load All Necessary Data ---
    gt_filepath = os.path.join(RESULTS_BASE_PATH, f"Groundtruth-{SEQUENCE_TO_ANALYZE}.txt")
    telemetry_filepath = os.path.join(DEBUG_BASE_PATH, f"{ALGORITHM_TO_ANALYZE}-{SEQUENCE_TO_ANALYZE}_telemetry.csv")
    imu_data_filepath = os.path.join(f"../data/{SEQUENCE_TO_ANALYZE}/mav0/imu0/data.csv") # Path to raw data

    gt_ts, gt_quats_xyzw = read_tum_trajectory(gt_filepath)
    telemetry = read_ahrs_telemetry_csv(telemetry_filepath)
    imu_ts_raw, imu_accel_raw = read_euroc_imu_data(imu_data_filepath)

    if gt_ts is None or telemetry is None or imu_ts_raw is None:
        print("\nAborting due to missing data.")
        return

    # --- 2. Synchronize Data ---
    # We synchronize everything to the telemetry timestamps
    est_ts = telemetry['timestamps']
    start_time = max(gt_ts[0], est_ts[0], imu_ts_raw[0])
    end_time = min(gt_ts[-1], est_ts[-1], imu_ts_raw[-1])

    sync_mask = (est_ts >= start_time) & (est_ts <= end_time)

    # Sync all telemetry data
    ts_synced = est_ts[sync_mask]
    for key in telemetry:
        telemetry[key] = telemetry[key][sync_mask]

    # Interpolate GT and IMU data to match telemetry timestamps
    gt_quats_synced_xyzw = np.array([np.interp(ts_synced, gt_ts, gt_quats_xyzw[:, i]) for i in range(4)]).T
    gt_quats_synced_xyzw /= np.linalg.norm(gt_quats_synced_xyzw, axis=1, keepdims=True)

    raw_accel_synced = np.array([np.interp(ts_synced, imu_ts_raw, imu_accel_raw[:, i]) for i in range(3)]).T

    # --- 3. Process Data for Plotting ---
    # Align estimated trajectory to the first ground truth orientation
    r_gt = R.from_quat(gt_quats_synced_xyzw)
    r_est_raw = R.from_quat(telemetry['quats_wxyz'][:, [1,2,3,0]])
    r_correction = r_gt[0] * r_est_raw[0].inv()
    r_est_aligned = r_correction * r_est_raw

    # Calculate orientation error
    error_rot = r_est_aligned * r_gt.inv()
    error_eul_deg = wrap_to_180(error_rot.as_euler('zyx', degrees=True)[:, ::-1])

    # Process accel-only data if available
    accel_error_eul_deg = None
    if 'q_accel_wxyz' in telemetry:
        print("Processing accelerometer-only attitude data...")
        r_accel_raw = R.from_quat(telemetry['q_accel_wxyz'][:, [1,2,3,0]])
        r_accel_aligned = r_correction * r_accel_raw
        accel_error_rot = r_accel_aligned * r_gt.inv()
        accel_error_eul_deg = wrap_to_180(accel_error_rot.as_euler('zyx', degrees=True)[:, ::-1])

    p_std_dev_rad = np.sqrt(telemetry['P_diag_att'])
    p_std_dev_deg = np.rad2deg(p_std_dev_rad)

    time_axis = ts_synced - ts_synced[0]

    # --- 4. Generate Plots ---
    plot_scored_error(time_axis, error_eul_deg, p_std_dev_deg, title_info, accel_errors_deg=accel_error_eul_deg)
    plot_bias_estimation(time_axis, telemetry, title_info)
    plot_internal_diagnostics(time_axis, telemetry, raw_accel_synced, title_info)

    print("\n--- Analysis Complete ---")
    plt.show()


if __name__ == '__main__':
    main()
