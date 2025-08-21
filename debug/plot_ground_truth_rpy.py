import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import argparse
import os
from typing import Optional

def plot_ground_truth_rpy(dataset_base_path: str, sequence_name: str, output_file: Optional[str] = None):
    """
    读取并绘制EuRoC数据集真值的Roll, Pitch, Yaw姿态角，以复现论文图表。

    关键点:
    - 世界坐标系 (World): ENU (East-North-Up)
    - 机体坐标系 (Body/Sensor): RUF (Right-Up-Forward)
    - 真值四元数 q_RS: 描述了从 RUF 到 ENU 的旋转 (q_ENU_from_RUF)。
    - 我们直接从这个原始四元数中提取ZYX欧拉角，不进行任何机体坐标系间的转换。
    """
    ground_truth_path = os.path.join(
        dataset_base_path,
        sequence_name,
        "mav0",
        "state_groundtruth_estimate0",
        "data.csv"
    )

    if not os.path.exists(ground_truth_path):
        print(f"错误: 真值文件未找到于 '{ground_truth_path}'")
        return

    print(f"正在读取真值数据: {ground_truth_path}")
    df = pd.read_csv(ground_truth_path)
    df.columns = [col.strip() for col in df.columns]

    # 1. 提取原始数据
    # EuRoC格式: #timestamp, ..., q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []
    # SciPy格式要求: (x, y, z, w)
    timestamps = df['#timestamp'].to_numpy() * 1e-9
    quats_wxyz = df[['q_RS_w []', 'q_RS_x []', 'q_RS_y []', 'q_RS_z []']].to_numpy()
    quats_xyzw = quats_wxyz[:, [1, 2, 3, 0]]

    # 2. 直接使用原始的姿态四元数创建旋转对象
    # 这个对象代表了从 RUF 机体坐标系到 ENU 世界坐标系的旋转
    rotations_enu_from_ruf = R.from_quat(quats_xyzw)

    # 3. 从原始姿态中直接提取欧拉角 (Roll, Pitch, Yaw)
    # 使用 'zyx' 序列，这是最标准的航空欧拉角序列，对应于 Yaw, Pitch, Roll。
    # as_euler('zyx') 返回 (yaw, pitch, roll)，我们需要反转顺序以匹配绘图习惯 (roll, pitch, yaw)。
    print("正在将四元数转换为欧拉角...")
    eul_angles_deg = rotations_enu_from_ruf.as_euler('zyx', degrees=True)
    
    # 提取并可能需要进行符号调整以匹配图表约定（根据'zyx'和右手系，通常无需调整）
    # scipy.as_euler('zyx') 返回 [yaw, pitch, roll]
    yaw = eul_angles_deg[:, 0]
    pitch = eul_angles_deg[:, 1]
    roll = eul_angles_deg[:, 2]

    # 创建绘图
    time_axis = timestamps - timestamps[0]
    # 将时间轴转换为分钟，以匹配论文图表的X轴
    time_axis_min = time_axis / 60.0

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f'Ground Truth Attitude (RPY) - {sequence_name} (Replicated)', fontsize=16)

    # 绘制Roll，匹配论文风格
    axes[0].plot(time_axis_min, roll, label='Roll', color='C0') # 使用蓝色
    axes[0].set_ylabel('roll (deg)')
    axes[0].grid(True)
    axes[0].set_ylim(140, 180) # 设置Y轴范围以匹配论文

    # 绘制Pitch
    axes[1].plot(time_axis_min, pitch, label='Pitch', color='C0')
    axes[1].set_ylabel('pitch (deg)')
    axes[1].grid(True)
    axes[1].set_ylim(-85, -55) # 设置Y轴范围以匹配论文

    # 绘制Yaw
    axes[2].plot(time_axis_min, yaw, label='Yaw', color='C0')
    axes[2].set_ylabel('yaw (deg)')
    axes[2].grid(True)
    axes[2].set_ylim(-50, 150) # 设置Y轴范围以匹配论文

    axes[2].set_xlabel('t (min)')
    plt.tight_layout(rect=[0, 0.03, 1, 0.96])

    if output_file:
        print(f"正在保存图像到: {output_file}")
        plt.savefig(output_file, dpi=300)
    else:
        plt.show()

if __name__ == '__main__':
    # 为了直接运行和测试，我们硬编码序列名称
    # python your_script_name.py --path ./EUROC --seq MH_04_difficult
    parser = argparse.ArgumentParser(description="复现论文中EuRoC真值的RPY姿态角图。")
    parser.add_argument('--path', type=str, default='./', help="数据集目录的基本路径。")
    parser.add_argument('--seq', type=str, default='MH_04_difficult', help='要处理的序列名称。')
    parser.add_argument('--save', type=str, default=None, help='可选的图像保存路径。')
    args = parser.parse_args()

    plot_ground_truth_rpy(args.path, args.seq, args.save)