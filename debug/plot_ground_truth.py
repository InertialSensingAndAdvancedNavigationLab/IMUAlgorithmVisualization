import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os
from typing import Optional

def plot_euroc_ground_truth(dataset_base_path: str, sequence_name: str, output_file: Optional[str] = None):
    """
    读取并绘制EuRoC数据集的真值轨迹。

    EuRoC的真值是在一个Vicon动捕系统定义的坐标系下，该坐标系遵循ENU约定：
    - p_x: East
    - p_y: North
    - p_z: Up

    Args:
        dataset_base_path (str): 数据集根目录的路径 (例如 './data')。
        sequence_name (str): 要处理的序列名称 (例如 'MH_01_easy')。
        output_file (str, optional): 如果提供，则将图表保存到此文件路径。
    """
    # 构造真值文件的完整路径
    ground_truth_path = os.path.join(
        dataset_base_path,
        sequence_name,
        "mav0",
        "state_groundtruth_estimate0",
        "data.csv"
    )

    if not os.path.exists(ground_truth_path):
        print(f"Error: Ground truth file not found at '{ground_truth_path}'")
        return

    # 使用pandas读取CSV文件。EuRoC的CSV文件使用'#'作为注释符
    print(f"Reading ground truth data from: {ground_truth_path}")
    df = pd.read_csv(ground_truth_path)
    df.columns = [col.strip() for col in df.columns]  # 清理列名中的前后空格

    # 提取位置数据 (p_x, p_y, p_z)
    # 根据EuRoC文档，这些列分别对应 East, North, Up
    x = df['p_RS_R_x [m]']
    y = df['p_RS_R_y [m]']
    z = df['p_RS_R_z [m]']

    # 创建3D绘图
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制轨迹
    ax.plot(x, y, z, label=f'Ground Truth: {sequence_name}')
    ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color='green', s=100, label='Start')
    ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color='red', s=100, label='End')

    # 设置坐标轴标签，明确表示这是ENU坐标系
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Up (m)')
    ax.set_title('Ground Truth Trajectory (ENU Frame)')
    ax.legend()
    ax.grid(True)

    # 保持坐标轴比例一致，这对于轨迹可视化非常重要
    ax.set_aspect('equal', 'box')

    if output_file:
        print(f"Saving plot to: {output_file}")
        plt.savefig(output_file, dpi=300)
    else:
        plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Plot EuRoC ground truth trajectory.")
    parser.add_argument('--path', type=str, default='./data', help='Base path to the dataset directory.')
    parser.add_argument('--seq', type=str, required=True, help='Sequence name to process (e.g., V2_01_easy).')
    parser.add_argument('--save', type=str, default=None, help='Optional path to save the plot image.')
    args = parser.parse_args()

    plot_euroc_ground_truth(args.path, args.seq, args.save)