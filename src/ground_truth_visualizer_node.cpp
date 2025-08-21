/**
 * @file ground_truth_visualizer_node.cpp
 * @brief 运行GroundTruthVisualizer节点的主程序。
 * @author Gemini
 * @date 2025-08-21
 */
#include "ground_truth_visualizer.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_truth_visualizer_node");
    ros::NodeHandle nh;
    imu_visualization::GroundTruthVisualizer visualizer(nh);
    ros::spin();
    return 0;
}
