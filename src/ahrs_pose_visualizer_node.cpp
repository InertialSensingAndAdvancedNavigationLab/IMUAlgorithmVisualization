/**
 * @file ahrs_pose_visualizer_node.cpp
 * @brief Main function to run the AhrsPoseVisualizerNode.
 * @author Gemini
 * @date 2025-08-22
 */
#include "ahrs_pose_visualizer.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ahrs_pose_visualizer");
    ros::NodeHandle nh;
    imu_visualization::AhrsPoseVisualizerNode node(nh);
    ros::spin();
    return 0;
}
