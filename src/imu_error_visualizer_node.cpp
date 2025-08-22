/**
 * @file imu_error_visualizer_node.cpp
 * @brief a ROS node that visualizes IMU errors.
 * @author Gemini
 * @date 2025-08-22
 */
#include "imu_error_visualizer.hpp"

/**
 * @brief Main function to run the ImuErrorVisualizer node.
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_error_visualizer");
    ros::NodeHandle nh;
    imu_visualization::ImuErrorVisualizer node(nh);
    ros::spin();
    return 0;
}
