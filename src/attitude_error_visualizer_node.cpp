/**
 * @file attitude_error_visualizer_node.cpp
 * @brief a ROS node that visualizes attitude errors.
 * @author Gemini
 * @date 2025-08-22
 */
#include "attitude_error_visualizer.hpp"

/**
 * @brief Main function to run the AttitudeErrorVisualizer node.
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "attitude_error_visualizer");
    ros::NodeHandle nh;
    imu_visualization::AttitudeErrorVisualizer node(nh);
    ros::spin();
    return 0;
}
