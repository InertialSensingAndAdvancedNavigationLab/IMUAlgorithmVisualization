/**
 * @file gravity_measurement_visualizer_node.cpp
 * @brief 运行GravityMeasurementVisualizer节点的主程序。
 * @author Gemini
 * @date 2025-08-21
 */
#include "gravity_measurement_visualizer.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gravity_measurement_visualizer_node");
    ros::NodeHandle nh;
    imu_visualization::GravityMeasurementVisualizer visualizer(nh);
    ros::spin();
    return 0;
}
