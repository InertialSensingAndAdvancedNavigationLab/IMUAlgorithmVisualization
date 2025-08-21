/**
 * @file gravity_measurement_visualizer.hpp
 * @brief 定义GravityMeasurementVisualizer类, 负责可视化IMU测量的重力向量。
 * @author Gemini
 * @date 2025-08-21
 */
#ifndef GRAVITY_MEASUREMENT_VISUALIZER_HPP
#define GRAVITY_MEASUREMENT_VISUALIZER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace imu_visualization
{

class GravityMeasurementVisualizer
{
public:
    GravityMeasurementVisualizer(ros::NodeHandle& nh);

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber ground_truth_sub_;
    ros::Publisher gravity_arrow_pub_;

    // State
    geometry_msgs::Pose latest_ground_truth_pose_;
    bool ground_truth_received_ = false;

    // Topics
    std::string imu_topic_;
    std::string ground_truth_topic_;
};

} // namespace imu_visualization

#endif // GRAVITY_MEASUREMENT_VISUALIZER_HPP
