/**
 * @file gravity_measurement_visualizer.hpp
 * @brief 定义GravityMeasurementVisualizer类, 负责可视化IMU测量的重力向量并提供标定功能。
 * @author Gemini
 * @date 2025-08-22
 */
#ifndef GRAVITY_MEASUREMENT_VISUALIZER_HPP
#define GRAVITY_MEASUREMENT_VISUALIZER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>

namespace imu_visualization
{

class GravityMeasurementVisualizer
{
public:
    GravityMeasurementVisualizer(ros::NodeHandle& nh);

private:
    // Internal helper to start calibration
    void startCalibration(double duration);

    // Callbacks
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool calibrateCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void publishCalibratedGravity(const ros::TimerEvent& event);

    // ROS Handles
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber ground_truth_sub_;
    ros::Publisher gravity_arrow_pub_;
    ros::Publisher calibrated_gravity_pub_;
    ros::ServiceServer calibrate_service_;
    ros::Timer pub_timer_;

    // State
    geometry_msgs::Pose latest_ground_truth_pose_;
    bool ground_truth_received_ = false;
    tf2::Vector3 calibrated_gravity_;

    // Calibration-related state
    bool auto_calibration_has_run_;
    double auto_calibrate_duration_param_;
    bool is_calibrating_;
    ros::Time calibration_start_time_;
    double calibration_duration_;
    std::vector<geometry_msgs::Vector3> gravity_samples_;

    // Topics
    std::string imu_topic_;
    std::string ground_truth_topic_;
};

} // namespace imu_visualization

#endif // GRAVITY_MEASUREMENT_VISUALIZER_HPP