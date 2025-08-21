/**
 * @file imu_visualizer.hpp
 * @brief IMU可视化工具的头文件，定义了ImuVisualizer类。
 * @author Gemini
 * @date 2025-08-21
 */
#ifndef IMU_ALGORITHM_VISUALIZATION_IMU_VISUALIZER_HPP
#define IMU_ALGORITHM_VISUALIZATION_IMU_VISUALIZER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <string>

namespace imu_visualization
{

/**
 * @class ImuVisualizer
 * @brief 核心类，负责处理数据、计算和发布所有可视化信息。
 */
class ImuVisualizer
{
public:
    ImuVisualizer(ros::NodeHandle& nh);

private:
    // --- Callback Functions ---
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // --- ROS Interface ---
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber ground_truth_sub_;
    ros::Publisher trajectory_marker_pub_;
    ros::Publisher drone_marker_pub_;
    ros::Publisher true_attitude_arrow_pub_;
    ros::Publisher accel_uncertainty_sphere_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // --- State & Data ---
    geometry_msgs::Pose latest_ground_truth_pose_;
    geometry_msgs::Quaternion latest_corrected_orientation_;
    bool ground_truth_received_ = false;
    std::vector<geometry_msgs::Point> trajectory_points_;

    // --- Parameters ---
    std::string imu_topic_;
    std::string ground_truth_pose_topic_;
    double uncertainty_radius_;
    double true_attitude_arrow_length_ = 1.5; // Length of the true attitude arrow
};

}  // namespace imu_visualization

#endif  // IMU_ALGORITHM_VISUALIZATION_IMU_VISUALIZER_HPP
