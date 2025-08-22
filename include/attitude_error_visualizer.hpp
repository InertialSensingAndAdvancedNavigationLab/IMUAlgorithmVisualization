/**
 * @file attitude_error_visualizer.hpp
 * @brief 定义AttitudeErrorVisualizer类, 在机体坐标系下可视化姿态误差。
 * @author Gemini
 * @date 2025-08-22
 */
#ifndef ATTITUDE_ERROR_VISUALIZER_HPP
#define ATTITUDE_ERROR_VISUALIZER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace imu_visualization
{

class AttitudeErrorVisualizer
{
public:
    AttitudeErrorVisualizer(ros::NodeHandle& nh);

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gravityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);

    // ROS Handles
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber gravity_sub_;
    ros::Publisher marker_pub_;
    ros::Timer update_timer_;

    // TF Handling
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Latest received data
    sensor_msgs::Imu latest_imu_;
    geometry_msgs::Pose latest_pose_;
    tf2::Vector3 calibrated_gravity_;

    // State flags
    bool imu_received_ = false;
    bool pose_received_ = false;
    bool gravity_received_ = false;

    // Config
    double error_circle_radius_;
};

} // namespace imu_visualization

#endif // ATTITUDE_ERROR_VISUALIZER_HPP
