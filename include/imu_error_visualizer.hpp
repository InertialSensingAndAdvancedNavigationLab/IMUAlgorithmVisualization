/**
 * @file imu_error_visualizer.hpp
 * @brief 定义ImuErrorVisualizer类, 对比真实与理想IMU读数并可视化误差。
 * @author Gemini
 * @date 2025-08-22
 */
#ifndef IMU_ERROR_VISUALIZER_HPP
#define IMU_ERROR_VISUALIZER_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace imu_visualization
{

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::AccelStamped, sensor_msgs::Imu> MySyncPolicy;

class ImuErrorVisualizer
{
public:
    ImuErrorVisualizer(ros::NodeHandle& nh);

private:
    void dataCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
                      const geometry_msgs::AccelStamped::ConstPtr& accel_msg,
                      const sensor_msgs::Imu::ConstPtr& imu_msg);
    
    void gravityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    // ROS Handles
    ros::NodeHandle nh_;
    ros::Publisher error_arrow_pub_;
    ros::Subscriber calibrated_gravity_sub_;

    // Message Filters
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::AccelStamped>> accel_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Imu>> imu_sub_;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    // State
    tf2::Vector3 calibrated_gravity_;
    bool gravity_received_ = false;
};

} // namespace imu_visualization

#endif // IMU_ERROR_VISUALIZER_HPP
