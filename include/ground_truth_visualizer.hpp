/**
 * @file ground_truth_visualizer.hpp
 * @brief 定义GroundTruthVisualizer类, 负责可视化无人机的真值位姿。
 * @author Gemini
 * @date 2025-08-21
 */
#ifndef GROUND_TRUTH_VISUALIZER_HPP
#define GROUND_TRUTH_VISUALIZER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <imu_algorithm_visualization/SetAttitude.h>
#include <imu_algorithm_visualization/SetLocation.h>


namespace imu_visualization
{

class GroundTruthVisualizer
{
public:
    GroundTruthVisualizer(ros::NodeHandle& nh);

private:
    void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // Service Callbacks
    bool setAttitudeCallback(imu_algorithm_visualization::SetAttitude::Request& req, imu_algorithm_visualization::SetAttitude::Response& res);
    bool setLocationCallback(imu_algorithm_visualization::SetLocation::Request& req, imu_algorithm_visualization::SetLocation::Response& res);
    bool resetPoseCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    ros::NodeHandle nh_;
    ros::Subscriber ground_truth_sub_;
    ros::Publisher body_marker_pub_;
    ros::Publisher down_vector_marker_pub_;
    ros::Publisher forward_arrow_marker_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Service Servers
    ros::ServiceServer set_attitude_service_;
    ros::ServiceServer set_location_service_;
    ros::ServiceServer reset_pose_service_;

    std::string ground_truth_topic_;

    // Locked pose state
    geometry_msgs::Pose locked_pose_;
    bool attitude_is_locked_;
    bool position_is_locked_;
};

} // namespace imu_visualization

#endif // GROUND_TRUTH_VISUALIZER_HPP
