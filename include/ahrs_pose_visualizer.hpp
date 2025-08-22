/**
 * @file AhrsPoseVisualizerNode.hpp
 * @brief Defines a node that visualizes a pose as a drone model.
 * @author Gemini
 * @date 2025-08-22
 */
#ifndef AHRS_POSE_VISUALIZER_NODE_HPP
#define AHRS_POSE_VISUALIZER_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>

namespace imu_visualization
{

class AhrsPoseVisualizerNode
{
public:
    AhrsPoseVisualizerNode(ros::NodeHandle& nh);

private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void groundTruthPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber ground_truth_pose_sub_;
    ros::Publisher model_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // For position following
    geometry_msgs::Pose latest_ground_truth_pose_;
    bool ground_truth_pose_received_;
    std::string child_frame_id_;
};

} // namespace imu_visualization

#endif // AHRS_POSE_VISUALIZER_NODE_HPP
