/**
 * @file AhrsPoseVisualizerNode.cpp
 * @brief Implementation of the AhrsPoseVisualizerNode class.
 * @author Gemini
 * @date 2025-08-22
 */
#include "ahrs_pose_visualizer.hpp"

namespace imu_visualization
{

AhrsPoseVisualizerNode::AhrsPoseVisualizerNode(ros::NodeHandle& nh) 
    : nh_(nh), ground_truth_pose_received_(false)
{
    ros::NodeHandle nh_private("~");
    std::string pose_topic;
    std::string model_topic;
    
    nh_private.param<std::string>("pose_topic", pose_topic, "/ahrs/pose");
    nh_private.param<std::string>("model_topic", model_topic, "/visualization/ahrs_drone");
    nh_private.param<std::string>("child_frame_id", child_frame_id_, "ahrs_body");

    pose_sub_ = nh_.subscribe(pose_topic, 10, &AhrsPoseVisualizerNode::poseCallback, this);
    model_pub_ = nh_.advertise<visualization_msgs::Marker>(model_topic, 10);

    // Subscribe to ground truth pose for position following
    ground_truth_pose_sub_ = nh_.subscribe("/ground_truth/pose", 10, &AhrsPoseVisualizerNode::groundTruthPoseCallback, this);
}

void AhrsPoseVisualizerNode::groundTruthPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    latest_ground_truth_pose_ = msg->pose;
    if (!ground_truth_pose_received_) {
        ground_truth_pose_received_ = true;
    }
}

void AhrsPoseVisualizerNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!ground_truth_pose_received_) {
        ROS_WARN_THROTTLE(2.0, "Waiting for ground truth pose for position following...");
        return;
    }

    // 1. Publish TF transform
    geometry_msgs::TransformStamped transform;
    transform.header = msg->header;
    transform.child_frame_id = child_frame_id_;
    transform.transform.translation.x = latest_ground_truth_pose_.position.x;
    transform.transform.translation.y = latest_ground_truth_pose_.position.y;
    transform.transform.translation.z = latest_ground_truth_pose_.position.z;
    transform.transform.rotation = msg->pose.orientation;
    tf_broadcaster_.sendTransform(transform);

    // 2. Publish drone model marker (Blue)
    visualization_msgs::Marker body_disk;
    body_disk.header = msg->header;
    body_disk.ns = "ahrs_drone_body";
    body_disk.id = 0;
    body_disk.type = visualization_msgs::Marker::CYLINDER;
    body_disk.action = visualization_msgs::Marker::ADD;
    body_disk.pose.position = latest_ground_truth_pose_.position; // Use ground truth position
    body_disk.pose.orientation = msg->pose.orientation;
    body_disk.scale.x = 0.8;
    body_disk.scale.y = 0.8;
    body_disk.scale.z = 0.02;
    body_disk.color.a = 0.7;
    body_disk.color.r = 0.2;
    body_disk.color.g = 0.4;
    body_disk.color.b = 1.0; // Blue
    model_pub_.publish(body_disk);
}

}