/**
 * @file imu_visualizer.cpp
 * @brief ImuVisualizer类的实现文件。
 * @author Gemini
 * @date 2025-08-21
 */

#include "imu_visualizer.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

namespace imu_visualization
{

// 构造函数
ImuVisualizer::ImuVisualizer(ros::NodeHandle& nh) : nh_(nh)
{
    // --- 从参数服务器加载参数 ---
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("imu_topic", imu_topic_, "/imu0");
    nh_private.param<std::string>("ground_truth_pose_topic", ground_truth_pose_topic_, "/ground_truth/pose");
    nh_private.param<double>("uncertainty_radius", uncertainty_radius_, 0.5);
    nh_private.param<double>("true_attitude_arrow_length", true_attitude_arrow_length_, 1.5);

    // --- 初始化ROS订阅者和发布者 ---
    imu_sub_ = nh_.subscribe(imu_topic_, 100, &ImuVisualizer::imuCallback, this);
    ground_truth_sub_ = nh_.subscribe(ground_truth_pose_topic_, 10, &ImuVisualizer::groundTruthCallback, this);

    trajectory_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization/trajectory", 10);
    drone_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization/drone_model", 10);
    true_attitude_arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization/true_attitude", 10);
    accel_uncertainty_sphere_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization/accel_uncertainty", 10);
}

// 地面真值回调函数
void ImuVisualizer::groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    latest_ground_truth_pose_ = msg->pose;
    ground_truth_received_ = true;

    // --- 使用最原始的姿态数据，不做任何额外修正 ---
    tf2::Quaternion q_original;
    tf2::fromMsg(msg->pose.orientation, q_original);

    // --- 1. 发布地面真值TF坐标系 (使用原始姿态) ---
    geometry_msgs::TransformStamped gt_transform;
    gt_transform.header.stamp = msg->header.stamp;
    gt_transform.header.frame_id = "world";
    gt_transform.child_frame_id = "ground_truth_body";
    gt_transform.transform.translation.x = msg->pose.position.x;
    gt_transform.transform.translation.y = msg->pose.position.y;
    gt_transform.transform.translation.z = msg->pose.position.z;
    gt_transform.transform.rotation = msg->pose.orientation; // 直接使用原始数据
    tf_broadcaster_.sendTransform(gt_transform);

    // --- 2. 发布真值无人机模型（灰色圆盘） ---
    visualization_msgs::Marker rotor_disk;
    rotor_disk.header.frame_id = "ground_truth_body";
    rotor_disk.header.stamp = msg->header.stamp;
    rotor_disk.ns = "drone_model";
    rotor_disk.id = 0;
    rotor_disk.type = visualization_msgs::Marker::CYLINDER;
    rotor_disk.action = visualization_msgs::Marker::ADD;
    rotor_disk.pose.orientation.w = 1.0;
    rotor_disk.scale.x = 0.8; rotor_disk.scale.y = 0.8; rotor_disk.scale.z = 0.01;
    rotor_disk.color.a = 0.7; rotor_disk.color.r = 0.8; rotor_disk.color.g = 0.8; rotor_disk.color.b = 0.8;
    drone_marker_pub_.publish(rotor_disk);

    // --- 3. 发布真值姿态箭头 (蓝色, 指向载体Z轴正上方) ---
    visualization_msgs::Marker true_arrow;
    true_arrow.header.frame_id = "world";
    true_arrow.header.stamp = msg->header.stamp;
    true_arrow.ns = "true_attitude_arrow";
    true_arrow.id = 0;
    true_arrow.type = visualization_msgs::Marker::ARROW;
    true_arrow.action = visualization_msgs::Marker::ADD;
    true_arrow.points.push_back(msg->pose.position);

    // 将箭头改为指向Z轴，以匹配加速度计的重力方向
    tf2::Vector3 z_vec(0, 0, true_attitude_arrow_length_); 
    tf2::Vector3 end_vec = tf2::quatRotate(q_original, z_vec);
    geometry_msgs::Point end_point;
    end_point.x = msg->pose.position.x + end_vec.x();
    end_point.y = msg->pose.position.y + end_vec.y();
    end_point.z = msg->pose.position.z + end_vec.z();
    true_arrow.points.push_back(end_point);
    true_arrow.scale.x = 0.05; true_arrow.scale.y = 0.1;
    true_arrow.color.a = 1.0; true_arrow.color.r = 0.0; true_arrow.color.g = 0.0; true_arrow.color.b = 1.0; // 蓝色
    true_attitude_arrow_pub_.publish(true_arrow);

    // --- 4. 发布运动轨迹（绿色线） ---
    trajectory_points_.push_back(msg->pose.position);
    visualization_msgs::Marker trajectory_line;
    trajectory_line.header.frame_id = "world";
    trajectory_line.header.stamp = msg->header.stamp;
    trajectory_line.ns = "trajectory";
    trajectory_line.id = 0;
    trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_line.action = visualization_msgs::Marker::ADD;
    trajectory_line.pose.orientation.w = 1.0;
    trajectory_line.scale.x = 0.05;
    trajectory_line.color.a = 1.0; trajectory_line.color.r = 0.0; trajectory_line.color.g = 1.0; trajectory_line.color.b = 0.0;
    trajectory_line.points = trajectory_points_;
    trajectory_marker_pub_.publish(trajectory_line);
}

// IMU回调函数
void ImuVisualizer::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (!ground_truth_received_) return;

    // --- 发布加速度测量不确定性球 (红色) ---
    visualization_msgs::Marker uncertainty_sphere;
    uncertainty_sphere.header.frame_id = "world";
    uncertainty_sphere.header.stamp = msg->header.stamp;
    uncertainty_sphere.ns = "accel_uncertainty_sphere";
    uncertainty_sphere.id = 0;
    uncertainty_sphere.type = visualization_msgs::Marker::SPHERE;
    uncertainty_sphere.action = visualization_msgs::Marker::ADD;

    // --- 核心计算逻辑 (简化) ---
    // 直接使用真值位姿中的原始旋转，因为我们假设IMU和载体坐标系现在是对齐的
    tf2::Quaternion q_original;
    tf2::fromMsg(latest_ground_truth_pose_.orientation, q_original);

    // 获取IMU在自己坐标系下测量的原始加速度向量
    tf2::Vector3 accel_in_imu_frame(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    // 使用原始旋转，将加速度向量从IMU系直接转换到世界系
    tf2::Vector3 accel_in_world_frame = tf2::quatRotate(q_original, accel_in_imu_frame);

    // 归一化并缩放到与真值箭头相同的长度，以进行比较
    accel_in_world_frame.normalize();
    accel_in_world_frame *= true_attitude_arrow_length_;

    // 计算球心最终位置: 真值位置 + 代表测量方向的向量
    uncertainty_sphere.pose.position.x = latest_ground_truth_pose_.position.x + accel_in_world_frame.x();
    uncertainty_sphere.pose.position.y = latest_ground_truth_pose_.position.y + accel_in_world_frame.y();
    uncertainty_sphere.pose.position.z = latest_ground_truth_pose_.position.z + accel_in_world_frame.z();
    uncertainty_sphere.pose.orientation.w = 1.0;

    // 球的半径由参数设定
    uncertainty_sphere.scale.x = uncertainty_radius_ * 2.0;
    uncertainty_sphere.scale.y = uncertainty_radius_ * 2.0;
    uncertainty_sphere.scale.z = uncertainty_radius_ * 2.0;
    uncertainty_sphere.color.a = 0.4; // 40%不透明度
    uncertainty_sphere.color.r = 1.0; uncertainty_sphere.color.g = 0.2; uncertainty_sphere.color.b = 0.2; // 红色
    
    accel_uncertainty_sphere_pub_.publish(uncertainty_sphere);
}

}  // namespace imu_visualization
