/**
 * @file ground_truth_visualizer.cpp
 * @brief GroundTruthVisualizer类的实现, 负责可视化无人机的真值位姿。
 * @author Gemini
 * @date 2025-08-21
 */
#include "ground_truth_visualizer.hpp"

namespace imu_visualization
{

GroundTruthVisualizer::GroundTruthVisualizer(ros::NodeHandle& nh) : nh_(nh)
{
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("ground_truth_pose_topic", ground_truth_topic_, "/ground_truth/pose");

    ground_truth_sub_ = nh_.subscribe(ground_truth_topic_, 10, &GroundTruthVisualizer::groundTruthCallback, this);

    // 一个发布者用于发布复合形状的不同部分，使用不同的id
    // Latching (true) is enabled to keep the last message available for new subscribers.
    body_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization/ground_truth_body", 1, true);
}

void GroundTruthVisualizer::groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 1. 发布TF变换
    geometry_msgs::TransformStamped gt_transform;
    gt_transform.header.stamp = msg->header.stamp;
    gt_transform.header.frame_id = "world";
    gt_transform.child_frame_id = "ground_truth_body";
        gt_transform.transform.translation.x = msg->pose.position.x;
    gt_transform.transform.translation.y = msg->pose.position.y;
    gt_transform.transform.translation.z = msg->pose.position.z;

    // -- Gemini: Applying URF to FLU coordinate system correction --
    // 从消息中获取原始姿态 (q_orig)
    tf2::Quaternion q_original;
    tf2::fromMsg(msg->pose.orientation, q_original);

    // 定义一个修正四元数 (q_corr)，用于将 URF (上-右-前) 坐标系旋转到 FLU (前-左-上) 坐标系
    // 这等效于围绕向量 (1, 0, 1) 旋转 180 度
    tf2::Quaternion q_correction(0.70710678, 0.0, 0.70710678, 0.0);

    // 将原始姿态右乘修正四元数，得到最终姿态 q_final = q_orig * q_corr
    tf2::Quaternion q_final = q_original * q_correction;
    q_final.normalize();

    // 更新TF变换中的旋转
    gt_transform.transform.rotation = tf2::toMsg(q_final);
    tf_broadcaster_.sendTransform(gt_transform);

    // 2. 发布复合形状的各个部分
    // 所有Marker都附着在 ground_truth_body 框架上，所以它们的pose都是相对于这个框架的

    // 2.1 大圆盘 (机身)
    visualization_msgs::Marker body_disk;
    body_disk.header.frame_id = "ground_truth_body";
    body_disk.header.stamp = msg->header.stamp;
    body_disk.ns = "drone_body";
    body_disk.id = 0;
    body_disk.type = visualization_msgs::Marker::CYLINDER;
    body_disk.action = visualization_msgs::Marker::ADD;
    body_disk.pose.orientation.w = 1.0;
    body_disk.scale.x = 0.8; // 直径
    body_disk.scale.y = 0.8; // 直径
    body_disk.scale.z = 0.02; // 厚度
    body_disk.color.a = 0.7; 
    body_disk.color.r = 0.8; 
    body_disk.color.g = 0.8; 
    body_disk.color.b = 0.8;
    body_marker_pub_.publish(body_disk);

    // 2.2 小圆柱 (向下的指示器)
    visualization_msgs::Marker down_cylinder;
    down_cylinder.header.frame_id = "ground_truth_body";
    down_cylinder.header.stamp = msg->header.stamp;
    down_cylinder.ns = "drone_body";
    down_cylinder.id = 1;
    down_cylinder.type = visualization_msgs::Marker::CYLINDER;
    down_cylinder.action = visualization_msgs::Marker::ADD;
    down_cylinder.pose.position.z = -0.11; // 把它放在大圆盘下方
    down_cylinder.pose.orientation.w = 1.0;
    down_cylinder.scale.x = 0.05;
    down_cylinder.scale.y = 0.05;
    down_cylinder.scale.z = 0.2; // 长度
    down_cylinder.color.a = 0.9;
    down_cylinder.color.r = 0.5;
    down_cylinder.color.g = 0.5;
    down_cylinder.color.b = 0.5;
    body_marker_pub_.publish(down_cylinder);

    // 2.3 小箭头 (前进方向指示器)
    visualization_msgs::Marker forward_arrow;
    forward_arrow.header.frame_id = "ground_truth_body";
    forward_arrow.header.stamp = msg->header.stamp;
    forward_arrow.ns = "drone_body";
    forward_arrow.id = 2;
    forward_arrow.type = visualization_msgs::Marker::ARROW;
    forward_arrow.action = visualization_msgs::Marker::ADD;
    forward_arrow.pose.position.z = -0.11; // 和小圆柱在同一高度
    // 箭头默认指向自己的+X方向，所以我们只需要设置好它的姿态
    // 我们想让它指向机体的+X方向，所以姿态不需要改变 (w=1)
    forward_arrow.pose.orientation.w = 1.0;
    forward_arrow.scale.x = 0.4; // 箭头长度
    forward_arrow.scale.y = 0.05; // 箭头宽度
    forward_arrow.scale.z = 0.05; // 箭头高度
    forward_arrow.color.a = 1.0;
    forward_arrow.color.r = 1.0;
    forward_arrow.color.g = 0.0;
    forward_arrow.color.b = 0.0;
    body_marker_pub_.publish(forward_arrow);
}

} // namespace imu_visualization
