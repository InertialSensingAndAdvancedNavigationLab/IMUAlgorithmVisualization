/**
 * @file gravity_measurement_visualizer.cpp
 * @brief GravityMeasurementVisualizer类的实现。
 * @author Gemini
 * @date 2025-08-21
 */
#include "gravity_measurement_visualizer.hpp"

namespace imu_visualization
{

GravityMeasurementVisualizer::GravityMeasurementVisualizer(ros::NodeHandle& nh) : nh_(nh)
{
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("imu_topic", imu_topic_, "/imu0");
    nh_private.param<std::string>("ground_truth_pose_topic", ground_truth_topic_, "/ground_truth/pose");

    imu_sub_ = nh_.subscribe(imu_topic_, 100, &GravityMeasurementVisualizer::imuCallback, this);
    ground_truth_sub_ = nh_.subscribe(ground_truth_topic_, 10, &GravityMeasurementVisualizer::groundTruthCallback, this);

    gravity_arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization/measured_gravity", 10);
}

void GravityMeasurementVisualizer::groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    latest_ground_truth_pose_ = msg->pose;
    if (!ground_truth_received_) {
        ground_truth_received_ = true;
    }
}

void GravityMeasurementVisualizer::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if (!ground_truth_received_) {
        return;
    }

    // 1. 获取IMU测量的加速度，并取反得到重力向量
    tf2::Vector3 gravity_vector_imu_frame(
        -msg->linear_acceleration.x,
        -msg->linear_acceleration.y,
        -msg->linear_acceleration.z
    );

    // 2. 获取最新的真值姿态，用于将重力向量转换到世界坐标系
    tf2::Quaternion q_world_to_body;
    tf2::fromMsg(latest_ground_truth_pose_.orientation, q_world_to_body);

    // 3. 将测量到的重力向量从IMU（载体）坐标系转换到世界坐标系
    tf2::Vector3 gravity_vector_world_frame = tf2::quatRotate(q_world_to_body, gravity_vector_imu_frame);

    // 4. 创建并发布可视化箭头
    visualization_msgs::Marker gravity_arrow;
    gravity_arrow.header.frame_id = "world";
    gravity_arrow.header.stamp = msg->header.stamp;
    gravity_arrow.ns = "measured_gravity";
    gravity_arrow.id = 0;
    gravity_arrow.type = visualization_msgs::Marker::ARROW;
    gravity_arrow.action = visualization_msgs::Marker::ADD;

    // 箭头的起点是无人机的当前位置
    gravity_arrow.points.push_back(latest_ground_truth_pose_.position);

    // 箭头的终点
    geometry_msgs::Point end_point;
    end_point.x = latest_ground_truth_pose_.position.x + gravity_vector_world_frame.x();
    end_point.y = latest_ground_truth_pose_.position.y + gravity_vector_world_frame.y();
    end_point.z = latest_ground_truth_pose_.position.z + gravity_vector_world_frame.z();
    gravity_arrow.points.push_back(end_point);

    // 设置箭头尺寸和颜色
    gravity_arrow.scale.x = 0.08; // shaft diameter
    gravity_arrow.scale.y = 0.15; // head diameter
    gravity_arrow.color.a = 0.9;
    gravity_arrow.color.r = 0.0;
    gravity_arrow.color.g = 1.0;
    gravity_arrow.color.b = 1.0; // Cyan

    gravity_arrow_pub_.publish(gravity_arrow);
}

} // namespace imu_visualization
