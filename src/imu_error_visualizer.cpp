/**
 * @file imu_error_visualizer.cpp
 * @brief ImuErrorVisualizer类的实现。
 * @author Gemini
 * @date 2025-08-22
 */
#include "imu_error_visualizer.hpp"

namespace imu_visualization
{

ImuErrorVisualizer::ImuErrorVisualizer(ros::NodeHandle& nh) : nh_(nh), gravity_received_(false)
{
    // Initialize publishers and subscribers
    error_arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization/imu_error", 10);
    calibrated_gravity_sub_ = nh_.subscribe("/calibration/gravity_vector", 1, &ImuErrorVisualizer::gravityCallback, this);

    // Initialize message filters subscribers
    pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::PoseStamped>>(nh_, "/ground_truth/pose", 10);
    accel_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::AccelStamped>>(nh_, "/ground_truth/accel", 10);
    imu_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::Imu>>(nh_, "/imu0", 100);

    // Setup synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), *pose_sub_, *accel_sub_, *imu_sub_);
    sync_->registerCallback(boost::bind(&ImuErrorVisualizer::dataCallback, this, _1, _2, _3));

    // Initialize gravity with a default value
    calibrated_gravity_.setValue(0.0, 0.0, -9.80665);
}

void ImuErrorVisualizer::gravityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    tf2::fromMsg(msg->vector, calibrated_gravity_);
    if (!gravity_received_) {
        gravity_received_ = true;
    }
}

void ImuErrorVisualizer::dataCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
                                    const geometry_msgs::AccelStamped::ConstPtr& accel_msg,
                                    const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    if (!gravity_received_) {
        ROS_WARN_THROTTLE(2.0, "Waiting for calibrated gravity vector...");
        return;
    }

    // 1. Get all necessary data in tf2 types
    tf2::Vector3 a_real_body;
    tf2::fromMsg(imu_msg->linear_acceleration, a_real_body);

    tf2::Vector3 a_motion_world;
    tf2::fromMsg(accel_msg->accel.linear, a_motion_world);

    tf2::Quaternion q_world_to_body;
    tf2::fromMsg(pose_msg->pose.orientation, q_world_to_body);

    // 2. Calculate the ideal IMU reading in the body frame
    // a_ideal_body = R_body_from_world * (a_motion_world - g_world)
    tf2::Vector3 g_world = calibrated_gravity_;
    tf2::Quaternion R_body_from_world = q_world_to_body.inverse();
    tf2::Vector3 a_ideal_body = tf2::quatRotate(R_body_from_world, (a_motion_world - g_world));

    // 3. Calculate the error vector
    tf2::Vector3 a_error_body = a_real_body - a_ideal_body;

    // 4. Visualize the error vector as an arrow
    visualization_msgs::Marker error_arrow;
    error_arrow.header.frame_id = "ground_truth_body";
    error_arrow.header.stamp = imu_msg->header.stamp;
    error_arrow.ns = "imu_error";
    error_arrow.id = 0;
    error_arrow.type = visualization_msgs::Marker::ARROW;
    error_arrow.action = visualization_msgs::Marker::ADD;

    // Arrow starts at the origin of the body frame
    geometry_msgs::Point start_point; // Defaults to 0,0,0
    error_arrow.points.push_back(start_point);

    // Arrow ends at the tip of the error vector
    geometry_msgs::Point end_point;
    tf2::toMsg(a_error_body, end_point);
    error_arrow.points.push_back(end_point);

    // Scale and color
    error_arrow.scale.x = 0.03; // shaft diameter
    error_arrow.scale.y = 0.06; // head diameter
    error_arrow.color.a = 1.0;
    error_arrow.color.r = 1.0;
    error_arrow.color.g = 1.0;
    error_arrow.color.b = 0.0; // Yellow

    error_arrow_pub_.publish(error_arrow);
}

} // namespace imu_visualization
