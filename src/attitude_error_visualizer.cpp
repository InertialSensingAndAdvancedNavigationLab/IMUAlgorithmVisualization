/**
 * @file attitude_error_visualizer.cpp
 * @brief AttitudeErrorVisualizer类的实现。
 * @author Gemini
 * @date 2025-08-22
 */
#include "attitude_error_visualizer.hpp"

namespace imu_visualization
{

AttitudeErrorVisualizer::AttitudeErrorVisualizer(ros::NodeHandle& nh) 
    : nh_(nh), imu_received_(false), pose_received_(false), gravity_received_(false)
{
    ros::NodeHandle nh_private("~");
    nh_private.param("error_circle_radius", error_circle_radius_, 0.1);

    // Initialize TF listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publishers and Subscribers
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/attitude_error", 10);
    imu_sub_ = nh_.subscribe("/imu0", 10, &AttitudeErrorVisualizer::imuCallback, this);
    pose_sub_ = nh_.subscribe("/ground_truth/pose", 10, &AttitudeErrorVisualizer::poseCallback, this);
    gravity_sub_ = nh_.subscribe("/calibration/gravity_vector", 1, &AttitudeErrorVisualizer::gravityCallback, this);

    // Timer to update visualization
    update_timer_ = nh_.createTimer(ros::Duration(0.02), &AttitudeErrorVisualizer::timerCallback, this); // 50 Hz
}

void AttitudeErrorVisualizer::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    latest_imu_ = *msg;
    if (!imu_received_) imu_received_ = true;
}

void AttitudeErrorVisualizer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    latest_pose_ = msg->pose;
    if (!pose_received_) pose_received_ = true;
}

void AttitudeErrorVisualizer::gravityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    tf2::fromMsg(msg->vector, calibrated_gravity_);
    if (!gravity_received_) gravity_received_ = true;
}

void AttitudeErrorVisualizer::timerCallback(const ros::TimerEvent& event)
{
    if (!imu_received_ || !pose_received_ || !gravity_received_) {
        return; // Wait for all data to be received
    }

    // --- Get transform from IMU frame to Body frame ---
    geometry_msgs::TransformStamped t_body_from_imu;
    try {
        // Use the timestamp from the IMU data to get the transform at that specific time
        t_body_from_imu = tf_buffer_->lookupTransform("ground_truth_body", latest_imu_.header.frame_id, latest_imu_.header.stamp, ros::Duration(0.1));
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(2.0, "Could not get transform from %s to ground_truth_body: %s", latest_imu_.header.frame_id.c_str(), ex.what());
        return;
    }

    // --- Calculations ---
    tf2::Quaternion q_world_to_body;
    tf2::fromMsg(latest_pose_.orientation, q_world_to_body);

    // Vector of true gravity in the body frame
    tf2::Vector3 g_true_body = tf2::quatRotate(q_world_to_body.inverse(), calibrated_gravity_);

    // Get measured acceleration in IMU frame
    tf2::Vector3 a_measured_imu;
    tf2::fromMsg(latest_imu_.linear_acceleration, a_measured_imu);

    // Transform measured acceleration to body frame using the lookup
    tf2::Quaternion r_body_from_imu;
    tf2::fromMsg(t_body_from_imu.transform.rotation, r_body_from_imu);
    tf2::Vector3 a_measured_body = tf2::quatRotate(r_body_from_imu, a_measured_imu);

    // Vector of measured gravity in the body frame
    tf2::Vector3 g_measured_body = -a_measured_body;

    // --- Visualization ---
    visualization_msgs::MarkerArray marker_array;
    ros::Time now = ros::Time::now();

    // 1. True Gravity Arrow (Green)
    visualization_msgs::Marker true_gravity_marker;
    true_gravity_marker.header.frame_id = "ground_truth_body";
    true_gravity_marker.header.stamp = now;
    true_gravity_marker.ns = "attitude_error";
    true_gravity_marker.id = 0;
    true_gravity_marker.type = visualization_msgs::Marker::ARROW;
    true_gravity_marker.action = visualization_msgs::Marker::ADD;
    true_gravity_marker.points.push_back(geometry_msgs::Point()); // Start at origin
    geometry_msgs::Point true_end_point;
    tf2::toMsg(g_true_body, true_end_point);
    true_gravity_marker.points.push_back(true_end_point);
    true_gravity_marker.scale.x = 0.02; true_gravity_marker.scale.y = 0.04;
    true_gravity_marker.color.a = 1.0; true_gravity_marker.color.r = 0.0; true_gravity_marker.color.g = 1.0; true_gravity_marker.color.b = 0.0;
    marker_array.markers.push_back(true_gravity_marker);

    // 2. Measured Gravity Arrow (Cyan)
    visualization_msgs::Marker measured_gravity_marker = true_gravity_marker;
    measured_gravity_marker.id = 1;
    measured_gravity_marker.points[1].x = g_measured_body.x();
    measured_gravity_marker.points[1].y = g_measured_body.y();
    measured_gravity_marker.points[1].z = g_measured_body.z();
    measured_gravity_marker.color.g = 1.0; measured_gravity_marker.color.b = 1.0; measured_gravity_marker.color.r = 0.0;
    marker_array.markers.push_back(measured_gravity_marker);

    // 3. Error Circle (Red)
    visualization_msgs::Marker circle_marker;
    circle_marker.header = true_gravity_marker.header;
    circle_marker.ns = "attitude_error";
    circle_marker.id = 2;
    circle_marker.type = visualization_msgs::Marker::CYLINDER;
    circle_marker.action = visualization_msgs::Marker::ADD;
    tf2::toMsg(g_true_body, circle_marker.pose.position);
    // Orientation: align cylinder's Z with the true gravity vector
    tf2::Vector3 z_axis(0,0,1);
    tf2::Vector3 axis = z_axis.cross(g_true_body.normalized());
    double angle = z_axis.angle(g_true_body.normalized());
    tf2::Quaternion q_rot;
    q_rot.setRotation(axis, angle);
    circle_marker.pose.orientation = tf2::toMsg(q_rot);
    circle_marker.scale.x = error_circle_radius_ * 2.0;
    circle_marker.scale.y = error_circle_radius_ * 2.0;
    circle_marker.scale.z = 0.005;
    circle_marker.color.a = 0.5; circle_marker.color.r = 1.0; circle_marker.color.g = 0.0; circle_marker.color.b = 0.0;
    marker_array.markers.push_back(circle_marker);

    marker_pub_.publish(marker_array);
}

} // namespace imu_visualization
