/**
 * @file gravity_measurement_visualizer.cpp
 * @brief GravityMeasurementVisualizer类的实现。
 * @author Gemini
 * @date 2025-08-22
 */
#include "gravity_measurement_visualizer.hpp"
#include <numeric>

namespace imu_visualization
{

GravityMeasurementVisualizer::GravityMeasurementVisualizer(ros::NodeHandle& nh) 
    : nh_(nh), ground_truth_received_(false), 
      auto_calibration_has_run_(false), is_calibrating_(false)
{
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("imu_topic", imu_topic_, "/imu0");
    nh_private.param<std::string>("ground_truth_pose_topic", ground_truth_topic_, "/ground_truth/pose");
    nh_private.param("auto_calibrate_duration", auto_calibrate_duration_param_, 0.0);

    imu_sub_ = nh_.subscribe(imu_topic_, 100, &GravityMeasurementVisualizer::imuCallback, this);
    ground_truth_sub_ = nh_.subscribe(ground_truth_topic_, 10, &GravityMeasurementVisualizer::groundTruthCallback, this);

    gravity_arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization/measured_gravity", 10);
    calibrated_gravity_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/calibration/gravity_vector", 10, true); // Latched

    calibrate_service_ = nh_.advertiseService("/calibrate_gravity", &GravityMeasurementVisualizer::calibrateCallback, this);

    pub_timer_ = nh_.createTimer(ros::Duration(0.1), &GravityMeasurementVisualizer::publishCalibratedGravity, this);

    // Initialize with standard gravity
    calibrated_gravity_.setValue(0.0, 0.0, -9.80665);
}

void GravityMeasurementVisualizer::startCalibration(double duration) {
    if (is_calibrating_) {
        ROS_WARN("Calibration already in progress.");
        return;
    }
    ROS_INFO("Starting gravity calibration for %.2f seconds...", duration);
    gravity_samples_.clear();
    calibration_duration_ = duration;
    calibration_start_time_ = ros::Time::now();
    is_calibrating_ = true;
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
    // Trigger auto-calibration on the first received message if enabled
    if (auto_calibrate_duration_param_ > 0.0 && !auto_calibration_has_run_) {
        startCalibration(auto_calibrate_duration_param_);
        auto_calibration_has_run_ = true;
    }

    if (is_calibrating_) {
        // Check if calibration duration has passed
        if ((ros::Time::now() - calibration_start_time_).toSec() < calibration_duration_) {
            geometry_msgs::Vector3 acc;
            acc.x = msg->linear_acceleration.x;
            acc.y = msg->linear_acceleration.y;
            acc.z = msg->linear_acceleration.z;
            gravity_samples_.push_back(acc);
        } else {
            // Finish calibration
            is_calibrating_ = false;
            if (gravity_samples_.empty() || !ground_truth_received_) {
                ROS_ERROR("Gravity calibration failed: No IMU messages or no ground truth pose received.");
            } else {
                tf2::Vector3 sum(0,0,0);
                for (const auto& acc : gravity_samples_) {
                    sum.setX(sum.x() + acc.x);
                    sum.setY(sum.y() + acc.y);
                    sum.setZ(sum.z() + acc.z);
                }
                tf2::Vector3 average_acc_body = sum / gravity_samples_.size();

                // BUG FIX: Rotate the measured vector from body to world before setting it as the calibrated gravity
                tf2::Quaternion q_world_to_body;
                tf2::fromMsg(latest_ground_truth_pose_.orientation, q_world_to_body);
                tf2::Vector3 average_acc_world = tf2::quatRotate(q_world_to_body, average_acc_body);
                
                calibrated_gravity_ = -average_acc_world;

                std::stringstream ss;
                ss << "Gravity calibrated successfully. New world vector: ["
                   << calibrated_gravity_.x() << ", "
                   << calibrated_gravity_.y() << ", "
                   << calibrated_gravity_.z() << "] Magnitude: " << calibrated_gravity_.length();
                ROS_INFO_STREAM(ss.str());
                gravity_samples_.clear();
            }
        }
    }

    if (!ground_truth_received_) {
        return;
    }

    // The rest of this function is for the original visualization and remains unchanged.
    tf2::Vector3 gravity_vector_imu_frame(-msg->linear_acceleration.x, -msg->linear_acceleration.y, -msg->linear_acceleration.z);
    tf2::Quaternion q_world_to_body;
    tf2::fromMsg(latest_ground_truth_pose_.orientation, q_world_to_body);
    tf2::Vector3 gravity_vector_world_frame = tf2::quatRotate(q_world_to_body, gravity_vector_imu_frame);

    visualization_msgs::Marker gravity_arrow;
    gravity_arrow.header.frame_id = "world";
    gravity_arrow.header.stamp = msg->header.stamp;
    gravity_arrow.ns = "measured_gravity";
    gravity_arrow.id = 0;
    gravity_arrow.type = visualization_msgs::Marker::ARROW;
    gravity_arrow.action = visualization_msgs::Marker::ADD;
    gravity_arrow.points.push_back(latest_ground_truth_pose_.position);
    geometry_msgs::Point end_point;
    end_point.x = latest_ground_truth_pose_.position.x + gravity_vector_world_frame.x();
    end_point.y = latest_ground_truth_pose_.position.y + gravity_vector_world_frame.y();
    end_point.z = latest_ground_truth_pose_.position.z + gravity_vector_world_frame.z();
    gravity_arrow.points.push_back(end_point);
    gravity_arrow.scale.x = 0.08; gravity_arrow.scale.y = 0.15; gravity_arrow.color.a = 0.9;
    gravity_arrow.color.r = 0.0; gravity_arrow.color.g = 1.0; gravity_arrow.color.b = 1.0;
    gravity_arrow_pub_.publish(gravity_arrow);
}

bool GravityMeasurementVisualizer::calibrateCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    startCalibration(2.0); // Default duration of 2 seconds for manual trigger
    res.success = true;
    res.message = "Gravity calibration started for 2.0 seconds.";
    return true;
}

void GravityMeasurementVisualizer::publishCalibratedGravity(const ros::TimerEvent& event)
{
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.vector.x = calibrated_gravity_.x();
    msg.vector.y = calibrated_gravity_.y();
    msg.vector.z = calibrated_gravity_.z();
    calibrated_gravity_pub_.publish(msg);
}

} // namespace imu_visualization