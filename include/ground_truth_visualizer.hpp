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


namespace imu_visualization
{

class GroundTruthVisualizer
{
public:
    GroundTruthVisualizer(ros::NodeHandle& nh);

private:
    void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber ground_truth_sub_;
    ros::Publisher body_marker_pub_;
    ros::Publisher down_vector_marker_pub_;
    ros::Publisher forward_arrow_marker_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string ground_truth_topic_;
};

} // namespace imu_visualization

#endif // GROUND_TRUTH_VISUALIZER_HPP
