#ifndef IMU_ALGORITHM_VISUALIZATION_IMU_VISUALIZER_HPP
#define IMU_ALGORITHM_VISUALIZATION_IMU_VISUALIZER_HPP

/// 包含 ROS C++ 客户端库
#include "ros/ros.h"
/// 包含标准 IMU 消息类型
#include "sensor_msgs/Imu.h"
/// 包含可视化标记消息类型
#include "visualization_msgs/Marker.h"
/// 包含几何点消息类型
#include "geometry_msgs/Point.h"

/**
 * @namespace   imu_visualization
 * @brief       封装 IMU 可视化相关的功能
 **/
namespace imu_visualization
{
/**
 * @class       ImuVisualizer
 * @brief       处理 IMU 数据并将其可视化为 RViz 中的标记
 *  @details     该类订阅 IMU 话题，并为每个接收到的消息发布一个表示加速度向量的箭头标记。
 **/
class ImuVisualizer
{
/// 公共接口
public:
    /**
     * @brief       ImuVisualizer 类的构造函数
     *  @details     初始化 ROS 节点、发布者和订阅者。
     *
     * @param       参数名称: nh                            数据类型:        ros::NodeHandle&
     *  @details     用于与 ROS 系统交互的节点句柄。
     **/
    ImuVisualizer(ros::NodeHandle& nh);

/// 私有实现细节
private:
    /**
     * @brief       IMU 消息的回调函数
     * @param       参数名称: msg                           数据类型:        const sensor_msgs::Imu::ConstPtr&
     *  @details     指向接收到的 IMU 消息的常量指针。
     **/
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    /// @brief 用于发布可视化标记的 ROS 发布者
    ros::Publisher markerPublisher;
    /// @brief 用于订阅 IMU 数据的 ROS 订阅者
    ros::Subscriber imuSubscriber;
};
}  // namespace imu_visualization

#endif  // IMU_ALGORITHM_VISUALIZATION_IMU_VISUALIZER_HPP