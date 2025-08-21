#ifndef IMU_ALGORITHM_VISUALIZATION_DUMMY_IMU_PUBLISHER_HPP
#define IMU_ALGORITHM_VISUALIZATION_DUMMY_IMU_PUBLISHER_HPP

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <cmath>

/**
 * @namespace   imu_visualization
 * @brief       封装 IMU 可视化相关的功能
 **/
namespace imu_visualization
{
/**
 * @class       DummyImuPublisher
 * @brief       发布模拟的 IMU 数据
 **/
class DummyImuPublisher
{
public:
    /**
     * @brief       构造函数
     * @param       nh ROS 节点句柄
     **/
    DummyImuPublisher(ros::NodeHandle& nh);
    /**
     * @brief       运行发布循环
     **/
    void run();
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher imuPublisher;
    ros::Rate loopRate;
    std::string imuFrameId;
};
}  // namespace imu_visualization
#endif  // IMU_ALGORITHM_VISUALIZATION_DUMMY_IMU_PUBLISHER_HPP