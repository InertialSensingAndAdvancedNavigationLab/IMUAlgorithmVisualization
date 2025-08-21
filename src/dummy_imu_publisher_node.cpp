#include "ros/ros.h"
#include "dummy_imu_publisher.hpp"

/**
 * @brief       节点的主函数
 *  @details     初始化 ROS，并以固定频率发布模拟的 IMU 数据。
 * @return      程序退出代码                          数据类型:        int
 **/
int main(int argc, char** argv)
{
    /// 初始化 ROS 节点，命名为 "dummy_imu_publisher"
    ros::init(argc, argv, "dummy_imu_publisher");
    /// 创建一个节点句柄
    ros::NodeHandle nh;
    /// 创建发布器实例并运行
    imu_visualization::DummyImuPublisher publisher(nh);
    publisher.run();
    return 0;
}