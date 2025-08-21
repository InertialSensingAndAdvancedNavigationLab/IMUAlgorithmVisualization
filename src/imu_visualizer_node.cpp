#include "ros/ros.h"
#include "imu_visualizer.hpp"

/**
 * @brief       节点的主函数
 *  @details     初始化 ROS，创建 ImuVisualizer 实例，并进入 ROS 事件循环。
 * @return      程序退出代码                          数据类型:        int
 **/
int main(int argc, char** argv)
{
    /// 初始化 ROS 节点，命名为 "imu_visualizer_node"
    ros::init(argc, argv, "imu_visualizer_node");
    /// 创建一个节点句柄
    ros::NodeHandle nh;
    /// 创建 ImuVisualizer 类的实例
    imu_visualization::ImuVisualizer visualizer(nh);
    /// 进入自旋，等待回调函数被触发
    ros::spin();
    /// 返回成功状态
    return 0;
}