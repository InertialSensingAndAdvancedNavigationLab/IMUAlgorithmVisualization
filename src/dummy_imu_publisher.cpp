#include "dummy_imu_publisher.hpp"

namespace imu_visualization
{
DummyImuPublisher::DummyImuPublisher(ros::NodeHandle& nh)
    : nodeHandle(nh), loopRate(50), imuFrameId("imu_link")
{
    imuPublisher = nodeHandle.advertise<sensor_msgs::Imu>("/imu/data", 100);
}

void DummyImuPublisher::run()
{
    while (ros::ok())
    {
        /// 创建一个 Imu 消息实例
        sensor_msgs::Imu imuMsg;
        /// 获取当前时间
        ros::Time currentTime = ros::Time::now();
        /// 填充消息头
        imuMsg.header.stamp = currentTime;
        /// 设置坐标系 ID
        imuMsg.header.frame_id = imuFrameId;
        /// 填充方向数据 (此处为单位四元数，无旋转)
        imuMsg.orientation.w = 1.0;
        /// 填充角速度数据 (此处为零)
        /// 填充线性加速度数据 (模拟一个动态变化的向量)
        /// 使用时间函数创建一个在 X-Y 平面内旋转的向量
        imuMsg.linear_acceleration.x = 9.8 * sin(currentTime.toSec());
        imuMsg.linear_acceleration.y = 9.8 * cos(currentTime.toSec());
        /// Z 轴加速度保持恒定 (模拟重力)
        imuMsg.linear_acceleration.z = 9.8;
        /// 发布 IMU 消息
        imuPublisher.publish(imuMsg);
        /// 处理任何挂起的回调
        ros::spinOnce();
        /// 按照设定的频率休眠
        loopRate.sleep();
    }
}

}  // namespace imu_visualization