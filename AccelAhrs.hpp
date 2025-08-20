/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含我们定义的算法基类
#include "ahrs/DirectIntegrator.hpp"
/// 包含全局配置管理器
#include "config/Configuration.hpp"

/**
 * @class       AccelAhrs
 * @brief       一个融合加速度计来修正姿态漂移的AHRS算法
 *  @extends     public DirectIntegrator
 *  @details     此类继承自 DirectIntegrator，并在此基础上增加了一个基于EKF思想的
 * 修正步骤。当IMU处于低动态（接近静止）时，它会利用加速度计测量
 * 到的重力向量来修正陀螺仪积分产生的横滚(Roll)和俯仰(Pitch)漂移。
 * 
 * 它的设计核心是为了“过程可观测性”，会记录包括“新息(innovation)”、
 * “卡尔曼增益(Gain)”和“协方差(P)”在内的详细遥测数据，以便于
 * 通过可视化来深入理解传感器融合的过程。
 **/
class AccelAhrs : public DirectIntegrator
{
/// 公共接口
public:
    /**
     * @brief       强制内存对齐，以安全地使用Eigen的固定大小类型。
     **/
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief       默认构造函数
     **/
    AccelAhrs();

    /**
     * @brief       虚析构函数
     **/
    ~AccelAhrs() override;

    /**
     * @brief       初始化算法
     *  @details     重写此函数以加载加速度计融合所需的特定配置参数。
     *
     * @param       参数名称: configManager                 数据类型:        const ConfigurationManager&
     * @return      初始化成功返回 true                  数据类型:        bool
     **/
    bool initialize(const ConfigurationManager& configManager) override;

    /**
     * @brief       根据单帧IMU数据更新算法状态
     *  @details     实现“预测-更新”循环。预测步骤复用父类逻辑，更新步骤则
     * 在满足条件时融合加速度计数据。
     *
     * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
     **/
    void update(const ImuMeasurement& imuData) override;

    /**
     * @brief       执行调试与数据导出任务
     *  @details     重写此函数以导出包含EKF内部状态的扩展遥测数据。
     *
     * @param       参数名称: outputFilePath              数据类型:        const std::string&
     **/
    void runDebugExport(const std::string& outputFilePath) const override;

/// 私有实现细节
private:
    /// @brief (用于调试导出) 存储整个过程的扩展遥测数据
    struct ExtendedTelemetryData
    {
        /// @brief 时间戳
        double timestamp;
        /// @brief 最终姿态
        Quaternion orientation;
        /// @brief 协方差矩阵P的对角线元素 (6x1)
        Eigen::Matrix<double, 6, 1> covarianceDiagonal;
        /// @brief 原始加速度计读数 (用于分析)
        Vector3d rawAcceleration;
        /// @brief 加速度计计算的姿态误差(新息) [rad]
        Vector2d innovation;
        /// @brief 用于修正姿态的卡尔曼增益对角线部分
        Vector2d kalmanGain;
        /// @brief 标记当前帧是否执行了修正步骤
        bool correctionStepActive;
    };
    /// @brief 存储扩展遥测数据的历史记录
    std::vector<ExtendedTelemetryData> extendedTelemetryHistory;

    /// @brief 指向全局配置管理器的指针，用于访问算法参数
    const AlgorithmConfig* algorithmConfig;
    
    /// @brief 测量噪声协方差矩阵 R (2x2)，对应Roll和Pitch的测量噪声
    Eigen::Matrix2d measurementNoiseR;

    /// @brief 地球重力加速度的期望大小 [m/s^2]
    double gravityMagnitude;
};