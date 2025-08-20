/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含我们定义的算法基类
#include "DefaultAhrs.hpp"

/**
 * @class       DirectIntegrator
 * @brief       一个基于陀螺仪数据进行直接积分的AHRS算法实现
 *  @extends     public DefaultAhrs
 *  @details     此类是 DefaultAhrs 接口的基础实现。它使用陀螺仪数据更新
 * 姿态，并使用一个包含零偏不稳定性的6x6协方差模型来跟踪理论
 * 上的不确定性增长。
 **/
class DirectIntegrator : public DefaultAhrs
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
    DirectIntegrator();

    /**
     * @brief       虚析构函数
     **/
    ~DirectIntegrator() override;

    /**
     * @brief       初始化算法
     *
     * @param       参数名称: configManager                 数据类型:        const ConfigurationManager&
     * @return      初始化成功返回 true                  数据类型:        bool
     **/
    bool initialize(const ConfigurationManager& configManager) override;

    /**
     * @brief       根据单帧IMU数据更新算法状态
     *
     * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
     **/
    void update(const ImuMeasurement& imUmuData) override;

    /**
     * @brief       获取当前算法的姿态结果
     *
     * @return      当前最优的姿态估计                数据类型:        Quaternion
     **/
    Quaternion getOrientation() const override;
    
    /**
     * @brief       检查算法是否已被初始化
     *
     * @return      如果已初始化则返回 true           数据类型:        bool
     **/
    bool isInitialized() const override;

    /**
     * @brief       执行调试与数据导出任务
     *
     * @param       参数名称: outputFilePath              数据类型:        const std::string&
     **/
    void runDebugExport(const std::string& outputFilePath) const override;

/// 保护成员 (允许派生类访问)
protected:
    /// @brief 名义状态：姿态四元数
    Quaternion orientation;

    /// @brief 误差状态协方差矩阵 P (6x6)。[δθ (3), δb (3)]
    Eigen::Matrix<double, 6, 6> covariance;
    
    /// @brief 过程噪声协方差矩阵 Q (6x6)。
    Eigen::Matrix<double, 6, 6> processNoiseQ;

    /// @brief 标记滤波器是否已经成功初始化
    bool bIsInitialized;
    
    /// @brief 记录上一个数据点的时间戳，用于计算deltaTime
    double lastTimestamp;

    /// @brief (用于调试导出) 存储整个过程的遥测数据
    struct TelemetryData
    {
        double timestamp;
        Quaternion orientation;
        /// *** 关键修正：将协方差对角线向量从 3x1 升级到 6x1 ***
        Eigen::Matrix<double, 6, 1> covarianceDiagonal;
    };
    std::vector<TelemetryData> telemetryHistory;
};