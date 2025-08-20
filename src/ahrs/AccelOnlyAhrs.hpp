/// 使用 #pragma once 防止头文件重复包含
#pragma once

#include "DefaultAhrs.hpp"
#include "IO/TumTrajectoryWriter.hpp" // For PoseStamped

/**
 * @class       AccelOnlyAhrs
 * @brief       一个仅使用加速度计并在线标定重力的姿态估计器
 *  @extends     public DefaultAhrs
 *  @details     此类是一个纯粹的“倾角计”(Tilt Sensor)。它在每一帧都
 * 假设加速度计测量到的向量是重力的反作用力方向，并以此来计算当前的
 * Roll 和 Pitch 角。Yaw角是不可观测的。
 *
 * 此版本包含一个在线初始化状态机：在开始阶段，它会自动利用
 * 静态数据（假设）来标定实际的重力加速度模长，以提高后续
 * 动态不确定性(R)计算的准确性。
 *
 *   @note        此算法无法估计偏航角(Yaw)。初始Yaw角将被任意设为0。
 **/
class AccelOnlyAhrs : public DefaultAhrs
{
/// 公共接口
public:
    /**
     * @brief       强制内存对齐以安全使用Eigen。
     **/
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief       默认构造函数
     **/
    AccelOnlyAhrs();
    
    /**
     * @brief       虚析构函数
     **/
    ~AccelOnlyAhrs() override = default;

    /**
     * @brief       配置算法并重置初始化状态
     * @param       参数名称: configManager                 数据类型:        const ConfigurationManager&
     * @return      初始化成功返回 true                  数据类型:        bool
     **/
    bool initialize(const ConfigurationManager& configManager) override;

    /**
     * @brief       根据单帧IMU数据更新算法状态 (状态机入口)
     * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
     **/
    void update(const ImuMeasurement& imuData) override;

    /**
     * @brief       获取当前算法的姿态结果
     * @return      当前最优的姿态估计                数据类型:        Quaternion
     **/
    Quaternion getOrientation() const override;
    
    /**
     * @brief       检查算法是否已被初始化
     * @return      如果已初始化则返回 true           数据类型:        bool
     **/
    bool isInitialized() const override;

    /**
     * @brief       执行调试与数据导出任务
     * @param       参数名称: outputFilePath              数据类型:        const std::string&
     **/
    void runDebugExport(const std::string& outputFilePath) const override;

/// 私有实现细节
private:
    /**
     * @brief       处理在线初始化阶段的数据 (私有方法)
     * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
     */
    void initializing(const ImuMeasurement& imuData);

    /// @brief 核心状态：姿态四元数
    Quaternion orientation;
    /// @brief 姿态协方差矩阵 P (3x3)，直接反映当前帧的观测不确定性
    Eigen::Matrix3d covariance;

    /// @brief 存储从配置中读取的参数
    const ConfigurationManager* configManager;
    /// @brief 加速度计测量噪声的方差 (sigma_a^2)
    double accelNoiseVariance;

    /// @brief 内部状态标志
    bool bIsInitialized;
    
    /// @brief 在线标定出的重力加速度模长
    double gravityMagnitude;

    // --- 初始化状态机所需的成员 ---
    /// @brief 静态初始化的目标结束时间戳
    double staticInitEndTime;
    /// @brief 用于初始化的加速度计数据累加和
    Vector3d accelSumForInit;
    /// @brief 用于初始化的样本计数器
    int sampleCountForInit;
    
    /// @brief 遥测数据历史记录
    struct TelemetryData
    {
        double timestamp;
        Quaternion orientation;
        Vector3d covarianceDiagonal;
        Vector3d rawAcceleration;
    };
    std::vector<TelemetryData> telemetryHistory;
};