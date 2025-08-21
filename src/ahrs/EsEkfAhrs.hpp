/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含抽象算法基类
#include "ahrs/DefaultAhrs.hpp"
/// 包含全局配置管理器
#include "config/Configuration.hpp"

/**
 * @class       EsEkfAhrs
 * @brief       一个基于误差状态扩展卡尔曼滤波(ES-EKF)的AHRS算法
 *  @extends     public DefaultAhrs
 *  @details     此类继承自 DirectIntegrator，并在此基础上增加了一个基于ES-EKF思想的
 * 修正步骤。当IMU处于低动态（接近静止）时，它会利用加速度计测量
 * 到的重力向量来修正陀螺仪积分产生的横滚(Roll)和俯仰(Pitch)漂移。
 * 
 * 它的设计核心是为了“过程可观测性”，会记录包括“新息(innovation)”、
 * “卡尔曼增益(Gain)”和“协方差(P)”在内的详细遥测数据，以便于
 * 通过可视化来深入理解传感器融合的过程。
 **/
class EsEkfAhrs : public DefaultAhrs
{
/// 公共接口
public:
    /// @brief 强制内存对齐，以安全地使用Eigen的固定大小类型
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief 默认构造函数
    EsEkfAhrs();

    /// @brief 虚析构函数
    ~EsEkfAhrs() override;

    /// @brief 初始化算法，加载加速度计融合所需的特定配置参数
    bool initialize(const ConfigurationManager& configManager) override;

    /// @brief 根据单帧IMU数据更新算法状态，实现“预测-更新”循环
    void update(const ImuMeasurement& imuData) override;

    /// @brief 获取当前算法的姿态结果
    Quaternion getOrientation() const override;

    /// @brief 检查算法是否已被初始化
    bool isInitialized() const override;

    /// @brief 执行调试与数据导出任务，导出包含EKF内部状态的扩展遥测数据
    void runDebugExport(const std::string& outputFilePath) const override;

/// 保护成员 (允许派生类访问)
private:
    /**
     * @struct      ExtendedTelemetryData
     * @brief       (用于调试导出) 存储整个过程的扩展遥测数据
    **/
    struct ExtendedTelemetryData
    {
        /// @brief 时间戳
        double timestamp;
        /// @brief 最终姿态
        Quaternion orientation;
        /// @brief (用于对比) 仅由加速度计计算的姿态
        Quaternion accelOnlyOrientation;
        /// @brief 陀螺仪零偏估计
        Vector3d gyroBias;
        /// @brief 协方差矩阵P的对角线元素 (6x1)
        Eigen::Matrix<double, 6, 1> covarianceDiagonal;
        /// @brief 原始加速度计读数 (用于分析)
        Vector3d rawAcceleration;
        /// @brief 加速度计计算的姿态误差(新息) [rad]，仅包含横滚和俯仰
        Vector2d innovation;
        /// @brief 用于修正姿态的卡尔曼增益对角线部分 (仅横滚和俯仰)
        Vector2d kalmanGain;
        /// @brief 标记当前帧是否执行了修正步骤
        bool correctionStepActive;
    };
    /// @brief 存储扩展遥测数据的历史记录
    std::vector<ExtendedTelemetryData> extendedTelemetryHistory;

    /// @brief 指向算法特定配置的指针
    const EsEkfAhrsConfig* esEkfConfig;
    
    // --- 名义状态 ---
    /// @brief 名义状态：姿态四元数
    Quaternion orientation;
    /// @brief 名义状态：陀螺仪零偏
    Vector3d gyroBias;

    // --- EKF 误差状态与协方差 ---
    /// @brief 误差状态协方差矩阵 P (6x6)
    Eigen::Matrix<double, 6, 6> covariance;
    /// @brief 过程噪声协方差矩阵 Q (6x6)
    Eigen::Matrix<double, 6, 6> processNoiseQ;
    /// @brief 观测噪声协方差矩阵 R (3x3)
    Eigen::Matrix3d measurementNoiseR;

    // --- 其他状态 ---
    /// @brief 算法的内部运行状态
    enum class State { UNCONFIGURED, INITIALIZING, RUNNING };
    /// @brief 当前的运行状态
    State currentState;
    /// @brief 标记滤波器是否已经成功初始化
    bool bIsInitialized;
    /// @brief 记录上一个数据点的时间戳
    double lastTimestamp;
    /// @brief 指向全局配置管理器的指针
    const ConfigurationManager* configManager;
    /// @brief 在线初始化结束的时间戳
    double staticInitEndTime;
    /// @brief 用于累加静态加速度数据以进行标定
    Vector3d accelSumForInit;
    /// @brief 用于静态标定的样本计数
    int sampleCountForInit;
    /// @brief 从IMU坐标系(RDF)到机体坐标系(FRD)的旋转矩阵
    Eigen::Matrix3d C_body_imu;
    /// @brief 重力向量参考
    Vector3d gravity;
};