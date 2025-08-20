/// 包含类头文件
#include "ahrs/AccelAhrs.hpp"

/// 包含其他标准库
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

/**
 * @brief       默认构造函数
 *  @details     初始化所有成员变量为安全默认值。
 **/
AccelAhrs::AccelAhrs()
    : DirectIntegrator()
    , algorithmConfig(nullptr)
    , measurementNoiseR(Eigen::Matrix2d::Zero())
    , gravityMagnitude(9.81)
{
    /// 构造函数体
}

/**
 * @brief       虚析构函数
 **/
AccelAhrs::~AccelAhrs()
{
    /// 析构函数体
}

/**
 * @brief       初始化算法
 **/
bool AccelAhrs::initialize(const ConfigurationManager& configManager)
{
    /// 1. 调用父类的初始化，它会设置好 Q 矩阵和初始 P 矩阵
    if (!DirectIntegrator::initialize(configManager))
    {
        /// 如果父类初始化失败，则直接返回
        return false;
    }

    /// 2. 存储算法特定配置的指针
    this->algorithmConfig = &configManager.algorithm;
    
    /// 3. 根据配置计算测量噪声矩阵 R
    double accelNoiseVar = std::pow(this->algorithmConfig->accelNoiseRms, 2);
    this->measurementNoiseR << accelNoiseVar, 0,
                               0, accelNoiseVar;

    /// 4. 存储重力大小
    this->gravityMagnitude = this->algorithmConfig->gravityMagnitude;

    /// 5. 清空扩展遥测历史记录
    this->extendedTelemetryHistory.clear();

    /// 打印初始化信息
    std::cout << "[AccelAhrs] Initialized successfully." << std::endl;
    std::cout << "  - Accel Noise RMS: " << this->algorithmConfig->accelNoiseRms << " m/s^2" << std::endl;
    std::cout << "  - Low Dynamic Threshold: " << this->algorithmConfig->lowDynamicThreshold << " m/s^2" << std::endl;

    /// 返回成功
    return true;
}

/**
 * @brief       根据单帧IMU数据更新算法状态
 **/
void AccelAhrs::update(const ImuMeasurement& imuData)
{
    /// 如果未初始化或配置丢失，则不执行任何操作
    if (!this->isInitialized() || !this->algorithmConfig)
    {
        /// 直接返回
        return;
    }

    /// 首次运行时，仅记录时间戳
    if (this->lastTimestamp < 0)
    {
        this->lastTimestamp = imuData.timestamp;
        /// 返回
        return;
    }

    /// --- 1. 预测阶段 (Prediction) ---
    /// 计算时间差
    double deltaTime = imuData.timestamp - this->lastTimestamp;
    /// 如果时间戳无效则跳过
    if (deltaTime <= 0) return;
    /// 更新时间戳
    this->lastTimestamp = imuData.timestamp;

    /// a. 姿态积分 (复用父类逻辑)
    const Vector3d& gyro = imuData.angularVelocity;
    Quaternion delta_q(1.0, 0.5 * gyro.x() * deltaTime, 0.5 * gyro.y() * deltaTime, 0.5 * gyro.z() * deltaTime);
    this->orientation = (this->orientation * delta_q).normalized();

    /// b. 协方差传播 (复用父类逻辑)
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity() * deltaTime;
    Eigen::Matrix<double, 6, 6> Q_d = this->processNoiseQ * deltaTime;
    this->covariance = F * this->covariance * F.transpose() + Q_d;

    /// --- 2. 更新/修正阶段 (Update/Correction) ---
    /// 初始化本帧的遥测变量
    Vector2d innovation = Vector2d::Zero();
    Vector2d kalmanGain = Vector2d::Zero();
    bool correctionStepActive = false;
    
    /// a. 检查是否处于低动态条件
    double accelMagnitude = imuData.linearAcceleration.norm();
    if (std::abs(accelMagnitude - this->gravityMagnitude) < this->algorithmConfig->lowDynamicThreshold)
    {
        /// 标记修正步骤已激活
        correctionStepActive = true;

        /// b. 计算新息 (Innovation) y = z - h(x)
        /// 测量值 z: 加速度计测量的重力方向
        Vector3d z_measured = imuData.linearAcceleration / accelMagnitude;
        /// 预测值 h(x): 当前姿态预测的重力方向
        Vector3d z_predicted = this->orientation.inverse() * Vector3d(0, 0, -1.0);
        /// 新息 y: 测量与预测的误差，对于小角度，近似为叉积
        /// 我们只关心对Roll(X)和Pitch(Y)的修正
        innovation = z_predicted.cross(z_measured).head<2>();

        /// c. 计算测量雅可比矩阵 H
        /// H 是测量模型对误差状态的偏导。对于重力向量观测，H = [skew(g_pred), 0_3x3]
        Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
        H(0, 1) = -z_predicted.z(); H(0, 2) = z_predicted.y(); /// d(err_x)/d(theta)
        H(1, 0) = z_predicted.z();  H(1, 2) = -z_predicted.x(); /// d(err_y)/d(theta)

        /// d. 计算卡尔曼增益 K
        Eigen::Matrix2d S = H * this->covariance * H.transpose() + this->measurementNoiseR;
        Eigen::Matrix<double, 6, 2> K = this->covariance * H.transpose() * S.inverse();
        /// 记录增益用于遥测
        kalmanGain << K(0, 0), K(1, 1);

        /// e. 更新状态 (姿态 和 零偏)
        Eigen::Matrix<double, 6, 1> errorState = K * innovation;
        Vector3d deltaAngle = errorState.head<3>();
        /// 修正姿态
        Quaternion delta_q_correction = Quaternion(1.0, 0.5 * deltaAngle.x(), 0.5 * deltaAngle.y(), 0.5 * deltaAngle.z()).normalized();
        this->orientation = (this->orientation * delta_q_correction).normalized();
        /// (可选) 修正零偏，这里暂不实现以简化模型，保持与DirectIntegrator一致
        
        /// f. 更新协方差 P
        Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
        this->covariance = I_KH * this->covariance * I_KH.transpose() + K * this->measurementNoiseR * K.transpose(); // Joseph form for stability
    }

    /// --- 3. 记录扩展遥测数据 ---
    this->extendedTelemetryHistory.push_back({
        imuData.timestamp,
        this->orientation,
        this->covariance.diagonal(),
        imuData.linearAcceleration,
        innovation,
        kalmanGain,
        correctionStepActive
    });
}

/**
 * @brief       执行调试与数据导出任务
 **/
void AccelAhrs::runDebugExport(const std::string& outputFilePath) const
{
    /// 如果历史记录为空，则不导出
    if (this->extendedTelemetryHistory.empty())
    {
        std::cout << "  [Debug Export] No telemetry data to export for AccelAhrs." << std::endl;
        return;
    }

    /// 创建并打开文件流
    std::ofstream outFile(outputFilePath);
    /// 检查文件是否成功打开
    if (!outFile.is_open())
    {
        std::cerr << "  Error: Could not open debug export file: " << outputFilePath << std::endl;
        return;
    }

    /// 写入CSV表头
    outFile << "timestamp,"
            << "q_w,q_x,q_y,q_z,"
            << "P_att_xx,P_att_yy,P_att_zz,P_bias_xx,P_bias_yy,P_bias_zz,"
            << "accel_x,accel_y,accel_z,"
            << "innovation_roll,innovation_pitch,"
            << "gain_roll,gain_pitch,"
            << "correction_active\n";

    /// 设置输出精度
    outFile << std::fixed << std::setprecision(9);

    /// 遍历历史记录并写入文件
    for (const auto& data : this->extendedTelemetryHistory)
    {
        outFile << data.timestamp << ","
                << data.orientation.w() << "," << data.orientation.x() << "," << data.orientation.y() << "," << data.orientation.z() << ","
                << data.covarianceDiagonal(0) << "," << data.covarianceDiagonal(1) << "," << data.covarianceDiagonal(2) << ","
                << data.covarianceDiagonal(3) << "," << data.covarianceDiagonal(4) << "," << data.covarianceDiagonal(5) << ","
                << data.rawAcceleration.x() << "," << data.rawAcceleration.y() << "," << data.rawAcceleration.z() << ","
                << data.innovation(0) << "," << data.innovation(1) << ","
                << data.kalmanGain(0) << "," << data.kalmanGain(1) << ","
                << (data.correctionStepActive ? 1 : 0) << "\n";
    }

    /// 关闭文件
    outFile.close();
    /// 打印成功信息
    std::cout << "  Debug export for AccelAhrs saved to: " << outputFilePath << std::endl;
}