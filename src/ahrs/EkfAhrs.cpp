/// 包含类头文件
#include "ahrs/EkfAhrs.hpp"

/// 包含其他标准库
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

/**
 * @brief       默认构造函数
 *  @details     初始化所有状态和矩阵。
 **/
EkfAhrs::EkfAhrs()
    : orientation(Quaternion::Identity())
    , gyroBias(Vector3d::Zero())
    , covariance(Eigen::Matrix<double, 6, 6>::Zero())
    , processNoiseQ(Eigen::Matrix<double, 6, 6>::Zero())
    , measurementNoiseR(Eigen::Matrix3d::Zero())
    , bIsInitialized(false)
    , lastTimestamp(-1.0)
    , gravity(0.0, 0.0, -9.81) // 内部坐标系 Z 轴朝下
{
    // 构造函数体为空
}

/**
 * @brief       配置算法
 *  @details     根据配置设置噪声参数(Q, R)和初始协方差(P)。
 *
 * @param       参数名称: configManager                 数据类型:        const ConfigurationManager&
 * @return      初始化成功返回 true                  数据类型:        bool
 **/
bool EkfAhrs::initialize(const ConfigurationManager& configManager)
{
    /// 获取硬件和算法配置
    const auto& hwConfig = configManager.hardware;
    const auto& algoConfig = configManager.ukfConfig; // 我们暂时复用UKF的配置

    /// --- 设置过程噪声 Q (6x6) ---
    /// Q 描述了陀螺仪随机游走和零偏不稳定性的不确定性
    /// Q = diag([sigma_g^2 * I, sigma_b^2 * I])
    double gyroNoiseVar = std::pow(hwConfig.gyroNoiseRms, 2);
    double gyroBiasVar = std::pow(hwConfig.gyroBiasInstabilityRms, 2);
    this->processNoiseQ.setZero();
    this->processNoiseQ.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyroNoiseVar;   // 姿态误差部分
    this->processNoiseQ.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * gyroBiasVar; // 零偏误差部分
    
    /// --- 设置初始协方差 P (6x6) ---
    /// P0 描述了初始状态的不确定性
    double initialAttitudeUncertaintyRad = algoConfig.initialAttitudeUncertaintyDeg * M_PI / 180.0;
    double initialGyroBiasUncertaintyRadPerSec = algoConfig.initialGyroBiasUncertaintyDps * M_PI / 180.0;
    this->covariance.setZero();
    this->covariance.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * std::pow(initialAttitudeUncertaintyRad, 2);
    this->covariance.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * std::pow(initialGyroBiasUncertaintyRadPerSec, 2);
    
    /// --- 设置基础观测噪声 R (3x3) ---
    double accelNoiseVar = std::pow(hwConfig.accelNoiseRms, 2);
    this->measurementNoiseR = Eigen::Matrix3d::Identity() * accelNoiseVar;

    /// --- 重置状态 ---
    this->orientation.setIdentity();
    this->gyroBias.setZero();
    this->lastTimestamp = -1.0;
    this->telemetryHistory.clear();
    this->bIsInitialized = false; // 等待第一帧数据来完成初始化

    return true;
}

/**
 * @brief       根据单帧IMU数据更新算法状态
 *  @details     执行完整的 ESKF 预测和更新步骤。
 *
 * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
 **/
void EkfAhrs::update(const ImuMeasurement& imuData)
{
    /// 如果是第一帧数据，则用它来初始化名义状态
    if (!this->bIsInitialized)
    {
        Vector3d idealUpVector(0.0, 0.0, -1.0);
        this->orientation = Quaternion::FromTwoVectors(imuData.linearAcceleration, idealUpVector).normalized();
        this->lastTimestamp = imuData.timestamp;
        this->bIsInitialized = true;
        return;
    }

    /// 计算时间步长
    double deltaTime = imuData.timestamp - this->lastTimestamp;
    if (deltaTime <= 0) { return; }
    this->lastTimestamp = imuData.timestamp;

    /// --- 1. 预测步骤 ---
    /// 1.1 传播名义状态 (直接积分)
    Vector3d correctedGyro = imuData.angularVelocity - this->gyroBias;
    this->propagateNominalState(correctedGyro, deltaTime);

    /// 1.2 传播误差协方差
    this->propagateErrorCovariance(correctedGyro, deltaTime);

    /// --- 2. 更新步骤 (使用加速度计) ---
    /// 2.1 计算观测雅可比 H
    Eigen::Matrix<double, 3, 6> jacobianH = this->computeObservationJacobian();

    /// 2.2 计算动态观测噪声 R
    Eigen::Matrix3d currentMeasurementNoiseR = this->computeMeasurementNoise(imuData.linearAcceleration.norm());
    
    /// 2.3 计算卡尔曼增益 K
    Eigen::Matrix<double, 6, 3> kalmanGain = this->computeKalmanGain(this->covariance, jacobianH, currentMeasurementNoiseR);
    
    /// 2.4 计算观测残差 (Innovation)
    /// innovation = z_measured - h(x_nominal)
    /// h(x_nominal) 是根据当前名义姿态预测的理论加速度值
    Vector3d predictedAccel = this->orientation.conjugate() * this->gravity;
    Vector3d innovation = imuData.linearAcceleration - predictedAccel;
    
    /// 2.5 & 3. 更新误差协方差并注入名义状态
    this->updateAndInject(this->covariance, kalmanGain, innovation, jacobianH);

    /// --- 4. 记录遥测数据 ---
    this->telemetryHistory.push_back({
        imuData.timestamp,
        this->orientation,
        this->gyroBias,
        this->covariance.diagonal(),
        currentMeasurementNoiseR,
        kalmanGain
    });
}

/**
 * @brief       步骤 1.1: 传播名义状态
 **/
void EkfAhrs::propagateNominalState(const Vector3d& correctedGyro, double deltaTime)
{
    Quaternion delta_q(1.0, 
                       0.5 * correctedGyro.x() * deltaTime,
                       0.5 * correctedGyro.y() * deltaTime,
                       0.5 * correctedGyro.z() * deltaTime);
    this->orientation = (this->orientation * delta_q.normalized()).normalized();
}

/**
 * @brief       步骤 1.2: 传播误差状态协方差
 **/
void EkfAhrs::propagateErrorCovariance(const Vector3d& correctedGyro, double deltaTime)
{
    /// 构造误差状态转移矩阵 F (6x6)
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    
    /// F(0:2, 0:2) = I - skew(w*dt)
    Eigen::Matrix3d R = this->orientation.toRotationMatrix();
    Eigen::Matrix3d C_nb = R; // 从导航系(n)到机体系(b)的旋转
    Eigen::Matrix3d gyro_skew;
    gyro_skew << 0, -correctedGyro.z(), correctedGyro.y(),
                 correctedGyro.z(), 0, -correctedGyro.x(),
                 -correctedGyro.y(), correctedGyro.x(), 0;

    F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() - gyro_skew * deltaTime;
    /// F(0:2, 3:5) = -I*dt
    F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity() * deltaTime;

    /// 过程噪声 Q_d = Q * dt
    Eigen::Matrix<double, 6, 6> Q_d = this->processNoiseQ * deltaTime;
    
    /// P_k⁻ = F * P * F^T + Q_d
    this->covariance = F * this->covariance * F.transpose() + Q_d;
}

/**
 * @brief       步骤 2.1: 计算观测雅可比矩阵 H
 **/
Eigen::Matrix<double, 3, 6> EkfAhrs::computeObservationJacobian() const
{
    /// H 是观测 h 对误差状态 δx 的偏导
    /// h(x) = C_bn * g = q⁻¹ * g * q
    /// z_accel = C_bn * g_n + a_b + v_a
    /// ∂z / ∂δθ = [ (C_bn * g_n) x ] = skew(predicted_accel)
    /// ∂z / ∂δb = 0
    
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    Vector3d predictedAccel = this->orientation.conjugate() * this->gravity;
    
    Eigen::Matrix3d accel_skew;
    accel_skew << 0, -predictedAccel.z(), predictedAccel.y(),
                  predictedAccel.z(), 0, -predictedAccel.x(),
                  -predictedAccel.y(), predictedAccel.x(), 0;

    H.block<3, 3>(0, 0) = accel_skew;
    return H;
}

/**
 * @brief       步骤 2.2: 计算动态观测噪声 R
 **/
Eigen::Matrix3d EkfAhrs::computeMeasurementNoise(double accelMagnitude) const
{
    // 这是一个简化的动态R模型，未来可以替换为CNN
    // 暂时我们只使用一个固定的基础R值
    (void)accelMagnitude; // 避免未使用参数警告
    return this->measurementNoiseR;
}

/**
 * @brief       步骤 2.3: 计算卡尔曼增益 K
 **/
Eigen::Matrix<double, 6, 3> EkfAhrs::computeKalmanGain(
    const Eigen::Matrix<double, 6, 6>& predictedCov,
    const Eigen::Matrix<double, 3, 6>& jacobianH,
    const Eigen::Matrix3d& noiseR) const
{
    /// S = H * P⁻ * H^T + R
    Eigen::Matrix3d S = jacobianH * predictedCov * jacobianH.transpose() + noiseR;
    /// K = P⁻ * H^T * S⁻¹
    Eigen::Matrix<double, 6, 3> K = predictedCov * jacobianH.transpose() * S.inverse();
    return K;
}

/**
 * @brief       步骤 2.4 & 3: 更新误差状态并注入名义状态
 **/
void EkfAhrs::updateAndInject(
    const Eigen::Matrix<double, 6, 6>& predictedCov,
    const Eigen::Matrix<double, 6, 3>& kalmanGain,
    const Vector3d& innovation,
    const Eigen::Matrix<double, 3, 6>& jacobianH)
{
    /// --- 更新误差协方差 ---
    /// P⁺ = (I - K * H) * P⁻
    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    this->covariance = (I - kalmanGain * jacobianH) * predictedCov;

    /// --- 计算误差状态估计 ---
    /// δx⁺ = K * y
    Eigen::Matrix<double, 6, 1> errorState = kalmanGain * innovation;
    
    /// --- 注入名义状态 ---
    /// 姿态注入： q <- q ⊗ quat(δθ)
    Vector3d deltaTheta = errorState.head<3>();
    Quaternion delta_q(1.0, 0.5 * deltaTheta.x(), 0.5 * deltaTheta.y(), 0.5 * deltaTheta.z());
    this->orientation = (this->orientation * delta_q.normalized()).normalized();

    /// 零偏注入： b <- b + δb
    this->gyroBias += errorState.tail<3>();
    
    // 误差状态在注入后被隐式地重置为0
}


// --- 剩余的通用函数 ---
Quaternion EkfAhrs::getOrientation() const { return this->orientation; }
bool EkfAhrs::isInitialized() const { return this->bIsInitialized; }

void EkfAhrs::runDebugExport(const std::string& outputFilePath) const
{
    if (this->telemetryHistory.empty())
    {
        std::cout << "  [Debug Export] No telemetry data to export for EkfAhrs." << std::endl;
        return;
    }

    std::ofstream outFile(outputFilePath);
    if (!outFile.is_open())
    {
        std::cerr << "  Error: Could not open debug export file: " << outputFilePath << std::endl;
        return;
    }

    // 写入CSV表头
    outFile << "timestamp,q_w,q_x,q_y,q_z,bias_x,bias_y,bias_z,"
            << "P_att_xx,P_att_yy,P_att_zz,P_bias_xx,P_bias_yy,P_bias_zz\n";

    outFile << std::fixed << std::setprecision(9);

    for (const auto& data : this->telemetryHistory)
    {
        outFile << data.timestamp << ","
                << data.orientation.w() << "," << data.orientation.x() << "," << data.orientation.y() << "," << data.orientation.z() << ","
                << data.gyroBias.x() << "," << data.gyroBias.y() << "," << data.gyroBias.z() << ","
                << data.covarianceDiagonal(0) << "," << data.covarianceDiagonal(1) << "," << data.covarianceDiagonal(2) << ","
                << data.covarianceDiagonal(3) << "," << data.covarianceDiagonal(4) << "," << data.covarianceDiagonal(5) << "\n";
    }

    outFile.close();
    std::cout << "  [Debug Export] Telemetry data for EkfAhrs saved to: " << outputFilePath << std::endl;
}