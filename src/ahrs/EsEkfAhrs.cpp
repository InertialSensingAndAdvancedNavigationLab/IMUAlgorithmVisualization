/// 包含类头文件
#include "ahrs/EsEkfAhrs.hpp"

/// 包含其他标准库
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

/**
 * @brief       默认构造函数
 *  @details     初始化所有成员变量为安全默认值。
 **/
EsEkfAhrs::EsEkfAhrs()
    : esEkfConfig(nullptr)
    , orientation(Quaternion::Identity())
    , gyroBias(Vector3d::Zero())
    , covariance(Eigen::Matrix<double, 6, 6>::Zero())
    , processNoiseQ(Eigen::Matrix<double, 6, 6>::Zero())
    , measurementNoiseR(Eigen::Matrix3d::Zero())
    , bIsInitialized(false)
    , currentState(State::UNCONFIGURED)
    , lastTimestamp(-1.0)
    , gravity(0.0, 0.0, -9.81)
    , configManager(nullptr)
    , staticInitEndTime(0.0)
    , accelSumForInit(Vector3d::Zero())
    , sampleCountForInit(0)
{
    /// C_body_imu 将在 initialize() 中从配置加载
}


/**
 * @brief       虚析构函数
 **/
EsEkfAhrs::~EsEkfAhrs()
{
    /// 析构函数体
}

/**
 * @brief       初始化算法，加载特定配置参数
 *  @details     重写此函数以加载加速度计融合所需的特定配置参数，并调用父类初始化。
 *
 * @param       参数名称: configManager                 数据类型:        const ConfigurationManager&
 *  @details     包含所有配置信息的配置管理器。
 * @return      初始化成功返回 true                  数据类型:        bool
 **/
bool EsEkfAhrs::initialize(const ConfigurationManager& configManager)
{
    /// 存储配置管理器指针
    this->configManager = &configManager;

    /// 存储算法特定配置的指针
    this->esEkfConfig = &configManager.esEkfAhrsConfig;
    const auto& hwConfig = configManager.hardware;

    /// 加载IMU到机体的旋转矩阵
    this->C_body_imu = configManager.runtime.imuToBodyTransform.rotationMatrix;

    /// --- 设置过程噪声 Q (6x6) ---
    double gyroNoiseVar = std::pow(hwConfig.gyroNoiseRms, 2);
    double gyroBiasVar = std::pow(hwConfig.gyroBiasInstabilityRms, 2);
    this->processNoiseQ.setZero();
    this->processNoiseQ.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyroNoiseVar;
    this->processNoiseQ.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * gyroBiasVar;

    /// --- 设置初始协方差 P (6x6) ---
    double initialAttitudeUncertaintyRad = this->esEkfConfig->initialAttitudeUncertaintyDeg * M_PI / 180.0;
    double initialGyroBiasUncertaintyRadPerSec = this->esEkfConfig->initialGyroBiasUncertaintyDps * M_PI / 180.0;
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
    this->bIsInitialized = false;

    /// --- 设置状态机为在线初始化模式 ---
    this->currentState = State::INITIALIZING;
    this->sampleCountForInit = 0;
    this->accelSumForInit.setZero();

    /// 清空扩展遥测历史记录
    this->extendedTelemetryHistory.clear();
    /// 打印初始化信息
    std::cout << "[EsEkfAhrs] Initialized successfully." << std::endl;
    std::cout << "  - Low Dynamic Threshold: " << this->esEkfConfig->lowDynamicThreshold << " m/s^2" << std::endl;
    /// 返回成功
    return true;
}

/**
 * @brief       根据单帧IMU数据更新算法状态
 *  @details     实现“预测-更新”循环。预测步骤复用父类(直接积分)的逻辑，
 * 在满足低动态条件时，更新步骤则利用加速度计数据进行融合修正。
 *
 * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
 *  @details     包含陀螺仪和加速度计读数的单帧IMU数据。
 **/
void EsEkfAhrs::update(const ImuMeasurement& imuData)
{
    /// 将原始IMU数据从传感器坐标系转换到机体坐标系 (FRD)
    const Vector3d accel_body = this->C_body_imu * imuData.linearAcceleration;
    const Vector3d gyro_body = this->C_body_imu * imuData.angularVelocity;

    /// 根据当前状态执行不同操作
    switch (this->currentState)
    {
        case State::UNCONFIGURED:
            return; // 如果未配置，则不执行任何操作

        case State::INITIALIZING:
        {
            /// --- 在线静态标定阶段 ---
            if (this->sampleCountForInit == 0)
            {
                this->staticInitEndTime = imuData.timestamp + this->configManager->runtime.staticDurationForInitSec;
                std::cout << "[EsEkfAhrs] Collecting static data to calibrate gravity until t=" << this->staticInitEndTime << "s" << std::endl;
            }

            if (imuData.timestamp < this->staticInitEndTime)
            {
                this->accelSumForInit += accel_body;
                this->sampleCountForInit++;
            }
            else
            {
                if (this->sampleCountForInit < 10)
                {
                    std::cerr << "Error: [EsEkfAhrs] Not enough static data for calibration. Retrying..." << std::endl;
                    this->sampleCountForInit = 0;
                    this->accelSumForInit.setZero();
                    return; // 等待下一帧数据重试
                }

                /// --- 执行最终标定 ---
                const Vector3d avg_accel = this->accelSumForInit / static_cast<double>(this->sampleCountForInit);
                const double g_mag = avg_accel.norm();
                this->gravity = Vector3d(0.0, 0.0, -g_mag); // 更新世界系下的重力向量大小
                
                /// --- 使用标定后的重力向量初始化姿态 ---
                this->orientation = Quaternion::FromTwoVectors(avg_accel, -this->gravity.normalized()).normalized();
                
                /// --- 完成初始化，切换到运行状态 ---
                this->currentState = State::RUNNING;
                this->bIsInitialized = true;
                this->lastTimestamp = imuData.timestamp; // 从当前帧开始积分
                
                std::cout << "  [EsEkfAhrs] Calibration complete. " << this->sampleCountForInit << " samples processed." << std::endl;
                std::cout << "  [EsEkfAhrs] Estimated Gravity Vector (Body Frame) [m/s^2]: " << avg_accel.transpose() << std::endl;
                std::cout << "  [EsEkfAhrs] Calibrated Gravity Magnitude: " << g_mag << " m/s^2" << std::endl;
                std::cout << "[EsEkfAhrs] State changed to RUNNING." << std::endl;
            }
            break; // 初始化阶段不执行积分
        }

        case State::RUNNING:
        {
            /// --- 正常运行阶段 ---
            /// 计算时间步长
            double deltaTime = imuData.timestamp - this->lastTimestamp;
            if (deltaTime <= 0) { return; }
            this->lastTimestamp = imuData.timestamp;

            /// --- 1. 预测步骤 ---
            /// 1.1 传播名义状态 (直接积分)
            Vector3d correctedGyro = gyro_body - this->gyroBias;
            Quaternion delta_q(1.0, 0.5 * correctedGyro.x() * deltaTime, 0.5 * correctedGyro.y() * deltaTime, 0.5 * correctedGyro.z() * deltaTime);
            this->orientation = (this->orientation * delta_q).normalized();

            /// 1.2 传播误差协方差
            Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
            F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity() * deltaTime;
            Eigen::Matrix<double, 6, 6> Q_d = this->processNoiseQ * deltaTime;
            this->covariance = F * this->covariance * F.transpose() + Q_d;

            /// --- 2. 更新/修正阶段 ---
            /// 初始化本帧的遥测变量
            Vector3d innovation = Vector3d::Zero();
            Eigen::Matrix<double, 6, 3> K = Eigen::Matrix<double, 6, 3>::Zero();
            Quaternion accelOnlyOrientation = Quaternion::Identity(); // 默认为单位四元数
            bool correctionStepActive = false;

            /// a. 检查是否处于低动态条件
            double accelMagnitude = accel_body.norm();
            if (std::abs(accelMagnitude - this->gravity.norm()) < this->esEkfConfig->lowDynamicThreshold)
            {
                /// 计算纯加速度计姿态 (仅横滚/俯仰有效)
                accelOnlyOrientation = Quaternion::FromTwoVectors(accel_body, -this->gravity.normalized());
                /// 标记修正步骤已激活
                correctionStepActive = true;

                /// b. 计算新息 (Innovation) y = z - h(x)
                Vector3d z_measured = accel_body.normalized();
                Vector3d z_predicted = this->orientation.conjugate() * this->gravity.normalized();
                innovation = z_measured - z_predicted;

                /// c. 计算测量雅可比矩阵 H
                Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
                Eigen::Matrix3d accel_skew;
                accel_skew << 0, -z_predicted.z(), z_predicted.y(),
                              z_predicted.z(), 0, -z_predicted.x(),
                              -z_predicted.y(), z_predicted.x(), 0;
                H.block<3, 3>(0, 0) = accel_skew;

                /// d. 计算卡尔曼增益 K
                Eigen::Matrix3d S = H * this->covariance * H.transpose() + this->measurementNoiseR;
                K = this->covariance * H.transpose() * S.inverse();

                /// e. 更新状态 (姿态 和 零偏)
                Eigen::Matrix<double, 6, 1> errorState = K * innovation;
                Vector3d deltaAngle = errorState.head<3>();
                Vector3d deltaBias = errorState.tail<3>();
                Quaternion delta_q_correction = Quaternion(1.0, 0.5 * deltaAngle.x(), 0.5 * deltaAngle.y(), 0.5 * deltaAngle.z()).normalized();
                this->orientation = (this->orientation * delta_q_correction).normalized();
                this->gyroBias += deltaBias;

                /// f. 更新协方差 P
                Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
                this->covariance = I_KH * this->covariance * I_KH.transpose() + K * this->measurementNoiseR * K.transpose(); // Joseph form for stability
            }

            /// --- 3. 记录扩展遥测数据 ---
            this->extendedTelemetryHistory.push_back({
                imuData.timestamp,
                this->orientation,
                accelOnlyOrientation,
                this->gyroBias,
                this->covariance.diagonal(), // 协方差
                accel_body,                   // 使用变换后的机体坐标系加速度
                // imuData.linearAcceleration,
                innovation.head<2>(), // 仅记录roll/pitch新息用于可视化
                K.block<2,2>(0,0).diagonal(), // 仅记录roll/pitch增益用于可视化
                correctionStepActive
            });
            break;
        }
    }
}

/**
 * @brief       执行调试与数据导出任务
 *  @details     重写此函数，将包含EKF内部状态（如新息、增益、协方差）的
 * 扩展遥测数据导出为CSV文件，用于后续分析。
 *
 * @param       参数名称: outputFilePath              数据类型:        const std::string&
 *  @details     要写入的调试文件的完整路径。
 **/
void EsEkfAhrs::runDebugExport(const std::string& outputFilePath) const
{
    /// 如果历史记录为空，则不导出
    if (this->extendedTelemetryHistory.empty())
    {
        std::cout << "  [Debug Export] No telemetry data to export for EsEkfAhrs." << std::endl;
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
            << "q_accel_w,q_accel_x,q_accel_y,q_accel_z,"
            << "bias_x,bias_y,bias_z,"
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
                << data.accelOnlyOrientation.w() << "," << data.accelOnlyOrientation.x() << "," << data.accelOnlyOrientation.y() << "," << data.accelOnlyOrientation.z() << "," // 纯加速度计姿态
                << data.gyroBias.x() << "," << data.gyroBias.y() << "," << data.gyroBias.z() << ","
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
    std::cout << "  Debug export for EsEkfAhrs saved to: " << outputFilePath << std::endl;
}

/**
 * @brief       获取当前算法的姿态结果
 *
 * @return      当前最优的姿态估计                数据类型:        Quaternion
 **/
Quaternion EsEkfAhrs::getOrientation() const
{
    return this->orientation;
}

/**
 * @brief       检查算法是否已被初始化
 *
 * @return      如果已初始化则返回 true           数据类型:        bool
 **/
bool EsEkfAhrs::isInitialized() const
{
    return this->currentState == State::RUNNING;
}