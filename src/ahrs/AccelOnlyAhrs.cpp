/// 包含类头文件
#include "ahrs/AccelOnlyAhrs.hpp"

/// 包含标准库
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

/**
 * @brief       处理在线初始化阶段的数据 (私有方法)
 *  @details     累积初始静态数据，以计算平均重力加速度模长。
 **/
void AccelOnlyAhrs::initializing(const ImuMeasurement& imuData)
{
    if (!this->configManager) { return; }

    if (this->sampleCountForInit == 0)
    {
        this->staticInitEndTime = imuData.timestamp + this->configManager->runtime.staticDurationForInitSec;
        std::cout << "AccelOnlyAhrs: Now in 'initializing' state..." << std::endl;
        std::cout << "  Collecting static data to calibrate gravity magnitude until t=" << this->staticInitEndTime << "s" << std::endl;
    }

    if (imuData.timestamp < this->staticInitEndTime)
    {
        this->accelSumForInit += imuData.linearAcceleration;
        this->sampleCountForInit++;
    }
    else
    {
        if (this->sampleCountForInit < 10)
        {
            std::cerr << "Error: [AccelOnlyAhrs] Not enough static data for calibration. Retrying..." << std::endl;
            this->sampleCountForInit = 0;
            this->accelSumForInit.setZero();
            return;
        }

        /// --- 执行最终标定 ---
        Vector3d averageAccel = this->accelSumForInit / static_cast<double>(this->sampleCountForInit);
        this->gravityMagnitude = averageAccel.norm();
        
        /// --- 完成初始化 ---
        this->bIsInitialized = true;
        
        std::cout << "  Calibration complete. " << this->sampleCountForInit << " samples processed." << std::endl;
        std::cout << "  Estimated Gravity Magnitude [m/s^2]: " << this->gravityMagnitude << std::endl;
        std::cout << "AccelOnlyAhrs is now initialized and running." << std::endl;
    }
}

/**
 * @brief       默认构造函数
 **/
AccelOnlyAhrs::AccelOnlyAhrs()
    : orientation(Quaternion::Identity())
    , covariance(Eigen::Matrix3d::Identity())
    , configManager(nullptr)
    , accelNoiseVariance(0.0)
    , bIsInitialized(false)
    , gravityMagnitude(9.81) // 默认值，将在初始化时被覆盖
    , staticInitEndTime(0.0)
    , accelSumForInit(Vector3d::Zero())
    , sampleCountForInit(0)
{
    // 构造函数体为空
}

/**
 * @brief       配置算法并重置初始化状态
 **/
bool AccelOnlyAhrs::initialize(const ConfigurationManager& configManager)
{
    this->configManager = &configManager;
    
    /// 从硬件配置中获取加速度计的物理噪声参数
    this->accelNoiseVariance = std::pow(configManager.hardware.accelNoiseRms, 2);
    
    /// 重置所有状态，准备进行在线初始化
    this->orientation.setIdentity();
    this->covariance.setIdentity(); // 初始不确定性设为单位阵 (1 rad^2)
    this->telemetryHistory.clear();
    
    this->bIsInitialized = false; // 强制进入在线初始化模式
    this->sampleCountForInit = 0;
    this->accelSumForInit.setZero();
    this->staticInitEndTime = 0.0;
    
    std::cout << "AccelOnlyAhrs configured. Accel Noise Var = " << this->accelNoiseVariance 
              << ". Ready for online initialization." << std::endl;
    return true;
}

/**
 * @brief       根据单帧IMU数据更新算法状态 (状态机入口)
 **/
void AccelOnlyAhrs::update(const ImuMeasurement& imuData)
{
    /// 如果还未完成在线标定，则继续进行
    if (!this->bIsInitialized)
    {
        this->initializing(imuData);
        // 在标定期间，姿态保持为初始值，但我们仍记录遥测
        this->telemetryHistory.push_back({
            imuData.timestamp,
            this->orientation,
            this->covariance.diagonal(),
            imuData.linearAcceleration
        });
        return;
    }

    /// --- 从这里开始，代码只会在初始化完成后执行 ---
    
    /// --- 1. 姿态计算 ---
    const Vector3d& accel = imuData.linearAcceleration;
    if (accel.norm() > 1e-6)
    {
        /// 在我们的内部FRD坐标系中，静止时的理想加速度(反作用力)指向上方，
        /// 即 Z 轴的负方向。
        Vector3d idealUpVector(0.0, 0.0, -1.0);
        
        /// 使用FromTwoVectors计算从理想“上”方向到测量方向的旋转
        this->orientation = Quaternion::FromTwoVectors(idealUpVector, accel).normalized();
    }
    
    /// --- 2. 不确定性计算 (基于物理模型) ---
    double accelMagnitude = accel.norm();
    
    /// a. 计算运动加速度大小的平方 (近似)
    double motionAccelMagnitudeSquared = std::pow(accelMagnitude - this->gravityMagnitude, 2);
    
    /// b. 计算总的等效噪声方差: R = (sigma_a^2 + ||a_motion||^2) / g^2
    double totalAccelErrorVariance = this->accelNoiseVariance + motionAccelMagnitudeSquared;
    double variance_rad_sq = totalAccelErrorVariance / std::pow(this->gravityMagnitude, 2);

    /// c. 增加保护，防止除零或g无效
    if (this->gravityMagnitude < 1.0) {
        // g无效时，给一个巨大的不确定性 (e.g., 90 degrees std dev)
        variance_rad_sq = std::pow(90.0 * M_PI / 180.0, 2);
    }
    
    /// d. 将协方差P直接设置为当前帧的观测不确定性R
    this->covariance = Eigen::Matrix3d::Identity() * variance_rad_sq;

    /// --- 3. 记录遥测数据 ---
    this->telemetryHistory.push_back({
        imuData.timestamp,
        this->orientation,
        this->covariance.diagonal(),
        imuData.linearAcceleration
    });
}

/**
 * @brief       获取当前算法的姿态结果
 **/
Quaternion AccelOnlyAhrs::getOrientation() const
{
    return this->orientation;
}

/**
 * @brief       检查算法是否已被初始化
 **/
bool AccelOnlyAhrs::isInitialized() const
{
    return this->bIsInitialized;
}

/**
 * @brief       执行调试与数据导出任务
 **/
void AccelOnlyAhrs::runDebugExport(const std::string& outputFilePath) const
{
    if (this->telemetryHistory.empty())
    {
        std::cout << "  [Debug Export] No telemetry data to export for AccelOnlyAhrs." << std::endl;
        return;
    }

    std::ofstream outFile(outputFilePath);
    if (!outFile.is_open())
    {
        std::cerr << "  Error: Could not open debug export file: " << outputFilePath << std::endl;
        return;
    }

    outFile << "timestamp,"
            << "q_w,q_x,q_y,q_z,"
            << "P_att_xx,P_att_yy,P_att_zz,"
            << "accel_x,accel_y,accel_z\n";

    outFile << std::fixed << std::setprecision(9);

    for (const auto& data : this->telemetryHistory)
    {
        outFile << data.timestamp << ","
                << data.orientation.w() << "," << data.orientation.x() << "," << data.orientation.y() << "," << data.orientation.z() << ","
                << data.covarianceDiagonal.x() << "," << data.covarianceDiagonal.y() << "," << data.covarianceDiagonal.z() << ","
                << data.rawAcceleration.x() << "," << data.rawAcceleration.y() << "," << data.rawAcceleration.z() << "\n";
    }

    outFile.close();
    std::cout << "  [Debug Export] Telemetry data for AccelOnlyAhrs saved to: " << outputFilePath << std::endl;
}