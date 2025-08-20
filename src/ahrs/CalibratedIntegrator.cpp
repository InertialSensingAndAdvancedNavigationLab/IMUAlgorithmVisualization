/// 包含类头文件
/// 包含类头文件
#include "ahrs/CalibratedIntegrator.hpp"

/// 包含IO工具类
#include "IO/EurocDataLoader.hpp"

/// 包含其他标准库
#include <iostream>
#include <numeric>
#include <iomanip> // <-- 新增的头文件
#include <cmath>   // For std::pow

/**
 * @brief       处理初始化阶段的数据 (私有方法)
// ... 剩余代码保持不变 ...

/**
 * @brief       处理初始化阶段的数据 (私有方法)
 *  @details     被 update() 在未初始化时调用，负责累积数据并
 * 在满足条件时完成最终的标定和初始化。
 *
 * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
 */
void CalibratedIntegrator::initializing(const ImuMeasurement& imuData)
{
    if (!this->configManager) { return; }

    if (this->sampleCountForInit == 0)
    {
        this->staticInitEndTime = imuData.timestamp + this->configManager->runtime.staticDurationForInitSec;
        std::cout << "CalibratedIntegrator: Now in 'initializing' state..." << std::endl;
        std::cout << "  Collecting static data until timestamp: " << this->staticInitEndTime << "s" << std::endl;
    }

    if (imuData.timestamp < this->staticInitEndTime)
    {
        this->gyroSumForInit += imuData.angularVelocity;
        this->accelSumForInit += imuData.linearAcceleration;
        this->sampleCountForInit++;
    }
    else
    {
        if (this->sampleCountForInit < 10)
        {
            std::cerr << "Error: [CalibratedIntegrator] Not enough static data for calibration. Retrying..." << std::endl;
            this->sampleCountForInit = 0;
            this->gyroSumForInit.setZero();
            this->accelSumForInit.setZero();
            return;
        }

        /// --- 执行最终标定 ---
        this->gyroBias = this->gyroSumForInit / static_cast<double>(this->sampleCountForInit);
        Vector3d averageAccel = this->accelSumForInit / static_cast<double>(this->sampleCountForInit);
        
        /// --- 用标定结果更新核心状态 ---
        /// 1. 更新姿态
        Vector3d idealUpVector(0.0, 0.0, -1.0);
        this->orientation = Quaternion::FromTwoVectors(averageAccel, idealUpVector).normalized();
        
        /// 2. *** 关键：重置协方差P以反映标定结果 ***
        double initialAttitudeUncertaintyRad = 1.0 * M_PI / 180.0; // 姿态初始化后的不确定性
        double calibratedGyroBiasUncertaintyRadPerSec = 0.01 * M_PI / 180.0; // 零偏标定后的不确定性（非常小）
        this->covariance.setZero();
        this->covariance.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * std::pow(initialAttitudeUncertaintyRad, 2);
        this->covariance.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * std::pow(calibratedGyroBiasUncertaintyRadPerSec, 2);
        
        /// --- 完成初始化 ---
        this->bIsInitialized = true;
        this->lastTimestamp = -1.0; // 强制让下一轮 update 的第一帧逻辑被触发
        
        std::cout << "  Calibration complete. " << this->sampleCountForInit << " samples processed." << std::endl;
        std::cout << std::fixed << std::setprecision(6);
        
        /// 打印出在我们内部 FRD 坐标系下的零偏
        std::cout << "  [C++ FRD Frame] Estimated Gyro Bias [rad/s]: (" 
                  << this->gyroBias.x() << ", " 
                  << this->gyroBias.y() << ", " 
                  << this->gyroBias.z() << ")" << std::endl;

        /// --- *** 关键诊断步骤 *** ---
        /// 将计算出的FRD零偏，临时转换回EuRoC原始(RUF)坐标系，以和MATLAB进行苹果对苹果的比较
        Vector3d biasInRufFrame;
        biasInRufFrame.x() = this->gyroBias.y();      // ruf.x = frd.y
        biasInRufFrame.y() = -this->gyroBias.z();     // ruf.y = -frd.z
        biasInRufFrame.z() = this->gyroBias.x();      // ruf.z = frd.x
        
        std::cout << "  [C++ RUF Frame (for MATLAB comparison)] Estimated Gyro Bias [rad/s]: ("
                  << biasInRufFrame.x() << ", "
                  << biasInRufFrame.y() << ", "
                  << biasInRufFrame.z() << ")" << std::endl;

        std::cout << "CalibratedIntegrator is now initialized and running." << std::endl;
    }
}


/**
 * @brief       默认构造函数
 **/
CalibratedIntegrator::CalibratedIntegrator()
    : DirectIntegrator()
    , gyroBias(Vector3d::Zero())
    , configManager(nullptr)
    , staticInitEndTime(0.0)
    , gyroSumForInit(Vector3d::Zero())
    , accelSumForInit(Vector3d::Zero())
    , sampleCountForInit(0)
{
    // 构造函数体为空
}

/**
 * @brief       析构函数
 **/
CalibratedIntegrator::~CalibratedIntegrator()
{
    // 析构函数体为空
}


/**
 * @brief       初始化算法
 **/
bool CalibratedIntegrator::initialize(const ConfigurationManager& configManager)
{
    /// 1. 调用父类的 initialize。它会设置好一个 6x6 的、高不确定性的 P 矩阵。
    if (!DirectIntegrator::initialize(configManager))
    {
        return false;
    }

    /// 2. 保存配置，并强制自己进入“未初始化”模式，以启动在线标定
    this->configManager = &configManager;
    this->bIsInitialized = false; 
    
    /// 3. 重置自己的初始化状态机
    this->sampleCountForInit = 0;
    this->gyroSumForInit.setZero();
    this->accelSumForInit.setZero();
    this->staticInitEndTime = 0.0;
    
    return true;
}


/**
 * @brief       根据单帧IMU数据更新算法状态 (状态机入口)
 **/
void CalibratedIntegrator::update(const ImuMeasurement& imuData)
{
    /// 如果配置丢失，不执行任何操作
    if (!this->configManager) return;
    
    /// 如果是第一帧数据，只记录时间戳
    if (this->lastTimestamp < 0)
    {
        this->lastTimestamp = imuData.timestamp;
        
        // 在线标定逻辑的启动
        if (!this->bIsInitialized)
        {
            this->staticInitEndTime = imuData.timestamp + this->configManager->runtime.staticDurationForInitSec;
            std::cout << "CalibratedIntegrator: Starting online calibration..." << std::endl;
            std::cout << "  Target end time: " << this->staticInitEndTime << "s" << std::endl;
        }
        return;
    }
    
    /// 计算时间步长
    double deltaTime = imuData.timestamp - this->lastTimestamp;
    if (deltaTime <= 0) return;
    this->lastTimestamp = imuData.timestamp;

    /// --- 在线初始化状态机 ---
    if (!this->bIsInitialized)
    {
        // 只要还没初始化完成，就持续累加数据
        if (imuData.timestamp < this->staticInitEndTime)
        {
            this->gyroSumForInit += imuData.angularVelocity;
            this->sampleCountForInit++;
        }
        else
        {
            // 时间到，执行最终标定
            if (this->sampleCountForInit >= 10)
            {
                this->gyroBias = this->gyroSumForInit / static_cast<double>(this->sampleCountForInit);
                this->bIsInitialized = true; // 翻转状态
                std::cout << "  Calibration complete. Estimated Bias: (" 
                          << this->gyroBias.x() << ", " << this->gyroBias.y() << ", " << this->gyroBias.z() << ")" << std::endl;
            }
            else
            {
                // 如果时间到了但样本不够，重置并重新开始
                std::cout << "  Warning: Not enough static samples. Restarting calibration window." << std::endl;
                this->sampleCountForInit = 0;
                this->gyroSumForInit.setZero();
                this->staticInitEndTime = imuData.timestamp + this->configManager->runtime.staticDurationForInitSec;
            }
        }
    }

    /// --- 姿态积分与不确定性传播 ---
    
    // 决定使用哪个陀螺仪数据
    Vector3d gyroToIntegrate = imuData.angularVelocity;
    if (this->bIsInitialized)
    {
        // 如果标定已完成，就使用修正后的数据
        gyroToIntegrate -= this->gyroBias;
    }
    
    // --- 姿态积分 (现在总是在执行) ---
    Quaternion delta_q(1.0, 
                       0.5 * gyroToIntegrate.x() * deltaTime,
                       0.5 * gyroToIntegrate.y() * deltaTime,
                       0.5 * gyroToIntegrate.z() * deltaTime);
    this->orientation = (this->orientation * delta_q.normalized()).normalized();

    // --- 不确定性传播 (现在总是在执行) ---
    // (注意：这里的P传播模型，应该使用 gyroToIntegrate 还是原始的 gyro？
    // 这是一个理论上的选择。ESKF通常使用修正后的。我们保持一致。)
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity() * deltaTime;
    Eigen::Matrix<double, 6, 6> Q_d = this->processNoiseQ * deltaTime;
    this->covariance = F * this->covariance * F.transpose() + Q_d;

    // --- 记录遥测 ---
    this->telemetryHistory.push_back({
        imuData.timestamp, this->orientation, this->covariance.diagonal()
    });
}