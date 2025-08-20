/// 包含类头文件
#include "ahrs/DirectIntegrator.hpp"

/// 包含其他标准库
#include <iostream>
#include <fstream>
#include <iomanip>

/**
 * @brief       默认构造函数
 *  @details     正确初始化所有成员变量，特别是将6x6的Eigen矩阵初始化为零。
 **/
DirectIntegrator::DirectIntegrator()
    : orientation(Quaternion::Identity())
    , covariance(Eigen::Matrix<double, 6, 6>::Zero())      // <-- 关键修正
    , processNoiseQ(Eigen::Matrix<double, 6, 6>::Zero())   // <-- 关键修正
    , bIsInitialized(false)
    , lastTimestamp(-1.0)
{
    /// 增加构造函数日志
    std::cout << "[Log] DirectIntegrator constructor called. this=" << this << std::endl;
}

/**
 * @brief       析构函数 (需要显式定义以添加日志)
 **/
DirectIntegrator::~DirectIntegrator()
{
    /// 增加析构函数日志
    std::cout << "[Log] DirectIntegrator destructor called. this=" << this << std::endl;
}


/**
 * @brief       初始化算法
 **/
bool DirectIntegrator::initialize(const ConfigurationManager& configManager)
{
    /// 增加初始化日志
    std::cout << "[Log] DirectIntegrator::initialize() called. this=" << this << std::endl;

    const auto& imuConfig = configManager.hardware;
    
    /// --- 计算过程噪声 Q (6x6) ---
    double gyroNoiseVar = std::pow(imuConfig.gyroNoiseRms, 2);
    double gyroBiasVar = std::pow(imuConfig.gyroBiasInstabilityRms, 2);
    this->processNoiseQ.setZero();
    this->processNoiseQ.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyroNoiseVar;
    this->processNoiseQ.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * gyroBiasVar;

    /// --- 设置初始不确定性 P (6x6) ---
    // 假设初始姿态不确定性为1度，初始零偏不确定性为0.1度/秒
    double initialAttitudeUncertaintyRad = 1.0 * M_PI / 180.0;
    double initialGyroBiasUncertaintyRadPerSec = 0.1 * M_PI / 180.0;
    this->covariance.setZero();
    this->covariance.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * std::pow(initialAttitudeUncertaintyRad, 2);
    this->covariance.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * std::pow(initialGyroBiasUncertaintyRadPerSec, 2);

    this->orientation = Quaternion::Identity();
    this->lastTimestamp = -1.0;
    this->telemetryHistory.clear();
    this->bIsInitialized = true;
    
    /// 增加初始化成功日志
    std::cout << "[Log] DirectIntegrator::initialize() finished successfully. bIsInitialized=" << this->bIsInitialized << std::endl;
    return true;
}

/**
 * @brief       根据单帧IMU数据更新算法状态
 *  @details     执行一步四元数积分，并累积过程噪声到协方差矩阵P中。
 *
 * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
 **/
void DirectIntegrator::update(const ImuMeasurement& imuData)
{
    if (!this->isInitialized()) return;

    if (this->lastTimestamp < 0)
    {
        this->lastTimestamp = imuData.timestamp;
        this->telemetryHistory.push_back({
            this->lastTimestamp, this->orientation, this->covariance.diagonal()
        });
        return;
    }

    double deltaTime = imuData.timestamp - this->lastTimestamp;
    if (deltaTime <= 0) return;
    this->lastTimestamp = imuData.timestamp;

    /// --- 1. 姿态积分 (精确复现 MATLAB) ---
    const Vector3d& gyro = imuData.angularVelocity;
    double gyro_norm = gyro.norm();

    Quaternion delta_q;
    /// 检查是否为非零旋转
    if (gyro_norm > 1e-9)
    {
        /// a. 计算旋转角度和轴 (精确模型)
        double angle = gyro_norm * deltaTime;
        Vector3d axis = gyro / gyro_norm;
        
        /// b. 从轴角精确构造 delta_q
        /// Eigen::AngleAxisd(angle, axis) 创建一个旋转对象
        /// 然后可以转换为四元数
        delta_q = Quaternion(Eigen::AngleAxisd(angle, axis));
    }
    else
    {
        /// 如果旋转很小，则 delta_q 是单位旋转
        delta_q.setIdentity();
    }
    
    /// --- *** 关键修正：匹配 MATLAB 的乘法顺序 *** ---
    /// MATLAB: quaternion(t, :) = quatmultiply(quaternion(t-1, :), delta_q);
    /// 这对应于 q_new = q_old * delta_q
    this->orientation = this->orientation * delta_q;
    
    /// 归一化
    this->orientation.normalize();
    
    /// --- 2. 不确定性传播 (保持不变) ---
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity() * deltaTime;
    Eigen::Matrix<double, 6, 6> Q_d = this->processNoiseQ * deltaTime;
    this->covariance = F * this->covariance * F.transpose() + Q_d;

    /// --- 3. 记录遥测数据 ---
    this->telemetryHistory.push_back({
        imuData.timestamp,
        this->orientation,
        this->covariance.diagonal()
    });
}

/**
 * @brief       获取当前算法的姿态结果
 *
 * @return      当前最优的姿态估计                数据类型:        Quaternion
 **/
Quaternion DirectIntegrator::getOrientation() const
{
    /// 返回当前姿态
    return this->orientation;
}

/**
 * @brief       检查算法是否已被初始化
 *
 * @return      如果已初始化则返回 true           数据类型:        bool
 **/
bool DirectIntegrator::isInitialized() const
{
    /// 返回初始化标志
    return this->bIsInitialized;
}

/**
 * @brief       执行调试与数据导出任务
 *  @details     将算法运行期间记录的轨迹和不确定性(P矩阵对角线)
 * 保存到CSV文件中，以便进行可视化分析。
 *
 * @param       参数名称: outputFilePath                数据类型:        const std::string&
 *  @details     用于保存调试文件的**完整**文件路径，由调用者(main)提供。
 **/
void DirectIntegrator::runDebugExport(const std::string& outputFilePath) const
{
    /// 如果历史记录为空，则不导出
    if (this->telemetryHistory.empty())
    {
        std::cout << "  [Debug Export] No telemetry data to export for DirectIntegrator." << std::endl;
        return;
    }

    /// 直接使用 main 函数提供的完整文件路径
    // std::string filePath = outputFolderPath + "/direct_integrator_telemetry.csv"; // <<< 移除这行错误的代码
    
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
            << "P_att_xx,P_att_yy,P_att_zz,"
            << "P_bias_xx,P_bias_yy,P_bias_zz\n";

    /// 设置输出精度
    outFile << std::fixed << std::setprecision(9);

    /// 遍历历史记录并写入文件
    for (const auto& data : this->telemetryHistory)
    {
        outFile << data.timestamp << ","
                << data.orientation.w() << "," << data.orientation.x() << "," << data.orientation.y() << "," << data.orientation.z() << ","
                << data.covarianceDiagonal(0) << "," << data.covarianceDiagonal(1) << "," << data.covarianceDiagonal(2) << ","
                << data.covarianceDiagonal(3) << "," << data.covarianceDiagonal(4) << "," << data.covarianceDiagonal(5) << "\n";
    }

    /// 关闭文件
    outFile.close();
    /// 打印成功信息
    std::cout << "  Debug export telemetry data saved to: " << outputFilePath << std::endl;
}