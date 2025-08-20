/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含标准库
#include <string>
#include <vector>
#include <optional>

/// 包含我们项目中的数学类型
#include "math/Vector.hpp"
#include "math/Quaternion.hpp"

/**
 * @struct      RuntimeConfig
 * @brief       存储程序运行时的宏观配置
 *  @details     这些参数与具体算法无关，定义了本次运行的数据源、输出和调度策略。
 **/
struct RuntimeConfig
{
    /// @brief 本次运行的描述性注释，用于记录
    std::string runComment;
    /// @brief 数据集的根目录路径
    std::string datasetBasePath;
    /// @brief 需要处理的数据集序列名称列表
    std::vector<std::string> sequencesToProcess;
    /// @brief 结果输出的根目录
    std::string outputBaseDir;
    /// @brief 调试日志(如遥测CSV)输出的根目录
    std::string debugOutputBaseDir; 
    /// @brief 要运行的算法名称 (例如 "UKF", "EKF")
    std::string algorithmName;
    /// @brief 选定算法的配置文件路径
    std::string algorithmConfigFile;
    /// @brief IMU硬件参数文件的路径
    std::string imuHardwareSpecFile;

    /// @brief 是否使用真值进行初始化
    bool useGroundTruthForInit;
    /// @brief 若不使用真值，用于静态初始化的时间长度 (秒)
    double staticDurationForInitSec;
};

/**
 * @struct      ImuHardwareConfig
 * @brief       存储IMU传感器的物理硬件参数
 *  @details     这些参数描述了传感器的固有特性，与所使用的滤波算法无关。
 **/
struct ImuHardwareConfig
{
    /// @brief 传感器型号
    std::string sensorModel;
    /// @brief 传感器采样频率 (Hz)
    double frequencyHz;
    /// @brief 陀螺仪噪声密度 RMS (rad/s/sqrt(Hz))
    double gyroNoiseRms;
    /// @brief 陀螺仪零偏不稳定性 RMS (rad/s)
    double gyroBiasInstabilityRms;
    /// @brief 加速度计噪声密度 RMS (m/s^2/sqrt(Hz))
    double accelNoiseRms;
    /// @brief 加速度计零偏不稳定性 RMS (m/s^2)
    double accelBiasInstabilityRms;
};

/**
 * @struct      UkfAlgorithmConfig
 * @brief       存储UKF算法特有的超参数
 **/
struct UkfAlgorithmConfig
{
    /// @brief 运动噪声缩放因子
    double motionNoiseScalingFactor;
    /// @brief 运动检测阈值 (m/s^2)
    double motionDetectionThresholdMs2;
    /// @brief UKF参数 alpha
    double ukfAlpha;
    /// @brief UKF参数 beta
    double ukfBeta;
    /// @brief UKF参数 kappa
    double ukfKappa;
    /// @brief 初始姿态不确定性 (度)
    double initialAttitudeUncertaintyDeg;
    /// @brief 初始陀螺仪零偏不确定性 (度/秒)
    double initialGyroBiasUncertaintyDps;
};

/**
 * @class       ConfigurationManager
 * @brief       负责加载、管理和提供所有配置信息的单例类
 *  @details     该类解析命令行参数以找到主配置文件，然后根据主配置文件
 * 的内容，级联加载所有必需的配置。
 **/
class ConfigurationManager
{
/// 公共接口
public:
    /**
     * @brief       加载所有配置信息
     *  @details     这是配置管理器的主要入口点。它解析主配置文件，
     * 然后加载关联的算法和硬件配置文件。
     *
     * @param       参数名称: mainConfigFilepath            数据类型:        const std::string&
     *  @details     主配置文件 (如 runtime_config.json) 的路径。
     *
     * @return      加载成功返回 true，否则返回 false   数据类型:        bool
     **/
    bool loadFromFiles(const std::string& mainConfigFilepath);

    /// @brief 运行时配置实例
    RuntimeConfig runtime;
    /// @brief IMU硬件配置实例
    ImuHardwareConfig hardware;
    /// @brief UKF算法配置实例
    UkfAlgorithmConfig ukfConfig;
};