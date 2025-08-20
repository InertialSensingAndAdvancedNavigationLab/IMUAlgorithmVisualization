/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含标准库
#include <string>
#include <memory>

/// 包含我们项目中的公共类型和接口
#include "config/Configuration.hpp" // 需要配置管理器来初始化
#include "IO/EurocDataLoader.hpp"   // 需要IMU测量数据类型

/**
 * @class       DefaultAhrs
 * @brief       所有AHRS算法的抽象基类
 *  @details     此类定义了一个AHRS算法必须遵守的通用接口“契约”。它封装了
 * 一个算法最核心的生命周期：初始化(initialize)、更新(update)和
 * 获取结果(getOrientation)。
 *
 * 它采用纯虚函数来强制所有具体的派生类（如 DirectIntegrator,
 * UkfAlgorithm）必须实现这些核心功能。
 *
 * 同时，它提供了一个可被重写(override)的虚函数 runDebugExport()，
 * 允许每个具体的算法自行决定如何实现调试信息的导出，从而保证了
 * 接口的通用性和可扩展性。
 *
 *   @note        这是一个抽象类，不能被直接实例化。
 **/
class DefaultAhrs
{
/// 公共接口
public:
    /**
     * @brief       虚析构函数
     *  @details     作为多态基类的必要条件，确保派生类的析构函数能被正确调用。
     **/
    virtual ~DefaultAhrs() = default;

    /**
     * @brief       初始化算法 (纯虚函数)
     *  @details     每个派生类必须实现此函数，以根据全局配置来设置其
     * 内部状态，例如初始姿态、协方差矩阵等。
     *
     * @param       参数名称: configManager                 数据类型:        const ConfigurationManager&
     *  @details     包含所有运行时、硬件和算法特定参数的配置管理器。
     *
     * @return      初始化成功返回 true                  数据类型:        bool
     **/
    virtual bool initialize(const ConfigurationManager& configManager) = 0;

    /**
     * @brief       根据单帧IMU数据更新算法状态 (纯虚函数)
     *  @details     这是算法的心跳。每个派生类必须实现此函数，以根据
     * 新的传感器测量值来演化其内部状态。
     *
     * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
     *  @details     包含时间戳、角速度和加速度的单帧IMU测量数据。
     **/
    virtual void update(const ImuMeasurement& imuData) = 0;

    /**
     * @brief       获取当前算法的姿态结果 (纯虚函数)
     *
     * @return      当前最优的姿态估计                数据类型:        Quaternion
     **/
    virtual Quaternion getOrientation() const = 0;

    /**
     * @brief       执行调试与数据导出任务 (虚函数)
     *  @details     此函数负责将算法运行过程中的内部状态或遥测数据保存
     * 到文件中。这是一个可选实现的接口。
     *
     * 基类提供了默认的空实现。派生类如果需要导出调试数据
     * (如UKF的P, R, K矩阵)，则必须重写(override)此函数。
     *
     * @param       参数名称: outputFolderPath              数据类型:        const std::string&
     *  @details     用于保存调试文件的目标文件夹路径。
     **/
    virtual void runDebugExport(const std::string& outputFolderPath) const
    {
        /// 默认实现为空。子类可以不实现此功能。
        (void)outputFolderPath; // 避免 "unused parameter" 警告
    }

    /**
     * @brief       检查算法是否已被初始化 (纯虚函数)
     *
     * @return      如果已初始化则返回 true           数据类型:        bool
     **/
    virtual bool isInitialized() const = 0;
};