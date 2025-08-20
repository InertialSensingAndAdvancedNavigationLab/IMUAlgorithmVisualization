/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含它的父类 DirectIntegrator
#include "DirectIntegrator.hpp"

/**
 * @class       CalibratedIntegrator
 * @brief       一个在初始化时标定并移除陀螺仪零偏的直接积分AHRS算法
 *  @extends     public DirectIntegrator
 *  @details     此类通过继承 DirectIntegrator 复用了其姿态积分和不确定性跟踪的
 * 逻辑。它的核心区别在于重写了 initialize 和 update 方法。
 *
 * `initialize` 负责配置。`update` 内部包含一个状态机：在未初始化时，
 * 它会调用私有的 `initializing` 方法来在线累积数据并计算零偏；
 * 一旦初始化完成，它就会从陀螺仪读数中减去零偏，并调用父类的
 * `update` 方法来执行通用的积分逻辑。
 **/
class CalibratedIntegrator : public DirectIntegrator
{
/// 公共接口 (重写父类的方法)
public:
    /**
     * @brief       默认构造函数
     *  @details     初始化所有状态为默认值。
     **/
    CalibratedIntegrator();

    /**
     * @brief       虚析构函数 (覆盖基类)
     **/
    ~CalibratedIntegrator() override;

    /**
     * @brief       配置算法并重置初始化状态
     *  @details     此函数非常轻量。它只负责配置参数，并调用父类的
     * initialize方法来建立一个“干净”的初始状态。
     *
     * @param       参数名称: configManager                 数据类型:        const ConfigurationManager&
     * @return      初始化成功返回 true                  数据类型:        bool
     **/
    bool initialize(const ConfigurationManager& configManager) override;

    /**
     * @brief       根据单帧IMU数据更新算法状态 (状态机入口)
     *  @details     如果算法未初始化，则调用私有的 `initializing` 方法来处理
     * 数据。一旦初始化完成，就从陀螺仪读数中减去零偏，并调用父类的
     * `update` 方法来执行通用的积分逻辑。
     *
     * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
     **/
    void update(const ImuMeasurement& imuData) override;
    
    // 我们直接继承和使用父类的 getOrientation(), isInitialized(), runDebugExport() 方法。
    // 注意：尽管我们重写了 initialize 和 update, 但其他方法仍然可以被继承和使用。
    // isInitialized() 会被父类在 initialize 时设为 true，但我们会在自己的 initialize 中
    // 将其重置为 false，以启动我们自己的在线初始化流程。

/// 私有实现细节
private:
    /**
     * @brief       处理初始化阶段的数据 (私有方法)
     *  @details     被 update() 在未初始化时调用，负责累积数据并
     * 在满足条件时完成最终的标定和初始化。
     *
     * @param       参数名称: imuData                       数据类型:        const ImuMeasurement&
     */
    void initializing(const ImuMeasurement& imuData);

    /// @brief 存储在初始化阶段估计出的陀螺仪零偏
    Vector3d gyroBias;

    /// @brief 指向配置管理器的常量指针，用于访问配置参数
    const ConfigurationManager* configManager;

    // --- 初始化状态机所需的成员 ---
    /// @brief 静态初始化的目标结束时间戳
    double staticInitEndTime;
    /// @brief 用于初始化的陀螺仪数据累加和
    Vector3d gyroSumForInit;
    /// @brief 用于初始化的加速度计数据累加和
    Vector3d accelSumForInit;
    /// @brief 用于初始化的样本计数器
    int sampleCountForInit;
};