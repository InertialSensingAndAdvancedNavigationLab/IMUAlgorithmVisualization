/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含标准库
#include <string>
#include <fstream>
#include <optional>

/// 包含我们项目中的数学类型
#include "math/Vector.hpp"
#include "math/Quaternion.hpp"

/**
 * @struct      ImuMeasurement
 * @brief       存储单次IMU测量的数据结构
 *  @details     该结构体包含时间戳、转换到内部坐标系后的角速度和线性加速度。
 * 内部坐标系约定为：X-前, Y-右, Z-下。
 **/
struct ImuMeasurement
{
    /// @brief 测量时间戳，单位：秒
    double timestamp;
    /// @brief 角速度，单位：rad/s (已转换到内部坐标系)
    Vector3d angularVelocity;
    /// @brief 线性加速度，单位：m/s^2 (已转换到内部坐标系)
    Vector3d linearAcceleration;
};

/**
 * @class       EurocDataLoader
 * @brief       一个用于流式读取EuRoC数据集IMU数据的加载器
 *  @details     此类模拟一个来自EuRoC数据集的实时IMU数据流，其设计哲学类似
 * 于串口或ROS数据包的读取方式。它逐行读取CSV文件，而不是一次性将
 * 全部数据加载到内存中。
 *
 * 在读取每一行数据时，它会自动将EuRoC的坐标系 (前-左-上) 转换为
 * 项目内部统一的坐标系 (前-右-下)。
 *
 *   @note        此类不处理真值(Ground Truth)数据，以确保算法模块无法“作弊”。
 *   @todo        增加对CSV文件中不同列顺序的鲁棒性处理。
 **/
class EurocDataLoader
{
/// 公共接口
public:
    /**
     * @brief       默认构造函数
     *  @details     初始化加载器状态为未打开。
     **/
    EurocDataLoader();

    /**
     * @brief       析构函数
     *  @details     确保在对象销毁时文件流被正确关闭。
     **/
    ~EurocDataLoader();

    /**
     * @brief       打开一个EuRoC IMU数据文件以供流式读取
     *  @details     此函数打开指定的 `data.csv` 文件，验证其格式，并读取CSV表头
     * 以准备后续的数据读取。如果文件已打开，它会先关闭旧文件。
     *
     * @param       参数名称: imuCsvPath                    数据类型:        const std::string&
     *  @details     指向IMU数据文件 (imu0/data.csv) 的完整路径。
     *
     * @return      如果文件成功打开并准备就绪，返回true 数据类型:        bool
     **/
    bool open(const std::string& imuCsvPath);

    /**
     * @brief       关闭当前打开的数据文件流
     **/
    void close();

    /**
     * @brief       检查数据流中是否还有下一条数据
     *  @details     **重要**: 此函数不是const的，因为它会调用非const的 fileStream.peek()。
     *
     * @return      如果未到达文件末尾，返回true          数据类型:        bool
     **/
    bool hasNext(); // <--- 主要改动：移除了 const

    /**
     * @brief       读取并返回数据流中的下一条IMU测量数据
     *  @details     这是实现流式读取的核心方法。它从文件当前位置读取一行，
     * 解析时间戳、角速度和线性加速度，进行坐标系转换，然后返回一个
     * `ImuMeasurement` 对象。读取后，内部指针自动前进到下一行。
     *
     * @return      包含下一条测量数据的optional对象    数据类型:        std::optional<ImuMeasurement>
     *  @retval      如果成功读取，optional包含有效数据。
     *  @retval      如果已到达文件末尾或发生读取错误，返回空的optional。
     **/
    std::optional<ImuMeasurement> readNext();

    /**
     * @brief       将数据流的读取位置重置到文件开头
     *  @details     此函数将文件指针移回文件开头，并重新跳过表头，
     * 允许从头开始重新“播放”数据流。
     **/
    void reset();

    /**
     * @brief       将数据流的读取位置快进到指定时间戳
     *  @details     重置数据流后，跳过所有时间戳小于`targetTimestamp`的数据行。
     * 这使得可以从数据流的任意时间点开始处理。
     *
     * @param       参数名称: targetTimestamp               数据类型:        double
     *  @details     目标开始时间，单位为秒。
     **/
    void seek(double targetTimestamp);

/// 私有成员变量
private:
    /// @brief C++文件输入流对象
    std::ifstream fileStream;

    /// @brief 存储当前打开文件的路径
    std::string currentFilePath;

    /// @brief 标记文件是否成功打开并准备就绪
    bool bIsOpen;
};