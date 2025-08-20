/// 包含类头文件
#include "IO/EurocDataLoader.hpp"

/// 包含其他标准库
#include <iostream>
#include <limits>
#include <sstream>

/**
 * @brief       默认构造函数
 *  @details     初始化加载器状态为未打开。
 **/
EurocDataLoader::EurocDataLoader() : bIsOpen(false)
{
    /// 构造函数体为空
}

/**
 * @brief       析构函数
 *  @details     确保在对象销毁时文件流被正确关闭。
 **/
EurocDataLoader::~EurocDataLoader()
{
    /// 调用 close 函数来关闭文件
    this->close();
}

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
bool EurocDataLoader::open(const std::string& imuCsvPath)
{
    /// 如果已有文件打开，先关闭它
    if (this->bIsOpen)
    {
        /// 调用 close 函数
        this->close();
    }
    /// 存储文件路径
    this->currentFilePath = imuCsvPath;
    /// 尝试打开新的文件流
    this->fileStream.open(this->currentFilePath);
    /// 检查文件是否成功打开
    if (!this->fileStream.is_open())
    {
        /// 在标准错误流中打印错误信息
        std::cerr << "Error: Could not open IMU data file: " << this->currentFilePath << std::endl;
        /// 更新状态标志
        this->bIsOpen = false;
        /// 返回失败
        return false;
    }
    /// 读取并丢弃CSV文件的表头行
    std::string headerLine;
    /// 读取一行
    std::getline(this->fileStream, headerLine);
    /// 检查读取表头后文件流的状态是否良好
    if (!this->fileStream.good() || headerLine.find("timestamp") == std::string::npos)
    {
        /// 打印错误信息
        std::cerr << "Error: Failed to read header from IMU data file or file is empty/invalid: " << this->currentFilePath << std::endl;
        /// 关闭文件
        this->close();
        /// 返回失败
        return false;
    }
    /// 更新状态标志
    this->bIsOpen = true;
    /// 返回成功
    return true;
}

/**
 * @brief       关闭当前打开的数据文件流
 **/
void EurocDataLoader::close()
{
    /// 如果文件流是打开的
    if (this->fileStream.is_open())
    {
        /// 关闭文件流
        this->fileStream.close();
    }
    /// 重置状态标志
    this->bIsOpen = false;
    /// 清空文件路径记录
    this->currentFilePath.clear();
}

/**
 * @brief       检查数据流中是否还有下一条数据
 *  @details     **重要**: 此函数不是const的，因为它会调用非const的 fileStream.peek()。
 *
 * @return      如果未到达文件末尾，返回true          数据类型:        bool
 **/
bool EurocDataLoader::hasNext() // <--- 主要改动：移除了 const
{
    /// 如果文件未打开，则没有数据
    if (!this->bIsOpen)
    {
        /// 返回 false
        return false;
    }
    /// 使用 peek() 函数检查下一个字符是否是文件结束符(EOF)
    /// peek() 不会移动文件指针
    return this->fileStream.peek() != EOF;
}

/**
 * @brief       读取并返回数据流中的下一条IMU测量数据
 **/
std::optional<ImuMeasurement> EurocDataLoader::readNext()
{
    if (!this->hasNext())
    {
        return std::nullopt;
    }

    std::string line;
    std::getline(this->fileStream, line);
    if (line.empty() || this->fileStream.fail())
    {
        return std::nullopt;
    }

    for (char& ch : line)
    {
        if (ch == ',')
        {
            ch = ' ';
        }
    }
    std::stringstream ss(line);
    
    ImuMeasurement data;
    unsigned long long timestamp_ns;
    Vector3d gyro_orig, accel_orig;

    ss >> timestamp_ns
       >> gyro_orig.x() >> gyro_orig.y() >> gyro_orig.z()
       >> accel_orig.x() >> accel_orig.y() >> accel_orig.z();

    if(ss.fail())
    {
        return std::nullopt;
    }

    data.timestamp = static_cast<double>(timestamp_ns) * 1e-9;
    
    /// --- 坐标系转换: EuRoC IMU Original (RUF) -> Internal Compute (FRD) ---
    /// 根据 COORDINATE_SYSTEMS.md v3.0 中的定义:
    /// Internal.X (Fwd)  = IMU_Orig.Z (Fwd)
    /// Internal.Y (Right) = IMU_Orig.X (Right)
    /// Internal.Z (Down)  = -IMU_Orig.Y (Up)
    data.angularVelocity << gyro_orig.z(), gyro_orig.x(), -gyro_orig.y();
    data.linearAcceleration << accel_orig.z(), accel_orig.x(), -gyro_orig.y();

    return data;
}

/**
 * @brief       将数据流的读取位置重置到文件开头
 *  @details     此函数将文件指针移回文件开头，并重新跳过表头，
 * 允许从头开始重新“播放”数据流。
 **/
void EurocDataLoader::reset()
{
    /// 如果文件未打开，则不执行任何操作
    if (!this->bIsOpen)
    {
        /// 返回
        return;
    }
    /// 清除文件流的任何错误状态 (如EOF)
    this->fileStream.clear();
    /// 将读取指针移到文件开头
    this->fileStream.seekg(0, std::ios::beg);
    /// 再次读取并丢弃表头行
    std::string headerLine;
    /// 读取一行
    std::getline(this->fileStream, headerLine);
}

/**
 * @brief       将数据流的读取位置快进到指定时间戳
 *  @details     重置数据流后，跳过所有时间戳小于`targetTimestamp`的数据行。
 * 这使得可以从数据流的任意时间点开始处理。
 *
 * @param       参数名称: targetTimestamp               数据类型:        double
 *  @details     目标开始时间，单位为秒。
 **/
void EurocDataLoader::seek(double targetTimestamp)
{
    /// 首先重置到文件开头
    this->reset();
    /// 如果文件未打开，则不执行任何操作
    if (!this->bIsOpen)
    {
        /// 返回
        return;
    }

    /// 循环读取，直到找到合适的时间戳
    while (this->hasNext())
    {
        /// 记录当前文件指针位置
        std::streampos currentPos = this->fileStream.tellg();
        /// 读取下一条数据以检查其时间戳
        auto dataOpt = this->readNext();
        /// 如果读取成功
        if (dataOpt)
        {
            /// 如果当前数据的时间戳已经大于或等于目标时间戳
            if (dataOpt->timestamp >= targetTimestamp)
            {
                /// 将文件指针移回到读取这条数据之前的位置
                this->fileStream.clear(); // 清除可能因为readNext()到达文件尾部而设置的EOF标志
                this->fileStream.seekg(currentPos);
                /// 找到了起始点，退出循环
                return;
            }
        }
        else
        {
            /// 如果读取失败(可能已到文件末尾)，退出循环
            break;
        }
    }
}