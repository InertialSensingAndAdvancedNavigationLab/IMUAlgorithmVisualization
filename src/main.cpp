
/// 包含标准库
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <iomanip>
#include <filesystem> // C++17 filesystem library

/// 包含我们自己的模块
#include "config/Configuration.hpp"
#include "IO/EurocDataLoader.hpp"
#include "IO/TumTrajectoryWriter.hpp"
#include "ahrs/AhrsFactory.hpp"
#include "ahrs/DefaultAhrs.hpp"
#include "math/CoordinateSystem.hpp"

// #include "ahrs/DirectIntegrator.hpp" // 未来会包含的算法
// #include "ahrs/UkfAlgorithm.hpp"

/**
 * @brief       打印加载的运行时配置信息
 *  @details     一个辅助函数，用于验证配置是否被正确加载。
 * 
 * @param       参数名称: config                        数据类型:        const RuntimeConfig&
 *  @details     要打印的运行时配置结构体。
 **/
void printRuntimeConfig(const RuntimeConfig& config)
{
    /// 打印一个清晰的标题
    std::cout << "--- Runtime Configuration Loaded ---" << std::endl;
    /// 打印运行注释
    std::cout << "Comment: " << config.runComment << std::endl;
    /// 打印数据集路径
    std::cout << "Dataset Path: " << config.datasetBasePath << std::endl;
    /// 打印输出路径
    std::cout << "Output Path: " << config.outputBaseDir << std::endl;
    /// 打印要处理的序列
    std::cout << "Sequences: ";
    /// 遍历并打印序列列表
    for (const auto& seq : config.sequencesToProcess) {
        std::cout << seq << " ";
    }
    /// 换行
    std::cout << std::endl;
    /// 打印选择的算法
    std::cout << "Algorithm Selected: " << config.algorithmName << " (config: " << config.algorithmConfigFile << ")" << std::endl;
    /// 打印IMU硬件配置
    std::cout << "IMU Spec File: " << config.imuHardwareSpecFile << std::endl;
    /// 打印初始化方法
    std::cout << "Init with Ground Truth: " << (config.useGroundTruthForInit ? "Yes" : "No") << std::endl;
    /// 打印一个分隔符
    std::cout << "------------------------------------" << std::endl;
}
/**
 * @brief       程序主入口
 *  @details     负责加载配置，并在一个循环中为每个指定的数据序列
 * 创建算法实例，调度数据加载器逐帧处理数据，收集通用结果，
 * 并最终触发轨迹保存和调试信息导出。
 *
 * @return      程序退出代码                          数据类型:        int
 *  @retval      0     程序成功执行。
 *  @retval      -1    发生错误。
 **/
int main()
{
    /// 硬编码主配置文件路径。假设程序从项目根目录执行。
    const std::string mainConfigFile = "config/runtime_config.json";
    /// 打印加载信息
    std::cout << "Attempting to load main config from hardcoded path: " << mainConfigFile << std::endl;

    /// 创建配置管理器实例
    ConfigurationManager configManager;
    /// 加载所有配置文件
    if (!configManager.loadFromFiles(mainConfigFile))
    {
        /// 打印错误信息
        std::cerr << "Failed to load configurations. Exiting." << std::endl;
        /// 返回错误代码
        return -1;
    }
    /// 打印加载成功的运行时配置
    printRuntimeConfig(configManager.runtime);

    /// --- 定义固定的坐标系变换 ---
    /// 根据 Readme.md，获取从 Internal (FRD) 系到 TUM/Viz (RDF) 系的旋转
    const Quaternion q_viz_from_internal(CoordinateSystemManager::C_viz_from_internal);

    /// --- 数据处理主循环 ---
    /// 遍历配置文件中指定的所有数据序列
    for (const auto& sequenceName : configManager.runtime.sequencesToProcess)
    {
        /// 打印当前正在处理的序列名称
        std::cout << "\n========================================================" << std::endl;
        std::cout << "Processing sequence: " << sequenceName << std::endl;
        std::cout << "========================================================" << std::endl;

        /// --- 算法创建与初始化 (在循环内部) ---
        /// 使用工厂为每个序列创建一个全新的、干净的算法实例
        auto algorithm = AhrsFactory::create(configManager.runtime.algorithmName);
        
        /// 检查算法是否被成功创建
        if (!algorithm)
        {
            std::cerr << "Error: Algorithm '" << configManager.runtime.algorithmName << "' is not supported. Skipping sequence." << std::endl;
            continue; // 跳到下一个序列
        }
        
        /// 初始化新创建的算法实例。此函数只负责配置，不接触数据。
        if (!algorithm->initialize(configManager))
        {
            std::cerr << "Error: Failed to initialize algorithm for sequence " << sequenceName << ". Skipping." << std::endl;
            continue; // 跳到下一个序列
        }

        /// 构造IMU数据文件的完整路径
        std::filesystem::path imuDataPath = std::filesystem::path(configManager.runtime.datasetBasePath) / sequenceName / "mav0" / "imu0" / "data.csv";
        
        /// 创建数据加载器实例
        EurocDataLoader dataLoader;
        /// 打开数据流
        if (!dataLoader.open(imuDataPath))
        {
            std::cerr << "Error: Failed to open data for sequence " << sequenceName << ". Skipping." << std::endl;
            continue;
        }

        /// --- 核心处理与数据收集 ---
        /// 创建一个容器来收集本序列的轨迹数据
        std::vector<PoseStamped> estimatedTrajectory;
        /// 初始化帧计数器
        long long frameCount = 0;

        /// 循环读取IMU数据流，直到结束
        while (dataLoader.hasNext())
        {
            /// 读取下一条IMU数据
            auto imuDataOpt = dataLoader.readNext();
            if (!imuDataOpt) { break; }

            /// 核心更新步骤 (算法内部将处理初始化状态机)
            algorithm->update(*imuDataOpt);
            
            /// 通用数据收集
            /// 只有在算法初始化完成后，我们收集的数据才有意义
            if(algorithm->isInitialized())
            {
                /// 算法输出的姿态是从 Internal (FRD) 系到 World (ENU) 系的旋转, q_world_from_internal
                Quaternion q_world_from_internal = algorithm->getOrientation();
                Vector3d currentPosition = Vector3d::Zero();
                /// 在写入TUM文件前，必须将姿态转换到 TUM/Viz (RDF) 系, q_world_from_viz = q_world_from_internal * q_internal_from_viz
                Quaternion q_world_from_viz = q_world_from_internal * q_viz_from_internal.inverse(); // q_internal_from_viz = q_viz_from_internal.inverse()
                estimatedTrajectory.push_back({imuDataOpt->timestamp, currentPosition, q_world_from_viz});
            }
            
            /// 递增帧计数器
            frameCount++;
        }

        /// --- 运行结束后的统一导出 ---
        std::cout << "\nSequence " << sequenceName << " processing finished. " << frameCount << " frames processed." << std::endl;

        /// 1. 构造【结果轨迹】的输出文件路径
        std::filesystem::path trajectoryOutputFile = std::filesystem::path(configManager.runtime.outputBaseDir) /
                                           (configManager.runtime.algorithmName + "-" +
                                            sequenceName + ".txt");
        
        /// 2. 调用通用的TUM写入器来保存【结果轨迹】
        std::cout << "  Saving estimated trajectory to results directory..." << std::endl;
        if (TumTrajectoryWriter::save(trajectoryOutputFile, estimatedTrajectory))
        {
            std::cout << "  Trajectory saved to: " << trajectoryOutputFile << std::endl;
        }
        else
        {
            std::cerr << "  Error: Failed to save trajectory file." << std::endl;
        }

        /// 3. 构造【私有调试数据】的输出文件路径
        std::filesystem::path debugOutputFile = std::filesystem::path(configManager.runtime.debugOutputBaseDir) /
                                      (configManager.runtime.algorithmName + "-" +
                                       sequenceName + "_telemetry.csv");

        /// 4. 调用算法私有的debug导出函数，写入【调试目录】
        std::cout << "  Running algorithm-specific debug export to debug directory..." << std::endl;
        algorithm->runDebugExport(debugOutputFile);
        
        std::cout << "--------------------------------------------------------" << std::endl;

        /// 关闭数据加载器
        dataLoader.close();
    }
    /// 打印程序结束信息
    std::cout << "\nAll sequences processed. Exiting." << std::endl;

    /// 正常退出
    return 0;
}