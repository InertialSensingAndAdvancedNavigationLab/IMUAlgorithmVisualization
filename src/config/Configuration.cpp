/// 包含类头文件
#include "config/Configuration.hpp"

/// 包含标准库
#include <fstream>
#include <iostream>
#include <map>

/// 包含 JSON 解析库。注意：我们现在将json.hpp放在third_party下，并通过-I告诉编译器
#include "json.hpp"

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
bool ConfigurationManager::loadFromFiles(const std::string& mainConfigFilepath)
{
    /// 使用 nlohmann::json 进行文件读取和解析
    using json = nlohmann::json;

    try
    {
        /// 打开并解析主配置文件
        std::ifstream mainFile(mainConfigFilepath);
        if (!mainFile.is_open())
        {
            /// 如果文件打不开，打印错误并返回
            std::cerr << "Error: Could not open main config file: " << mainConfigFilepath << std::endl;
            return false;
        }
        json mainJson = json::parse(mainFile);

        /// --- 加载运行时配置 ---
        this->runtime.runComment = mainJson.value("run_comment", "No comment");
        this->runtime.datasetBasePath = mainJson.at("dataset_base_path").get<std::string>();
        this->runtime.sequencesToProcess = mainJson.at("sequences_to_process").get<std::vector<std::string>>();
        this->runtime.outputBaseDir = mainJson.at("output_base_dir").get<std::string>();
        this->runtime.debugOutputBaseDir = mainJson.value("debug_output_base_dir", "./debug_logs"); // 使用.value提供默认值
        this->runtime.algorithmName = mainJson.at("algorithm_to_run").at("name").get<std::string>();
        this->runtime.algorithmConfigFile = mainJson.at("algorithm_to_run").at("config_file").get<std::string>();
        this->runtime.imuHardwareSpecFile = mainJson.at("imu_hardware_spec_file").get<std::string>();
        this->runtime.useGroundTruthForInit = mainJson.at("initialization_method").at("use_ground_truth").get<bool>();
        this->runtime.staticDurationForInitSec = mainJson.at("initialization_method").at("static_duration_sec").get<double>();

        /// --- 加载IMU到机体的变换配置 ---
        const auto& transformJson = mainJson.at("imu_to_body_transform");
        this->runtime.imuToBodyTransform.method = transformJson.at("method").get<std::string>();

        if (this->runtime.imuToBodyTransform.method == "predefined")
        {
            this->runtime.imuToBodyTransform.predefinedFormat = transformJson.at("predefined_format").get<std::string>();
            
            // 从预定义格式字符串生成旋转矩阵 (R_frd_from_this)
            static const std::map<std::string, Matrix3d> predefined_rotations = {
                {"FRD", (Matrix3d() << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished()},
                {"FLU", (Matrix3d() << 1, 0, 0, 0,-1, 0, 0, 0,-1).finished()},
                {"RDF", (Matrix3d() << 0, 1, 0, 0, 0, 1, 1, 0, 0).finished()},
                {"RUF", (Matrix3d() << 0, 0, 1,-1, 0, 0, 0,-1, 0).finished()},
                {"NWU", (Matrix3d() << 1, 0, 0, 0,-1, 0, 0, 0,-1).finished()},
                {"LFD", (Matrix3d() << 0, 1, 0,-1, 0, 0, 0, 0, 1).finished()},
                {"RUB", (Matrix3d() << 0, 0,-1, 1, 0, 0, 0,-1, 0).finished()}
            };

            auto it = predefined_rotations.find(this->runtime.imuToBodyTransform.predefinedFormat);
            if (it != predefined_rotations.end())
            {
                this->runtime.imuToBodyTransform.rotationMatrix = it->second;
            }
            else
            {
                std::cerr << "Error: Unknown predefined IMU format '" << this->runtime.imuToBodyTransform.predefinedFormat << "'" << std::endl;
                return false;
            }
        }
        else if (this->runtime.imuToBodyTransform.method == "matrix")
        {
            // 直接从JSON解析旋转矩阵
            auto matrix_data = transformJson.at("rotation_matrix").get<std::vector<std::vector<double>>>();
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) this->runtime.imuToBodyTransform.rotationMatrix(i, j) = matrix_data[i][j];
        }
        else
        {
            std::cerr << "Error: Invalid imu_to_body_transform method '" << this->runtime.imuToBodyTransform.method << "'" << std::endl;
            return false;
        }

        /// --- 加载IMU硬件配置 ---
        std::ifstream imuFile(this->runtime.imuHardwareSpecFile);
        if (!imuFile.is_open())
        {
            std::cerr << "Error: Could not open IMU hardware config file: " << this->runtime.imuHardwareSpecFile << std::endl;
            return false;
        }
        json imuJson = json::parse(imuFile);
        this->hardware.sensorModel = imuJson.at("sensor_model").get<std::string>();
        this->hardware.frequencyHz = imuJson.at("frequency_hz").get<double>();
        this->hardware.gyroNoiseRms = imuJson.at("gyroscope").at("noise_density_rms").get<double>();
        this->hardware.gyroBiasInstabilityRms = imuJson.at("gyroscope").at("bias_instability_rms").get<double>();
        this->hardware.accelNoiseRms = imuJson.at("accelerometer").at("noise_density_rms").get<double>();
        this->hardware.accelBiasInstabilityRms = imuJson.at("accelerometer").at("bias_instability_rms").get<double>();
        
        /// --- 加载算法特定配置 ---
        if (this->runtime.algorithmName == "UKF")
        {
            std::ifstream algoFile(this->runtime.algorithmConfigFile);
            if (!algoFile.is_open())
            {
                    std::cerr << "Error: Could not open algorithm config file: " << this->runtime.algorithmConfigFile << std::endl;
                    return false;
            }
            json algoJson = json::parse(algoFile);
            this->ukfConfig.motionNoiseScalingFactor = algoJson.at("parameters").at("motion_noise_scaling_factor").get<double>();
            this->ukfConfig.motionDetectionThresholdMs2 = algoJson.at("parameters").at("motion_detection_threshold_ms2").get<double>();
            this->ukfConfig.ukfAlpha = algoJson.at("parameters").at("ukf_alpha").get<double>();
            this->ukfConfig.ukfBeta = algoJson.at("parameters").at("ukf_beta").get<double>();
            this->ukfConfig.ukfKappa = algoJson.at("parameters").at("ukf_kappa").get<double>();
            this->ukfConfig.initialAttitudeUncertaintyDeg = algoJson.at("initial_covariance_P0").at("attitude_deg").get<double>();
            this->ukfConfig.initialGyroBiasUncertaintyDps = algoJson.at("initial_covariance_P0").at("gyro_bias_dps").get<double>();
        }
        else if (this->runtime.algorithmName == "ESEKF")
        {
            std::ifstream algoFile(this->runtime.algorithmConfigFile);
            if (!algoFile.is_open())
            {
                    std::cerr << "Error: Could not open algorithm config file: " << this->runtime.algorithmConfigFile << std::endl;
                    return false;
            }
            json algoJson = json::parse(algoFile);
            this->esEkfAhrsConfig.lowDynamicThreshold = algoJson.at("parameters").at("low_dynamic_threshold").get<double>();
            this->esEkfAhrsConfig.initialAttitudeUncertaintyDeg = algoJson.at("initial_covariance_P0").at("attitude_deg").get<double>();
            this->esEkfAhrsConfig.initialGyroBiasUncertaintyDps = algoJson.at("initial_covariance_P0").at("gyro_bias_dps").get<double>();
        }
        
    }
    catch (const json::exception& e)
    {
        /// 捕获JSON解析或访问错误
        std::cerr << "JSON configuration error: " << e.what() << std::endl;
        return false;
    }

    /// 所有配置加载成功
    return true;
}