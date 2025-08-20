/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含我们定义的算法基类
#include "DefaultAhrs.hpp"

/// 包含IO和标准库依赖
#include "IO/TumTrajectoryWriter.hpp" // For PoseStamped
#include <vector>

/**
 * @class       EkfAhrs
 * @brief       一个基于误差状态扩展卡尔曼滤波(ESKF)的AHRS算法
 *  @extends     public DefaultAhrs
 *  @details     此类实现了一个高度透明和模块化的ESKF-AHRS，便于学习和调试。
 * 它的所有核心状态和协方差矩阵都为`public`，所有核心数学步骤都被
 * 封装在独立的`protected`函数中。
 *
 * **核心ESKF方程:**
 * 1. **误差状态预测 (Error State Prediction):**
 *    \f[
 *    \delta x_k^- = F_k \delta x_{k-1}
 *    \f]
 *    \f[
 *    P_k^- = F_k P_{k-1} F_k^T + Q_k
 *    \f]
 *
 * 2. **误差状态更新 (Error State Update):**
 *    \f[
 *    y_k = z_k - h(\hat{x}_k^-)
 *    \f]
 *    \f[
 *    S_k = H_k P_k^- H_k^T + R_k
 *    \f]
 *    \f[
 *    K_k = P_k^- H_k^T S_k^{-1}
 *    \f]
 *    \f[
 *    \delta x_k^+ = K_k y_k
 *    \f]
 *    \f[
 *    P_k^+ = (I - K_k H_k) P_k^-
 *    \f]
 *
 * 3. **名义状态注入 (Nominal State Injection):**
 *    \f[
 *    \hat{q}_k^+ \leftarrow \hat{q}_k^- \otimes \text{quat}(\delta\theta_k^+)
 *    \f]
 *    \f[
 *    \hat{b}_k^+ \leftarrow \hat{b}_k^- + \delta b_k^+
 *    \f]
 **/
class EkfAhrs : public DefaultAhrs
{
/// 公共接口 (实现基类)
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EkfAhrs();
    ~EkfAhrs() override = default;

    bool initialize(const ConfigurationManager& configManager) override;
    void update(const ImuMeasurement& imuData) override;
    Quaternion getOrientation() const override;
    bool isInitialized() const override;
    void runDebugExport(const std::string& outputFilePath) const override;

/// 核心状态与协方差 (Public for easy access & visualization)
public:
    // --- 名义状态 ---
    /// @brief 名义状态：姿态四元数
    Quaternion orientation;
    /// @brief 名义状态：陀螺仪零偏
    Vector3d gyroBias;

    // --- EKF 误差状态与协方差 ---
    /// @brief 误差状态协方差矩阵 P (6x6)
    Eigen::Matrix<double, 6, 6> covariance;
    /// @brief 过程噪声协方差矩阵 Q (6x6)
    Eigen::Matrix<double, 6, 6> processNoiseQ;
    /// @brief 观测噪声协方差矩阵 R (3x3)
    Eigen::Matrix3d measurementNoiseR;

    // --- 其他状态 ---
    /// @brief 标记滤波器是否已经成功初始化
    bool bIsInitialized;
    /// @brief 记录上一个数据点的时间戳
    double lastTimestamp;
    /// @brief 重力向量参考
    Vector3d gravity;
    
    /// @brief (用于调试导出) 遥测数据历史记录
    struct TelemetryData
    {
        double timestamp;
        Quaternion orientation;
        Vector3d gyroBias;
        Eigen::Matrix<double, 6, 1> covarianceDiagonal;
        Eigen::Matrix3d measurementNoiseR; // 记录当时的R
        Eigen::Matrix<double, 6, 3> kalmanGain; // 记录当时的K
    };
    std::vector<TelemetryData> telemetryHistory;

/// 模块化的保护成员函数 (Protected for internal use & potential extension)
protected:
    /**
     * @brief       步骤 1.1: 传播名义状态
     *  @details     使用(已修正零偏的)陀螺仪数据进行一步四元数积分。
     *
     * @param       参数名称: gyroData, deltaTime         ...
     **/
    void propagateNominalState(const Vector3d& gyroData, double deltaTime);

    /**
     * @brief       步骤 1.2: 传播误差状态协方差
     *  @details     执行 EKF 预测步骤: P_k⁻ = F * P * F^T + Q
     *
     * @param       参数名称: correctedGyro, deltaTime    ...
     **/
    void propagateErrorCovariance(const Vector3d& correctedGyro, double deltaTime);

    /**
     * @brief       步骤 2.1: 计算观测雅可比矩阵 H
     *  @details     计算加速度计观测模型关于误差状态的雅可比矩阵。
     * \f$ H = \frac{\partial h(\hat{x}^-, \delta x)}{\partial \delta x} \bigg|_{\delta x=0} \f$
     *
     * @return      3x6 的观测雅可比矩阵 H           数据类型:        Eigen::Matrix<double, 3, 6>
     **/
    Eigen::Matrix<double, 3, 6> computeObservationJacobian() const;

    /**
     * @brief       步骤 2.2: 计算动态观测噪声 R
     *  @details     根据当前加速度的模长，动态调整对加速度计的信任度。
     *
     * @param       参数名称: accelMagnitude              ...
     * @return      3x3 的观测噪声矩阵 R              数据类型:        Eigen::Matrix3d
     **/
    Eigen::Matrix3d computeMeasurementNoise(double accelMagnitude) const;
    
    /**
     * @brief       步骤 2.3: 计算卡尔曼增益 K
     *  @details     执行 K = P⁻ * H^T * (H * P⁻ * H^T + R)⁻¹
     *
     * @param       参数名称: jacobianH, noiseR           ...
     * @return      6x3 的卡尔曼增益矩阵 K            数据类型:        Eigen::Matrix<double, 6, 3>
     **/
    Eigen::Matrix<double, 6, 3> computeKalmanGain(
        const Eigen::Matrix<double, 6, 6>& predictedCov,
        const Eigen::Matrix<double, 3, 6>& jacobianH,
        const Eigen::Matrix3d& noiseR) const;

    /**
     * @brief       步骤 2.4 & 3: 更新误差状态并注入名义状态
     *  @details     计算误差状态估计，更新协方差，并将误差注入回名义状态中，
     * 最后重置误差状态（在我们的实现中是隐式的）。
     *
     * @param       参数名称: kalmanGain, innovation, jacobianH ...
     **/
    void updateAndInject(
        const Eigen::Matrix<double, 6, 6>& predictedCov,
        const Eigen::Matrix<double, 6, 3>& kalmanGain,
        const Vector3d& innovation,
        const Eigen::Matrix<double, 3, 6>& jacobianH);
};