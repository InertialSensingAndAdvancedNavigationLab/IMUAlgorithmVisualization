/// 包含类头文件
#include "math/CoordinateSystem.hpp"

/// @brief 从 IMU 原始坐标系 (RUF) 到 内部计算坐标系 (FRD) 的旋转矩阵
/// @details v_internal = C_internal_from_imu * v_imu_orig
/// Internal.X(Fwd) = IMU.Z(Fwd), Internal.Y(Right) = IMU.X(Right), Internal.Z(Down) = -IMU.Y(Up)
const Eigen::Matrix3d CoordinateSystemManager::C_internal_from_imu =
    (Eigen::Matrix3d() << 0, 0, 1,
                         1, 0, 0,
                         0, -1, 0).finished();

/// @brief 从 内部计算坐标系 (FRD) 到 可视化坐标系 (RDF) 的旋转矩阵
/// @details v_viz = C_viz_from_internal * v_internal
/// VIZ.X(Right) = Internal.Y(Right), VIZ.Y(Down) = Internal.Z(Down), VIZ.Z(Fwd) = Internal.X(Fwd)
const Eigen::Matrix3d CoordinateSystemManager::C_viz_from_internal =
    (Eigen::Matrix3d() << 0, 1, 0,
                         0, 0, 1,
                         1, 0, 0).finished();