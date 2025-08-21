/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含我们项目中的数学类型
#include "math/Quaternion.hpp"

/**
 * @enum        CoordinateFrame
 * @brief       定义项目中使用的所有标准坐标系
**/
enum class CoordinateFrame
{
    /// @brief EuRoC IMU 原始坐标系 (RUF: 右-上-前)
    IMU_ORIG_RUF,
    /// @brief 内部计算坐标系 (FRD: 前-右-下)
    INTERNAL_FRD,
    /// @brief TUM/可视化坐标系 (RDF: 右-下-前)
    TUM_VIZ_RDF
};

/**
 * @class       CoordinateSystemManager
 * @brief       管理所有坐标系之间固定变换关系的静态工具类
 *  @details     此类提供了一个中心化的位置来定义和获取所有固定的
 * 坐标系旋转。所有定义均严格遵守 Readme.md 中的约定。
**/
class CoordinateSystemManager
{
public:
    /// @brief 从 IMU 原始坐标系 (RUF) 到 内部计算坐标系 (FRD) 的旋转矩阵
    /// @details C_internal_from_imu, 使得 v_internal = C_internal_from_imu * v_imu_orig
    static const Eigen::Matrix3d C_internal_from_imu;

    /// @brief 从 内部计算坐标系 (FRD) 到 可视化坐标系 (RDF) 的旋转矩阵
    /// @details C_viz_from_internal, 使得 v_viz = C_viz_from_internal * v_internal
    static const Eigen::Matrix3d C_viz_from_internal;
};