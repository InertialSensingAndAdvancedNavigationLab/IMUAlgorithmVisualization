/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含Eigen的几何模块，其中定义了四元数
#include <Eigen/Geometry>
/// 包含我们的向量别名文件，因为辅助函数需要用到
#include "Vector.hpp"

/**
 * @using       Quaternion
 * @brief       定义一个双精度浮点型四元数
 *  @details     这是对 Eigen::Quaternion<double> 的类型别名，用于在整个项目中
 * 统一表示姿态。
 **/
using Quaternion = Eigen::Quaterniond;

/**
 * @brief       从一个(w, x, y, z)数组安全地创建一个四元数
 *  @details     封装了Eigen构造函数的顺序，确保在项目中创建四元数时
 * 的顺序始终是 w-x-y-z，避免混淆。
 *
 * @param       参数名称: w                             数据类型:        double
 * @param       参数名称: x                             数据类型:        double
 * @param       参数名称: y                             数据类型:        double
 * @param       参数名称: z                             数据类型:        double
 *
 * @return      构造好的四元数                        数据类型:        Quaternion
 **/
inline Quaternion createQuaternionFromWXYZ(double w, double x, double y, double z)
{
    /// Eigen的构造函数接收 (w, x, y, z)
    return Quaternion(w, x, y, z);
}

/**
 * @brief       以(w, x, y, z)的顺序获取四元数的系数
 *  @details     提供一个统一的接口来访问四元数的元素，隐藏Eigen内部
 * [x, y, z, w] 的存储顺序。
 *
 * @param       参数名称: q                             数据类型:        const Quaternion&
 *
 * @return      一个包含(w, x, y, z)的四维向量      数据类型:        Vector4d
 **/
inline Vector4d getWXYZ(const Quaternion& q)
{
    /// 返回一个 Vector4d，其顺序是我们约定的 w, x, y, z
    return Vector4d(q.w(), q.x(), q.y(), q.z());
}