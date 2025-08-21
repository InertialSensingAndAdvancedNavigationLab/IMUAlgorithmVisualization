/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含Eigen的核心密集矩阵和向量库
#include <Eigen/Dense>

/**
 * @using       Vector2d
 * @brief       定义一个二维双精度浮点型向量
 *  @details     这是对 Eigen::Matrix<double, 2, 1> 的类型别名。
 **/
using Vector2d = Eigen::Vector2d;

/**
 * @using       Vector3d
 * @brief       定义一个三维双精度浮点型向量
 *  @details     这是对 Eigen::Matrix<double, 3, 1> 的类型别名，用于在整个项目中
 * 统一表示三维向量，例如位置、速度、加速度和角速度。
 **/
using Vector3d = Eigen::Vector3d;

/**
 * @using       Vector4d
 * @brief       定义一个四维双精度浮点型向量
 *  @details     这是对 Eigen::Matrix<double, 4, 1> 的类型别名。
 **/
using Vector4d = Eigen::Vector4d;