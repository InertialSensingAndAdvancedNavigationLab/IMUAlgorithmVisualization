/// 使用 #pragma once 防止头文件重复包含
#pragma once

#include <string>
#include <vector>
#include "math/Vector.hpp"
#include "math/Quaternion.hpp"

/**
 * @struct      PoseStamped
 * @brief       存储带有时间戳的单个位姿（位置+姿态）
 **/
struct PoseStamped
{
    double timestamp;
    Vector3d position;
    Quaternion orientation;
};

/**
 * @class       TumTrajectoryWriter
 * @brief       一个用于将轨迹保存为TUM格式的静态工具类
 **/
class TumTrajectoryWriter
{
public:
    /**
     * @brief       将一个位姿轨迹保存为TUM格式的文件
     *  @details     这是一个工具函数，可以将任何符合格式的轨迹数据写入
     * 到一个文件中。它会自动处理四元数的顺序转换。
     *
     * @param       参数名称: filePath                      数据类型:        const std::string&
     * @param       参数名称: trajectory                    数据类型:        const std::vector<PoseStamped>&
     * @return      保存成功返回 true                      数据类型:        bool
     **/
    static bool save(const std::string& filePath, const std::vector<PoseStamped>& trajectory);
};