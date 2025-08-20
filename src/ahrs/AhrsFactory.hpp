/// 使用 #pragma once 防止头文件重复包含
#pragma once

/// 包含标准库
#include <string>
#include <memory>

/// 包含我们定义的算法基类
#include "DefaultAhrs.hpp"

/**
 * @class       AhrsFactory
 * @brief       根据名称创建不同AHRS算法实例的工厂类
 *  @details     此类采用了工厂设计模式，将对象的创建逻辑与使用逻辑
 * 分离开。`main`函数或其他客户端代码只需要与此工厂交互，
 * 提供一个算法名称字符串，即可获得一个指向具体算法实例的
 * 基类指针，而无需知道任何具体算法类的细节。
 *
 * 当需要添加新的算法时，只需要在此工厂的 `create` 方法中
 * 添加一个新的 `else if` 分支即可，客户端代码无需任何改动。
 **/
class AhrsFactory
{
/// 公共接口
public:
    /**
     * @brief       根据算法名称字符串创建算法实例
     *
     * @param       参数名称: algorithmName                 数据类型:        const std::string&
     *  @details     要创建的算法的名称，例如 "DirectIntegrator", "UKF"。
     *
     * @return      指向创建的算法实例的智能指针        数据类型:        std::unique_ptr<DefaultAhrs>
     *  @retval      如果名称匹配，返回一个有效的指针。
     *  @retval      如果名称未找到，返回一个空的 `nullptr`。
     **/
    static std::unique_ptr<DefaultAhrs> create(const std::string& algorithmName);
};