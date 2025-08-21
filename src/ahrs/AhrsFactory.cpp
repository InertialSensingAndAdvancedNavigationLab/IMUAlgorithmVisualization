/// 包含类头文件
#include "ahrs/AhrsFactory.hpp"

/// 在这里包含所有具体的算法实现头文件
#include "ahrs/DirectIntegrator.hpp"
#include "ahrs/CalibratedIntegrator.hpp"
#include "ahrs/AccelOnlyAhrs.hpp"
#include "ahrs/EsEkfAhrs.hpp"
// #include "ahrs/UkfAlgorithm.hpp" // 未来会添加

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
std::unique_ptr<DefaultAhrs> AhrsFactory::create(const std::string& algorithmName)
{
    /// 如果名称是 "DirectIntegrator"
    if (algorithmName == "DirectIntegrator")
    {
        /// 创建并返回 DirectIntegrator 的实例
        return std::make_unique<DirectIntegrator>();
    }
    /// 如果名称是 "CalibratedIntegrator"
    else if (algorithmName == "CalibratedIntegrator") // <-- 新增分支
    {
        /// 创建并返回 CalibratedIntegrator 的实例
        return std::make_unique<CalibratedIntegrator>();
    }
    else if (algorithmName == "AccelOnlyAhrs")
    {
        return std::make_unique<AccelOnlyAhrs>();
    }
    else if (algorithmName == "ESEKF")
    {
        return std::make_unique<EsEkfAhrs>();
    }
    /// 如果名称是 "UKF" (未来)
    else if (algorithmName == "UKF")
    {
        // return std::make_unique<UkfAlgorithm>();
    }

    /// 如果没有找到匹配的算法名称，返回空指针
    return nullptr;
}