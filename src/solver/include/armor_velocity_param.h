//
// Created by plutoli on 2022/1/28.
//

#ifndef CUBOT_BRAIN_ARMOR_VELOCITY_PARAM_H
#define CUBOT_BRAIN_ARMOR_VELOCITY_PARAM_H

#include <vector>
#include "system_configurator.h"

/**
 * @brief 装甲板速度补偿参数
 */
class ArmorVelocityParam
{
public:
    std::vector<EWorkMode> WorkModes;    ///< 装甲板速度补偿参数适配的工作模式集合
    EPolynomialOrder Order;              ///< 装甲板速度补偿参数的阶数
    int MaxIterationNumber;              ///< 装甲板速度补偿多项式拟合时的最大迭代次数
    int SampleNumber;                    ///< 装甲板速度补偿多项式拟合时的样本数量

    /**
    * @brief 构造函数
    */
    ArmorVelocityParam();

    /**
     * @brief 析构函数
     */
    ~ArmorVelocityParam() = default;
};

#endif //CUBOT_BRAIN_ARMOR_VELOCITY_PARAM_H