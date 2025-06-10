//
// Created by plutoli on 2021/8/3.
//

#ifndef CUBOT_BRAIN_CLASSICAL_ARMOR_EVALUATION_WEIGHT_H
#define CUBOT_BRAIN_CLASSICAL_ARMOR_EVALUATION_WEIGHT_H

#include <vector>
#include "classical_armor.h"

/**
 * @brief 装甲板评估权值
 */
class ClassicalArmorEvaluationWeight
{
public:
    EClassicalArmorNumber ArmorNumber;      ///< 装甲板编号
    float OffsetCoeff;                      ///< 装甲板的中心偏转距离系数
    float AreaCoeff;                        ///< 装甲板的面积系数
    float ImportanceCoeff;                  ///< 装甲板的重要性系数

    /**
     * @brief 构造函数
     */
    ClassicalArmorEvaluationWeight();

    /**
     * @brief 析构函数
     */
    ~ClassicalArmorEvaluationWeight() = default;
};

#endif //CUBOT_BRAIN_CLASSICAL_ARMOR_EVALUATION_WEIGHT_H