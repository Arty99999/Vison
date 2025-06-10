//
// Created by plutoli on 2021/8/2.
//

#include "classical_armor_solver.h"

// ******************************  ClassicalLightBar类的公有函数  ******************************

// 转换灯条颜色
bool ClassicalLightBar::ConvertToClassicalLightBarColor(const int &input, EClassicalLightBarColor *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换灯条颜色
    switch (input)
    {
        case 1:
            *output = EClassicalLightBarColor::Red;
            break;

        case 2:
            *output = EClassicalLightBarColor::Blue;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}