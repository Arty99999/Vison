//
// Created by plutoli on 2021/8/2.
//

#include "classical_light_bar.h"

// ******************************  ClassicalLightBar类的公有函数  ******************************

// 构造函数
ClassicalLightBar::ClassicalLightBar():
    Contour(),
    MinRotatedRect(),
    MinEnclosedRect(),
    LeftUpper(0.0, 0.0),
    LeftLower(0.0, 0.0),
    RightLower(0.0, 0.0),
    RightUpper(0.0, 0.0),
    Center(0.0, 0.0),
    Width(0.0),
    Height(0.0),
    Angle(0.0),
    Area(0.0)
{
}

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