//
// Created by plutoli on 2021/8/2.
//

#include "classical_armor.h"

// ******************************  ClassicalArmor类的公有函数  ******************************

// 构造函数
ClassicalArmor::ClassicalArmor():
    LeftLightBar(),
    RightLightBar(),
    Number(EClassicalArmorNumber::Invalid),
    Area(0.0),
    Offset(0.0),
    LeftUpper(0.0, 0.0),
    LeftLower(0.0, 0.0),
    RightUpper(0.0, 0.0),
    RightLower(0.0, 0.0),
    Center(0.0, 0.0),
    Image(),
    Timestamp(0)
{
}

// 转换装甲板编号
bool ClassicalArmor::ConvertToClassicalArmorNumber(const int &input, EClassicalArmorNumber *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换装甲板编号
    switch (input)
    {
        case 0:
            *output = EClassicalArmorNumber::Invalid;
            break;

        case 1:
            *output = EClassicalArmorNumber::One;
            break;

        case 2:
            *output = EClassicalArmorNumber::Two;
            break;

        case 3:
            *output = EClassicalArmorNumber::Three;
            break;

        case 4:
            *output = EClassicalArmorNumber::Four;
            break;

        case 5:
            *output = EClassicalArmorNumber::Five;
            break;

        case 6:
            *output = EClassicalArmorNumber::Sentry;
            break;

        case 7:
            *output = EClassicalArmorNumber::Outpost;
            break;

        case 8:
            *output = EClassicalArmorNumber::Base;
            break;

        default:
            result = false;
            *output = EClassicalArmorNumber::Invalid;
            break;
    }

    // 返回转换结果
    return result;
}