//
// Created by plutoli on 2021/8/2.
//

#include "classical_armor.h"

// ******************************  ClassicalArmor类的公有函数  ******************************

// 构造函数
ClassicalArmor::ClassicalArmor():
    Number(EClassicalArmorNumber::Invalid),
    Area(0.0),
    Offset(0.0),
    boxHeight(0.0),
    boxWidth(0.0),
    boxScore(0.0),
    Center(0.0, 0.0),
    boxCenter(0.0, 0.0),
    LeftUpper(0.0, 0.0),
    boxLeftUpper(0.0, 0.0),
    LeftLower(0.0, 0.0),
    RightUpper(0.0, 0.0),
    RightLower(0.0, 0.0),
    boxRightLower(0.0, 0.0),
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
            *output = EClassicalArmorNumber::BlueSentry;
            break;

        case 1:
            *output = EClassicalArmorNumber::BlueOne;
            break;

        case 2:
            *output = EClassicalArmorNumber::BlueTwo;
            break;

        case 3:
            *output = EClassicalArmorNumber::BlueThree;
            break;

        case 4:
            *output = EClassicalArmorNumber::BlueFour;
            break;

        case 5:
            *output = EClassicalArmorNumber::BlueFive;
            break;

        case 6:
            *output = EClassicalArmorNumber::BlueOutpost;
            break;

        case 7:
            *output = EClassicalArmorNumber::BlueBase;
            break;

        case 8:
            *output = EClassicalArmorNumber::BlueBaseBig;
            break;

        case 9:
            *output = EClassicalArmorNumber::RedSentry;
            break;

        case 10:
            *output = EClassicalArmorNumber::RedOne;
            break;

        case 11:
            *output = EClassicalArmorNumber::RedTwo;
            break;

        case 12:
            *output = EClassicalArmorNumber::RedThree;
            break;

        case 13:
            *output = EClassicalArmorNumber::RedFour;
            break;

        case 14:
            *output = EClassicalArmorNumber::RedFive;
            break;

        case 15:
            *output = EClassicalArmorNumber::RedOutpost;
            break;

        case 16:
            *output = EClassicalArmorNumber::RedBase;
            break;

        case 17:
            *output = EClassicalArmorNumber::RedBaseBig;
            break;

        default:
            result = false;
            *output = EClassicalArmorNumber::Invalid;
            break;
    }

    // 返回转换结果
    return result;
}