//
// Created by plutoli on 2021/8/13.
//

#include "robot_fight_data.h"

// ******************************  RobotFightData类的公有函数  ******************************

// 构造函数
RobotFightData::RobotFightData():
    Type(EFightDataType::None),
    Image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255))
{
}

// 转换机器人大脑内核的战斗数据类型
bool RobotFightData::ConvertToFightDataType(const int &input, EFightDataType *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换机器人大脑内核的战斗数据类型
    switch (input)
    {
        case 0:
            *output = EFightDataType::None;
            break;

        case 1:
            *output = EFightDataType::RawImage;
            break;

        case 2:
            *output = EFightDataType::PolishedArmorsImage;
            break;

        case 3:
            *output = EFightDataType::FilteredArmorsImage;
            break;

        case 4:
            *output = EFightDataType::EvaluatedArmorsImage;
            break;

        case 5:
            *output = EFightDataType::LockedArmorImage;
            break;

        case 6:
            *output = EFightDataType::ComparedArmorImage;
            break;

        case 7:
            *output=EFightDataType::PredictorArmorImage;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}