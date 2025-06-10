//
// Created by plutoli on 2021/8/13.
//

#include "robot_buff_data.h"

// ******************************  RobotBuffData类的公有函数  ******************************

// 构造函数
RobotBuffData::RobotBuffData():
    Type(EBuffDataType::None),
    Image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255))
{
}

// 转换机器人大脑内核的Buff数据类型
bool RobotBuffData::ConvertToBuffDataType(const int &input, EBuffDataType *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换机器人大脑内核的Buff数据类型
    switch (input)
    {
        case 0:
            *output = EBuffDataType::None;
            break;

        case 1:
            *output = EBuffDataType::RawImage;
            break;

        case 2:
            *output = EBuffDataType::PreprocessedImage;
            break;

        case 3:
            *output = EBuffDataType::WindmillContoursImage;
            break;

        case 4:
            *output = EBuffDataType::WindmillLogosImage;
            break;

        case 5:
            *output = EBuffDataType::WindmillFansImage;
            break;

        case 6:
            *output = EBuffDataType::WindmillObjectsImage;
            break;

        case 7:
            *output = EBuffDataType::WindmillAnglesImage;
            break;

        case 8:
            *output = EBuffDataType::LockedFanImage;
            break;

        case 9:
            *output = EBuffDataType::ComparedFanImage;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}