//
// Created by plutoli on 2022/1/30.
//

#include "robot_brain_control_command.h"
#include <iostream>
// ******************************  RobotBrainControlCommand类的公有函数  ******************************

// 构造函数
RobotBrainControlCommand::RobotBrainControlCommand():
        ID(1),
        Yaw(0.0),
        Pitch(0.0),
        Top(false),
        IsFire(false)
{
}

// 获取控制指令打包之后的数据帧字节长度
unsigned int RobotBrainControlCommand::GetFrameSize()
{
    // 初始化打包之后的字节总长度
    unsigned int totalSize = 0;

    // 数据帧的帧头占用1个字节
    unsigned int headerSize = 1;
    totalSize += headerSize;

    // 数据帧的类型占用1个字节
    unsigned int typeSize = 1;
    totalSize += typeSize;

    // 机器人大脑内核ID占用1个字节
    unsigned int idSize = 1;
    totalSize += idSize;

    // Yaw轴的偏转角度占用2个字节
    unsigned int yawSize = 2;
    totalSize += yawSize;

    // Pitch轴的偏转角度占用2个字节
    unsigned int pitchSize = 2;
    totalSize += pitchSize;

    // 击打目标点到枪口的距离占用1个字节
    unsigned int distanceSize = 1;
    totalSize += distanceSize;

    // 开火标志占用1个字节
    unsigned int isFireSize = 1;
    totalSize += isFireSize;

    unsigned int yawSize1 = 2;
    totalSize += yawSize1;

    // 数据帧的帧尾占用1个字节
    unsigned int tailSize = 1;
    totalSize += tailSize;

    // 返回数据长度
    return totalSize;
}

// 将控制指令封装成数据帧
void RobotBrainControlCommand::EncapsulateToFrame(unsigned char *frame) const
{
    // 初始化字节索引
    unsigned int index = 0;

    // 写入数据帧的帧头
    *(frame + index) = 0xAA;
    index += 1;

    // 写入数据帧的类型
    *(frame + index) = 0x01;
    index += 1;

    // 写入机器人大脑内核ID
    *(frame + index) = ID;
    index += 1;

    // 提取Yaw轴偏转角度的整数部分和小数部分
    double yawInteger;
    double yawFraction = std::modf(Yaw, &yawInteger);

    // 写入Yaw轴偏转角度的整数部分
    double yawIntegerAbs = (std::abs(yawInteger) > 63.0) ? 63.0 : std::abs(yawInteger);
    auto yawIntegerValue = static_cast<unsigned char>(yawIntegerAbs);
    if (Yaw < 0)
    {
        *(frame + index) = 0x40 + yawIntegerValue;
    }
    else
    {
        *(frame + index) = yawIntegerValue;
    }
    index += 1;

    // 写入Yaw轴偏转角度的小数部分
    double yawFractionAbs = std::round(std::abs(yawFraction * 100.0));
    auto yawFractionValue = static_cast<unsigned char>(yawFractionAbs);
    *(frame + index) = yawFractionValue;
    index += 1;
//    std::cout<<"yaw"<<yawIntegerAbs<<"."<<yawFractionAbs<<std::endl;

    // 提取Pitch轴偏转角度的整数部分和小数部分
    double pitchInteger;
    double pitchFraction = std::modf(Pitch, &pitchInteger);

    // 写入Pitch轴偏转角度的整数部分
    double pitchIntegerAbs = (std::abs(pitchInteger) > 63.0) ? 63.0 : std::abs(pitchInteger);
    auto pitchIntegerValue = static_cast<unsigned char>(pitchIntegerAbs);
    if (Pitch < 0)
    {
        *(frame + index) = 0x40 + pitchIntegerValue;
    }
    else
    {
        *(frame + index) = pitchIntegerValue;
    }
    index += 1;
    // 写入Pitch轴偏转角度的小数部分
    double pitchFractionAbs = std::round(std::abs(pitchFraction * 100.0));
    auto pitchFractionValue = static_cast<unsigned char>(pitchFractionAbs);
    *(frame + index) = pitchFractionValue;
    index += 1;

    // 写入击打目标点到枪口的距离
    double distanceAbs = (std::abs(Distance / 100.0) > 127.0) ? 127.0 : std::abs(Distance / 100.0);
    *(frame + index) = static_cast<unsigned char>(std::round(distanceAbs));
    index += 1;

    // 写入是否开火标志
    if (IsFire)
    {
        *(frame + index) = 0x01;
    }
    else
    {
        *(frame + index) = 0x00;
    }
    index += 1;

    double yawInteger1;
    double yawFraction1 = std::modf(Yaw1, &yawInteger1);

    // 写入Yaw轴偏转角度的整数部分
    double yawIntegerAbs1 = (std::abs(yawInteger1) > 63.0) ? 63.0 : std::abs(yawInteger1);
    auto yawIntegerValue1 = static_cast<unsigned char>(yawIntegerAbs1);
    if (Yaw1 < 0)
    {
        *(frame + index) = 0x40 + yawIntegerValue1;
    }
    else
    {
        *(frame + index) = yawIntegerValue1;
    }
    index += 1;
    //std::cout<<"yaw"<<yawIntegerAbs<<std::endl;
    // 写入Yaw轴偏转角度的小数部分
    double yawFractionAbs1 = std::round(std::abs(yawFraction1 * 100.0));
    auto yawFractionValue1 = static_cast<unsigned char>(yawFractionAbs1);
    *(frame + index) = yawFractionValue1;
    index += 1;

    // 写入数据帧的帧尾
    *(frame + index) = 0xDD;
}