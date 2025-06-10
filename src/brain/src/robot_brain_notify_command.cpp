//
// Created by plutoli on 2022/1/30.
//

#include "robot_brain_notify_command.h"

// ******************************  RobotBrainNotifyCommand类的公有函数  ******************************

// 构造函数
RobotBrainNotifyCommand::RobotBrainNotifyCommand():
    ID(1),
    WorkMode(EWorkMode::Manual),
    BulletVelocity(EBulletVelocity::MPS_10),
    IsInitialized(false),
    IsOpened(false),
    IsCameraConnected(false),
    IsCameraWorking(false)
{
}

// 获取提示指令打包之后的数据帧字节长度
unsigned int RobotBrainNotifyCommand::GetFrameSize()
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

    // 机器人大脑内核的工作模式占用1个字节
    unsigned int workModeSize = 1;
    totalSize += workModeSize;

    // 机器人大脑内核的子弹速度占用1个字节
    unsigned int bulletVelocitySize = 1;
    totalSize += bulletVelocitySize;

    // 机器人大脑内核的状态位占用1个字节
    unsigned int flagSize = 1;
    totalSize += flagSize;

    // 数据帧的帧尾占用1个字节
    unsigned int tailSize = 1;
    totalSize += tailSize;

    // 返回数据长度
    return totalSize;
}

// 将提示指令封装成数据帧
void RobotBrainNotifyCommand::EncapsulateToFrame(unsigned char *frame) const
{
    // 初始化字节索引
    unsigned int index = 0;

    // 写入数据帧的帧头
    *(frame + index) = 0xAA;
    index += 1;

    // 写入数据帧的类型
    *(frame + index) = 0x02;
    index += 1;

    // 写入机器人大脑内核ID
    *(frame + index) = ID;
    index += 1;

    // 写入机器人大脑内核的工作模式
    auto workMode = static_cast<unsigned char>(WorkMode);
    *(frame + index) = workMode;
    index += 1;

    // 写入机器人大脑内核的子弹速度
    auto bulletVelocity = static_cast<unsigned char>(BulletVelocity);
    *(frame + index) = bulletVelocity;
    index += 1;

    // 写入机器人大脑内核的标志位
    unsigned char flag = 0x00;
    if (IsInitialized)
    {
        flag = flag | 0x01;
    }
    if (IsOpened)
    {
        flag = flag | 0x02;
    }
    if (IsCameraConnected)
    {
        flag = flag | 0x04;
    }
    if (IsCameraWorking)
    {
        flag = flag | 0x08;
    }
    *(frame + index) = flag;
    index += 1;

    // 写入数据帧的帧尾
    *(frame + index) = 0xDD;
}