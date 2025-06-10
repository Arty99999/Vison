//
// Created by plutoli on 2022-06-16.
//

#include <iostream>
#include "robot_brain_control_command.h"
#include "serial_port.h"

// Linux命令行列出所有的串口设备：dmesg | grep ttyS*
int main(int, char* [])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 创建串口
    SerialPort serialPort;

    // 读取串口参数
    SerialPortParam serialPortParam;
    std::string yameFile = "config/infantry_3/basement/serial_port_param.yaml";
    if (!SerialPortParam::LoadFromYamlFile(yameFile, &serialPortParam))
    {
        return -1;
    }

    // 设置串口参数
    if (!serialPort.SetParam(serialPortParam))
    {
        return -1;
    }

    // 初始化串口
    if (!serialPort.Init())
    {
        return -1;
    }

    // 打开串口
    if (!serialPort.Open())
    {
        return -1;
    }

    // 初始化控制指令索引
    unsigned int commandIndex = 0;

    // 循环发送数据
    while (true)
    {
        // 创建控制指令数据
        RobotBrainControlCommand command;
        command.ID = 1;
        command.Yaw = 1.1;
        command.Pitch = 2.2;
        command.Distance = 234.0;
        command.IsFire = true;

        // 生成控制指令数据帧
        unsigned int commandFrameSize = RobotBrainControlCommand::GetFrameSize();
        unsigned char commandFrame[commandFrameSize];
        command.EncapsulateToFrame(commandFrame);

        // 将指令数据帧发送给下位机
        serialPort.Write(commandFrameSize, commandFrame);

        // 更新并显示指令数据帧索引
        commandIndex++;
        std::cout << "control command index: " << commandIndex << std::endl;

        // 延时1秒钟
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // 关闭串口
    serialPort.Close();

    // 释放串口资源
    serialPort.Release();

    return 0;
}