//
// Created by plutoli on 2022-05-19.
//

#include <iostream>
#include "robot_brain_control_command.h"
#include "robot_brain_notify_command.h"
#include "serial_port_param.h"
#include "serial_port.h"

// 串口数据接收缓冲区
unsigned char receivedBuffer[1000];

// 串口数据接收回调函数
void HandleDataReceived(const unsigned int &bytesToRead, void* userData)
{
    // 读取请求数据帧
    auto serialPort = (SerialPort *)userData;
    unsigned int frameSize = serialPort->ReadLine(receivedBuffer);

    // 获取当前时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    uint64_t timestamp = now.time_since_epoch().count();

    // 解析机器人本体的请求
    if ((frameSize >= 3) && (receivedBuffer[0] == 0xAA) && (receivedBuffer[frameSize - 1] == 0xDD))
    {
        ERequestType type;
        if (SystemConfigurator::ConvertToRequestType(receivedBuffer[1], &type))
        {
            std::cout << "Received valid request type: " << static_cast<int>(type) << std::endl;
            std::cout << "received request timestamp: " << timestamp / 1000000 << std::endl << std::endl;
        }
        else
        {
            std::cout << "Received invalid request type" << std::endl;
            std::cout << "received request timestamp: " << timestamp / 1000000 << std::endl << std::endl;
        }
    }
}

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
    std::string yameFile = "config/infantry_4/basement/serial_port_param.yaml";
    if (!SerialPortParam::LoadFromYamlFile(yameFile, &serialPortParam))
    {
        return -1;
    }

    // 设置串口参数
    if (!serialPort.SetParam(serialPortParam))
    {
        return -1;
    }

    // 注册串口的数据接收回调函数
    serialPort.RegisterDataReceivedHandler(HandleDataReceived, &serialPort);

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

        // 更新指令数据帧索引
        commandIndex++;

        // 获取时间戳
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        uint64_t timestamp = now.time_since_epoch().count();

        // 显示输出信息
        std::cout << "command index: " << commandIndex << std::endl;
        std::cout << "serialPort status: " << serialPort.IsNormal() << std::endl;
        std::cout << "issue command timestamp: " << timestamp / 1000000 << std::endl << std::endl;

        // 延时1000毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}