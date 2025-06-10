//
// Created by plutoli on 2022/1/26.
//

#include "keyboard_scanner.h"

// ******************************  KeyboardScanner类的公有函数  ******************************

// 需要回车操作，直接扫描键盘按下的键值
int KeyboardScanner::GetPressedKeyWithoutEnter()
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 获取终端配置信息
    struct termios rawSettings;
    if (::tcgetattr(0, &rawSettings) == -1)
    {
        log = "Terminal's raw settings was got failure";
        logger.Save(ELogType::Error, log);
    }

    // 修改终端配置信息
    struct termios newSettings;
    newSettings = rawSettings;
    newSettings.c_lflag &= (~ICANON);
    newSettings.c_lflag &= (~ECHO);
    newSettings.c_lflag &= (~ISIG);
    newSettings.c_cc[VTIME] = 0;
    newSettings.c_cc[VMIN] = 1;
    if (::tcsetattr(0, TCSANOW, &newSettings) == -1)
    {
        log = "Terminal's new settings was set failure";
        logger.Save(ELogType::Error, log);
    }

    // 通过阻塞方式读取键值
    int keyValue = std::getchar();

    // 恢复终端配置信息
    if (::tcsetattr(0, TCSANOW, &rawSettings) == -1)
    {
        log = "Terminal's raw settings was restored failure";
        logger.Save(ELogType::Error, log);
    }

    // 返回读取到的键值
    return keyValue;
}