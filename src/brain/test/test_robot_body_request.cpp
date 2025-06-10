//
// Created by plutoli on 22-4-17.
//

#include <string>
#include "robot_body_request.h"

int main(int argc, char *argv[])
{
    // 初始化机器人本体请求
    // RobotBodyRequest request;
    // request.Type = ERequestType::SwitchWorkMode;
    // request.Datas.emplace_back(1);
    // request.Datas.emplace_back(3);

    // 初始化机器人本体请求
    RobotBodyRequest request;
    request.Type = ERequestType::SaveLog;
    request.Datas.emplace_back(2);
    request.Datas.emplace_back('H');
    request.Datas.emplace_back('e');
    request.Datas.emplace_back('l');
    request.Datas.emplace_back('l');
    request.Datas.emplace_back('o');
    request.Datas.emplace_back(' ');
    request.Datas.emplace_back('w');
    request.Datas.emplace_back('o');
    request.Datas.emplace_back('r');
    request.Datas.emplace_back('l');
    request.Datas.emplace_back('d');

    switch (request.Type)
    {
        // 切换工作模式
        case ERequestType::SwitchWorkMode:
        {
            // 判断请求的数据长度是否合法
            if (request.Datas.size() != 2)
            {
                break;
            }

            // 测试break指令的跳出范围
            for (int i = 0; i < 5; ++i)
            {
                if (i == 2)
                {
                    break;
                }
            }

            // 转换机器人本体请求切换的工作模式
            EWorkMode workMode = EWorkMode::AutomaticShoot;
            if (!SystemConfigurator::ConvertToWorkMode(request.Datas[1], &workMode))
            {
                break;
            }

            break;
        }

        // 保存日志
        case ERequestType::SaveLog:
        {
            // 提取请求的机器人内核ID
            std::string id = std::to_string(request.Datas[0]);

            // 提取请求的日志内容
            std::string message;
            message.assign(request.Datas.begin() + 1, request.Datas.end());

            break;
        }

        default:
            break;
    }

    return 0;
}