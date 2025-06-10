//
// Created by plutoli on 2021/12/29.
//

#include "easy_logger.h"
#include "robot_brain_param.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 加载机器人大脑参数
    RobotBrainParam param;
    std::string yamlFileName = "config/infantry_3/basement/robot_brain_param.yaml";
    bool result = RobotBrainParam::LoadFromYamlFile(yamlFileName, &param);

    return 0;
}