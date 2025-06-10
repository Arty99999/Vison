//
// Created by plutoli on 2022/2/24.
//

#include "robot_brain_control_command.h"

int main(int argc, char *argv[])
{
    // 初始化机器人大脑指令
    RobotBrainControlCommand command;
    command.ID = 1;
    command.Yaw = 23.45;
    command.Pitch = -12.34;
    command.Distance = 5792.8;

    // 创建指令数据帧
    unsigned char commandFrame[RobotBrainControlCommand::GetFrameSize()];
    command.EncapsulateToFrame(commandFrame);

    return 0;
}