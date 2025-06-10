//
// Created by plutoli on 2022/2/24.
//

#include "robot_brain_notify_command.h"

int main(int argc, char *argv[])
{
    // 初始化机器人大脑内核指令
    RobotBrainNotifyCommand command;
    command.ID = 1;
    command.WorkMode = EWorkMode::AutomaticShoot;
    command.BulletVelocity = EBulletVelocity::MPS_18;
    command.IsInitialized = true;
    command.IsOpened = true;
    command.IsCameraConnected = true;
    command.IsCameraWorking = true;

    // 创建指令数据帧
    unsigned char commandFrame[RobotBrainNotifyCommand::GetFrameSize()];
    command.EncapsulateToFrame(commandFrame);

    return 0;
}