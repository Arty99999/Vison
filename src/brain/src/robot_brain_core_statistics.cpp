//
// Created by plutoli on 2022-06-11.
//

#include "robot_brain_core_statistics.h"

// ******************************  RobotBrainCoreStatistics类的公有函数  ******************************

// 构造函数
RobotBrainCoreStatistics::RobotBrainCoreStatistics():
    ReadCameraDataIndex(0),
    ProcessedCameraDataIndex(0),
    FightControlCommandIndex(0),
    BuffControlCommandIndex(0),
    ProcessTime(0),
    Fps(0)
{
}