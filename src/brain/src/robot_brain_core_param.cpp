//
// Created by plutoli on 2021/12/28.
//

#include "robot_brain_core_param.h"

// ******************************  RobotBrainCoreParam类的公有函数  ******************************

// 构造函数
RobotBrainCoreParam::RobotBrainCoreParam():
    Key(),
    ID(1),
    HuarayCameraParamFileName(),
    ClassicalArmorRecognizerParamFileName(),
    ClassicalWindmillRecognizerParamFileName(),
    SolverParamFileName(),
    WorkMode(EWorkMode::Manual),
    BulletVelocity(EBulletVelocity::MPS_30),
    IgnoredArmorNumber(EClassicalArmorNumber::Invalid),
    ShootDelay(0),
    FightControlPeriod(10),
    BuffControlPeriod(500),
    CachedFightDataType(EFightDataType::None),
    CachedBuffDataType(EBuffDataType::None),
    ControlBodyCpuCore(-1),
    NotifyBodyCpuCore(-1),
    DetectArmorCpuCore(-1),
    FittingCpuCore(-1)
{
}