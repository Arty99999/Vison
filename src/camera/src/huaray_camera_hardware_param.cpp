//
// Created by plutoli on 2022/4/5.
//

#include "huaray_camera_hardware_param.h"

// ******************************  HuarayCameraHardwareParam类的公有函数  ******************************

// 构造函数
HuarayCameraHardwareParam::HuarayCameraHardwareParam():
    WorkModes(),
    IsBalanceWhiteAuto(true),
    IsExposureAuto(false),
    ExposureTime(1500.0),
    GainRaw(1.0),
    Gamma(0.8),
    Brightness(40),
    IsSelected(false)
{
}