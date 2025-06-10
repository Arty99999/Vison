//
// Created by plutoli on 2022-06-12.
//

#include "large_buff_velocity_param.h"

// ******************************  VelocityPolarParam类的公有函数  ******************************

// 构造函数
LargeBuffVelocityParam::LargeBuffVelocityParam():
    PredictTime(500),
    HighSpeedThreshold(0.0),
    HighSpeedOffsetRadian(0.0),
    LowSpeedThreshold(0.0),
    LowSpeedOffsetRadian(0.0),
    SampleNumber(0)
{
}