//
// Created by plutoli on 2022/1/28.
//

#include "armor_velocity_param.h"

// ******************************  ArmorVelocityParam类的公有函数  ******************************

// 构造函数
ArmorVelocityParam::ArmorVelocityParam():
    WorkModes(),
    Order(EPolynomialOrder::One),
    MaxIterationNumber(10),
    SampleNumber(5)
{
}