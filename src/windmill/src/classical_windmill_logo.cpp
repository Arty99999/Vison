//
// Created by plutoli on 2022/2/5.
//

#include "classical_windmill_logo.h"

// ******************************  ClassicalWindmillLogo类的公有函数  ******************************

// 构造函数
ClassicalWindmillLogo::ClassicalWindmillLogo():
    IsValid(false),
    Contour(),
    ContourArea(0.0),
    ContourCenter(0.0, 0.0),
    MinEnclosedRect(),
    Image(),
    AspectRatio(0.0),
    Timestamp(0)
{
}