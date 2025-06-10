//
// Created by plutoli on 2022/1/23.
//

#include "classical_windmill_fan.h"

// ******************************  ClassicalWindmillFan类的公有函数  ******************************

// 构造函数
ClassicalWindmillFan::ClassicalWindmillFan():
        IsValid(false),
        Contour(),
        ContourCenter(0.0, 0.0),
        ContourArea(0.0),
        RotatedRect(),
        AspectRatio(0.0),
        PolyNum(0.0),
        ContourPoly(),
        targetContour(),
        targetCenter(0.0, 0.0),
        targetContourArea(0.0),
        targetRect(),
        TargetBoundingPoints(),
        EigenTargetCenter(),
        Angle(0.0),
        OffsetAngle(0.0),
        Timestamp(0)
{
}