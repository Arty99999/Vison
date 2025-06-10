//
// Created by plutoli on 2022/4/5.
//

#include "huaray_camera_status.h"

// ******************************  HuarayCameraStatus类的公有函数  ******************************

// 构造函数
HuarayCameraStatus::HuarayCameraStatus():
    Key(),
    IsConnected(false),
    IsWorking(false),
    ErrorFrame(0),
    LostPacketFrame(0),
    TotalFrame(0),
    BandWidth(0.0),
    FPS(0.0)
{
}