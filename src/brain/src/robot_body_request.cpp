//
// Created by plutoli on 2022/2/25.
//

#include "robot_body_request.h"

// ******************************  RobotBodyRequest类的公有函数  ******************************

// 构造函数
RobotBodyRequest::RobotBodyRequest():
    Type(ERequestType::HeartBeat),
    Datas()
{
}