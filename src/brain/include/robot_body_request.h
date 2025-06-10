//
// Created by plutoli on 2022/2/25.
//

#ifndef CUBOT_BRAIN_ROBOT_BODY_REQUEST_H
#define CUBOT_BRAIN_ROBOT_BODY_REQUEST_H

#include <vector>
#include "system_configurator.h"

/**
 * @brief 机器人本体的请求
 */
class RobotBodyRequest
{
public:
    ERequestType Type;                  ///< 机器人本体请求的类型
    std::vector<unsigned char> Datas;   ///< 机器人本体请求的数据

    /**
    * @brief 构造函数
    */
    RobotBodyRequest();

    /**
     * @brief 析构函数
     */
    ~RobotBodyRequest() = default;
};

#endif //CUBOT_BRAIN_ROBOT_BODY_REQUEST_H