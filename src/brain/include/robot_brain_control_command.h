//
// Created by plutoli on 2022/1/30.
//

#ifndef CUBOT_BRAIN_ROBOT_BRAIN_CONTROL_COMMAND_H
#define CUBOT_BRAIN_ROBOT_BRAIN_CONTROL_COMMAND_H

#include <cmath>

/**
 * @brief 机器人大脑的控制指令
 */
class RobotBrainControlCommand
{
public:
    unsigned char ID;     ///< 机器人大脑内核ID
    float Yaw;            ///< 机器人本体Yaw轴的偏转角度
    float Yaw1;            ///< 机器人本体Yaw轴的偏转角度
    float Pitch;          ///< 机器人本体Pitch轴的偏转角度
    bool Top;       ///< 击打目标点到枪口的距离；单位：毫米
    float Distance;
    bool IsFire;          ///< 是否开火

    /**
    * @brief 构造函数
    */
    RobotBrainControlCommand();

    /**
     * @brief 析构函数
     */
    ~RobotBrainControlCommand() = default;

    /**
     * @brief 获取控制指令打包之后的数据帧字节长度
     * @return 控制指令打包之后的数据帧字节长度
     */
    static unsigned int GetFrameSize();

    /**
     * @brief 将控制指令封装成数据帧
     * @param[out] frame 存储数据帧的字节数组
     */
    void EncapsulateToFrame(unsigned char *frame) const;
};

#endif //CUBOT_BRAIN_ROBOT_BRAIN_CONTROL_COMMAND_H