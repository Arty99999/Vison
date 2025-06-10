//
// Created by plutoli on 2022/1/30.
//

#ifndef CUBOT_BRAIN_ROBOT_BRAIN_NOTIFY_COMMAND_H
#define CUBOT_BRAIN_ROBOT_BRAIN_NOTIFY_COMMAND_H

#include "system_configurator.h"

/**
 * @brief 机器人大脑向机器人本体发布的提示指令
 */
class RobotBrainNotifyCommand
{
public:
    unsigned char ID;                  ///< 机器人大脑内核ID
    EWorkMode WorkMode;                ///< 机器人大脑内核的工作模式
    EBulletVelocity BulletVelocity;    ///< 机器人大脑内核的目标解算器补偿参考的子弹速度
    bool IsInitialized;                ///< 机器人大脑内核的初始化状态
    bool IsOpened;                     ///< 机器人大脑内核的打开状态
    bool IsCameraConnected;            ///< 机器人大脑内核的相机连接状态
    bool IsCameraWorking;              ///< 机器人大脑内核的相机工作状态

    /**
    * @brief 构造函数
    */
    RobotBrainNotifyCommand();

    /**
     * @brief 析构函数
     */
    ~RobotBrainNotifyCommand() = default;

    /**
     * @brief 获取提示指令打包之后的数据帧字节长度
     * @return 提示指令打包之后的数据帧字节长度
     */
    static unsigned int GetFrameSize();

    /**
     * @brief 将提示指令封装成数据帧
     * @param[out] frame 存储数据帧的字节数组
     */
    void EncapsulateToFrame(unsigned char *frame) const;
};

#endif //CUBOT_BRAIN_ROBOT_BRAIN_NOTIFY_COMMAND_H