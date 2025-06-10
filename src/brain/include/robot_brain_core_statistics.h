//
// Created by plutoli on 2022-06-11.
//

#ifndef CUBOT_BRAIN_ROBOT_BRAIN_CORE_STATISTICS_H
#define CUBOT_BRAIN_ROBOT_BRAIN_CORE_STATISTICS_H

#include <cstdint>

/**
 * @brief 机器人大脑内核的统计信息
 */
class RobotBrainCoreStatistics
{
public:
    uint64_t ReadCameraDataIndex;            ///< 机器人大脑内核读取的相机数据索引
    uint64_t ProcessedCameraDataIndex;       ///< 机器人大脑内核处理的相机数据索引
    uint64_t FightControlCommandIndex;       ///< 机器人大脑内核发布的战斗控制指令索引
    uint64_t BuffControlCommandIndex;        ///< 机器人大脑内核发布的Buff控制指令索引
    uint64_t ProcessTime;                    ///< 机器人大脑内核的当前相机数据处理时间；单位：纳秒
    uint64_t Fps;                            ///< 机器人大脑内核数据处理帧率

    /**
    * @brief 构造函数
    */
    RobotBrainCoreStatistics();

    /**
     * @brief 析构函数
     */
    ~RobotBrainCoreStatistics() = default;
};

#endif //CUBOT_BRAIN_ROBOT_BRAIN_CORE_STATISTICS_H
