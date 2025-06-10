//
// Created by plutoli on 2021/12/28.
//

#ifndef CUBOT_BRAIN_ROBOT_BRAIN_CORE_PARAM_H
#define CUBOT_BRAIN_ROBOT_BRAIN_CORE_PARAM_H

#include <string>
#include "system_configurator.h"
#include "robot_fight_data.h"
#include "robot_buff_data.h"
#include "classical_armor.h"

/**
 * @brief 机器人大脑内核参数
 */
class RobotBrainCoreParam
{
public:
    std::string Key;                                      ///< 机器人大脑内核的标识符
    unsigned char ID;                                     ///< 机器人大脑内核的编号
    std::string HuarayCameraParamFileName;                ///< 机器人大脑内核的华睿相机参数配置文件名
    std::string ClassicalArmorRecognizerParamFileName;    ///< 机器人大脑内核的经典装甲板识别器参数配置文件名
    std::string ClassicalWindmillRecognizerParamFileName; ///< 机器人大脑内核的经典风车识别器参数配置文件名
    std::string SolverParamFileName;                      ///< 机器人大脑内核的目标解算器参数配置文件名
    std::string PredictorParamFileName;                   ///< 机器人大脑内核的目标预测器参数配置文件名
    EWorkMode WorkMode;                                   ///< 机器人大脑内核的工作模式
    EBulletVelocity BulletVelocity;                       ///< 机器人大脑内核的子弹速度；单位：米/秒
    EClassicalArmorNumber IgnoredArmorNumber;             ///< 机器人大脑内核忽略的装甲板编号
    float ShootDelay;                                     ///< 机器人大脑内核的发射机构射击延迟时间；单位：毫秒
    unsigned int FightControlPeriod;                      ///< 机器人大脑内核的战斗模式控制周期；单位：毫秒
    unsigned int BuffControlPeriod;                       ///< 机器人大脑内核的Buff模式控制周期；单位：毫秒
    EFightDataType CachedFightDataType;                   ///< 机器人大脑内核缓存的战斗数据类型
    EBuffDataType CachedBuffDataType;                     ///< 机器人大脑内核缓存的Buff数据类型
    int ControlBodyCpuCore;                               ///< 机器人大脑内核执行机器人本体控制任务的CPU内核编号；默认值：-1
    int NotifyBodyCpuCore;                                ///< 机器人大脑内核执行机器人本体提示任务的CPU内核编号；默认值：-1
    int DetectArmorCpuCore;                               ///< 机器人大脑内核执行机器人本体提示任务的CPU内核编号；默认值：-1
    int FittingCpuCore;                                   ///< 机器人速度方程解算任务的CPU内核编号；默认值：-1

    /**
    * @brief 构造函数
    */
    RobotBrainCoreParam();

    /**
     * @brief 析构函数
     */
    ~RobotBrainCoreParam() = default;
};

#endif //CUBOT_BRAIN_ROBOT_BRAIN_CORE_PARAM_H