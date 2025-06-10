//
// Created by plutoli on 2021/8/12.
//

#ifndef CUBOT_BRAIN_ROBOT_BRAIN_PARAM_H
#define CUBOT_BRAIN_ROBOT_BRAIN_PARAM_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "robot_brain_core_param.h"

/**
 * @brief 机器人大脑参数
 */
class RobotBrainParam
{
public:
    std::string Key;                                          ///< 机器人大脑的标识符
    int ScanCpuCore;                                          ///< 机器人大脑的自检任务CPU内核编号；默认为-1
    std::string SerialPortParamFileName;                      ///< 机器人大脑的串口参数配置文件名
    std::vector<RobotBrainCoreParam> RobotBrainCoreParams;    ///< 机器人大脑内核参数集合

    /**
    * @brief 构造函数
    */
    RobotBrainParam();

    /**
     * @brief 析构函数
     */
    ~RobotBrainParam() = default;

    /**
     * @brief 从yaml配置文件中加载机器人大脑参数
     * @param[in]  yamlFileName     机器人大脑参数配置文件名
     * @param[out] robotBrainParam  机器人大脑参数
     * @return 加载结果\n
     *         -<em>false</em> 加载失败\n
     *         -<em>true</em> 加载成功\n
     */
    static bool LoadFromYamlFile(const std::string &yamlFileName, RobotBrainParam *robotBrainParam);
};

#endif //CUBOT_BRAIN_ROBOT_BRAIN_PARAM_H