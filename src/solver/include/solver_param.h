//
// Created by plutoli on 2021/8/13.
//

#ifndef CUBOT_BRAIN_SOLVER_PARAM_H
#define CUBOT_BRAIN_SOLVER_PARAM_H

#include <string>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "easy_logger.h"
#include "armor_distance_param.h"
#include "armor_velocity_param.h"
#include "buff_distance_param.h"
#include "small_buff_velocity_param.h"
#include "large_buff_velocity_param.h"
#include "angle_distance_offset_param.h"

/**
 * @brief 目标解算器参数
 */
class SolverParam
{
public:
    std::string Key;                                            ///< 目标解算器的标识符
    std::vector<ArmorDistanceParam> ArmorDistanceParameters;    ///< 目标解算器的装甲板距离补偿参数集合
    std::vector<ArmorVelocityParam> ArmorVelocityParameters;    ///< 目标解算器的装甲板速度补偿参数集合
    BuffDistanceParam BuffDistanceParameter;                    ///< 目标解算器的Buff距离补偿参数
    SmallBuffVelocityParam SmallBuffVelocityParameter;          ///< 目标解算器的小符速度补偿参数
    LargeBuffVelocityParam LargeBuffVelocityParameter;          ///< 目标解算器的大符速度补偿参数
    AngleDistanceOffset    AngleDistanceOffsetParameter;        ///< 目标结算器的角度结算补偿参数
    /**
    * @brief 构造函数
    */
    SolverParam();

    /**
     * @brief 析构函数
     */
    ~SolverParam() = default;

    /**
     * @brief 从yaml配置文件中加载目标解算器参数
     * @param[in]  yamlFileName     目标解算器参数配置文件名
     * @param[out] solverParam      目标解算器参数
     * @return 加载结果\n
     *         -<em>false</em> 加载失败\n
     *         -<em>true</em> 加载成功\n
     */
    static bool LoadFromYamlFile(const std::string &yamlFileName, SolverParam *solverParam);
};

#endif //CUBOT_BRAIN_SOLVER_PARAM_H