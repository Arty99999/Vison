//
// Created by tony on 2022/4/23.
//

#ifndef CUBOT_BRAIN_PREDICTOR_PARAM_H
#define CUBOT_BRAIN_PREDICTOR_PARAM_H

#include "predictor_tool.h"
#include "easy_logger.h"

class PredictorParam{
public:

    /**
     * @brief 构造函数
     */
    PredictorParam();

    /**
     * @brief 析构函数
     */
    ~PredictorParam()=default;

    /**
     * @brief 从yaml配置文件中加载预测器参数
     * @param[in]  yamlFileName     预测器参数配置文件名称
     * @param[out] dahuaCameraParam 预测器参数
     * @return 预测器参数加载结果\n
     *         -<em>false</em> 加载失败\n
     *         -<em>true</em> 加载成功\n
     */
    static bool LoadFromYamlFile(const std::string &yamlFileName, PredictorParam *predictParam);

    std::string Key;                   ///< 预测参数标识符
    int GapIndex;                      ///< 间隔预测数据帧
    Eigen::Matrix<double, 9, 9> Q;     ///< 预测过程协方差
    Eigen::Matrix<double, 4, 4> R;     ///< 观测过程协方差
    double Q00;                        ///< 预测过程协方差的对角线的数值
    double Q11;
    double Q22;
    double Q33;
    double Q44;
    double Q55;
    double Q66;
    double Q77;
    double Q88;
    double R00;                        ///< 观测过程协方差的对角线的数值
    double R11;
    double R22;
    double R33;
    double sigma_X;
    double sigma_Y;
    double sigma_R;
    double sigma_W;
};
#endif //CUBOT_BRAIN_PREDICTOR_PARAM_H
