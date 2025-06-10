//
// Created by plutoli on 2022-06-12.
//

#ifndef CUBOT_BRAIN_LARGE_BUFF_VELOCITY_PARAM_H
#define CUBOT_BRAIN_LARGE_BUFF_VELOCITY_PARAM_H

/**
 * @brief 大符速度补偿参数
 */
class LargeBuffVelocityParam
{
public:
    int PredictTime;                ///< 大符速度补偿预测时间；单位：毫秒
    float HighSpeedThreshold;       ///< 大符高速阈值
    float HighSpeedOffsetRadian;    ///< 大符高速补偿弧度
    float LowSpeedThreshold;        ///< 大符低速阈值
    float LowSpeedOffsetRadian;     ///< 大符低速补偿弧度
    int SampleNumber;               ///< 大符速度补偿样本数量

    /**
    * @brief 构造函数
    */
    LargeBuffVelocityParam();

    /**
     * @brief 析构函数
     */
    ~LargeBuffVelocityParam() = default;
};

#endif //CUBOT_BRAIN_LARGE_BUFF_VELOCITY_PARAM_H