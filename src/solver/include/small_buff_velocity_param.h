//
// Created by plutoli on 2022/2/6.
//

#ifndef CUBOT_BRAIN_SMALL_BUFF_VELOCITY_PARAM_H
#define CUBOT_BRAIN_SMALL_BUFF_VELOCITY_PARAM_H

/**
 * @brief 小符速度补偿参数
 */
class SmallBuffVelocityParam
{
public:
    int PredictTime;        ///< 小符速度补偿预测时间；单位：毫秒
    float OffsetRadian;     ///< 小符速度补偿弧度
    int SampleNumber;       ///< 小符速度补偿样本数量

    /**
    * @brief 构造函数
    */
    SmallBuffVelocityParam();

    /**
     * @brief 析构函数
     */
    ~SmallBuffVelocityParam() = default;
};

#endif //CUBOT_BRAIN_SMALL_BUFF_VELOCITY_PARAM_H