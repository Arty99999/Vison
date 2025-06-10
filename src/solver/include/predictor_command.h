//
// Created by cubot on 2022/5/2.
//

#ifndef CUBOT_BRAIN_BRAIN_PREDICTOR_COMMAND_H
#define CUBOT_BRAIN_BRAIN_PREDICTOR_COMMAND_H

#include <array>
#include <cstring>
#include <chrono>
#include "iostream"
#include <vector>
#include <shared_mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
/**
 * command部分存放预防需要数据
 */
class PredictorCommand{

public:
    unsigned char ID;                                   ///< 机器人本体对应的机器人大脑内核ID
    int16_t index;                                      ///< 串口发来的的本周期发送四元数的序号
    std::array<float, 4> Q;                             ///< 串口发来的的四元数
    uint64_t TimeStamp;                                 ///< 串口发来的当前时间戳
    uint64_t nowTime;                                   ///< 系统内部的的时间戳
    bool IsBlue;
    Eigen::Matrix3d Rotation;
    int stand;                                          ///< fight linear action or top

    /**
     * @brief 构造函数
     */
    PredictorCommand();

    /**
     * @brief 析构函数
     */
    ~PredictorCommand()=default;

    static bool Parse(Eigen::Quaterniond quaterniond, PredictorCommand *predictorCommand);
    /**
     * @brief 根据输入的数据帧解析机器人预测命令
     * @param[in] frameSize 数据帧字节长度
     * @param[in] frame     数据帧
     * @param[out] request  机器人预测数据
     * @return 机器人本体的预测数据析结果\n
     *         -<em>false</em> 解析失败\n
     *         -<em>true</em> 解析成功\n
     */
    static bool Parse(unsigned char *robotBodyRequest, PredictorCommand *predictorCommand);
};
#endif //CUBOT_BRAIN_BRAIN_PREDICTOR_COMMAND_H
