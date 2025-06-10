//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_BRAIN_HUARAY_CAMERA_STATUS_H
#define CUBOT_BRAIN_HUARAY_CAMERA_STATUS_H

#include <string>

/**
 * @brief 华睿相机状态帧
 * @note 封装了相机的标识符、连接状态、工作状态和数据统计信息
 */
class HuarayCameraStatus
{
public:
    std::string Key;                ///< 华睿相机的标识符
    bool IsConnected;               ///< 华睿相机的连接状态
    bool IsWorking;                 ///< 华睿相机的工作状态
    unsigned int ErrorFrame;        ///< 华睿相机的错误帧数统计
    unsigned int LostPacketFrame;   ///< 华睿相机的丢包帧数统计
    unsigned int TotalFrame;        ///< 华睿相机的总帧数统计
    double BandWidth;               ///< 华睿相机的带宽统计
    double FPS;                     ///< 华睿相机的帧率统计

    /**
     * @brief 构造函数
     */
    HuarayCameraStatus();

    /**
     * @brief 析构函数
     */
    ~HuarayCameraStatus() = default;
};


#endif //CUBOT_BRAIN_HUARAY_CAMERA_STATUS_H
