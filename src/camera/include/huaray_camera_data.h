//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_BRAIN_HUARAY_CAMERA_DATA_H
#define CUBOT_BRAIN_HUARAY_CAMERA_DATA_H

#include <opencv2/opencv.hpp>
#include <chrono>

/**
 * @brief 华睿相机数据帧
 * @note 基于cv::Mat封装的华睿相机数据帧
 */
class HuarayCameraData
{
public:
    /**
     * @brief 华睿相机数据帧索引
     */
    uint64_t Index;

    /**
     * @brief 华睿相机数据帧的时间戳；单位：纳秒；换算关系：1秒(s)=1000毫秒(ms)=1000,000微秒(us)=1000,000,000纳秒(ns)\n
     * @code
     *  std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
     *  uint64_t timestamp = now.time_since_epoch().count();
     * @endcode
     * @note 1-如果相机数据来源为硬件设备，时间戳从硬件设备中读取
     *       2-如果相机数据来源为离线视频，时间戳从计算机中读取
     */
    uint64_t Timestamp;

    /**
     * @brief 华睿相机数据帧图像
     */
    cv::Mat Image;

    /**
     * @brief 构造函数
     */
    HuarayCameraData();

    /**
     * @brief 析构函数
     */
    ~HuarayCameraData() = default;
};

#endif //CUBOT_BRAIN_HUARAY_CAMERA_DATA_H