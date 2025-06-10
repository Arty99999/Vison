//
// Created by plutoli on 2022/4/5.
//

#ifndef CUBOT_BRAIN_HUARAY_CAMERA_HARDWARE_PARAM_H
#define CUBOT_BRAIN_HUARAY_CAMERA_HARDWARE_PARAM_H

#include <string>
#include <vector>
#include "system_configurator.h"

/**
 * @brief 华睿相机的硬件参数
 */
class HuarayCameraHardwareParam
{
public:
    std::vector<EWorkMode> WorkModes;   ///< 相机硬件参数适配的工作模式集合
    bool IsBalanceWhiteAuto;            ///< 相机是否自动白平衡
    bool IsExposureAuto;                ///< 相机是否自动曝光
    double ExposureTime;                ///< 相机的曝光时间
    double GainRaw;                     ///< 相机的增益值
    double Gamma;                       ///< 相机的伽马值
    int64_t Brightness;                 ///< 相机的亮度
    bool IsSelected;                    ///< 相机硬件参数的选择状态

    /**
     * @brief 构造函数
     */
    HuarayCameraHardwareParam();

    /**
     * @brief 析构函数
     */
    ~HuarayCameraHardwareParam() = default;
};

#endif //CUBOT_BRAIN_HUARAY_CAMERA_HARDWARE_PARAM_H