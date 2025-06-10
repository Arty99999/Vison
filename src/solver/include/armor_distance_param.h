//
// Created by plutoli on 2022/1/28.
//

#ifndef CUBOT_BRAIN_ARMOR_DISTANCE_PARAM_H
#define CUBOT_BRAIN_ARMOR_DISTANCE_PARAM_H

#include <vector>
#include "system_configurator.h"
#include "armor_distance_offset.h"

/**
 * @brief 装甲板距离补偿参数
 */
class ArmorDistanceParam
{
public:
    std::vector<EWorkMode> WorkModes;            ///< 距离补偿参数适配的工作模式集合
    EBulletVelocity BulletVelocity;              ///< 距离补偿参数适配的子弹速度；单位：米/秒
    std::vector<ArmorDistanceOffset> Offsets;    ///< 距离分段补偿值集合

    /**
    * @brief 构造函数
    */
    ArmorDistanceParam();

    /**
     * @brief 析构函数
     */
    ~ArmorDistanceParam() = default;
};

#endif //CUBOT_BRAIN_ARMOR_DISTANCE_PARAM_H