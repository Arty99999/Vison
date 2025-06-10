//
// Created by plutoli on 2022-06-12.
//

#ifndef CUBOT_BRAIN_BUFF_DISTANCE_PARAM_H
#define CUBOT_BRAIN_BUFF_DISTANCE_PARAM_H

#include <vector>
#include "buff_distance_offset.h"

/**
 * @brief Buff距离补偿参数
 */
class BuffDistanceParam
{
public:
    std::vector<BuffDistanceOffset> Offsets;  ///< 距离分段补偿值集合

    /**
    * @brief 构造函数
    */
    BuffDistanceParam();

    /**
     * @brief 析构函数
     */
    ~BuffDistanceParam() = default;
};

#endif //CUBOT_BRAIN_BUFF_DISTANCE_PARAM_H