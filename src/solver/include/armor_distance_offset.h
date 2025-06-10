//
// Created by plutoli on 2022/1/17.
//

#ifndef CUBOT_BRAIN_ARMOR_DISTANCE_OFFSET_H
#define CUBOT_BRAIN_ARMOR_DISTANCE_OFFSET_H

/**
 * @brief 装甲板距离分段补偿值
 */
class ArmorDistanceOffset
{
public:
    float Lower;        ///< 距离分段补偿值的作用域下限
    float Upper;        ///< 距离分段补偿值的作用域上限
    float Offset_x;     ///< 距离分段补偿值的x轴偏移
    float Offset_y;     ///< 距离分段补偿值的y轴偏移

    /**
     * @brief 构造函数
     */
    ArmorDistanceOffset();

    /**
     * @brief 默认析构函数
     */
    ~ArmorDistanceOffset() = default;
};

#endif //CUBOT_BRAIN_ARMOR_DISTANCE_OFFSET_H