//
// Created by xcx on 22-11-21.
//

#ifndef CUBOT_BRAIN_ANGLE_DISTANCE_OFFSET_PARAM_H
#define CUBOT_BRAIN_ANGLE_DISTANCE_OFFSET_PARAM_H
#include <vector>
/**
 *  自动根据距离进行角度补偿的参数
 */
 class AngleDistanceOffset{
 public:

     int max_iter;                     ///< 使用迭代法求解pitch补偿的最大迭代次数
     int R_K_iter;                     ///< 龙格库塔法求解落点的迭代次数
     float stop_error;                 ///< 停止迭代的最小误差(单位m)
     float k;                          ///< 弹丸飞行的空气摩擦力
     float g;                          ///< 重力加速度
     float bullet_speed;               ///< 弹丸飞行速度(单位m/s) 其实在其他部分有读取方法

     /**
     * @brief 构造函数
     */
     AngleDistanceOffset();

     /**
      * @brief 析构函数
      */
     ~AngleDistanceOffset() = default;
 };
#endif //CUBOT_BRAIN_ANGLE_DISTANCE_OFFSET_PARAM_H
