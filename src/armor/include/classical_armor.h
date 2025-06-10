//
// Created by plutoli on 2021/8/2.
//

#ifndef CUBOT_BRAIN_CLASSICAL_ARMOR_H
#define CUBOT_BRAIN_CLASSICAL_ARMOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "classical_light_bar.h"

/**
 * @brief 装甲板编号
 */
enum class EClassicalArmorNumber
{
    Invalid = 0,    ///< 无效装甲板
    One = 1,        ///< 1号机器人装甲板
    Two = 2,        ///< 2号机器人装甲板
    Three = 3,      ///< 3号机器人装甲板
    Four = 4,       ///< 4号机器人装甲板
    Five = 5,       ///< 5号机器人装甲板
    Sentry = 6,     ///< 哨兵机器人装甲板
    Outpost = 7,    ///< 前哨站机器人装甲板
    Base = 8        ///< 基地机器人装甲板
};

/**
 * @brief 装甲板
 */
class ClassicalArmor
{
public:
    ClassicalLightBar LeftLightBar;           ///< 装甲板的左侧灯条
    ClassicalLightBar RightLightBar;          ///< 装甲板的右侧灯条
    EClassicalArmorNumber Number;             ///< 装甲板的编号
    float Area;                               ///< 装甲板的面积；单位：像素
    float Offset;                             ///< 装甲板的图像中心偏转距离；单位：像素
    cv::Point2f LeftUpper;                    ///< 装甲板的左上角顶点坐标
    cv::Point2f LeftLower;                    ///< 装甲板的左下角顶点坐标
    cv::Point2f RightUpper;                   ///< 装甲板的右上角顶点坐标
    cv::Point2f RightLower;                   ///< 装甲板的右下角顶点坐标
    cv::Point2f Center;                       ///< 装甲板的中心点坐标
    cv::Mat Image;                            ///< 装甲板的轮廓区域经过透视变换之后的图像
    uint64_t Timestamp;                       ///< 装甲板的时间戳
    int type;
    /**
    * @brief 构造函数
    */
    ClassicalArmor();

    /**
     * @brief 析构函数
     */
    ~ClassicalArmor() = default;

    /**
     * @brief 转换装甲板编号
     * @param[in]   input   输入的装甲板编号数值
     * @param[out]  output  转换得到的装甲板编号
     * @return 装甲板编号转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 装甲板编号的取值范围为0/1/2/3/4/5/6/7/8，输入数据不在此范围内，则转换失败，返回Invalid\n
     *         -<em>0</em> 无效装甲板\n
     *         -<em>1</em> 1号机器人装甲板\n
     *         -<em>2</em> 2号机器人装甲板\n
     *         -<em>3</em> 3号机器人装甲板\n
     *         -<em>4</em> 4号机器人装甲板\n
     *         -<em>5</em> 5号机器人装甲板\n
     *         -<em>6</em> 哨兵机器人装甲板\n
     *         -<em>7</em> 前哨站机器人装甲板\n
     *         -<em>8</em> 基地机器人装甲板\n
     */
    static bool ConvertToClassicalArmorNumber(const int &input, EClassicalArmorNumber *output);
};

#endif //CUBOT_BRAIN_CLASSICAL_ARMOR_H