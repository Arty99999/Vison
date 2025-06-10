//
// Created by plutoli on 2021/8/13.
//

#ifndef CUBOT_BRAIN_ROBOT_FIGHT_DATA_H
#define CUBOT_BRAIN_ROBOT_FIGHT_DATA_H

#include <opencv2/opencv.hpp>

/**
 * @brief 机器人大脑内核的战斗数据类型
 */
enum class EFightDataType
{
    None = 0,                     ///< 无图像
    RawImage = 1,                 ///< 原始图像
    PolishedArmorsImage = 2,      ///< 精装甲板图像
    FilteredArmorsImage = 3,      ///< 过滤后的精装甲板图像
    EvaluatedArmorsImage = 4,     ///< 评估后的精装甲板图像
    LockedArmorImage = 5,        ///< 锁定装甲板图像
    ComparedArmorImage = 6,       ///< 比较装甲板图像
    PredictorArmorImage= 7
};

/**
 * @brief 机器人大脑内核的战斗数据
 */
class RobotFightData
{
public:
    EFightDataType Type;           ///< 战斗数据类型
    cv::Mat Image;                 ///< 战斗图像

    /**
    * @brief 构造函数
    */
    RobotFightData();

    /**
     * @brief 析构函数
     */
    ~RobotFightData() = default;

    /**
     * @brief 转换机器人大脑内核的战斗数据类型
     * @param[in] input     输入的机器人大脑内核战斗数据类型数值
     * @param[out] output   转换得到的机器人大脑内核战斗数据类型
     * @return 机器人大脑内核战斗数据类型转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 机器人大脑内核战斗数据类型的取值范围为[0,11]的整数，输入数据不在此范围内，则转换失败\n
     *         -<em>0</em> None\n
     *         -<em>1</em> RawImage\n
     *         -<em>2</em> PreprocessedImage\n
     *         -<em>3</em> RawLightBarsImage\n
     *         -<em>4</em> PolishedLightBarsImage\n
     *         -<em>5</em> LightBarPairsImage\n
     *         -<em>6</em> RawArmorsImage\n
     *         -<em>7</em> PolishedArmorsImage\n
     *         -<em>8</em> FilteredArmorsImage\n
     *         -<em>9</em> EvaluatedArmorsImage\n
     *         -<em>10</em> LockedArmorImage\n
     *         -<em>11</em> ComparedArmorImage\n
     */
    static bool ConvertToFightDataType(const int &input, EFightDataType *output);
};

#endif //CUBOT_BRAIN_ROBOT_FIGHT_DATA_H