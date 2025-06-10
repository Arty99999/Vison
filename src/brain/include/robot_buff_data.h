//
// Created by plutoli on 2021/8/13.
//

#ifndef CUBOT_BRAIN_ROBOT_BUFF_DATA_H
#define CUBOT_BRAIN_ROBOT_BUFF_DATA_H

#include <opencv2/opencv.hpp>

/**
 * @brief 机器人大脑内核的Buff数据类型
 */
enum class EBuffDataType
{
    None = 0,                     ///< 无图像
    RawImage = 1,                 ///< 原始图像
    PreprocessedImage = 2,        ///< 预处理图像
    WindmillContoursImage = 3,    ///< 风车轮廓图像
    WindmillLogosImage = 4,       ///< 风车Logo图像
    WindmillFansImage = 5,        ///< 风车扇叶图像
    WindmillObjectsImage = 6,     ///< 风车目标图像
    WindmillAnglesImage = 7,      ///< 风车角度图像
    LockedFanImage = 8,           ///< 锁定扇叶图像
    ComparedFanImage = 9          ///< 比较扇叶图像
};

/**
 * @brief 机器人大脑内核的Buff数据
 */
class RobotBuffData
{
public:
    EBuffDataType Type;            ///< Buff数据类型
    cv::Mat Image;                 ///< Buff图像

    /**
    * @brief 构造函数
    */
    RobotBuffData();

    /**
     * @brief 析构函数
     */
    ~RobotBuffData() = default;

    /**
     * @brief 转换机器人大脑内核的Buff数据类型
     * @param[in] input     输入的机器人大脑内核Buff数据类型数值
     * @param[out] output   转换得到的机器人大脑内核Buff数据类型
     * @return 机器人大脑内核Buff数据类型转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 机器人大脑内核Buff数据类型的取值范围为[0,9]的整数，输入数据不在此范围内，则转换失败\n
     *         -<em>0</em> None\n
     *         -<em>1</em> RawImage\n
     *         -<em>2</em> PreprocessedImage\n
     *         -<em>3</em> WindmillContoursImage\n
     *         -<em>4</em> WindmillLogosImage\n
     *         -<em>5</em> WindmillFansImage\n
     *         -<em>6</em> WindmillObjectsImage\n
     *         -<em>7</em> WindmillAnglesImage\n
     *         -<em>8</em> LockedFanImage\n
     *         -<em>9</em> ComparedFanImage\n
     */
    static bool ConvertToBuffDataType(const int &input, EBuffDataType *output);
};

#endif //CUBOT_BRAIN_ROBOT_BUFF_DATA_H