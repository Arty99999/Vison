//
// Created by plutoli on 2021/8/2.
//

#ifndef CUBOT_BRAIN_CLASSICAL_LIGHT_BAR_H
#define CUBOT_BRAIN_CLASSICAL_LIGHT_BAR_H

#include <vector>
#include <opencv2/opencv.hpp>

/**
 * @brief 灯条颜色
 */
enum class EClassicalLightBarColor
{
    Red = 1,        ///< 红色灯条
    Blue = 2        ///< 蓝色灯条
};

/**
 * @brief 装甲板两侧的灯条
 * @note 关于cv::RotatedRect中的width/height/angle的说明参见如下网址：\n
 *       https://blog.csdn.net/mailzst1/article/details/83141632 \n
 *       https://www.cnblogs.com/panxiaochun/p/5478555.html \n
 */
class ClassicalLightBar
{
public:
    /**
    * @brief 构造函数
    */
    ClassicalLightBar();

    /**
     * @brief 析构函数
     */
    ~ClassicalLightBar() = default;

    /**
     * @brief 转换灯条颜色
     * @param[in]   input   输入的灯条颜色数值
     * @param[out]  output  转换得到的灯条颜色
     * @return 灯条颜色转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 灯条颜色的取值为1/2，输入数据不在此范围内，则转换失败\n
     *         -<em>1</em> 红色灯条\n
     *         -<em>2</em> 蓝色灯条\n
     */
    static bool ConvertToClassicalLightBarColor(const int &input, EClassicalLightBarColor *output);
};

#endif //CUBOT_BRAIN_CLASSICAL_LIGHT_BAR_H