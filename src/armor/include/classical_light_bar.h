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
    std::vector<cv::Point> Contour;   ///< 灯条在原始图像上的外部轮廓
    cv::RotatedRect MinRotatedRect;   ///< 灯条在原始图像上的最小旋转外接矩形
    cv::Rect MinEnclosedRect;         ///< 灯条在原始图像上的最小正外接矩形
    cv::Point2f LeftUpper;            ///< 灯条在原始图像上的左上角顶点坐标
    cv::Point2f LeftLower;            ///< 灯条在原始图像上的左下角顶点坐标
    cv::Point2f RightUpper;           ///< 灯条在原始图像上的右上角顶点坐标
    cv::Point2f RightLower;           ///< 灯条在原始图像上的右下角顶点坐标
    cv::Point2f Center;               ///< 灯条在原始图像上的最小旋转外接矩形中心点坐标
    float Width;                      ///< 灯条在原始图像上的宽度(最小旋转外接矩形短边的长度)
    float Height;                     ///< 灯条在原始图像上的高度(最小旋转外接矩形长边的长度)
    float Angle;                      ///< 灯条的最小旋转外接矩形长边与像素坐标系x轴正方向的夹角；取值范围：[0°, 180°)
    float Area;                       ///< 灯条的像素面积

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