//
// Created by plutoli on 2022/2/5.
//

#ifndef CUBOT_BRAIN_CLASSICAL_WINDMILL_LOGO_H
#define CUBOT_BRAIN_CLASSICAL_WINDMILL_LOGO_H

#include <opencv2/opencv.hpp>
#include "Eigen/Eigen"

/**
 * @brief 风车Logo
 */
class ClassicalWindmillLogo
{
public:
    bool IsValid;                                   ///< 风车Logo是否合法
    std::vector<cv::Point> Contour;                 ///< 风车Logo在原始图像上的外部轮廓
    float ContourArea;                              ///< 风车Logo在原始图像上的外部轮廓面积
    cv::Point2f ContourCenter;                      ///< 风车Logo在原始图像上的外部轮廓中心点坐标
    cv::Rect MinEnclosedRect;                       ///< 风车Logo在原始图像上的最小正外接矩形
    cv::Mat Image;                                  ///< 风车Logo的图像(与Hog特征描述子窗体相同大小的灰度图)
    float AspectRatio;                              ///< 风车Logo的纵横比；纵横比=Logo高度/Logo宽度
    uint64_t Timestamp;                             ///< 风车Logo的时间戳
    std::vector<cv::Point2f> LogoBoundingPoints;    ///< 风车Logo最小正外界矩形的四点坐标
    Eigen::Vector3d WorldCoordinate;                ///< 风车Logo世界坐标
    Eigen::Vector3d CradleHeadCoordinate;           ///< 风车Logo云台坐标

    /**
    * @brief 构造函数
    */
    ClassicalWindmillLogo();

    /**
     * @brief 析构函数
     */
    ~ClassicalWindmillLogo() = default;
};

#endif //CUBOT_BRAIN_CLASSICAL_WINDMILL_LOGO_H