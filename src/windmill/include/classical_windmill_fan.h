//
// Created by plutoli on 2022/1/23.
//

#ifndef CUBOT_BRAIN_CLASSICAL_WINDMILL_FAN_H
#define CUBOT_BRAIN_CLASSICAL_WINDMILL_FAN_H

#include <opencv2/opencv.hpp>
#include "eigen3/Eigen/Core"

/**
 * @brief 风车扇叶
 */
class ClassicalWindmillFan
{
public:
    bool IsValid;                                       ///< 风车扇叶是否合法
    std::vector<cv::Point> Contour;                     ///< 风车扇叶在原始图像上的外部轮廓
    cv::Point2f ContourCenter;                          ///< 风车扇叶在原始图像上的外部轮廓中心点坐标
    float ContourArea;                                  ///< 风车扇叶在原始图像上的外部轮廓面积；单位：像素
    cv::RotatedRect RotatedRect;                        ///< 风车扇叶在原始图像上的外部轮廓最小旋转外接矩形
    float AspectRatio;                                  ///< 风车扇叶在原始图像上的外部轮廓纵横比；纵横比=扇叶高度/扇叶宽度
    std::vector<cv::Point> ContourPoly;                 ///< 风车扇叶在原始图像上的角点轮廓
    float PolyNum;                                      ///< 风车扇叶在原始图像上的角点数目
    std::vector<cv::Point> targetContour;               ///< 风车扇叶在原始图像上的靶心轮廓
    cv::Point2f targetCenter;                           ///< 风车扇叶在原始图像上的靶心轮廓中心点坐标
    float targetContourArea;                            ///< 风车扇叶在原始图像上的靶心轮廓面积；单位：像素
    cv::Rect targetRect;                                ///< 风车扇叶在原始图像上的靶心轮廓最小正外接矩形
    std::vector<cv::Point2f> TargetBoundingPoints;      ///< 靶心最小正外接矩形的四点坐标
    Eigen::Vector3d EigenTargetCenter;                  ///< 风车扇叶在原始图像上的Eigen格式的靶心三维坐标
    double Angle;                                       ///< 风车扇叶矢量角
    float OffsetAngle;                                  ///< 风车扇叶与靶心的偏转角
    uint64_t Timestamp;                                 ///< 风车扇叶的时间戳

    /**
    * @brief 构造函数
    */
    ClassicalWindmillFan();

    /**
     * @brief 析构函数
     */
    ~ClassicalWindmillFan() = default;
};

#endif //CUBOT_BRAIN_CLASSICAL_WINDMILL_FAN_H