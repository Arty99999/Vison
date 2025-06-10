//
// Created by plutoli on 2022/2/14.
//

#include <opencv2/opencv.hpp>
#include <chrono>
#include "delta_cv.h"

int main(int argc, char *argv[])
{
    // 打开视频
    cv::VideoCapture capture("/home/plutoli/development/cubot_brain/data/video/2021-09-05/red3.avi");
    if (!capture.isOpened())
    {
        return -1;
    }

    // 读取并处理视频
    cv::Mat rawImage;
    while (capture.read(rawImage))
    {
        // 对原始图像进行快速BGR阈值分割
        BGRWeight bgrWeight;
        bgrWeight.Blue = 0.05;
        bgrWeight.Green = 0.05;
        bgrWeight.Red = 0.90;
        cv::Mat binaryImageBGR(rawImage.rows, rawImage.cols, CV_8UC1);
        DeltaCV::FastWeidhtedBinarize(rawImage,
                                      bgrWeight,
                                      120,
                                      &binaryImageBGR);

        // 将原始图像转换到HSV颜色空间
        cv::Mat hsvImage;
        cv::cvtColor(rawImage, hsvImage, cv::COLOR_BGR2HSV);

        // 对原始图像进行快速HSV阈值分割
        HSVThreshold hsvThreshold;
        hsvThreshold.HueLower = 0;
        hsvThreshold.HueUpper = 40;
        hsvThreshold.SaturationLower = 0;
        hsvThreshold.SaturationUpper = 255;
        hsvThreshold.ValueLower = 180;
        hsvThreshold.ValueUpper = 255;
        cv::Mat binaryImageHSV(rawImage.rows, rawImage.cols, CV_8UC1);
        DeltaCV::FastInRange(hsvImage, hsvThreshold, &binaryImageHSV);

        // 显示原始图像和灰度图像
        cv::imshow("rawImage", rawImage);
        cv::imshow("binaryImageBGR", binaryImageBGR);
        cv::imshow("binaryImageHSV", binaryImageHSV);
        cv::waitKey(0);
    }

    // 释放视频读取器
    capture.release();

    return 0;
}