//
// Created by plutoli on 2021/8/9.
//

#include <chrono>
#include "delta_cv.h"

int main()
{
    // 读取原始图像
    cv::Mat rawImage = imread("/home/plutoli/development/cubot_brain/data/picture/002_tree_1920x1080.jpg",
                              cv::IMREAD_COLOR);
    if (rawImage.empty())
    {
        std::cout << "Error. Could not read picture." << std::endl;
        return -1;
    }

    // 初始化加权阈值分割参数
    BGRWeight weights;
    weights.Blue = 0.9;
    weights.Green = 0.05;
    weights.Red = 0.05;
    cv::Mat weightedBinarizeImage(rawImage.rows, rawImage.cols, CV_8UC1);
    cv::Mat fastWeightedBinarizeImage(rawImage.rows, rawImage.cols, CV_8UC1);

    // 计算DeltaCV::WeightedBinarize()函数的运算时间
    std::chrono::time_point<std::chrono::steady_clock> weightedBinarizeBeginTime = std::chrono::steady_clock::now();
    uint64_t weightedBinarizeBeginTimestamp = weightedBinarizeBeginTime.time_since_epoch().count();
    for (unsigned int i = 0; i< 1000; ++i)
    {
        DeltaCV::WeightedBinarize(rawImage, weights, 120, &weightedBinarizeImage);
    }
    std::chrono::time_point<std::chrono::steady_clock> weightedBinarizeEndTime = std::chrono::steady_clock::now();
    uint64_t weightedBinarizeEndTimestamp = weightedBinarizeEndTime.time_since_epoch().count();
    uint64_t weightedBinarizeInterval = (weightedBinarizeEndTimestamp - weightedBinarizeBeginTimestamp) / 1000;
    std::cout << "WeightedBinarize(): " << weightedBinarizeInterval / 1000 << " us" << std::endl;

    // 计算DeltaCV::FastWeidhtedBinarize()函数的运算时间
    std::chrono::time_point<std::chrono::steady_clock> fastWeightedBinarizeBeginTime = std::chrono::steady_clock::now();
    uint64_t fastWeightedBinarizeBeginTimestamp = fastWeightedBinarizeBeginTime.time_since_epoch().count();
    for (unsigned int i = 0; i< 1000; ++i)
    {
        DeltaCV::FastWeidhtedBinarize(rawImage, weights, 120, &fastWeightedBinarizeImage);
    }
    std::chrono::time_point<std::chrono::steady_clock> fastWeightedBinarizeEndTime = std::chrono::steady_clock::now();
    uint64_t fastWeightedBinarizeEndTimestamp = fastWeightedBinarizeEndTime.time_since_epoch().count();
    uint64_t fastWeightedBinarizeInterval = (fastWeightedBinarizeEndTimestamp - fastWeightedBinarizeBeginTimestamp) / 1000;
    std::cout << "FastWeightedBinarize(): " << fastWeightedBinarizeInterval / 1000 << " us" << std::endl;

    // 初始化HSV分割参数
    HSVThreshold hsvThreshold;
    hsvThreshold.HueLower = 0;
    hsvThreshold.HueUpper = 10;
    hsvThreshold.SaturationLower = 43;
    hsvThreshold.SaturationUpper = 255;
    hsvThreshold.ValueLower = 46;
    hsvThreshold.ValueUpper = 255;
    cv::Mat inRangeImage(rawImage.rows, rawImage.cols, CV_8UC1);
    cv::Mat fastInRangeImage(rawImage.rows, rawImage.cols, CV_8UC1);

    // 计算DeltaCV::InRange()函数的运算时间
    std::chrono::time_point<std::chrono::steady_clock> inRangeBeginTime = std::chrono::steady_clock::now();
    uint64_t inRangeBeginTimestamp = inRangeBeginTime.time_since_epoch().count();
    for (unsigned int i = 0; i < 1000; i++)
    {
        DeltaCV::InRange(rawImage, hsvThreshold, &inRangeImage);
    }
    std::chrono::time_point<std::chrono::steady_clock> inRangeEndTime = std::chrono::steady_clock::now();
    uint64_t inRangeEndTimestamp = inRangeEndTime.time_since_epoch().count();
    uint64_t inRangeInterval = (inRangeEndTimestamp - inRangeBeginTimestamp) / 1000;
    std::cout << "InRange(): " << inRangeInterval / 1000 << " us" << std::endl;

    // 计算DeltaCV::FastInRange()函数的运算时间
    std::chrono::time_point<std::chrono::steady_clock> fastInRangeBeginTime = std::chrono::steady_clock::now();
    uint64_t fastInRangeBeginTimestamp = fastInRangeBeginTime.time_since_epoch().count();
    for (unsigned int i = 0; i < 1000; i++)
    {
        DeltaCV::FastInRange(rawImage, hsvThreshold, &fastInRangeImage);
    }
    std::chrono::time_point<std::chrono::steady_clock> fastInRangeEndTime = std::chrono::steady_clock::now();
    uint64_t fastInRangeEndTimestamp = fastInRangeEndTime.time_since_epoch().count();
    uint64_t fastInRangeInterval = (fastInRangeEndTimestamp - fastInRangeBeginTimestamp) / 1000;
    std::cout << "FastInRange(): " << fastInRangeInterval / 1000 << " us" << std::endl;

    // 显示图像
    cv::imshow("rawImage", rawImage);
    cv::imshow("weightedBinarizeImage", weightedBinarizeImage);
    cv::imshow("fastWeightedBinarizeImage", fastWeightedBinarizeImage);
    cv::imshow("inRangeImage", inRangeImage);
    cv::imshow("fastInRangeImage", fastInRangeImage);
    cv::waitKey(0);

    return 0;
}