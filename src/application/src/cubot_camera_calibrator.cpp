//
// Created by plutoli on 2021/9/26.
//

#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include "easy_logger.h"

/**
 * @brief 相机标定工具
 * @note 参考网址：https://blog.csdn.net/qq_37791134/article/details/80942171 \n
 *               https://www.cnblogs.com/haoxing990/p/4588566.html \n
 *               https://www.cnblogs.com/li-yao7758258/p/5929145.html \n
 *               http://www.elecfans.com/emb/app/20171204593598.html \n
 */
int main()
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 初始化日志信息
    std::string log;

    // 根据标定板上每行每列的角点个数，初始化角点检测的patternSize
    cv::Size patternSize;
    patternSize.width = 6;      // 标定板每行角点的个数
    patternSize.height = 9;     // 标定板每列角点的个数

    // 初始化标定板棋盘格角点的世界坐标
    float squareWidth = 27.5;   // 棋盘格宽度，单位：毫米
    float squareHeight = 27.5;  // 棋盘格高度，单位：毫米
    std::vector<cv::Point3f> worldCorners;
    for (int i = 0; i < patternSize.height; ++i)
    {
        for (int j = 0; j < patternSize.width; ++j)
        {
            // 假设标定板在世界坐标系中的z=0平面上，根据棋盘格的物理大小，计算每个角点的世界坐标
            cv::Point3f worldCorner;
            worldCorner.x = static_cast<float>(j) * squareWidth;
            worldCorner.y = static_cast<float>(i) * squareHeight;
            worldCorner.z = 0;
            worldCorners.push_back(worldCorner);
        }
    }

    // 初始化标定图像的宽度和高度
    cv::Size imageSize;
    imageSize.width = 1280;
    imageSize.height = 1024;

    // 加载相机的标定图像的绝对名称
    std::vector<cv::String> imageNameVector;
    cv::String imagePath = "/home/cubot/MVviewer/pictures/*.bmp";
    cv::glob(imagePath, imageNameVector, false);

    // 根据标定图像的绝对名称，寻找并保存每张图片的角点信息
    std::vector<std::vector<cv::Point2f>> pixelCornersVector;
    std::vector<std::vector<cv::Point3f>> worldCornersVector;
    for (int i = 0; i < imageNameVector.size(); ++i)
    {
        // 读取标定图像，判断标定图像的尺寸是否合法
        cv::Mat rawImage = cv::imread(imageNameVector[i]);
        if ((rawImage.cols != imageSize.width) || (rawImage.rows != imageSize.height))
        {
            log = "Image's size is invalid. Name: " + static_cast<std::string>(imageNameVector[i]);
            logger.Save(ELogType::Error, log);
            continue;
        }

        // 检测标定图像中的角点
        std::vector<cv::Point2f> pixelCorners;
        if (cv::findChessboardCorners(rawImage, patternSize, pixelCorners))
        {
            // 判断角点个数是否合法
            if (pixelCorners.size() != patternSize.width * patternSize.height)
            {
                log = "Corners' number is invalid. Name: " + static_cast<std::string>(imageNameVector[i]);
                logger.Save(ELogType::Error, log);
                continue;
            }
        }
        else
        {
            log = "Corners can't be found. Name: " + static_cast<std::string>(imageNameVector[i]);
            logger.Save(ELogType::Error, log);
            continue;
        }

        // 对角点进行亚像素处理
        cv::Mat grayScaleImage;
        cv::cvtColor(rawImage, grayScaleImage, CV_RGB2GRAY);
        if (!cv::find4QuadCornerSubpix(grayScaleImage, pixelCorners, cv::Size(5,5)))
        {
            log = "Corners were polished failure. Name: " + static_cast<std::string>(imageNameVector[i]);
            logger.Save(ELogType::Error, log);
            continue;
        }

        // 记录日志信息
        log = "Image was processed successful. Name: " + static_cast<std::string>(imageNameVector[i]);
        logger.Save(ELogType::Info, log);

        // 保存角点
        pixelCornersVector.emplace_back(pixelCorners);
        worldCornersVector.emplace_back(worldCorners);

        // 绘制角点并显示结果
        cv::drawChessboardCorners(rawImage, patternSize, pixelCorners,true);
        cv::imshow("CornerImage",rawImage);
        cv::waitKey(10);
    }

    // 初始化相机内参数矩阵、畸变系数、平移向量和旋转向量
    cv::Mat cameraMatrix = cv::Mat(3,3,CV_32FC1,cv::Scalar::all(0));
    cv::Mat distCoeffs = cv::Mat(1,5,CV_32FC1,cv::Scalar::all(0));
    std::vector<cv::Mat> tvecs;
    std::vector<cv::Mat> rvecs;

    // 相机标定
    cv::calibrateCamera(worldCornersVector,
                        pixelCornersVector,
                        imageSize,
                        cameraMatrix,
                        distCoeffs,
                        rvecs,
                        tvecs,
                        0);

    // 记录日志信息
    log = "Camera was calibrated successful";
    logger.Save(ELogType::Info, log);

    // 显示内参矩阵
    std::cout << std::endl << "Internal matrix:" << std::endl;
    std::cout.setf(std::ios::showpoint);    // 设置小数点后自动补零
    for (int i = 0; i < cameraMatrix.rows; ++i)
    {
        std::cout << "    ";
        for (int j = 0; j < cameraMatrix.cols; ++j)
        {
            // 必须使用cameraMatrix.at<double>，不能使用cameraMatrix.at<float>
            // 否则输出结果不正确
            std::cout << std::setprecision(10) << cameraMatrix.at<double>(i, j) << "    ";
        }
        std::cout << std::endl;
    }

    // 显示畸变向量
    std::cout << std::endl << "Distortion vector:" << std::endl;
    std::cout.setf(std::ios::showpoint);    // 设置小数点后自动补零
    std::cout << "    ";
    for (int i = 0; i < distCoeffs.cols; ++i)
    {
        // 必须使用distCoeffs.at<double>，不能使用distCoeffs.at<float>
        // 否则输出结果不正确
        std::cout << std::setprecision(10) << distCoeffs.at<double>(0, i) << "    ";
    }
    std::cout << std::endl;

    return 0;
}