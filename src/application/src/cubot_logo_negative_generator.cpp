//
// Created by plutoli on 2022/2/6.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_windmill_recognizer_param.h"

// 按ESC键，退出系统
// 按Enter键，保存样本
// 按其它键，继续处理
int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 初始化日志信息
    std::string log;

    // 创建相机
    HuarayCamera camera;

    // 读取相机参数
    HuarayCameraParam cameraParam;
    std::string cameraYaml = "config/infantry_3/basement/huaray_camera_param.yaml";
    if (!HuarayCameraParam::LoadFromYamlFile(cameraYaml, &cameraParam))
    {
        return -1;
    }

    // 设置相机参数
    if (!camera.SetParam(cameraParam))
    {
        return -1;
    }

    // 初始化相机
    if (!camera.Init())
    {
        return -1;
    }

    // 打开相机
    if (!camera.Open())
    {
        return -1;
    }

    // 读取风车识别器参数
    ClassicalWindmillRecognizerParam recognizerParam;
    std::string recognizerYaml = "config/infantry_3/basement/classical_windmill_recognizer_param.yaml";
    if (!ClassicalWindmillRecognizerParam::LoadFromYamlFile(recognizerYaml, &recognizerParam))
    {
        return -1;
    }

    // 初始化负样本存储路径
    std::string samplePath = "generator/";
    std::string sampleNegativePath = "generator/logo/";

    // 创建samplePath
    if (::access(samplePath.c_str(), F_OK) == -1)
    {
        if (::mkdir(samplePath.c_str(), 07777) == 0)
        {
            log = "[" + samplePath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + samplePath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + samplePath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleNegativePath
    if (::access(sampleNegativePath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleNegativePath.c_str(), 0777) == 0)
        {
            log = "[" + sampleNegativePath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + sampleNegativePath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + sampleNegativePath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 初始化显示窗体
    cv::namedWindow("rawImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("rawImage", 800, 600);
    cv::namedWindow("grayScaleImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("grayScaleImage", 800, 600);
    cv::namedWindow("negativeImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("negativeImage", 50, 50);

    // 显示临时图像
    cv::Mat bigImage(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::Mat smallImage(50, 50, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("rawImage", bigImage);
    cv::imshow("windmillLogosImage", bigImage);
    cv::imshow("negativeImage", smallImage);

    // 打印提示信息
    log = "Press 'ESC' to eixt. Press 'Enter' to save samples. Press other key to continue: ";
    logger.Save(ELogType::Info, log);

    // 从视频流中抽取负样本
    int sampleIndex = 0;
    while (camera.IsOpened())
    {
        // 读取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 如果原始图像是彩色图像，将其转为灰度图像
        cv::Mat grayScaleImage;
        if (cameraData.Image.type() == CV_8UC3)
        {
            cv::cvtColor(cameraData.Image, grayScaleImage, cv::COLOR_BGR2GRAY);
            cv::medianBlur(grayScaleImage, grayScaleImage, 3);
        }
        else
        {
            cameraData.Image.copyTo(grayScaleImage);
        }

        // 显示图像
        cv::imshow("rawImage", cameraData.Image);
        cv::imshow("grayScaleImage", grayScaleImage);

        // 当前线程阻塞，读取键盘按键的键值
        int keyValue = cv::waitKey(0);

        // 如果按下“ESC”键，则退出系统
        if (keyValue == 27)
        {
            break;
        }

        // 如果按下"Enter"键，保存负样本图像
        if (keyValue != 13)
        {
            continue;
        }

        // 按照风车Logo的Hog特征窗体大小依次对原始图像进行分割
        int sampleIndexOffset = 0;
        int rowNumber = grayScaleImage.rows / recognizerParam.LogoHogWindowHeight;
        int colNumber = grayScaleImage.cols / recognizerParam.LogoHogWindowWidth;
        for (int i = 0; i < colNumber; ++i)
        {
            for (int j = 0; j < rowNumber; ++j)
            {
                // 提取负样本图片
                cv::Rect rect(i * recognizerParam.LogoHogWindowWidth,
                              j * recognizerParam.LogoHogWindowHeight,
                              recognizerParam.LogoHogWindowWidth,
                              recognizerParam.LogoHogWindowHeight);
                cv::Mat negativeImage = grayScaleImage(rect);

                // 显示负样本图片
                cv::imshow("negativeImage", negativeImage);

                // 保存负样本图片
                sampleIndex++;
                std::stringstream ss;
                ss << "negative_" << std::setw(5) << std::setfill('0') << sampleIndex << ".bmp";
                std::string fileName = sampleNegativePath + ss.str();
                cv::imwrite(fileName, negativeImage);

                // 记录日志信息
                log = "Negatives[" + std::to_string(sampleIndex) + "] has been generated successful";
                logger.Save(ELogType::Info, log);
            }
        }
    }

    // 关闭相机
    camera.Close();

    // 释放相机资源
    camera.Release();

    return 0;
}