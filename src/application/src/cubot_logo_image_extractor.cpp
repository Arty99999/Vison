//
// Created by plutoli on 2022/2/6.
//

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_windmill_recognizer.h"

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

    // 创建风车识别器
    ClassicalWindmillRecognizer recognizer;

    // 加载风车识别器参数
    ClassicalWindmillRecognizerParam recognizerParam;
    std::string recognizerYaml = "config/infantry_3/basement/classical_windmill_recognizer_param.yaml";
    if (!ClassicalWindmillRecognizerParam::LoadFromYamlFile(recognizerYaml, &recognizerParam))
    {
        return -1;
    }

    // 设置风车识别器参数
    if (!recognizer.SetParam(recognizerParam))
    {
        return -1;
    }

    // 初始化风车识别器
    if (!recognizer.Init())
    {
        return -1;
    }

    // 初始化样本存储路径
    std::string samplePath = "logo/";
    std::string samplePositivePath = "logo/positive/";
    std::string sampleNegativePath = "logo/negative/";

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

    // 创建samplePositivePath
    if (::access(samplePositivePath.c_str(), F_OK) == -1)
    {
        if (::mkdir(samplePositivePath.c_str(), 0777) == 0)
        {
            log = "[" + samplePositivePath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + samplePositivePath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + samplePositivePath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleNegativePath
    if (::access(sampleNegativePath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleNegativePath.c_str(), 07777) == 0)
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
    cv::namedWindow("windmillLogosImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("windmillLogosImage", 800, 600);
    cv::namedWindow("positiveImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("positiveImage", 50, 50);
    cv::namedWindow("negative", cv::WINDOW_NORMAL);
    cv::resizeWindow("negative", 50, 50);

    // 显示临时图像
    cv::Mat bigImage(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::Mat smallImage(50, 50, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("rawImage", bigImage);
    cv::imshow("windmillLogosImage", bigImage);
    cv::imshow("positiveImage", smallImage);
    cv::imshow("negativeImage", smallImage);

    // 初始化样本索引
    int samplePositiveIndex = 0;
    int sampleNegativeIndex = 0;

    // 打印提示信息
    log = "Press 'ESC' to eixt. Press 'Enter' to save samples. Press other key to continue: ";
    logger.Save(ELogType::Info, log);

    // 读取并处理视频
    while (camera.IsOpened())
    {
        // 读取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 原始图像预处理
        cv::Mat binaryImage;
        recognizer.Preprocess(cameraData.Image, &binaryImage);

        // 检测风车Logo
        std::vector<ClassicalWindmillLogo> windmillLogos;
        recognizer.DetectWindmillLogos(binaryImage, cameraData.Image, &windmillLogos);

        // 创建风车Logo图像
        cv::Mat windmillLogosImage;
        ClassicalWindmillRecognizer::CreateWindmillLogosImage(windmillLogos,
                                                              cameraData.Image,
                                                              &windmillLogosImage);

        // 显示图像
        cv::imshow("rawImage", cameraData.Image);
        cv::imshow("windmillLogosImage", windmillLogosImage);

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

        // 保存风车Logo图片
        for(unsigned int i = 0; i < windmillLogos.size(); ++i)
        {
            if (windmillLogos[i].IsValid)
            {
                // 显示图片
                cv::imshow("positiveImage", windmillLogos[i].Image);

                // 索引累加
                samplePositiveIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "positive_" << std::setw(5) << std::setfill('0') << samplePositiveIndex << ".bmp";
                std::string fileName = samplePositivePath + ss.str();
                cv::imwrite(fileName, windmillLogos[i].Image);

                // 记录日志信息
                log = "Positives[" + std::to_string(samplePositiveIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                // 显示图片
                cv::imshow("negativeImage", windmillLogos[i].Image);

                // 索引累加
                sampleNegativeIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "negative_" << std::setw(5) << std::setfill('0') << sampleNegativeIndex << ".bmp";
                std::string fileName = sampleNegativePath + ss.str();
                cv::imwrite(fileName, windmillLogos[i].Image);

                // 记录日志信息
                log = "Negatives[" + std::to_string(samplePositiveIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }
        }
    }

    // 关闭相机
    camera.Close();

    // 释放相机资源
    camera.Release();

    // 释放风车识别器资源
    recognizer.Release();

    return 0;
}