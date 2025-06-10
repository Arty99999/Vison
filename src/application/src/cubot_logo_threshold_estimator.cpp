//
// Created by plutoli on 2022-06-06.
//

#include <iostream>
#include <vector>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_windmill_recognizer.h"

// 使用说明：1-将classical_windmill_recognizer_param.yaml文件中的MaxLogoLocationOffset设置一个足够大的值(例如：1000) \n
//         2-将classical_windmill_recognizer_param.yaml文件中的MaxMemoryLength设置一个足够大的值(例如：5000) \n
//         3-运行程序，提取装甲板位置缓冲区相邻时刻的最大位置偏差 \n

// 按Enter键，启动/暂停系统
// 按ESC键，退出系统
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

    // 初始化显示窗体
    cv::namedWindow("trackedLogoImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("trackedLogoImage", 800, 600);

    // 显示临时图像
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("trackedLogoImage", image);

    // 读取并处理视频
    while (true)
    {
        // 读取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 原始图像预处理
        cv::Mat binaryImage;
        recognizer.Preprocess(cameraData.Image, &binaryImage);

        // 检测风车Logo
        std::vector<ClassicalWindmillLogo> logos;
        recognizer.DetectWindmillLogos(binaryImage, cameraData.Image, &logos);

        // 判断是否检测到合法的风车Logo
        bool isLogoReady = false;
        ClassicalWindmillLogo validLogo;
        for (unsigned int i = 0; i < logos.size(); ++i)
        {
            if (logos[i].IsValid)
            {
                isLogoReady = true;
                validLogo = logos[i];
                validLogo.Timestamp = cameraData.Timestamp;
                break;
            }
        }

        // 初始化风车Logo历史位置信息
        std::vector<cv::Point2f> locations;

        // 如果检测到合法的风车Logo，提取风车Logo的历史位置信息
        if (isLogoReady)
        {
            // 更新Logo位置缓冲区
            recognizer.UpdateLogoBuffer(validLogo);

            // 获取风车Logo位置时序
            std::vector<std::pair<cv::Point2f, uint64_t>> locationSequence;
            recognizer.GetLogoLocationSequence(&locationSequence);

            // 提取风车Logo历史位置信息
            for (unsigned int i = 0; i < locationSequence.size(); ++i)
            {
                locations.emplace_back(locationSequence[i].first);
            }
        }

        // 创建并显示追踪Logo图像
        cv::Mat trackedLogoImage;
        ClassicalWindmillRecognizer::CreateTrackedLogoImage(validLogo,
                                                            locations,
                                                            cameraData.Image,
                                                            &trackedLogoImage);
        cv::imshow("trackedLogoImage", trackedLogoImage);

        // 读取按键的ASCII码；注意：cv::waitKey()返回的是按键的ASCII码
        int keyValue = cv::waitKey(10);

        // 如果按下ESC键，退出系统
        if (keyValue == 27)
        {
            break;
        }

        // 如果按下Enter键，暂停系统
        if (keyValue == 13)
        {
            cv::waitKey(0);
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