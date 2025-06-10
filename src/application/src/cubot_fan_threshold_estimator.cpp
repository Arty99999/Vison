//
// Created by plutoli on 2022-06-06.
//

#include <iostream>
#include <vector>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_windmill_recognizer.h"

// 使用说明：1-将classical_windmill_recognizer_param.yaml文件中的MaxFanLocationOffset设置一个足够大的值(例如：1000) \n
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
    cv::namedWindow("trackedFanImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("trackedFanImage", 800, 600);

    // 显示临时图像
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("trackedFanImage", image);

    // 读取并处理视频
    while (true)
    {
        // 读取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 原始图像预处理
        cv::Mat binaryImage;
        recognizer.Preprocess(cameraData.Image, &binaryImage);

        // 检测风车扇叶
        std::vector<ClassicalWindmillFan> fans;
        recognizer.DetectWindmillFans(binaryImage, &fans);

        // 判断是否检测到合法的风车扇叶
        bool isFanReady = false;
        ClassicalWindmillFan validFan;
        for (unsigned int i = 0; i < fans.size(); ++i)
        {
            if (fans[i].IsValid)
            {
                isFanReady = true;
                validFan = fans[i];
                validFan.Timestamp = cameraData.Timestamp;
                break;
            }
        }

        // 初始化风车扇叶历史位置信息
        std::vector<cv::Point2f> locations;

        // 如果检测到合法的风车扇叶，提取风车扇叶的历史位置信息
        if (isFanReady)
        {
            // 更新扇叶位置缓冲区
            recognizer.UpdateFanBuffer(validFan);

            // 获取风车扇叶位置时序
            std::vector<std::pair<cv::Point2f, uint64_t>> locationSequence;
            recognizer.GetFanLocationSequence(&locationSequence);

            // 提取风车扇叶历史位置信息
            for (unsigned int i = 0; i < locationSequence.size(); ++i)
            {
                locations.emplace_back(locationSequence[i].first);
            }
        }

        // 创建并显示追踪扇叶图像
        cv::Mat trackedFanImage;
        ClassicalWindmillRecognizer::CreateTrackedFanImage(validFan,
                                                            locations,
                                                            cameraData.Image,
                                                            &trackedFanImage);
        cv::imshow("trackedFanImage", trackedFanImage);

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