//
// Created by plutoli on 2022-06-13.
//

#include <iostream>
#include <vector>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "solver.h"
#include "classical_windmill_recognizer.h"

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
    cv::namedWindow("objectsImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("objectsImage", 800, 600);

    // 显示临时图像
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("objectsImage", image);

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

        // 检测风车扇叶
        std::vector<ClassicalWindmillFan> fans;
        recognizer.DetectWindmillFans(binaryImage, &fans);

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

        // 初始化最大角速度和最小角速度
        float minPalstance = FLT_MAX;
        float maxPalstance = FLT_MIN;

        // 如果同时检测到合法的风车Logo和风车扇叶，则计算角速度
        if (isLogoReady && isFanReady)
        {
            // 更新Logo位置缓冲区和扇叶位置缓冲区
            recognizer.UpdateLogoBuffer(validLogo);
            recognizer.UpdateFanBuffer(validFan);

            // 获取风车Logo位置时序和风车扇叶位置时序
            std::vector<std::pair<cv::Point2f, uint64_t>> logoLocationSequence;
            std::vector<std::pair<cv::Point2f, uint64_t>> fanLocationSequence;
            recognizer.GetLogoLocationSequence(&logoLocationSequence);
            recognizer.GetFanLocationSequence(&fanLocationSequence);

            // 计算风车Logo位置坐标之和
            float total_x = 0.0;
            float total_y = 0.0;
            for (int i = 0; i < logoLocationSequence.size(); ++i)
            {
                total_x += logoLocationSequence[i].first.x;
                total_y += logoLocationSequence[i].first.y;
            }

            // 计算风车Logo位置坐标均值
            cv::Point2f logoLocation;
            logoLocation.x = total_x / static_cast<float>(logoLocationSequence.size());
            logoLocation.y = total_y / static_cast<float>(logoLocationSequence.size());

            // 计算弧度样本集合
            std::vector<std::pair<float, float>> samples;
            Solver::ComputeRadianSamples(logoLocation, fanLocationSequence, &samples);

            // 计算角速度样本集合
            std::vector<float> palstances;
            for (unsigned int i = 1; i < samples.size(); ++i)
            {
                float deltaTime = (samples[i].first - samples[i - 1].first) / 1000;
                float deltaRadian = samples[i].second - samples[i - 1].second;
                palstances.emplace_back(deltaRadian / deltaTime);
            }

            // 更新角速度的最大最小值
            unsigned int size = palstances.size();
            if (size > 3)
            {
                // 计算平均角速度
                float palstance = (palstances[size - 1] + palstances[size - 2] + palstances[size - 3]) / 3;

                // 更新最大角速度
                if (palstance > maxPalstance)
                {
                    maxPalstance = palstance;
                }

                // 更新最小角速度
                if (palstance < minPalstance)
                {
                    minPalstance = palstance;
                }
            }
        }

        // 输出最大/最小角速度
        std::cout << "**************************************************" << std::endl;
        std::cout << "max palstance: " << maxPalstance << " radian/s" << std::endl;
        std::cout << "min palstance: " << minPalstance << " radian/s" << std::endl;
        std::cout << "**************************************************" << std::endl << std::endl;

        // 创建并显示目标图像
        cv::Mat objectsImage;
        ClassicalWindmillRecognizer::CreateWindmillObjectsImage(logos,
                                                                fans,
                                                                cameraData.Image,
                                                                &objectsImage);
        cv::imshow("objectsImage", objectsImage);

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