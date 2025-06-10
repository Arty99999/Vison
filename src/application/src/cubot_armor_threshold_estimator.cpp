//
// Created by plutoli on 2022-06-06.
//

#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"

// 使用说明：1-将classical_armor_recognizer_param.yaml文件中的MaxLocationOffset设置一个足够大的值(例如：1000) \n
//         2-将classical_armor_recognizer_param.yaml文件中的MaxMemoryLength设置一个足够大的值(例如：5000) \n
//         3-运行程序，提取装甲板位置缓冲区相邻时刻的最大位置偏差 \n

// 按Enter键，启动/暂停系统
// 按ESC键，退出系统
int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

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

    // 创建装甲板识别器
    ClassicalArmorRecognizer recognizer;

    // 读取装甲板识别器参数
    ClassicalArmorRecognizerParam recognizerParam;
    std::string recognizerYaml = "config/infantry_3/basement/classical_armor_recognizer_param.yaml";
    if (!ClassicalArmorRecognizerParam::LoadFromYamlFile(recognizerYaml, &recognizerParam))
    {
        return -1;
    }

    // 设置装甲板识别器参数
    if (!recognizer.SetParam(recognizerParam))
    {
        return -1;
    }

    // 初始化装甲板识别器
    if (!recognizer.Init())
    {
        return -1;
    }

    // 初始化显示窗体
    cv::namedWindow("trackedArmorImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("trackedArmorImage", 800, 600);

    // 显示临时图像
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("rangedArmorImage", image);

    // 循环处理相机数据
    while (true)
    {
        // 读取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 原始图像预处理
        cv::Mat binaryImage;
        recognizer.Preprocess(cameraData.Image, &binaryImage);

        // 检测粗灯条
        std::vector<ClassicalLightBar> rawLightBars;
        recognizer.DetectRawLightBars(binaryImage, &rawLightBars);

        // 检测精灯条
        std::vector<ClassicalLightBar> polishedLightBars;
        recognizer.DetectPolishedLightBars(rawLightBars, &polishedLightBars);

        // 检测灯条对
        std::vector<std::pair<ClassicalLightBar, ClassicalLightBar>> lightBarPairs;
        recognizer.DetectLightBarPairs(polishedLightBars, &lightBarPairs);

        // 检测粗装甲板
        std::vector<ClassicalArmor> rawArmors;
        recognizer.DetectRawArmors(lightBarPairs, cameraData.Image, &rawArmors);

        // 检测精装甲板
        std::vector<ClassicalArmor> polishedArmors;
        recognizer.DetectPolishedArmors(rawArmors, &polishedArmors);

        // 判断是否检测到目标装甲板
        bool isTargetArmorReady = false;
        ClassicalArmor targetArmor;
        for (unsigned int i = 0; i < polishedArmors.size(); ++i)
        {
            if (polishedArmors[i].Number != EClassicalArmorNumber::Three)
            {
                isTargetArmorReady = true;
                targetArmor = polishedArmors[i];
                break;
            }
        }

        // 初始化目标装甲板的历史位置信息
        std::vector<cv::Point2f> locations;

        // 如果检测到目标装甲板，提取目标装甲板的历史位置信息
        if (isTargetArmorReady)
        {
            // 更新装甲板数据缓冲区
            recognizer.UpdateArmorBuffer(targetArmor);

            // 读取装甲板位置时序
            std::vector<std::pair<cv::Point2f, uint64_t>> locationSequence;
            recognizer.GetArmorLocationSequence(targetArmor.Number, &locationSequence);

            // 提取装甲板的历史位置信息
            for (unsigned int i = 0; i < locationSequence.size(); ++i)
            {
                locations.emplace_back(locationSequence[i].first);
            }
        }

        // 创建并显示追踪装甲板图像
        cv::Mat trackedArmorImage;
        ClassicalArmorRecognizer::CreateTrackedArmorImage(targetArmor,
                                                          locations,
                                                          cameraData.Image,
                                                          &trackedArmorImage);
        cv::imshow("trackedArmorImage",trackedArmorImage);

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

    // 释放装甲板识别器资源
    recognizer.Release();

    return 0;
}