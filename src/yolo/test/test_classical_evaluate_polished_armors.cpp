//
// Created by plutoli on 2021/9/18.
//

#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"

// 按ESC键，退出系统
// 按Enter键，暂停系统
// 按任意键，继续处理
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

    // 创建图像显示窗口
    cv::namedWindow("rawImage",cv::WINDOW_NORMAL);
    cv::resizeWindow("rawImage", 800, 600);
    cv::namedWindow("binaryImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("binaryImage", 800, 600);
    cv::namedWindow("evaluatedArmorsImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("evaluatedArmorsImage", 800, 600);

    // 记录初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
    uint64_t initTimestamp = initTime.time_since_epoch().count();

    // 初始化数据帧索引
    uint64_t frameIndex = 0;

    // 循环处理相机数据
    while (true)
    {
        // 记录起始时间戳
        std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
        uint64_t beginTimestamp = beginTime.time_since_epoch().count();

        // 获取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 累加数据帧索引
        frameIndex++;

        std::vector<ClassicalArmor> Armors;
        recognizer.DetectArmorsByYolo(cameraData.Image, Armors);

        // 评估精装甲板
        std::vector<float> scores;
        recognizer.EvaluatePolishedArmors(Armors, &scores);

        // 记录截止时间戳
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();

        // 输出处理结果
        std::cout << "**************************************************" << std::endl;
        std::cout << "frame index: " << frameIndex << std::endl;
        std::cout << "fps: " << (frameIndex * 1000000000) / (endTimestamp - initTimestamp) << std::endl;
        std::cout << "process time: " << (endTimestamp - beginTimestamp) / 1000000 << "ms" << std::endl;
        std::cout << "**************************************************" << std::endl << std::endl;

        // 创建评估装甲板图像
        cv::Mat evaluatedArmorsImage;
        ClassicalArmorRecognizer::CreateEvaluatedArmorsImage(Armors,
                                                             scores,
                                                             cameraData.Image,
                                                             &evaluatedArmorsImage);

        // 显示图像
        cv::imshow("rawImage", cameraData.Image);
        cv::imshow("evaluatedArmorsImage", evaluatedArmorsImage);

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