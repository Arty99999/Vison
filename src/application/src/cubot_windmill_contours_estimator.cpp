//
// Created by plutoli on 2022-06-02.
//

#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_windmill_recognizer.h"

// 按ESC键，退出系统
// 按Enter键，暂停系统
// 按任意键，继续运行
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

    // 创建图像显示窗口
    cv::namedWindow("rawImage",cv::WINDOW_NORMAL);
    cv::resizeWindow("rawImage", 800, 600);
    cv::namedWindow("binaryImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("binaryImage", 800, 600);
    cv::namedWindow("windmillContoursImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("windmillContoursImage", 800, 600);

    // 显示临时图像
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("rawImage", image);
    cv::imshow("binaryImage", image);
    cv::imshow("windmillContoursImage", image);

    // 循环处理相机数据
    while (true)
    {
        // 获取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 原始图像预处理
        cv::Mat binaryImage;
        recognizer.Preprocess(cameraData.Image, &binaryImage);

        // 创建风车轮廓图像
        cv::Mat windmillContoursImage;
        ClassicalWindmillRecognizer::CreateWindmillContoursImage(binaryImage,
                                                                 recognizerParam,
                                                                 cameraData.Image,
                                                                 &windmillContoursImage);

        // 显示图像
        cv::imshow("rawImage", cameraData.Image);
        cv::imshow("binaryImage", binaryImage);
        cv::imshow("windmillContoursImage", windmillContoursImage);

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