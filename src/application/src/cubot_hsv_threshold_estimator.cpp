//
// Created by plutoli on 2022-05-14.
//

#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "delta_cv.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer_param.h"

// 按ESC键，退出系统
int main()
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

    // 读取装甲板识别器参数
    ClassicalArmorRecognizerParam recognizerParam;
    std::string recognizerYaml = "config/infantry_3/basement/classical_armor_recognizer_param.yaml";
    if (!ClassicalArmorRecognizerParam::LoadFromYamlFile(recognizerYaml, &recognizerParam))
    {
        return -1;
    }

    // 初始化HSV分割阈值
    int hueLower_1 = 0;             ///< 色度下限1
    int hueUpper_1 = 180;           ///< 色度上限1
    int hueLower_2 = 0;             ///< 色度下限2
    int hueUpper_2 = 180;           ///< 色度上限2
    int saturationLower = 0;        ///< 饱和度下限
    int saturationUpper = 255;      ///< 饱和度上限
    int valueLower = 0;             ///< 亮度下限
    int valueUpper = 255;           ///< 亮度上限

    // 初始化HSV分割阈值
    if (recognizerParam.LightBarColor == EClassicalLightBarColor::Blue)
    {
        hueLower_1 = recognizerParam.HSVThresholdForBlue.HueLower;
        hueUpper_1 = recognizerParam.HSVThresholdForBlue.HueUpper;
        hueLower_2 = 0;
        hueUpper_2 = 0;
        saturationLower = recognizerParam.HSVThresholdForBlue.SaturationLower;
        saturationUpper = recognizerParam.HSVThresholdForBlue.SaturationUpper;
        valueLower = recognizerParam.HSVThresholdForBlue.ValueLower;
        valueUpper = recognizerParam.HSVThresholdForBlue.ValueUpper;
    }
    else
    {
        hueLower_1 = recognizerParam.HSVThresholdForRed_1.HueLower;
        hueUpper_1 = recognizerParam.HSVThresholdForRed_1.HueUpper;
        hueLower_2 = recognizerParam.HSVThresholdForRed_2.HueLower;                                                           ///< 色度下限2
        hueUpper_2 = recognizerParam.HSVThresholdForRed_2.HueUpper;                                                           ///< 色度上限2
        saturationLower = recognizerParam.HSVThresholdForRed_1.SaturationLower;
        saturationUpper = recognizerParam.HSVThresholdForRed_1.SaturationUpper;
        valueLower = recognizerParam.HSVThresholdForRed_1.ValueLower;
        valueUpper = recognizerParam.HSVThresholdForRed_1.ValueUpper;
    }

    // 创建图像显示窗口
    cv::namedWindow("rawImage",cv::WINDOW_NORMAL);
    cv::resizeWindow("rawImage", 800, 600);
    cv::namedWindow("binaryImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("binaryImage", 800, 600);
    cv::namedWindow("controller", cv::WINDOW_NORMAL);
    cv::resizeWindow("controller", 500, 10);

    // 显示临时图像
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("rawImage", image);
    cv::imshow("binaryImage", image);

    // 创建色度下限控制条1
    cv::createTrackbar("          HueLower_1",
                       "controller",
                       &hueLower_1,
                       180);

    // 创建色度上限控制条1
    cv::createTrackbar("          HueUpper_1",
                       "controller",
                       &hueUpper_1,
                       180);

    // 创建色度下限控制条2
    cv::createTrackbar("          HueLower_2",
                       "controller",
                       &hueLower_2,
                       180);

    // 创建色度上限控制条2
    cv::createTrackbar("          HueUpper_2",
                       "controller",
                       &hueUpper_2,
                       180);

    // 创建饱和度下限控制条
    cv::createTrackbar("SaturationLower",
                       "controller",
                       &saturationLower,
                       255);

    // 创建饱和度上限控制条
    cv::createTrackbar("SaturationUpper",
                       "controller",
                       &saturationUpper,
                       255);

    // 创建亮度下限控制条
    cv::createTrackbar("          ValueLower",
                       "controller",
                       &valueLower,
                       255);

    // 创建亮度上限控制条
    cv::createTrackbar("          ValueUpper",
                       "controller",
                       &valueUpper,
                       255);

    // 循环处理相机数据
    while (camera.IsOpened())
    {
        // 获取原始图像
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 将原始图像转换为HSV图像
        cv::Mat hsvImage;
        cv::cvtColor(cameraData.Image, hsvImage, cv::COLOR_BGR2HSV);

        // 创建HSV阈值1
        HSVThreshold hsvThreshold1;
        hsvThreshold1.HueLower = hueLower_1;
        hsvThreshold1.HueUpper = hueUpper_1;
        hsvThreshold1.SaturationLower = saturationLower;
        hsvThreshold1.SaturationUpper = saturationUpper;
        hsvThreshold1.ValueLower = valueLower;
        hsvThreshold1.ValueUpper = valueUpper;

        // 使用HSV阈值1对HSV图像进行分割
        cv::Mat binaryImage_1(cameraData.Image.rows, cameraData.Image.cols, CV_8UC1);
        DeltaCV::FastInRange(hsvImage, hsvThreshold1, &binaryImage_1);

        // 创建HSV阈值2
        HSVThreshold hsvThreshold2;
        hsvThreshold2.HueLower = hueLower_2;
        hsvThreshold2.HueUpper = hueUpper_2;
        hsvThreshold2.SaturationLower = saturationLower;
        hsvThreshold2.SaturationUpper = saturationUpper;
        hsvThreshold2.ValueLower = valueLower;
        hsvThreshold2.ValueUpper = valueUpper;

        // 使用HSV阈值2对HSV图像进行分割
        cv::Mat binaryImage_2(cameraData.Image.rows, cameraData.Image.cols, CV_8UC1);
        DeltaCV::FastInRange(hsvImage, hsvThreshold2, &binaryImage_2);

        // 创建合并之后的二值图像
        cv::Mat binaryImage(cameraData.Image.rows, cameraData.Image.cols, CV_8UC1);
        binaryImage = binaryImage_1 + binaryImage_2;

        // 对二值图进行膨胀操作
        cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_CROSS,
                                                          cv::Size(2, 2),
                                                          cv::Point(-1, -1));
        cv::dilate(binaryImage,
                   binaryImage,
                   dilateElement,
                   cv::Point(-1, -1),
                   1);

        // 显示原始图像和分割之后的二值图像
        imshow("rawImage", cameraData.Image);
        cv::imshow("binaryImage", binaryImage);

        // 按下ESC键，退出系统；cv::waitKey()返回的是按键的ASCII码
        int keyValue = cv::waitKey(10);
        if (keyValue == 27)
        {
            break;
        }
    }

    // 关闭相机
    camera.Close();

    // 释放相机资源
    camera.Release();

    return 0;
}