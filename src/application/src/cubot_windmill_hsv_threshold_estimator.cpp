//
// Created by plutoli on 2022-05-14.
//

#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "delta_cv.h"
#include "huaray_camera.h"
#include "classical_windmill_recognizer_param.h"

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
    ClassicalWindmillRecognizerParam recognizerParam;
    std::string recognizerYaml = "config/infantry_3/basement/classical_windmill_recognizer_param.yaml";
    if (!ClassicalWindmillRecognizerParam::LoadFromYamlFile(recognizerYaml, &recognizerParam))
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
    if (recognizerParam.WindmillColor == EClassicalWindmillColor::Blue)
    {
        hueLower_1 = recognizerParam.TargetHSVThresholdForBlue.HueLower;
        hueUpper_1 = recognizerParam.TargetHSVThresholdForBlue.HueUpper;
        hueLower_2 = 0;
        hueUpper_2 = 0;
        saturationLower = recognizerParam.TargetHSVThresholdForBlue.SaturationLower;
        saturationUpper = recognizerParam.TargetHSVThresholdForBlue.SaturationUpper;
        valueLower = recognizerParam.TargetHSVThresholdForBlue.ValueLower;
        valueUpper = recognizerParam.TargetHSVThresholdForBlue.ValueUpper;
    }
    else
    {
        hueLower_1 = recognizerParam.TargetHSVThresholdForRed_1.HueLower;
        hueUpper_1 = recognizerParam.TargetHSVThresholdForRed_1.HueUpper;
        hueLower_2 = recognizerParam.TargetHSVThresholdForRed_2.HueLower;                                                           ///< 色度下限2
        hueUpper_2 = recognizerParam.TargetHSVThresholdForRed_2.HueUpper;                                                           ///< 色度上限2
        saturationLower = recognizerParam.TargetHSVThresholdForRed_1.SaturationLower;
        saturationUpper = recognizerParam.TargetHSVThresholdForRed_1.SaturationUpper;
        valueLower = recognizerParam.TargetHSVThresholdForRed_1.ValueLower;
        valueUpper = recognizerParam.TargetHSVThresholdForRed_1.ValueUpper;
    }

    // 初始化形态学处理各个参数；开运算可以用来消除小物体，在纤细点处分离物体，平滑较大物体的同时不改变其面积；闭运算，先膨胀后腐蚀，先让白色便多，修复裂痕等；
    // 形态学梯度，是膨胀图和腐蚀图之差，可以让物体边缘突出出来；顶帽运算，开运算先进行腐蚀再进行膨胀，放大了裂缝和局部低亮度区域，因此通过顶帽运算得到比原图像轮廓周围区域更明亮的区域
    // 黑帽运算，得到比原图区域更暗的区域
    int view = 1;               ///< 模式切换
    int kernelSize = 2;        ///< 核的大小
//    int kernelSizeDilate = 1;
    int kernelSizeErode = 1;

    // 创建图像显示窗口
    cv::namedWindow("rawImage",cv::WINDOW_NORMAL);
    cv::resizeWindow("rawImage", 800, 600);
    cv::namedWindow("binaryImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("binaryImage", 800, 600);
    cv::namedWindow("controller", cv::WINDOW_NORMAL);
    cv::resizeWindow("controller", 500, 20);
    cv::namedWindow("binaryImageFan", cv::WINDOW_NORMAL);
    cv::resizeWindow("binaryImageFan", 800, 600);
    cv::namedWindow("binaryImageTarget", cv::WINDOW_NORMAL);
    cv::resizeWindow("binaryImageTarget", 800, 600);

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

    cv::createTrackbar("Model",
                       "controller",
                       &view,
                       7);

    cv::createTrackbar("kernelSize",
                       "controller",
                       &kernelSize,
                       20);

//    cv::createTrackbar("kernelSizeDilate",
//                       "controller",
//                       &kernelSizeDilate,
//                       20);
//
    cv::createTrackbar("kernelSizeErode",
                       "controller",
                       &kernelSizeErode,
                       20);


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
        cv::Mat binaryImageFan;
        cv::Mat binaryImageTarget;

        cv::Mat dstImage;
        if(view)
        {
            // 对二值化图像进行形态学处理
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                        cv::Size(kernelSize,kernelSize),
                                                        cv::Point(-1,-1));

            cv::Mat elementErode = cv::getStructuringElement(cv::MORPH_RECT,
                                                             cv::Size(kernelSizeErode,kernelSizeErode),
                                                             cv::Point(-1,-1));

            cv::Mat elementErodeFan = cv::getStructuringElement(cv::MORPH_RECT,
                                                                cv::Size(1,1),
                                                                cv::Point(-1,-1));

            cv::Mat elementErodeFanCom = cv::getStructuringElement(cv::MORPH_RECT,
                                                                   cv::Size(1, 1),
                                                                   cv::Point(-1, -1));
//            cv::morphologyEx(binaryImage, dstImage, cv::MORPH_CLOSE, element);
//            cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_RECT,
//                                                        cv::Size(kernelSizeDilate,kernelSizeDilate),
//                                                        cv::Point(-1,-1));
//            cv::Mat elementErode = cv::getStructuringElement(cv::MORPH_RECT,
//                                                             cv::Size(kernelSizeErode,kernelSizeErode),
//                                                             cv::Point(-1,-1));
//
//            cv::erode(dstImage,dstImage,kernelSizeErode);
//            cv::dilate(binaryImage, dstImage,elementDilate);

            switch(view)
            {
                // 1、开运算
                case 1:
                    cv::erode(binaryImage, binaryImageFan, elementErodeFanCom);
                    cv::morphologyEx(binaryImageFan, binaryImageFan, cv::MORPH_CLOSE, element);
                    cv::morphologyEx(binaryImage, binaryImageTarget, cv::MORPH_CLOSE, element);
                    cv::erode(binaryImageTarget, binaryImageTarget, elementErode);
                    cv::putText(binaryImageFan, "Com", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 1);
                    break;
                case 2:
                    cv::erode(binaryImage, binaryImageFan, elementErodeFan);
//                    cv::dilate(binaryImageFan, binaryImageFan, element);
                    cv::erode(binaryImageFan, binaryImageTarget, elementErodeFan);
                    cv::morphologyEx(binaryImageTarget, binaryImageTarget, cv::MORPH_CLOSE, element);
//                    cv::erode(binaryImage, binaryImageTarget, elementErode);
//                    cv::morphologyEx(binaryImage, binaryImageTarget, cv::MORPH_CLOSE, element);
                    cv::erode(binaryImageTarget, binaryImageTarget, elementErode);
                    cv::putText(binaryImageFan, "CLOSE", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 1);
                    break;
                case 3:
                    cv::morphologyEx(binaryImage, binaryImageFan, cv::MORPH_GRADIENT, element);
                    cv::dilate(binaryImageFan, binaryImageTarget, elementErode);
                    cv::putText(binaryImageFan, "GRADIENT", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 1);
                    break;
                case 4:
                    cv::morphologyEx(binaryImage, binaryImageFan, cv::MORPH_TOPHAT, element);
                    cv::dilate(binaryImageFan, binaryImageTarget, elementErode);
                    cv::putText(binaryImageFan, "TOPHAT", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 1);
                    break;
                case 5:
                    cv::morphologyEx(binaryImage, binaryImageFan, cv::MORPH_BLACKHAT, element);
                    cv::dilate(binaryImageFan, binaryImageTarget, elementErode);
                    cv::putText(binaryImageFan, "BLACKHAT", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 1);
                    break;
                case 6:
                    cv::morphologyEx(binaryImage, binaryImageFan, cv::MORPH_ERODE, element);
                    cv::dilate(binaryImageFan, binaryImageTarget, elementErode);
                    cv::putText(binaryImageFan, "ERODE", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 1);
                    break;
                case 7:
                    cv::morphologyEx(binaryImage, binaryImageFan, cv::MORPH_DILATE, element);
                    cv::dilate(binaryImageFan, binaryImageTarget, elementErode);
                    cv::putText(binaryImageFan, "DILATE", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,255,255), 1);
                    break;
            }
        }
        else
        {
            binaryImage.copyTo(binaryImageFan);
            binaryImage.copyTo(binaryImageTarget);
        }

        // 显示原始图像和分割之后的二值图像
        imshow("rawImage", cameraData.Image);
        cv::imshow("binaryImage", binaryImage);
        cv::imshow("binaryImageFan", binaryImageFan);
        cv::imshow("binaryImageTarget", binaryImageTarget);

        // 按下ESC键，退出系统；cv::waitKey()返回的是按键的ASCII码
        int keyValue = cv::waitKey(10);
        if (keyValue == 27)
        {
            break;
        }
        if (keyValue == 13)
        {
            cv::waitKey(0);
        }
    }

    // 关闭相机
    camera.Close();

    // 释放相机资源
    camera.Release();

    return 0;
}