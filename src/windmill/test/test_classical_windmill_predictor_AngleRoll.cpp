//
// Created by gusha on 23-10-28.
//

#include <opencv2/opencv.hpp>
#include <chrono>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_windmill_recognizer.h"
#include "classcal_windmill_cerese_predictor.h"

//cv::VideoCapture video("/home/gusha/下载/关灯-红方大能量机关-正在激活状态.MP4");
HuarayCameraData cameraData;

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

    //  读取相机参数
    HuarayCameraParam cameraParam;
    std::string cameraYaml = "config/infantry_3/basement/huaray_camera_param.yaml";
    if (!HuarayCameraParam::LoadFromYamlFile(cameraYaml, &cameraParam))
    {
        return -1;
    }

    // 设置相机参数
    if (!camera.SetParam(cameraParam))   return -1;

    //  初始化相机
    if (!camera.Init())   return -1;

    // 打开相机
    if (!camera.Open())   return -1;

    // 创建风车识别器
    ClassicalWindmillRecognizer recognizer;

    // 加载风车识别器参数
    ClassicalWindmillRecognizerParam recognizerParam;
    std::string recognizerYaml = "config/infantry_3/basement/classical_windmill_recognizer_param.yaml";
    if (!ClassicalWindmillRecognizerParam::LoadFromYamlFile(recognizerYaml, &recognizerParam))
    {
        return -1;
    }

//    std::cout << " --------------- " << int(recognizerParam.TargetHSVThresholdForBlue.HueUpper) << std::endl;

    // 设置风车识别器参数
    if (!recognizer.SetParam(recognizerParam))   return -1;


    // 初始化风车识别器
    if (!recognizer.Init())    return -1;


    // 创建图像显示窗口
    float k = 0.7;
//    cv::namedWindow("rawimage", cv::WINDOW_NORMAL);
//    cv::resizeWindow("rawimage", int(1600 * k) , int(1200 * k) );
    cv::namedWindow("pre_coordinate_image", cv::WINDOW_NORMAL);
    cv::resizeWindow("pre_coordinate_image", int(1600 * k) , int(1200 * k) );

    // 记录初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
    uint64_t initTimestamp = initTime.time_since_epoch().count();

    // 初始化数据帧索引和风车扇叶索引
    uint64_t frameIndex = 0;
    double                                   param[3];
    std::vector<ClassicalWindmillLogo>       ValidLogos;
    std::vector<uint64_t>                    w_Timestamps;
    std::vector<float>                       angles;
    std::vector<W_Time_data>                 sin_datas;
    cv::Mat tempimage;
    cv::VideoCapture video ("/home/cubot/Documents/windmill_red.avi") ;

    // 循环处理相机数据
    while (true)
    {
        std::chrono::time_point<std::chrono::steady_clock> bg = std::chrono::steady_clock::now();
        uint64_t bgtime = bg.time_since_epoch().count();

        camera.GetData(&cameraData);
//        video >> cameraData.Image;

        Eigen::Quaterniond Q_raw(1,0,0,0);
        Eigen::Matrix3d QMatrix = Q_raw.toRotationMatrix();

//        cv::imshow("rawimage",cameraData.Image);
        cameraData.Image.copyTo(tempimage);
        // 累加数据帧索引
        frameIndex++;

        Fitcoordinate Fit (SMALL_RUN ,300,1);

        // 原始图像预处理
        cv::Mat binaryImageFan;
        cv::Mat binaryImageTarget;
        recognizer.Preprocess(cameraData.Image, &binaryImageFan, &binaryImageTarget);

//        cv::imshow ( " binaryImageFan " ,  binaryImageFan);
//        cv::imshow ( " binaryImageTarget " ,  binaryImageTarget);

        // 检测风车扇叶集合
        ClassicalWindmillFan windmillFan;
        std::vector<ClassicalWindmillFan> targetContours;
        recognizer.DetectWindmillFans(binaryImageFan, binaryImageTarget, cameraData.Image, &windmillFan, &targetContours);

        bool isfanok = false;
        if (windmillFan.AspectRatio != 0 )  isfanok = true;
        ClassicalWindmillRecognizer::drawRotatedRect(tempimage, windmillFan.RotatedRect, cv::Scalar(255,255,255), 3);

        std::vector<ClassicalWindmillLogo> WindmillLogos;
        recognizer.DetectWindmillLogos(binaryImageFan, cameraData.Image, &WindmillLogos);


        for (unsigned int i = 0; i < WindmillLogos.size(); i++)
        {
            if (WindmillLogos[i].IsValid)
            {

                Fit.ValidLogo = WindmillLogos[i];
                ClassicalWindmillLogo templogo;

//                Fit.IsLogoReady = Fitcoordinate::_3DComputeLogoHOP(Fit.ValidLogo,ValidLogos,QMatrix,camera.GetParam().ModelParam);

//                Fit.IsLogoReady = Fitcoordinate::_2DComputeLogoHOP(Fit.ValidLogo,ValidLogos);

                  Fit.IsLogoReady = true;

                if (Fit.IsLogoReady){
                    ClassicalWindmillRecognizer::DrawRect(tempimage, Fit.ValidLogo.MinEnclosedRect,cv::Scalar(255,255,255), 3);
                    break;
                }
            }
        }


        recognizer.CheckWindmillTargetsByAngle(Fit.ValidLogo.ContourCenter, &windmillFan, &targetContours);

        ClassicalWindmillFan ValidTarget;
        for (unsigned int i = 0; i < targetContours.size(); i++)
        {
            if (targetContours[i].IsValid )
            {
                Fit.IsTargetReady = true;
                Fit.ValidTarget = targetContours[i];
                cv::circle(tempimage,Fit.ValidTarget.targetCenter,6,cv::Scalar(255,255,0),3);
            }
        }

//        std::cout <<  Fit.IsTargetReady << std::endl;

        cv::Point2f Pre_point;
        Eigen::Vector3d Pre_Coordinate;
        HuarayCameraModelParam modelParam = camera.GetParam().ModelParam;

        Fit.Run(angles ,w_Timestamps ,sin_datas , param , modelParam ,Pre_point,Pre_Coordinate );

        Pre_Coordinate = QMatrix * Pre_Coordinate;
        cv::circle(tempimage,Pre_point,6,cv::Scalar(255,0,0),3);

//        std::cout << " Pre_Coordinate  : " << std::endl << Pre_Coordinate << std::endl;
        std::cout << "logo = " << Fit.IsLogoReady << " target = " << Fit.IsTargetReady << " isfanok =  " << isfanok << std::endl;

        if (Fit.IsLogoReady && Fit.IsTargetReady)
            cv::imshow("pre_coordinate_image", tempimage);
        else
            cv::imshow("pre_coordinate_image", cameraData.Image);

        std::chrono::time_point<std::chrono::steady_clock> ed = std::chrono::steady_clock::now();
        uint64_t edtime = ed.time_since_epoch().count();

        std::cout << "**************************************************" << std::endl;
        std::cout << "frame index: " << frameIndex << std::endl;
        std::cout << "fps: " << (frameIndex * 1000000000) / (edtime - initTimestamp) << std::endl;
        std::cout << "process time: " << (edtime - bgtime) / 1000000 << "ms" << std::endl;
        std::cout << "**************************************************" << std::endl << std::endl;

        // 读取按键的ASCII码；注意：cv::waitKey()返回的是按键的ASCII码
        int keyValue = cv::waitKey(1);

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

//     关闭相机
    camera.Close();

    // 释放相机资源
    camera.Release();

    // 释放风车识别器资源
    recognizer.Release();

    // 记录截止时间戳
    std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
    uint64_t endTimestamp = endTime.time_since_epoch().count();

    // 输出处理结果
    std::cout << "**************************************************" << std::endl;
    std::cout << "frame index: " << frameIndex << std::endl;
    std::cout << "fps: " << (frameIndex * 1000000000) / (endTimestamp - initTimestamp) << std::endl;
    std::cout << "process time: " << (endTimestamp - initTimestamp) / 1000000 << "ms" << std::endl;
    std::cout << "**************************************************" << std::endl << std::endl;



    return 0;

}