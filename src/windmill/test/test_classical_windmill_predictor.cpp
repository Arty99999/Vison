//
// Created by whales on 23-5-17.
//

#include <opencv2/opencv.hpp>
#include <chrono>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_windmill_recognizer.h"
#include "classical_windmill_predictor.h"
#include "predictor_tool.h"
#include "ceres/ceres.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    Fitting fitting_;
    fitting_.runeMode = Mode::RUNE;

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
    cv::namedWindow("rawImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("rawImage", 800, 600);
    cv::namedWindow("binaryImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("binaryImage", 800, 600);

    // 记录初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
    uint64_t initTimestamp = initTime.time_since_epoch().count();

    // 初始化数据帧索引和风车扇叶索引
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

        // 原始图像预处理
        cv::Mat binaryImageFan;
        cv::Mat binaryImageTarget;
        recognizer.Preprocess(cameraData.Image, &binaryImageFan, &binaryImageTarget);

        // 检测风车扇叶集合
        ClassicalWindmillFan windmillFan;
        std::vector<ClassicalWindmillFan> targetContours;
        recognizer.DetectWindmillFans(binaryImageFan, binaryImageTarget, cameraData.Image, &windmillFan, &targetContours);


        std::vector<ClassicalWindmillLogo> WindmillLogos;
        recognizer.DetectWindmillLogos(binaryImageFan, cameraData.Image, &WindmillLogos);

        ClassicalWindmillLogo ValidLogo;
        bool IsLogoReady = false;
        for (unsigned int i = 0; i < WindmillLogos.size(); i++)
        {
            if (WindmillLogos[i].IsValid)
            {
                IsLogoReady = true;
                ValidLogo = WindmillLogos[i];
                ValidLogo.Timestamp = cameraData.Timestamp;
                cv::circle(cameraData.Image, ValidLogo.ContourCenter, 3, cv::Scalar(0,0,0),3);
                cv::imshow("image", ValidLogo.Image);
                break;
            }
        }

        recognizer.CheckWindmillTargetsByAngle(ValidLogo.ContourCenter, &windmillFan, &targetContours);

        ClassicalWindmillFan ValidTarget;
        bool IsTargetReady = false;
        for (unsigned int i = 0; i < targetContours.size(); i++)
        {
            if (targetContours[i].IsValid)
            {
                IsTargetReady = true;
                ValidTarget = targetContours[i];
                cv::circle(cameraData.Image, ValidTarget.targetCenter, 3, cv::Scalar(255,255,255),3);
                break;
            }
        }

        if((!IsLogoReady) || (!IsTargetReady))
        {
            continue;
        }

        double n_x = ValidTarget.targetCenter.x - ValidLogo.ContourCenter.x;
        if(n_x > 0)
        {
            fitting_.IsPositive = true;
        }
        else
        {
            fitting_.IsPositive = false;
        }

        HuarayCameraModelParam modelParam = camera.GetParam().ModelParam;

        Eigen::Quaterniond Q_raw(1,0,0,0);
        Eigen::Matrix3d QMatrix = Q_raw.toRotationMatrix();
        Eigen::Vector3d euler_t = Q_raw.toRotationMatrix().eulerAngles(2,1,0);
//        std::cout<<" YAW: "<<euler_t(0)*57.3<<" Pitch: "<<euler_t(1)*57.3<<" Roll: "<<euler_t(2)*57.3<<std::endl;

        ValidTarget.Timestamp = cameraData.Timestamp;
        std::cout << "TimeStamp: " << cameraData.Timestamp;

        double distanceTarget = 7420;
        double distanceLogo = 7420;

        Eigen::Vector3d LogoCoordinate;
        PredictorTool::GetRelativeCorrdinate(distanceLogo,
                                             ValidLogo.ContourCenter,
                                             false,
                                             modelParam,
                                             &LogoCoordinate);

        Eigen::Vector3d WorldLogoCoordinate = QMatrix * LogoCoordinate;
        std::cout << "WorldCoordinate: " << WorldLogoCoordinate << std::endl;

        if(fitting_.IsJudgement)
        {
            Eigen::Vector3d RightUpperCoordinate;
            PredictorTool::GetRelativeCorrdinate(distanceLogo,
                                                 ValidLogo.LogoBoundingPoints[1],
                                                 false,
                                                 modelParam,
                                                 &RightUpperCoordinate);
            Eigen::Vector3d WorldRightUpperPoint = QMatrix * RightUpperCoordinate;
            fitting_.JudgeCoordinate(WorldRightUpperPoint, WorldLogoCoordinate);
            fitting_.IsJudgement = false;
        }

        Eigen::Vector3d TargetCoordinate;
        PredictorTool::GetRelativeCorrdinate(distanceTarget,
                                             ValidTarget.targetCenter,
                                             false,
                                             modelParam,
                                             &TargetCoordinate);
        Eigen::Vector3d WorldTargetCoordinate = QMatrix * TargetCoordinate;
        ValidTarget.EigenTargetCenter.x() = WorldTargetCoordinate.x();
        ValidTarget.EigenTargetCenter.y() = WorldTargetCoordinate.y();
        ValidTarget.EigenTargetCenter.z() = WorldTargetCoordinate.z();

        ValidTarget.Angle = fitting_.calAngle(ValidTarget.EigenTargetCenter, WorldLogoCoordinate);
        std::cout << "ValidTargetAngle: " << ValidTarget.Angle * 180 / M_PI << std::endl;

        // 判断是否为同一目标
        if((!fitting_.buffArmor_.empty()))
        {
            double OffDis = fitting_.calDistance(ValidTarget.EigenTargetCenter, fitting_.buffArmor_[fitting_.buffArmor_.size() - 1].EigenTargetCenter);
            if(OffDis > 1000)
            {
//            std::cout << "fittingData was cleared" << std::endl;
                fitting_.clearData();
            }
        }

        if(fitting_.buffArmor_.empty())
        {
            fitting_.armorStatus = EClassicalStatus::First;
        }
        else
        {
            fitting_.armorStatus = EClassicalStatus::Shoot;
        }
//    std::cout << "***************FITTING BEGIN*****************" << std::endl;
        fitting_.DataStatus(ValidTarget);

        Eigen::Vector3d predictWorldCoordinate;
        Eigen::Vector3d predictCradleHeadCoordinate;
        fitting_.run(ValidTarget, WorldLogoCoordinate, &predictWorldCoordinate);
//    std::cout << "PredictWorldCoordinate1: " << *predictWorldCoordinate << std::endl;
//    std::cout << "WorldLogoX: " << WorldLogoCoordinate.x() <<"  WorldLogoY: " << WorldLogoCoordinate.y() << "  WorldLogoZ: " << WorldLogoCoordinate.z() << std::endl;
//    *predictWorldCoordinate = WorldTargetCoordinate;
//    std::cout << "PredictWorldCoordinate2: " << WorldTargetCoordinate << std::endl;
//    std::cout << "****************FITTING END****************" << std::endl;

        predictCradleHeadCoordinate = QMatrix.inverse() * predictWorldCoordinate;

        cv::Point2f predictPixel;
        cv::Point2f LogoPixel;
        PredictorTool::GetPixelLocation(distanceTarget, &predictPixel, modelParam, predictCradleHeadCoordinate);
        PredictorTool::GetPixelLocation(distanceLogo, &LogoPixel, modelParam, LogoCoordinate);
        cv::circle(cameraData.Image, predictPixel, 5, cv::Scalar(255, 255, 255), 3);
        cv::circle(cameraData.Image, LogoPixel, 5, cv::Scalar(255,255,255),3);
        // 记录截止时间戳
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();

        // 输出处理结果
        std::cout << "frame index: " << frameIndex << std::endl;
        std::cout << "fps: " << (frameIndex * 1000000000) / (endTimestamp - initTimestamp) << std::endl;
        std::cout << "process time: " << (endTimestamp - beginTimestamp) / 1000000 << "ms" << std::endl;
        std::cout << "**************************************************" << std::endl << std::endl;

        fitting_.call_param();

        // 显示图像
        cv::imshow("rawImage", cameraData.Image);
        cv::imshow("binaryImageFan", binaryImageFan);
        cv::imshow("binaryImageTarget", binaryImageTarget);

//        cv::waitKey(0);
        // 读取按键的ASCII码；注意：cv::waitKey()返回的是按键的ASCII码
        int keyValue = cv::waitKey(5);

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
