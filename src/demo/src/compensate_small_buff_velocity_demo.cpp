//
// Created by plutoli on 2022-05-30.
//

#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_windmill_recognizer.h"
#include "solver.h"

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

    // 创建风车识别器
    ClassicalWindmillRecognizer recognizer;

    // 读取风车识别器参数
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

    // 创建目标解算器
    Solver solver;

    // 读取目标解算器参数
    SolverParam solverParam;
    std::string solverYaml = "config/infantry_3/basement/solver_param.yaml";
    if (!SolverParam::LoadFromYamlFile(solverYaml, &solverParam))
    {
        return -1;
    }

    // 设置目标解算器参数
    if (!solver.SetParam(solverParam))
    {
        return -1;
    }

    // 初始化目标解算器
    if (!solver.Init())
    {
        return -1;
    }

    // 初始化显示窗体
    cv::namedWindow("comparedFanImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("comparedFanImage", 800, 600);
    cv::namedWindow("lockedFanImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("lockedFanImage", 800, 600);

    // 初始化上一帧相机数据时间戳
    uint64_t previousFrameTimestamp = 0;

    // 循环处理相机数据
    while (true)
    {
        // 获取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 判断读取到的相机数据是否已经处理完毕
        if (cameraData.Timestamp > previousFrameTimestamp)
        {
            previousFrameTimestamp = cameraData.Timestamp;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 原始图像预处理
        cv::Mat binaryImage;
        recognizer.Preprocess(cameraData.Image, &binaryImage);

        // 检测风车Logo集合
        std::vector<ClassicalWindmillLogo> logos;
        recognizer.DetectWindmillLogos(binaryImage,
                                       cameraData.Image,
                                       &logos);

        // 检测风车扇叶集合
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

        // 初始化距离补偿和速度补偿；
        std::pair<float, float> distanceCompensation(0.0, 0.0);
        std::pair<float, float> velocityCompensation(0.0, 0.0);

        // 初始化Logo位置缓冲区和扇叶位置缓冲区
        std::vector<std::pair<cv::Point2f, uint64_t>> logoLocationSequence;
        std::vector<std::pair<cv::Point2f, uint64_t>> fanLocationSequence;

        // 初始化历史扇叶位置集合
        std::vector<cv::Point2f> fanCenters;

        // 初始化历史击打目标点集合
        std::vector<std::pair<cv::Point2f, uint64_t>> oldTargets;

        // 判断是否同时检测到合法的风车Logo和风车扇叶
        if (isLogoReady && isFanReady)
        {
            // 创建击打目标点
            cv::Point2f target = validFan.HeadContourCenter;

            // 更新Logo位置缓冲区和扇叶位置缓冲区
            recognizer.UpdateLogoBuffer(validLogo);
            recognizer.UpdateFanBuffer(validFan);

            // 获取风车Logo位置时序和风车扇叶位置时序
            recognizer.GetLogoLocationSequence(&logoLocationSequence);
            recognizer.GetFanLocationSequence(&fanLocationSequence);

            // 填充历史扇叶位置集合
            for (unsigned int i = 0; i < fanLocationSequence.size(); ++i)
            {
                fanCenters.emplace_back(fanLocationSequence[i].first);
            }

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

            // 计算风车扇叶的速度补偿
            solver.CompensateBuffVelocity(EWorkMode::ShootSmallBuff,
                                          logoLocation,
                                          fanLocationSequence,
                                          &velocityCompensation);

            // 对击打目标点进行速度补偿
            target.x += velocityCompensation.first;
            target.y += velocityCompensation.second;

            // 更新击打目标点数据缓冲区
            recognizer.UpdateTargetBuffer(target, cameraData.Timestamp);

            // 搜索匹配的历史击打目标点集合
            uint64_t prediceTime = solverParam.SmallBuffVelocityParameter.PredictTime * 1000000;
            uint64_t searchTimestamp = cameraData.Timestamp - prediceTime;
            uint64_t timestampOffset = 20000000;
            recognizer.GetTarget(searchTimestamp,
                                 timestampOffset,
                                 &oldTargets);
        }

        // 创建并显示锁定扇叶图像
        cv::Mat lockedFanImage;
        ClassicalWindmillRecognizer::CreateLockedFanImage(validFan,
                                                          fanCenters,
                                                          distanceCompensation,
                                                          velocityCompensation,
                                                          cameraData.Image,
                                                          &lockedFanImage);
        cv::imshow("lockedFanImage", lockedFanImage);


        // 创建并显示比较扇叶图像
        cv::Mat comparedFanImage;
        ClassicalWindmillRecognizer::CreateComparedFanImage(validFan,
                                                            oldTargets,
                                                            cameraData.Image,
                                                            &comparedFanImage);
        cv::imshow("comparedFanImage", comparedFanImage);

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

    // 关闭相机，释放相机资源
    camera.Close();
    camera.Release();

    // 释放风车识别器系统资源
    recognizer.Release();

    // 释放目标解算器系统资源
    solver.Release();

    return 0;
}