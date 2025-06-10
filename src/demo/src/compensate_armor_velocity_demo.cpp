//
// Created by plutoli on 2022-05-30.
//

#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"
#include "solver.h"

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
    cv::namedWindow("lockedArmorImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("lockedArmorImage", 800, 600);
    cv::namedWindow("comparedArmorImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("comparedArmorImage", 800, 600);

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

        // 初始化锁定装甲板图像参数
        float distance = 0.0;
        std::vector<cv::Point2f> armorLocations;
        std::pair<float, float> distanceCompensation(0.0, 0.0);
        std::pair<float, float> velocityCompensation(0.0, 0.0);

        // 初始化锁定装甲板图像
        cv::Mat lockedArmorImage;
        cameraData.Image.copyTo(lockedArmorImage);

        // 判断是否检测到目标装甲板
        bool isTargetArmorReady = false;
        ClassicalArmor targetArmor;
        for (unsigned int i = 0; i < polishedArmors.size(); ++i)
        {
            if (polishedArmors[i].Number == EClassicalArmorNumber::Two)
            {
                isTargetArmorReady = true;
                targetArmor = polishedArmors[i];
                targetArmor.Timestamp = cameraData.Timestamp;
                break;
            }
        }

        // 补偿装甲板的运动速度
        if (isTargetArmorReady)
        {
            // 更新装甲板数据缓冲区
            recognizer.UpdateArmorBuffer(targetArmor);

            // 获取装甲板的位置时序
            std::vector<std::pair<cv::Point2f, uint64_t>> armorLocationSequence;
            recognizer.GetArmorLocationSequence(targetArmor.Number,
                                                &armorLocationSequence);

            // 计算速度补偿，预测未来100毫秒的装甲板的位置偏差
            if (solver.CompensateArmorVelocity(EWorkMode::AutomaticShoot,
                                               armorLocationSequence,
                                               100.0,
                                               &velocityCompensation))
            {
                cv::Point2f compensatedTarget;
                compensatedTarget.x = targetArmor.Center.x + velocityCompensation.first;
                compensatedTarget.y = targetArmor.Center.y + velocityCompensation.second;
                recognizer.UpdateTargetBuffer(targetArmor.Number,
                                              compensatedTarget,
                                              targetArmor.Timestamp);
            }
        }

        // 创建并显示锁定装甲板图像
        ClassicalArmorRecognizer::CreateLockedArmorImage(targetArmor,
                                                         distance,
                                                         armorLocations,
                                                         distanceCompensation,
                                                         velocityCompensation,
                                                         cameraData.Image,
                                                         &lockedArmorImage);
        cv::imshow("lockedArmorImage", lockedArmorImage);

        // 获取比较装甲板的历史目标
        std::vector<std::pair<cv::Point2f, uint64_t>> oldTargets;
        uint64_t searchTimestamp = targetArmor.Timestamp - 100000000;
        uint64_t timestampOffset = 20000000;
        recognizer.GetTarget(targetArmor.Number,
                             searchTimestamp,
                             timestampOffset,
                             &oldTargets);

        // 创建并显示比较装甲板图像
        cv::Mat comparedArmorImage;
        ClassicalArmorRecognizer::CreateComparedArmorImage(targetArmor,
                                                           oldTargets,
                                                           cameraData.Image,
                                                           &comparedArmorImage);
        cv::imshow("comparedArmorImage", comparedArmorImage);

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

    // 释放装甲板识别器系统资源
    recognizer.Release();

    // 释放目标解算器系统资源
    solver.Release();

    return 0;
}