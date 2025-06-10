//
// Created by plutoli on 2021/10/3.
//

#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"
#include "solver.h"
#include "serial_port.h"
#include "robot_brain_control_command.h"

// 按下Enter键，启动/暂停系统
// 按下ESC键，退出系统
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

    // 创建通信串口
    SerialPort serialPort;

    // 读取通信串口参数
    SerialPortParam serialPortParam;
    std::string serialPortYaml = "config/infantry_3/basement/serial_port_param.yaml";
    if (!SerialPortParam::LoadFromYamlFile(serialPortYaml, &serialPortParam))
    {
        return -1;
    }

    // 设置通信串口参数
    if (!serialPort.SetParam(serialPortParam))
    {
        return -1;
    }

    // 初始化通信串口
    if (!serialPort.Init())
    {
        return -1;
    }

    // 打开通信串口
    if (!serialPort.Open())
    {
        return -1;
    }

    // 初始化显示窗体
    cv::namedWindow("rangedArmorImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("rangedArmorImage", 800, 600);

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
            if (polishedArmors[i].Number != EClassicalArmorNumber::Invalid)
            {
                isTargetArmorReady = true;
                targetArmor = polishedArmors[i];
                targetArmor.Timestamp = cameraData.Timestamp;
                break;
            }
        }

        // 创建目标装甲板集合和距离集合
        std::vector<ClassicalArmor> targetArmors;
        std::vector<float> distances;

        // 如果检测到目标装甲板，则发送机器人本体控制指令
        if (isTargetArmorReady)
        {
            // 初始化装甲板四个角点的世界坐标
            std::vector<cv::Point3f> objectPoints;
            if((targetArmor.Number == EClassicalArmorNumber::One) ||
               (targetArmor.Number == EClassicalArmorNumber::Sentry))
            {
                // 获取大装甲板的宽度和高度
                float width = recognizerParam.LargeArmorPhysicalWidth - 2 * recognizerParam.LightBarPhysicalWidth;
                float height = recognizerParam.LargeArmorPhysicalHeight;
                objectPoints.emplace_back(-width / 2,-height / 2, 0.0);
                objectPoints.emplace_back(width / 2, -height / 2, 0.0);
                objectPoints.emplace_back(width / 2, height / 2,  0.0);
                objectPoints.emplace_back(-width / 2,height / 2,  0.0);
            }
            else
            {
                // 获取小装甲板的宽度和高度
                float width = recognizerParam.SmallArmorPhysicalWidth - 2 * recognizerParam.LightBarPhysicalWidth;
                float height = recognizerParam.SmallArmorPhysicalHeight;
                objectPoints.emplace_back(-width / 2,-height / 2, 0.0);
                objectPoints.emplace_back(width / 2, -height / 2, 0.0);
                objectPoints.emplace_back(width / 2, height / 2,  0.0);
                objectPoints.emplace_back(-width / 2,height / 2,  0.0);
            }

            // 初始化装甲板四个角点的像素坐标
            std::vector<cv::Point2f> pixelPoints;
            pixelPoints.emplace_back(targetArmor.LeftUpper);
            pixelPoints.emplace_back(targetArmor.RightUpper);
            pixelPoints.emplace_back(targetArmor.RightLower);
            pixelPoints.emplace_back(targetArmor.LeftLower);

            // 解算目标装甲板到枪口的距离
            float distance = Solver::ComputeDistance(objectPoints, pixelPoints, cameraParam.ModelParam).first;

            // 计算击打目标点的距离补偿
            std::pair<float, float> compensation(0.0, 0.0);
            bool isCompensated = solver.CompensateArmorDistance(EWorkMode::AutomaticShoot,
                                                                EBulletVelocity::MPS_15,
                                                                distance,
                                                                &compensation);

            // 计算补偿之后的击打目标点
            cv::Point2f target = targetArmor.Center;
            target.x += compensation.first;
            target.y += compensation.second;

            // 对补偿之后的击打目标点进行解算
            std::pair<float, float> offsetAngle;
            Solver::Solve(distance,
                          target,
                          false,
                          cameraParam.ModelParam,
                          &offsetAngle);

            // 初始化控制指令
            RobotBrainControlCommand command;
            command.ID = 1;
            command.Yaw = offsetAngle.first;
            command.Pitch = offsetAngle.second;
            command.Distance = distance;

            // 生成控制指令数据帧
            unsigned int commandFrameSize = RobotBrainControlCommand::GetFrameSize();
            unsigned char commandFrame[commandFrameSize];
            command.EncapsulateToFrame(commandFrame);

            // 将指令数据帧发送给下位机
            serialPort.Write(commandFrameSize, commandFrame);

            // 填充目标装甲板集合和距离集合
            targetArmors.emplace_back(targetArmor);
            distances.emplace_back(distance);

            // 输出解算结果
            std::cout << "****************************************" << std::endl;
            std::cout << "is compensated: " << isCompensated << std::endl;
            std::cout << "comensation: [" << compensation.first << "," << compensation.second << "]" << std::endl;
            std::cout << "distance: " << distance << std::endl;
            std::cout << "target: [" << target.x << ", " << target.y << "]" << std::endl;
            std::cout << "offset angle: [" << offsetAngle.first << ", " << offsetAngle.second << "]" << std::endl;
            std::cout << "****************************************" << std::endl << std::endl;
        }

        // 创建并显示测距装甲板图像
        cv::Mat rangedArmorImage;
        ClassicalArmorRecognizer::CreateRangedArmorImage(targetArmors,
                                                         distances,
                                                         cameraData.Image,
                                                         &rangedArmorImage);
        cv::imshow("rangedArmorImage",rangedArmorImage);

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

    // 释放装甲板识别器资源
    recognizer.Release();

    // 释放目标解算器资源
    solver.Release();

    // 关闭通信窗口，释放通信串口资源
    serialPort.Close();
    serialPort.Release();

    return 0;
}