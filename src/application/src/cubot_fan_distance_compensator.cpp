//
// Created by plutoli on 2022-05-29.
//

#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_windmill_recognizer.h"
#include "solver.h"
#include "serial_port.h"
#include "robot_brain_control_command.h"

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
    if (!ClassicalWindmillRecognizerParam::LoadFromYamlFile(recognizerYaml,
                                                            &recognizerParam))
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
    cv::namedWindow("windmillAnglesImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("windmillAnglesImage", 800, 600);

    // 显示临时图像
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("windmillAnglesImage", image);

    // 循环处理相机数据
    while (true)
    {
        // 获取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

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

        // 创建风车扇叶中心点集合
        std::vector<cv::Point2f> fanCenters;

        // 如果同时检测到合法的风车Logo和风车扇叶，则发送机器人本体控制指令
        if (isFanReady && isLogoReady)
        {
            // 计算Logo中心点到扇叶中心点的方向矢量与x轴正方向的夹角；取值范围为[0°, 360°)
            float angle;
            ClassicalWindmillRecognizer::ComputeWindmillFanAngle(validLogo.ContourCenter,
                                                                 validFan.HeadContourCenter,
                                                                 &angle);

            // 计算击打目标点的距离补偿
            std::pair<float, float> distanceCompensation(0.0, 0.0);
            solver.CompensateBuffDistance(angle, &distanceCompensation);

            // 计算补偿之后的击打目标点
            cv::Point2f target = validFan.HeadContourCenter;
            target.x += distanceCompensation.first;
            target.y += distanceCompensation.second;

            // 对击打目标点进行解算
            float distance = 7000.0;
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

            // 输出解算结果
            std::cout << "****************************************" << std::endl;
            std::cout << "target: [" << target.x << ", " << target.y << "]" << std::endl;
            std::cout << "offset angle: [" << offsetAngle.first << ", " << offsetAngle.second << "]" << std::endl;
            std::cout << "****************************************" << std::endl << std::endl;

            // 填充风车扇叶中心点集合
            fanCenters.emplace_back(validFan.HeadContourCenter);
        }

        // 创建并显示风车角度图像
        cv::Mat windmillAnglesImage;
        ClassicalWindmillRecognizer::CreateWindmillAnglesImage(validLogo.ContourCenter,
                                                               fanCenters,
                                                               cameraData.Image,
                                                               &windmillAnglesImage);
        cv::imshow("windmillAnglesImage", windmillAnglesImage);

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

    // 释放风车识别器资源
    recognizer.Release();

    // 释放目标解算器资源
    solver.Release();

    // 关闭通信窗口，释放通信串口资源
    serialPort.Close();
    serialPort.Release();

    return 0;
}