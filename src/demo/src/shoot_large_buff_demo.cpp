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

/**
 * @brief 机器人本体控制开关
 */
bool controlSwitch = false;

/**
 * @brief 机器人本体控制函数
 * @param[in] camera        华睿相机
 * @param[in] recognizer    大符识别器
 * @param[in] solver        目标解算器
 * @param[in] serialPort    通信串口
 * @param[out] image        图像数据
 */
void ControlBody(HuarayCamera *camera,
                 ClassicalWindmillRecognizer *recognizer,
                 Solver *solver,
                 SerialPort *serialPort,
                 cv::Mat *image)
{
    // 读取相机参数和目标解算器参数
    HuarayCameraParam cameraParam = camera->GetParam();
    SolverParam solverParam = solver->GetParam();

    // 记录初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
    uint64_t initTimestamp = initTime.time_since_epoch().count();

    // 记录上次击打时间戳
    std::chrono::time_point<std::chrono::steady_clock> previousHitTime = std::chrono::steady_clock::now();
    uint64_t previousHitTimestamp = previousHitTime.time_since_epoch().count();

    // 初始化数据帧索引和上一帧相机数据时间戳
    uint64_t frameIndex = 0;
    uint64_t previousFrameTimestamp = 0;

    // 循环处理相机图像
    while (controlSwitch)
    {
        // 记录起始时间戳
        std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
        uint64_t beginTimestamp = beginTime.time_since_epoch().count();

        // 获取相机数据
        HuarayCameraData cameraData;
        camera->GetData(&cameraData);

        // 判断读取到的相机数据是否已经处理完毕
        if (cameraData.Timestamp > previousFrameTimestamp)
        {
            previousFrameTimestamp = cameraData.Timestamp;
            frameIndex++;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 原始图像预处理
        cv::Mat binaryImage;
        recognizer->Preprocess(cameraData.Image, &binaryImage);

        // 检测风车Logo集合
        std::vector<ClassicalWindmillLogo> logos;
        recognizer->DetectWindmillLogos(binaryImage,
                                        cameraData.Image,
                                        &logos);

        // 检测风车扇叶集合
        std::vector<ClassicalWindmillFan> fans;
        recognizer->DetectWindmillFans(binaryImage, &fans);

        // 创建目标图像
        ClassicalWindmillRecognizer::CreateWindmillObjectsImage(logos,
                                                                fans,
                                                                cameraData.Image,
                                                                image);

        // 判断是否需要击打风车扇叶；大符击打的时间间隔为500ms
        std::chrono::time_point<std::chrono::steady_clock> currentHitTime = std::chrono::steady_clock::now();
        uint64_t currentHitTimestamp = currentHitTime.time_since_epoch().count();
        if ((currentHitTimestamp - previousHitTimestamp) < 500000000)
        {
            continue;
        }

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

        // 判断是否同时检测到合法的风车Logo和风车扇叶
        if (!(isLogoReady && isFanReady))
        {
            continue;
        }

        // 创建击打目标点
        cv::Point2f target = validFan.HeadContourCenter;

        // 更新Logo位置缓冲区和扇叶位置缓冲区
        recognizer->UpdateLogoBuffer(validLogo);
        recognizer->UpdateFanBuffer(validFan);

        // 获取风车Logo位置时序和风车扇叶位置时序
        std::vector<std::pair<cv::Point2f, uint64_t>> logoLocationSequence;
        std::vector<std::pair<cv::Point2f, uint64_t>> fanLocationSequence;
        recognizer->GetLogoLocationSequence(&logoLocationSequence);
        recognizer->GetFanLocationSequence(&fanLocationSequence);

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
        std::pair<float, float> velocityCompensation(0.0, 0.0);
        bool isVelocityCompensated = solver->CompensateBuffVelocity(EWorkMode::ShootLargeBuff,
                                                                    logoLocation,
                                                                    fanLocationSequence,
                                                                    &velocityCompensation);

        // 对击打目标点进行速度补偿
        target.x += velocityCompensation.first;
        target.y += velocityCompensation.second;

        // 使用速度补偿之后的击打目标点计算扇叶角度
        float angle;
        ClassicalWindmillRecognizer::ComputeWindmillFanAngle(logoLocation,
                                                             target,
                                                             &angle);

        // 使用扇叶角度计算距离补偿
        std::pair<float, float> distanceCompensation(0.0, 0.0);
        bool isDistanceCompensated = solver->CompensateBuffDistance(angle, &distanceCompensation);

        // 对击打目标点进行距离补偿
        target.x += distanceCompensation.first;
        target.y += distanceCompensation.second;

        // 如果补偿失败，继续处理视频
        if (!(isVelocityCompensated && isDistanceCompensated))
        {
            continue;
        }

        // 对补偿之后的击打目标点进行解算
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
        serialPort->Write(commandFrameSize, commandFrame);

        // 记录截止时间戳
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();

        // 更新上次击打时间戳
        previousHitTimestamp = endTimestamp;

        // 清空风车Logo位置缓冲区和风车扇叶位置缓冲区
        recognizer->ClearLogoBuffer();
        recognizer->ClearFanBuffer();

        // 计算帧率和数据处理时间
        uint64_t processTime = endTimestamp - beginTimestamp;
        uint64_t fps = (frameIndex * 1000000000) / (endTimestamp - initTimestamp);

        // 输出解算结果
        std::cout << "offset angle: " << offsetAngle.first << ", " << offsetAngle.second << std::endl;
        std::cout << "process time: " << processTime / 1000000 << "ms" << std::endl;
        std::cout << "fps: " << fps << std::endl << std::endl << std::endl;
    }
}

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

    // 初始化缓存图像和显示窗体
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::namedWindow("image", cv::WINDOW_NORMAL);
    cv::resizeWindow("image", 800, 600);

    // 设置机器人本体控制开关
    controlSwitch = true;

    // 启动机器人本体控制线程
    std::thread controlThread = std::thread(ControlBody,
                                            &camera,
                                            &recognizer,
                                            &solver,
                                            &serialPort,
                                            &image);

    // 循环处理相机数据
    while (true)
    {
        // 显示图像
        cv::imshow("image", image);

        // 读取按键的ASCII码；注意：cv::waitKey()返回的是按键的ASCII码
        int keyValue = cv::waitKey(20);

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

    // 等待机器人本体控制线程结束
    controlSwitch = false;
    if (controlThread.joinable())
    {
        controlThread.join();
    }

    // 关闭相机，释放相机资源
    camera.Close();
    camera.Release();

    // 释放装甲板识别器资源
    recognizer.Release();

    // 释放目标解算器资源
    solver.Release();

    // 关闭通信串口，释放串口资源
    serialPort.Close();
    serialPort.Release();
}