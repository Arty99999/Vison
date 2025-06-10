//
// Created by plutoli on 2022-06-05.
//

#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"
#include "solver.h"
#include "serial_port.h"
#include "robot_brain_control_command.h"

/**
 * @brief 机器人本体控制开关
 */
bool controlSwitch = false;

/**
 * @brief 机器人本体控制函数
 * @param[in] workMode          工作模式
 * @param[in] bulletVelocity    子弹速度
 * @param[in] targetArmorNumber 目标装甲板编号
 * @param[in] id                控制编号
 * @param[in] camera            华睿相机
 * @param[in] recognizer        装甲板识别器
 * @param[in] solver            目标解算器
 * @param[in] serialPort        通信串口
 * @param[out] image            图像数据
 */
void ControlBody(EWorkMode workMode,
                 EBulletVelocity bulletVelocity,
                 EClassicalArmorNumber targetArmorNumber,
                 int id,
                 HuarayCamera *camera,
                 ClassicalArmorRecognizer *recognizer,
                 Solver *solver,
                 SerialPort *serialPort,
                 cv::Mat *image)
{
    // 记录初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
    uint64_t initTimestamp = initTime.time_since_epoch().count();

    // 获取相机参数和识别器参数
    HuarayCameraParam cameraParam = camera->GetParam();
    ClassicalArmorRecognizerParam recognizerParam = recognizer->GetParam();

    // 初始化数据帧索引和上一帧相机数据时间戳
    uint64_t frameIndex = 0;
    uint64_t commandIndex = 0;
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

        // 检测粗灯条
        std::vector<ClassicalLightBar> rawLightBars;
        recognizer->DetectRawLightBars(binaryImage, &rawLightBars);

        // 检测精灯条
        std::vector<ClassicalLightBar> polishedLightBars;
        recognizer->DetectPolishedLightBars(rawLightBars, &polishedLightBars);

        // 检测灯条对
        std::vector<std::pair<ClassicalLightBar, ClassicalLightBar>> lightBarPairs;
        recognizer->DetectLightBarPairs(polishedLightBars, &lightBarPairs);

        // 检测粗装甲板
        std::vector<ClassicalArmor> rawArmors;
        recognizer->DetectRawArmors(lightBarPairs, cameraData.Image, &rawArmors);

        // 检测精装甲板
        std::vector<ClassicalArmor> polishedArmors;
        recognizer->DetectPolishedArmors(rawArmors, &polishedArmors);

        // 创建精装甲板图像
        ClassicalArmorRecognizer::CreateCommonArmorsImage(polishedArmors,
                                                          cameraData.Image,
                                                          image);

        // 判断是否检测到目标装甲板
        bool isTargetArmorReady = false;
        ClassicalArmor targetArmor;
        for (unsigned int i = 0; i < polishedArmors.size(); ++i)
        {
            if (polishedArmors[i].Number == targetArmorNumber)
            {
                isTargetArmorReady = true;
                targetArmor = polishedArmors[i];
                targetArmor.Timestamp = cameraData.Timestamp;
                break;
            }
        }

        // 如果没有检测到目标装甲板，继续获取并处理图像
        if (!isTargetArmorReady)
        {
            continue;
        }

        // 创建击打目标点
        cv::Point2f target = targetArmor.Center;

        // 更新装甲板数据缓冲区
        recognizer->UpdateArmorBuffer(targetArmor);

        // 获取装甲板的位置时序
        std::vector<std::pair<cv::Point2f, uint64_t>> armorLocationSequence;
        recognizer->GetArmorLocationSequence(targetArmorNumber,
                                             &armorLocationSequence);

        // 计算速度补偿，预测未来100毫秒的装甲板的位置偏差
        std::pair<float, float> velocityCompensation(0.0, 0.0);
        bool isVelocityCompensated = solver->CompensateArmorVelocity(workMode,
                                                                     armorLocationSequence,
                                                                     100.0,
                                                                     &velocityCompensation);

        // 对击打目标点进行速度补偿
        target.x += velocityCompensation.first;
        target.y += velocityCompensation.second;

        // 计算目标装甲板顶点的世界坐标
        std::vector<cv::Point3f> objectPoints;
        if(targetArmor.Number == EClassicalArmorNumber::One)
        {
            // 根据大装甲板的宽度和高度，计算目标装甲板顶点的世界坐标
            float width = recognizerParam.LargeArmorPhysicalWidth - 2 * recognizerParam.LightBarPhysicalWidth;
            float height = recognizerParam.LargeArmorPhysicalHeight;
            objectPoints.emplace_back(-width / 2,-height / 2, 0.0);
            objectPoints.emplace_back(width / 2, -height / 2, 0.0);
            objectPoints.emplace_back(width / 2, height / 2,  0.0);
            objectPoints.emplace_back(-width / 2,height / 2,  0.0);
        }
        else
        {
            // 根据小装甲板的宽度和高度，计算目标装甲板顶点的世界坐标
            float width = recognizerParam.SmallArmorPhysicalWidth - 2 * recognizerParam.LightBarPhysicalWidth;
            float height = recognizerParam.SmallArmorPhysicalHeight;
            objectPoints.emplace_back(-width / 2,-height / 2, 0.0);
            objectPoints.emplace_back(width / 2, -height / 2, 0.0);
            objectPoints.emplace_back(width / 2, height / 2,  0.0);
            objectPoints.emplace_back(-width / 2,height / 2,  0.0);
        }

        // 计算目标装甲板顶点的像素坐标
        std::vector<cv::Point2f> pixelPoints;
        pixelPoints.emplace_back(targetArmor.LeftUpper);
        pixelPoints.emplace_back(targetArmor.RightUpper);
        pixelPoints.emplace_back(targetArmor.RightLower);
        pixelPoints.emplace_back(targetArmor.LeftLower);

        // 计算目标装甲板到相机的距离
        float distance = Solver::ComputeDistance(objectPoints, pixelPoints, cameraParam.ModelParam);

        // 计算距离补偿
        std::pair<float, float> distanceCompensation(0.0, 0.0);
        bool isDistanceCompensated = solver->CompensateArmorDistance(workMode,
                                                                     bulletVelocity,
                                                                     distance,
                                                                     &distanceCompensation);

        // 对击打目标点进行距离补偿
        target.x += distanceCompensation.first;
        target.y += distanceCompensation.second;

        // 对目标点进行解算
        std::pair<float, float> offsetAngle;
        Solver::Solve(distance,
                      target,
                      false,
                      cameraParam.ModelParam,
                      &offsetAngle);

        // 初始化控制指令
        RobotBrainControlCommand command;
        command.ID = id;
        command.Yaw = offsetAngle.first;
        command.Pitch = offsetAngle.second;
        command.Distance = distance;

        // 生成控制指令数据帧
        unsigned int commandFrameSize = RobotBrainControlCommand::GetFrameSize();
        unsigned char commandFrame[commandFrameSize];
        command.EncapsulateToFrame(commandFrame);

        // 将指令数据帧发送给下位机
        serialPort->Write(commandFrameSize, commandFrame);

        // 指令索引累加
        commandIndex++;

        // 记录截止时间戳
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();

        // 计算数据获取帧率和控制指令帧率
        uint64_t frameFps = (frameIndex * 1000000000) / (endTimestamp - initTimestamp);
        uint64_t commandFps = (commandIndex * 1000000000) / (endTimestamp - initTimestamp);

        // 输出处理结果
        std::cout << "**************************************************" << std::endl;
        std::cout << "id: " << id << std::endl;
        std::cout << "frame index: " << frameIndex << std::endl;
        std::cout << "frame fps: " << frameFps << std::endl;
        std::cout << "command index: " << commandIndex << std::endl;
        std::cout << "command fps: " << commandFps << std::endl;
        std::cout << "distance: " << distance << std::endl;
        std::cout << "is velocity compensated: " << isVelocityCompensated << std::endl;
        std::cout << "velocity comensation: [" << velocityCompensation.first << "," << velocityCompensation.second << "]" << std::endl;
        std::cout << "is distance compensated: " << isDistanceCompensated << std::endl;
        std::cout << "distance comensation: [" << distanceCompensation.first << "," << distanceCompensation.second << "]" << std::endl;
        std::cout << "target: [" << target.x << ", " << target.y << "]" << std::endl;
        std::cout << "offset angle: " << offsetAngle.first << ", " << offsetAngle.second << std::endl;
        std::cout << "process time: " << (endTimestamp - beginTimestamp) / 1000000 << "ms" << std::endl;
        std::cout << "**************************************************" << std::endl << std::endl;
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
    HuarayCamera camera1;
    HuarayCamera camera2;

    // 读取相机参数
    HuarayCameraParam cameraParam1;
    HuarayCameraParam cameraParam2;
    std::string cameraYaml1 = "config/sentry/basement/huaray_camera_param_1.yaml";
    std::string cameraYaml2 = "config/sentry/basement/huaray_camera_param_2.yaml";
    if (!HuarayCameraParam::LoadFromYamlFile(cameraYaml1, &cameraParam1))
    {
        return -1;
    }
    if (!HuarayCameraParam::LoadFromYamlFile(cameraYaml2, &cameraParam2))
    {
        return -1;
    }

    // 设置相机参数
    if (!camera1.SetParam(cameraParam1))
    {
        return -1;
    }
    if (!camera2.SetParam(cameraParam2))
    {
        return -1;
    }

    // 初始化相机
    if (!camera1.Init())
    {
        return -1;
    }
    if (!camera2.Init())
    {
        return -1;
    }

    // 打开相机
    if (!camera1.Open())
    {
        return -1;
    }
    if (!camera2.Open())
    {
        return -1;
    }

    // 创建装甲板识别器
    ClassicalArmorRecognizer recognizer1;
    ClassicalArmorRecognizer recognizer2;

    // 读取装甲板识别器参数
    ClassicalArmorRecognizerParam recognizerParam1;
    ClassicalArmorRecognizerParam recognizerParam2;
    std::string recognizerYaml1 = "config/sentry/basement/classical_armor_recognizer_param_1.yaml";
    std::string recognizerYaml2 = "config/sentry/basement/classical_armor_recognizer_param_2.yaml";
    if (!ClassicalArmorRecognizerParam::LoadFromYamlFile(recognizerYaml1,
                                                         &recognizerParam1))
    {
        return -1;
    }
    if (!ClassicalArmorRecognizerParam::LoadFromYamlFile(recognizerYaml2,
                                                         &recognizerParam2))
    {
        return -1;
    }

    // 设置装甲板识别器参数
    if (!recognizer1.SetParam(recognizerParam1))
    {
        return -1;
    }
    if (!recognizer2.SetParam(recognizerParam2))
    {
        return -1;
    }

    // 初始化装甲板识别器
    if (!recognizer1.Init())
    {
        return -1;
    }
    if (!recognizer2.Init())
    {
        return -1;
    }

    // 创建目标解算器
    Solver solver1;
    Solver solver2;

    // 读取目标解算器参数
    SolverParam solverParam1;
    SolverParam solverParam2;
    std::string solverYaml1 = "config/sentry/basement/solver_param_1.yaml";
    std::string solverYaml2 = "config/sentry/basement/solver_param_2.yaml";
    if (!SolverParam::LoadFromYamlFile(solverYaml1, &solverParam1))
    {
        return -1;
    }
    if (!SolverParam::LoadFromYamlFile(solverYaml2, &solverParam2))
    {
        return -1;
    }

    // 设置目标解算器参数
    if (!solver1.SetParam(solverParam1))
    {
        return -1;
    }
    if (!solver2.SetParam(solverParam2))
    {
        return -1;
    }

    // 初始化目标解算器
    if (!solver1.Init())
    {
        return -1;
    }
    if (!solver2.Init())
    {
        return -1;
    }

    // 创建通信串口
    SerialPort serialPort;

    // 读取通信串口参数
    SerialPortParam serialPortParam;
    std::string serialPortYaml = "config/sentry/basement/serial_port_param.yaml";
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
    cv::Mat image1(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::Mat image2(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::namedWindow("image1", cv::WINDOW_NORMAL);
    cv::resizeWindow("image1", 800, 600);
    cv::namedWindow("image2", cv::WINDOW_NORMAL);
    cv::resizeWindow("image2", 800, 600);

    // 设置机器人本体控制开关
    controlSwitch = true;

    // 启动机器人本体控制线程1
    std::thread controlThread1 = std::thread(ControlBody,
                                             EWorkMode::AutomaticShoot,
                                             EBulletVelocity::MPS_30,
                                             EClassicalArmorNumber::Two,
                                             1,
                                             &camera1,
                                             &recognizer1,
                                             &solver1,
                                             &serialPort,
                                             &image1);

    // 启动机器人本体控制线程2
    std::thread controlThread2 = std::thread(ControlBody,
                                             EWorkMode::AutomaticShoot,
                                             EBulletVelocity::MPS_30,
                                             EClassicalArmorNumber::Two,
                                             2,
                                             &camera2,
                                             &recognizer2,
                                             &solver2,
                                             &serialPort,
                                             &image2);

    // 循环处理相机数据
    while (true)
    {
        // 显示图像
        cv::imshow("image1", image1);
        cv::imshow("image2", image2);

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

    // 重置机器人本体控制开关
    controlSwitch = false;

    // 等待机器人本体控制线程1结束
    if (controlThread1.joinable())
    {
        controlThread1.join();
    }

    // 等待机器人本体控制线程2结束
    if (controlThread2.joinable())
    {
        controlThread2.join();
    }

    // 关闭相机，释放相机资源
    camera1.Close();
    camera2.Close();
    camera1.Release();
    camera2.Release();

    // 释放装甲板识别器资源
    recognizer1.Release();
    recognizer2.Release();

    // 释放目标解算器资源
    solver1.Release();
    solver2.Release();

    // 关闭通信串口，释放串口资源
    serialPort.Close();
    serialPort.Release();
}