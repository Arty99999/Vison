//
// Created by plutoli on 2021/9/15.
//

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"

// 按ESC键，退出系统
// 按Enter键，保存样本
// 按其它键，继续处理
int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 初始化日志信息
    std::string log;

    // 创建相机
   // HuarayCamera camera;
//
   // // 读取相机参数
   // HuarayCameraParam cameraParam;
   // std::string cameraYaml = "config/infantry_3/basement/huaray_camera_param.yaml";
   // if (!HuarayCameraParam::LoadFromYamlFile(cameraYaml, &cameraParam))
   // {
   //     return -1;
   // }
//
   // // 设置相机参数
   // if (!camera.SetParam(cameraParam))
   // {
   //     return -1;
   // }
//
   // // 初始化相机
   // if (!camera.Init())
   // {
   //     return -1;
   // }
//
   // // 打开相机
   // if (!camera.Open())
   // {
   //     return -1;
   // }

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

    // 初始化样本存储路径，以“/”结束
    std::string samplePath = "armor/";
    std::string sampleInvalidPath = "/home/cubot/cubot_brain_1.3/cubot_brain_2022-06-24/data/sample/armor/2022.12.12/Invalid/";
    std::string sampleOnePath = "/home/cubot/cubot_brain_1.3/cubot_brain_2022-06-24/data/sample/armor/2022.12.12/One/";
    std::string sampleTwoPath = "/home/cubot/cubot_brain_1.3/cubot_brain_2022-06-24/data/sample/armor/2022.12.12/Two/";
    std::string sampleThreePath = "/home/cubot/cubot_brain_1.3/cubot_brain_2022-06-24/data/sample/armor/2022.12.12/Three/";
    std::string sampleFourPath = "/home/cubot/cubot_brain_1.3/cubot_brain_2022-06-24/data/sample/armor/2022.12.12/Four/";
    std::string sampleFivePath = "/home/cubot/cubot_brain_1.3/cubot_brain_2022-06-24/data/sample/armor/2022.12.12/Five/";
    std::string sampleSentryPath = "armor/sentry/";
    std::string sampleOutpostPath = "armor/outpost/";
    std::string sampleBasePath = "armor/base/";

    // 创建samplePath
    if (::access(samplePath.c_str(), F_OK) == -1)
    {
        if (::mkdir(samplePath.c_str(), 07777) == 0)
        {
            log = "[" + samplePath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + samplePath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + samplePath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleInvalidPath
    if (::access(sampleInvalidPath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleInvalidPath.c_str(), 07777) == 0)
        {
            log = "[" + sampleInvalidPath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + sampleInvalidPath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + sampleInvalidPath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleOnePath
    if (::access(sampleOnePath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleOnePath.c_str(), 07777) == 0)
        {
            log = "[" + sampleOnePath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + sampleOnePath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + sampleOnePath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleTwoPath
    if (::access(sampleTwoPath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleTwoPath.c_str(), 07777) == 0)
        {
            log = "[" + sampleTwoPath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + sampleTwoPath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + sampleTwoPath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleThreePath
    if (::access(sampleThreePath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleThreePath.c_str(), 07777) == 0)
        {
            log = "[" + sampleThreePath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + sampleThreePath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + sampleThreePath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleFourPath
    if (::access(sampleFourPath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleFourPath.c_str(), 07777) == 0)
        {
            log = "[" + sampleFourPath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + sampleFourPath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + sampleFourPath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleFivePath
    if (::access(sampleFivePath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleFivePath.c_str(), 07777) == 0)
        {
            log = "[" + sampleFivePath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + sampleFivePath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + sampleFivePath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleSentryPath
    if (::access(sampleSentryPath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleSentryPath.c_str(), 07777) == 0)
        {
            log = "[" + sampleSentryPath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + sampleSentryPath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + sampleSentryPath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleOutpostPath
    if (::access(sampleOutpostPath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleOutpostPath.c_str(), 07777) == 0)
        {
            log = "[" + sampleOutpostPath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + sampleOutpostPath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + sampleOutpostPath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建sampleBasePath
    if (::access(sampleBasePath.c_str(), F_OK) == -1)
    {
        if (::mkdir(sampleBasePath.c_str(), 07777) == 0)
        {
            log = "[" + sampleBasePath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + sampleBasePath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + sampleBasePath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 初始化显示窗体
    cv::namedWindow("rawImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("rawImage", 800, 600);
    cv::namedWindow("polishedArmorsImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("polishedArmorsImage", 800, 600);
    cv::namedWindow("sampleInvalid", cv::WINDOW_NORMAL);
    cv::resizeWindow("sampleInvalid", 50, 50);
    cv::namedWindow("sampleOne", cv::WINDOW_NORMAL);
    cv::resizeWindow("sampleOne", 50, 50);
    cv::namedWindow("sampeTwo", cv::WINDOW_NORMAL);
    cv::resizeWindow("sampeTwo", 50, 50);
    cv::namedWindow("sampleThree", cv::WINDOW_NORMAL);
    cv::resizeWindow("sampleThree", 50, 50);
    cv::namedWindow("sampleFour", cv::WINDOW_NORMAL);
    cv::resizeWindow("sampleFour", 50, 50);
    cv::namedWindow("sampleFive",cv::WINDOW_NORMAL);
    cv::resizeWindow("sampleFive", 50, 50);
    cv::namedWindow("sampleSentry",cv::WINDOW_NORMAL);
    cv::resizeWindow("sampleSentry", 50, 50);
    cv::namedWindow("sampleOutpost",cv::WINDOW_NORMAL);
    cv::resizeWindow("sampleOutpost", 50, 50);
    cv::namedWindow("sampleBase",cv::WINDOW_NORMAL);
    cv::resizeWindow("sampleBase", 50, 50);

    // 显示临时图像
    cv::Mat bigImage(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::Mat smallImage(50, 50, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("rawImage", bigImage);
    cv::imshow("polishedArmorsImage", bigImage);
    cv::imshow("sampleInvalid", smallImage);
    cv::imshow("sampleOne", smallImage);
    cv::imshow("sampeTwo", smallImage);
    cv::imshow("sampleThree", smallImage);
    cv::imshow("sampleFour", smallImage);
    cv::imshow("sampleFive", smallImage);
    cv::imshow("sampleSentry", smallImage);
    cv::imshow("sampleOutpost", smallImage);
    cv::imshow("sampleBase", smallImage);

    // 初始化样本索引
    int sampleInvalidIndex = 0;
    int sampleOneIndex = 0;
    int sampleTwoIndex = 0;
    int sampleThreeIndex = 1080;
    int sampleFourIndex = 2325;
    int sampleFiveIndex = 0;
    int sampleSentryIndex = 0;
    int sampleOutpostIndex = 0;
    int sampleBaseIndex = 0;
    int count = 0;
    cv::VideoCapture c("/home/cubot/MVviewer/videos/A5131CU210_BA12121AAK00034/Video_2022_12_15_163138_1.avi");
    // 打印提示信息
    log = "Press 'ESC' to eixt. Press 'Enter' to save samples. Press other key to continue: ";
    logger.Save(ELogType::Info, log);
    // 读取并处理视频
    while (true)
    {
        count++;
        // 读取相机数据
       // HuarayCameraData cameraData;
       // camera.GetData(&cameraData);
        cv::Mat srcImage;
        c>>srcImage;
        // 原始图像预处理
        cv::Mat binaryImage;
        recognizer.Preprocess(srcImage, &binaryImage);

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
        recognizer.DetectRawArmors(lightBarPairs, srcImage, &rawArmors);

        // 检测精装甲板
        std::vector<ClassicalArmor> polishedArmors;
        recognizer.DetectPolishedArmors(rawArmors, &polishedArmors);

        // 创建精装甲板图像
        cv::Mat polishedArmorsImage;
        ClassicalArmorRecognizer::CreateCommonArmorsImage(polishedArmors,
                                                          srcImage,
                                                          &polishedArmorsImage);

        if (count!=5)
        {
            continue;
        }
        count=0;

        // 保存样本图片
        for (unsigned int i = 0; i < polishedArmors.size(); ++i)
        {
            // 保存无效装甲板图片
            if (polishedArmors[i].Number == EClassicalArmorNumber::Invalid)
            {
                // 显示图片
                cv::imshow("sampleInvalid", polishedArmors[i].Image);

                // 索引累加
                sampleInvalidIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "invalid_" << std::setw(5) << std::setfill('0') << sampleInvalidIndex << ".bmp";
                std::string fileName = sampleInvalidPath + ss.str();
                cv::imwrite(fileName, polishedArmors[i].Image);

                // 记录日志信息
                log = "Invalids[" + std::to_string(sampleInvalidIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }

            // 保存1号装甲板图片
            if (polishedArmors[i].Number == EClassicalArmorNumber::One)
            {
                // 显示图片
                cv::imshow("sampleOne", polishedArmors[i].Image);

                // 索引累加
                sampleOneIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "one_" << std::setw(5) << std::setfill('0') << sampleOneIndex << ".bmp";
                std::string fileName = sampleOnePath + ss.str();
                cv::imwrite(fileName, polishedArmors[i].Image);

                // 记录日志信息
                log = "Ones[" + std::to_string(sampleOneIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }

            // 保存2号装甲板图片
            if(polishedArmors[i].Number == EClassicalArmorNumber::Two)
            {
                // 显示图片
                cv::imshow("sampleTwo", polishedArmors[i].Image);

                // 索引累加
                sampleTwoIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "two_" << std::setw(5) << std::setfill('0') << sampleTwoIndex << ".bmp";
                std::string fileName = sampleTwoPath + ss.str();
                cv::imwrite(fileName, polishedArmors[i].Image);

                // 记录日志信息
                log = "Twos[" + std::to_string(sampleTwoIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }

            // 保存3号装甲板图片
            if(polishedArmors[i].Number == EClassicalArmorNumber::Three)
            {
                // 显示图片
                cv::imshow("sampleThree", polishedArmors[i].Image);

                // 索引累加
                sampleThreeIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "three_" << std::setw(5) << std::setfill('0') << sampleThreeIndex << ".bmp";
                std::string fileName = sampleThreePath + ss.str();
                cv::imwrite(fileName, polishedArmors[i].Image);

                // 记录日志信息
                log = "Threes[" + std::to_string(sampleThreeIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }

            // 保存4号装甲板图片
            if(polishedArmors[i].Number == EClassicalArmorNumber::Four)
            {
                // 显示图片
                cv::imshow("sampleFour", polishedArmors[i].Image);

                // 索引累加
                sampleFourIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "four_" << std::setw(5) << std::setfill('0') << sampleFourIndex << ".bmp";
                std::string fileName = sampleFourPath + ss.str();
                cv::imwrite(fileName, polishedArmors[i].Image);

                // 记录日志信息
                log = "Fours[" + std::to_string(sampleFourIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }

            // 保存5号装甲板图片
            if(polishedArmors[i].Number == EClassicalArmorNumber::Five)
            {
                // 显示图片
                cv::imshow("sampleFive", polishedArmors[i].Image);

                // 索引累加
                sampleFiveIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "five_" << std::setw(5) << std::setfill('0') << sampleFiveIndex << ".bmp";
                std::string fileName = sampleFivePath + ss.str();
                cv::imwrite(fileName, polishedArmors[i].Image);

                // 记录日志信息
                log = "Fives[" + std::to_string(sampleFiveIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }

            // 保存哨兵装甲板图片
            if(polishedArmors[i].Number == EClassicalArmorNumber::Sentry)
            {
                // 显示图片
                cv::imshow("sampleSentry", polishedArmors[i].Image);

                // 编号累加
                sampleSentryIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "sentry_" << std::setw(5) << std::setfill('0') << sampleSentryIndex << ".bmp";
                std::string fileName = sampleSentryPath + ss.str();
                cv::imwrite(fileName, polishedArmors[i].Image);

                // 记录日志信息
                log = "Sentries[" + std::to_string(sampleSentryIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }

            // 保存前哨站装甲板图片
            if(polishedArmors[i].Number == EClassicalArmorNumber::Outpost)
            {
                // 显示图片
                cv::imshow("sampleOutpost", polishedArmors[i].Image);

                // 编号累加
                sampleOutpostIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "outpost_" << std::setw(5) << std::setfill('0') << sampleOutpostIndex << ".bmp";
                std::string fileName = sampleOutpostPath + ss.str();
                cv::imwrite(fileName, polishedArmors[i].Image);

                // 记录日志信息
                log = "Outposts[" + std::to_string(sampleOutpostIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }

            // 保存基地装甲板图片
            if(polishedArmors[i].Number == EClassicalArmorNumber::Base)
            {
                // 显示图片
                cv::imshow("sampleBase", polishedArmors[i].Image);

                // 编号累加
                sampleBaseIndex++;

                // 保存图片
                std::stringstream ss;
                ss << "base_" << std::setw(5) << std::setfill('0') << sampleBaseIndex << ".bmp";
                std::string fileName = sampleBasePath + ss.str();
                cv::imwrite(fileName, polishedArmors[i].Image);

                // 记录日志信息
                log = "Bases[" + std::to_string(sampleBaseIndex) + "] has been sampled successful";
                logger.Save(ELogType::Info, log);
            }
        }
    }

    // 关闭相机
   //camera.Close();

   //// 释放相机资源
   //camera.Release();

    // 释放装甲板识别器资源
    recognizer.Release();

    return 0;
}