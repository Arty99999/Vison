//
// Created by plutoli on 2022/1/25.
//

#include "classical_windmill_recognizer.h"
#include "huaray_camera.h"

// 按ESC键，退出系统
// 按Enter键，保存模板
// 按其它键，继续处理
int main()
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 初始化日志信息
    std::string log;

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

    // 初始化风车扇叶模板文件的存储路径，以“/”结束
    std::string templatePath = "/home/plutoli/data/";

    // 创建templatePath
    if (::access(templatePath.c_str(), F_OK) == -1)
    {
        if (::mkdir(templatePath.c_str(), 07777) == 0)
        {
            log = "[" + templatePath + "] was created successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + templatePath + "] was created failure";
            logger.Save(ELogType::Error, log);
            return -1;
        }
    }
    else
    {
        log = "[" + templatePath + "] exist";
        logger.Save(ELogType::Info, log);
    }

    // 创建图像显示窗口
    cv::namedWindow("rawImage",cv::WINDOW_NORMAL);
    cv::resizeWindow("rawImage", 800, 600);
    cv::namedWindow("binaryImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("binaryImage", 800, 600);
    cv::namedWindow("windmillFanImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("windmillFanImage", 800, 600);

    // 显示临时图像
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("rawImage", image);
    cv::imshow("binaryImage", image);
    cv::imshow("windmillFanImage", image);

    // 打印提示信息
    log = "Press 'ESC' to eixt. Press 'Enter' to save template. Press other key to continue: ";
    logger.Save(ELogType::Info, log);

    // 循环处理相机数据
    while (camera.IsOpened())
    {
        // 读取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 对原始图像进行预处理得到二值图像
        cv::Mat binaryImage;
        recognizer.Preprocess(cameraData.Image, &binaryImage);

        // 显示原始图像和预处理之后的二值图像
        cv::imshow("rawImage", cameraData.Image);
        cv::imshow("binaryImage", binaryImage);

        // 当前线程阻塞，读取键盘按键的键值
        int keyValue = cv::waitKey(0);

        // 如果按下“ESC”键，则退出系统
        if (keyValue == 27)
        {
            break;
        }

        // 如果按下"Enter"键，保存模板
        if (keyValue != 13)
        {
            continue;
        }

        // 在二值图像中搜索所有轮廓
        std::vector<cv::Vec4i> hierarchy;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binaryImage,
                         contours,
                         hierarchy,
                         cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_NONE,
                         cv::Point(0, 0));

        // 判断搜索得到的轮廓集合是否为空
        if (contours.empty())
        {
            log = "There is no contour in current binary image";
            logger.Save(ELogType::Error, log);
            continue;
        }

        // 将面积最大的轮廓作为风车扇叶的轮廓模板
        double area = cv::contourArea(contours[0]);
        std::vector<cv::Point> windmillFanContour = contours[0];
        for (unsigned int i = 1; i < contours.size(); ++i)
        {
            double tempArea = cv::contourArea(contours[i]);
            if (tempArea > area)
            {
                area = tempArea;
                windmillFanContour = contours[i];
            }
        }

        // 创建一个和二值图像大小相同的灰度图
        cv::Mat windmillFanImage(binaryImage.rows, binaryImage.cols, CV_8UC1, cv::Scalar(0));

        // 在灰度图上绘制并填充轮廓风车扇叶轮廓
        std::vector<std::vector<cv::Point>> windmillFanContours;
        windmillFanContours.emplace_back(windmillFanContour);
        cv::drawContours(windmillFanImage,
                         windmillFanContours,
                         -1,
                         cv::Scalar(255),
                         -1);   // 当thickness为-1时，标识填充轮廓内部

        // 显示风车扇叶轮廓图像
        cv::imshow("windmillFanImage", windmillFanImage);

        // 获取当前日期时间字符串，使用日期时间字符串创建风车扇叶模板文件名
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&now_time);
        std::stringstream stream;
        stream << std::put_time(&tm, "%Y-%m-%d_%H:%M:%S");
        std::string dateTimeString = stream.str();
        std::string windmillFanTemplateName = templatePath.append("windmill_fan_template_" + dateTimeString + ".bmp");

        // 保存风车扇叶模板文件
        if (cv::imwrite(windmillFanTemplateName, windmillFanImage))
        {
            log = "Windmill fan template was saved successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "Windmill fan template was saved failure";
            logger.Save(ELogType::Error, log);
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