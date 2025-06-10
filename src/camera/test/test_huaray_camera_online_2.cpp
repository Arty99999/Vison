//
// Created by plutoli on 2022-05-12.
//

#include <string>
#include "huaray_camera.h"

// 按ESC键，退出系统
// 按Enter键，暂停系统
// 按任意键，继续处理
int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 创建相机
    HuarayCamera camera1;
    HuarayCamera camera2;

    // 读取相机参数
    HuarayCameraParam param1;
    HuarayCameraParam param2;
    std::string yamlFile1 = "config/sentry/basement/huaray_camera_param_1.yaml";
    std::string yamlFile2 = "config/sentry/basement/huaray_camera_param_2.yaml";
    if (!HuarayCameraParam::LoadFromYamlFile(yamlFile1, &param1))
    {
        return -1;
    }
    if (!HuarayCameraParam::LoadFromYamlFile(yamlFile2, &param2))
    {
        return -1;
    }

    // 设置相机参数
    if (!camera1.SetParam(param1))
    {
        return -1;
    }
    if (!camera2.SetParam(param2))
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

    // 初始化显示窗体
    cv::namedWindow("camera1", cv::WINDOW_NORMAL);
    cv::resizeWindow("camera1", 800, 600);
    cv::namedWindow("camera2", cv::WINDOW_NORMAL);
    cv::resizeWindow("camera2", 800, 600);

    // 初始化数据帧索引和上一帧图像的时间戳
    uint64_t frameIndex1 = 0;
    uint64_t frameIndex2 = 0;
    uint64_t previousFrameTimestamp1 = 0;
    uint64_t previousFrameTimestamp2 = 0;

    // 播放在线视频
    while (true)
    {
        // 获取相机数据1
        HuarayCameraData cameraData1;
        camera1.GetData(&cameraData1);
        if (cameraData1.Timestamp > previousFrameTimestamp1)
        {
            previousFrameTimestamp1 = cameraData1.Timestamp;
            frameIndex1++;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 获取相机数据2
        HuarayCameraData cameraData2;
        camera2.GetData(&cameraData2);
        if (cameraData2.Timestamp > previousFrameTimestamp2)
        {
            previousFrameTimestamp2 = cameraData2.Timestamp;
            frameIndex2++;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 线程延迟，模拟图像处理的消耗
        // std::this_thread::sleep_for(std::chrono::milliseconds(6));

        // 输出播放数据帧索引和相机数据索引
        std::cout << std::endl;
        std::cout << "cameraDataIndex_1: " << cameraData1.Index << std::endl;
        std::cout << "displayedFrameNumber_1: " << frameIndex1 << std::endl;
        std::cout << "cameraDataIndex_2: " << cameraData2.Index << std::endl;
        std::cout << "displayedFrameNumber_2: " << frameIndex2 << std::endl;
        std::cout << "**************************************************" << std::endl;

        // 读取并输出相机状态1
        HuarayCameraStatus status1;
        if (camera1.GetStatus(&status1))
        {
            std::cout << "Key_1: " << status1.Key << std::endl;
            std::cout << "IsConnected_1: " << status1.IsConnected << std::endl;
            std::cout << "IsWorking_1: " << status1.IsWorking << std::endl;
            std::cout << "ErrorFrame_1: " << status1.ErrorFrame << std::endl;
            std::cout << "LostPacketFrame_1: " << status1.LostPacketFrame << std::endl;
            std::cout << "TotalFrame_1: " << status1.TotalFrame << std::endl;
            std::cout << "BandWidth_1: " << status1.BandWidth << std::endl;
            std::cout << "FPS_1: " << status1.FPS << std::endl;
            std::cout << "**************************************************" << std::endl;
        }

        // 读取并输出相机状态2
        HuarayCameraStatus status2;
        if (camera2.GetStatus(&status2))
        {
            std::cout << "Key_2: " << status2.Key << std::endl;
            std::cout << "IsConnected_2: " << status2.IsConnected << std::endl;
            std::cout << "IsWorking_2: " << status2.IsWorking << std::endl;
            std::cout << "ErrorFrame_2: " << status2.ErrorFrame << std::endl;
            std::cout << "LostPacketFrame_2: " << status2.LostPacketFrame << std::endl;
            std::cout << "TotalFrame_2: " << status2.TotalFrame << std::endl;
            std::cout << "BandWidth_2: " << status2.BandWidth << std::endl;
            std::cout << "FPS_2: " << status2.FPS << std::endl;
            std::cout << "**************************************************" << std::endl;
        }

        // 播放视频
        cv::imshow("camera1", cameraData1.Image);
        cv::imshow("camera2", cameraData2.Image);

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

    // 关闭相机
    camera1.Close();
    camera2.Close();

    // 清理相机资源
    camera1.Release();
    camera2.Release();

    return 0;
}