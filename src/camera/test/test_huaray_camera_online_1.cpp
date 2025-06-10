//
// Created by plutoli on 2021/8/14.
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

    // 初始化显示窗体
    cv::namedWindow("camera", cv::WINDOW_NORMAL);
    cv::resizeWindow("camera", 800, 600);

    // 初始化播放数据帧索引和上一帧图像的时间戳
    uint64_t frameIndex = 0;
    uint64_t previousFrameTimestamp = 0;

    // 播放在线视频
    while (true)
    {
        // 获取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);
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

        // 线程延迟，模拟图像处理的消耗
        std::this_thread::sleep_for(std::chrono::milliseconds(6));

        // 输出播放数据帧索引和相机数据索引
        std::cout << "**************************************************" << std::endl;
        std::cout << "frameIndex: " << frameIndex << std::endl;
        std::cout << "cameraDataIndex: " << cameraData.Index << std::endl;
        std::cout << "**************************************************" << std::endl;

        // 读取并输出相机状态
        HuarayCameraStatus status;
        if (camera.GetStatus(&status))
        {
            std::cout << "Key: " << status.Key << std::endl;
            std::cout << "IsConnected: " << status.IsConnected << std::endl;
            std::cout << "IsWorking: " << status.IsWorking << std::endl;
            std::cout << "ErrorFrame: " << status.ErrorFrame << std::endl;
            std::cout << "LostPacketFrame: " << status.LostPacketFrame << std::endl;
            std::cout << "TotalFrame: " << status.TotalFrame << std::endl;
            std::cout << "BandWidth: " << status.BandWidth << std::endl;
            std::cout << "FPS: " << status.FPS << std::endl;
            std::cout << "**************************************************" << std::endl << std::endl;
        }

        // 播放视频
        cv::imshow("camera", cameraData.Image);

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
    camera.Close();

    // 清理相机资源
    camera.Release();

    return 0;
}