//
// Created by plutoli on 2022-06-05.
//

#include "huaray_camera.h"

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

    // 初始化显示窗体
    cv::namedWindow("camera", cv::WINDOW_NORMAL);
    cv::resizeWindow("camera", 800, 600);

    // 输出提示信息
    std::cout << "VideoRecorder is working. Please press 'ESC' to exit......" << std::endl;

    // 循环处理相机数据
    while (true)
    {
        // 获取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 播放相机数据
        cv::imshow("camera", cameraData.Image);

        // 按ESC键，停止录制视频；cv::waitKey()返回的是按键的ASCII码
        int keyValue = cv::waitKey(10);
        if (keyValue == 27)
        {
            break;
        }
    }

    // 关闭相机
    camera.Close();

    // 释放相机资源
    camera.Release();

    // 输出提示信息
    std::cout << "VideoRecorder exit completely" << std::endl;

    return 0;
}