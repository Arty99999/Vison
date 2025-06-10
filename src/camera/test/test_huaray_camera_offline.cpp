//
// Created by plutoli on 2022/4/6.
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
    HuarayCameraParam param;
    std::string yamlFile = "config/infantry_3/basement/huaray_camera_param.yaml";
    if (!HuarayCameraParam::LoadFromYamlFile(yamlFile, &param))
    {
        return -1;
    }

    // 设置相机参数
    if (!camera.SetParam(param))
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
    cv::namedWindow("video", cv::WINDOW_NORMAL);
    cv::resizeWindow("video", 800, 600);

    // 播放离线视频
    while (true)
    {
        // 获取相机数据
        HuarayCameraData data;
        camera.GetData(&data);

        // 播放相机数据
        cv::imshow("video", data.Image);

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