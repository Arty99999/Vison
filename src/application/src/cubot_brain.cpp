//
// Created by plutoli on 2021/8/24.
//

#include <atomic>
#include "keyboard_scanner.h"
#include "robot_brain.h"
#include "easy_logger.h"

/**
 * @brief 机器人大脑的暂停状态
 */
std::atomic<bool> IsRobotBrainPaused(false);

/**
 * @brief 切换机器人大脑状态
 * @note 可以在暂停状态和运行状态之间进行切换，便于系统调试工作
 */
void ToggleRobotBrain()
{
    auto GetPressedKeyWithoutEnter = [ ]()->int {
        // 初始化日志信息，获取日志记录器的单例引用
        std::string log;
        EasyLogger &logger = EasyLogger::GetSingleInstance();

        // 获取终端配置信息
        struct termios rawSettings;
        if (::tcgetattr(0, &rawSettings) == -1)
        {
            log = "Terminal's raw settings was got failure";
            logger.Save(ELogType::Error, log);
        }

        // 修改终端配置信息
        struct termios newSettings;
        newSettings = rawSettings;
        newSettings.c_lflag &= (~ICANON);
        newSettings.c_lflag &= (~ECHO);
        newSettings.c_lflag &= (~ISIG);
        newSettings.c_cc[VTIME] = 0;
        newSettings.c_cc[VMIN] = 1;
        if (::tcsetattr(0, TCSANOW, &newSettings) == -1)
        {
            log = "Terminal's new settings was set failure";
            logger.Save(ELogType::Error, log);
        }

        // 通过阻塞方式读取键值
        int keyValue = std::getchar();

        // 恢复终端配置信息
        if (::tcsetattr(0, TCSANOW, &rawSettings) == -1)
        {
            log = "Terminal's raw settings was restored failure";
            logger.Save(ELogType::Error, log);
        }

        // 返回读取到的键值
        return keyValue;
    };
    // 获取机器人大脑
    RobotBrain& robotBrain = RobotBrain::GetSingleInstance();

    // 如果机器人大脑处于存活状态，则循环检测是否切换机器人大脑状态
    while (robotBrain.IsAlive())
    {
        // 当前线程阻塞，等待按键按下
        int keyValue = GetPressedKeyWithoutEnter();

        // 判断是否按下Enter键
        if (keyValue != 10)
        {
            continue;
        }

        // 获取机器人大脑内核集合
        std::vector<std::shared_ptr<RobotBrainCore>> robotBrainCores = robotBrain.GetRobotBrainCores();

        // 切换机器人大脑状态
        if (IsRobotBrainPaused)
        {
            // 机器人大脑内核恢复工作
            for (unsigned int i = 0; i < robotBrainCores.size(); ++i)
            {
                if (robotBrainCores[i]->IsPaused())
                {
                    robotBrainCores[i]->Resume();
                }
            }

            // 更新机器人大脑的暂停状态
            IsRobotBrainPaused = false;
        }
        else
        {
            // 机器人大脑内核暂停工作
            for (unsigned int i = 0; i < robotBrainCores.size(); ++i)
            {
                if (!robotBrainCores[i]->IsPaused())
                {
                    robotBrainCores[i]->Pause();
                }
            }

            // 更新机器人大脑的暂停状态
            IsRobotBrainPaused = true;
        }
    }
}

/**
 * @brief 主函数
 */
int main(int argc, char *argv[])
{

    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 获取机器人大脑的单例引用
    RobotBrain& robotBrain = RobotBrain::GetSingleInstance();

    // 读取机器人大脑参数
    RobotBrainParam robotBrainParam;
    std::string robotBrainYaml = "config/infantry_3/basement/robot_brain_param.yaml";
    if (!RobotBrainParam::LoadFromYamlFile(robotBrainYaml, &robotBrainParam))
    {
        return -1;
    }

    // 设置机器人大脑参数
    if (!robotBrain.SetParam(robotBrainParam))
    {
        return -1;
    }

    // 初始化机器人大脑
    robotBrain.Init();

    // 打开机器人大脑
    robotBrain.Open();

    // 启动机器人大脑自检
    robotBrain.StartScan();

    // 启动机器人大脑切换线程
    // 注意：正式使用时，请注释掉该行代码
    // std::thread toggleRobotBrainThread(ToggleRobotBrain);

    // 初始化显示窗体
    cv::namedWindow("cachedImage", cv::WINDOW_NORMAL);
    cv::resizeWindow("cachedImage", 800, 600);

    // 临时显示图像
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(0, 0, 255));
    cv::imshow("cachedImage", image);

    // 创建上一帧统计信息时间戳
    std::chrono::time_point<std::chrono::steady_clock> previousStatisticsTime = std::chrono::steady_clock::now();
    uint64_t previousStatisticsTimestamp = previousStatisticsTime.time_since_epoch().count();

    // 主线程循环扫描
    while (robotBrain.IsAlive())
    {
        // 获取机器人大脑内核
        std::shared_ptr<RobotBrainCore> brainCore = robotBrain.GetRobotBrainCores()[0];

        // 获取当前时间戳
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        uint64_t nowTimestamp = now.time_since_epoch().count();

        // 每秒钟更新1次机器人大脑内核的统计信息
        if ((nowTimestamp - previousStatisticsTimestamp) > 1000000000)
        {
            // 更新时间戳
            previousStatisticsTimestamp = nowTimestamp;

            // 获取并显示统计信息
            RobotBrainCoreStatistics statistics = brainCore->GetStatistics();
            std::cout << std::endl;
            std::cout << "****************************************************" << std::endl;
            std::cout << "read camera data index: " << statistics.ReadCameraDataIndex << std::endl;
            std::cout << "processed camera data index: " << statistics.ProcessedCameraDataIndex << std::endl;
            std::cout << "fight control command index: " << statistics.FightControlCommandIndex << std::endl;
            std::cout << "buff control command index: " << statistics.BuffControlCommandIndex << std::endl;
            std::cout << "process time: " << statistics.ProcessTime / 1000000 << "ms" << std::endl;
            std::cout << "fps: " << statistics.Fps << std::endl;
            std::cout << "****************************************************" << std::endl;
            std::cout << std::endl;
        }

        // 判断机器人大脑内核是否工作在战斗模式
        bool isFightMode = true;

        // 显示战斗缓存数据
        if (isFightMode)
        {
            RobotFightData fightData;
            brainCore->GetFightData(&fightData);
            cv::imshow("cachedImage", fightData.Image);
        }

        cv::waitKey(10);

    }

    // 停止机器人大脑自检
    if (robotBrain.IsScanning())
    {
        robotBrain.StopScan();
    }

    // 关闭机器人大脑
    if (robotBrain.IsOpened())
    {
        robotBrain.Close();
    }

    // 释放机器人大脑资源
    if (robotBrain.IsInitialized())
    {
        robotBrain.Release();
    }

    return 0;
}