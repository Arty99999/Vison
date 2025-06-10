//
// Created by plutoli on 2021/8/12.
//

#include "robot_brain.h"
/**
* 定义全局变量  大脑预测指令缓存区
*/
extern std::vector<PredictorCommand> BodyPredictorCommand;
// ******************************  RobotBrain类的公有函数  ******************************

// 析构函数
RobotBrain::~RobotBrain()
{
    // 停止机器人大脑自检
    if (IsScanning())
    {
        StopScan();
    }

    // 关闭机器人大脑
    if (IsOpened())
    {
        Close();
    }

    // 释放机器人大脑系统资源
    if (IsInitialized())
    {
        Release();
    }
}

// 获取机器人大脑的单例引用
RobotBrain& RobotBrain::GetSingleInstance()
{
    static RobotBrain robotBrain;
    return robotBrain;
}

// 获取机器人大脑的通信串口引用
SerialPort& RobotBrain::GetSerialPort()
{
    return serialPort_;
}

// 获取机器人大脑参数
RobotBrainParam RobotBrain::GetParam()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return param_;
}

// 设置机器人大脑参数
bool RobotBrain::SetParam(const RobotBrainParam &param)
{
    // 锁定机器人大脑的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - RobotBrainParam was set failure because RobotBrain has been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 记录机器人大脑参数
    param_ = param;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrainParam was set successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回设置结果
    return true;
}

// 获取机器人大脑的存活状态
bool RobotBrain::IsAlive()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isAlive_;
}

// 获取机器人大脑的扫描状态
bool RobotBrain::IsScanning()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return scanSwitch_;
}

// 获取机器人大脑的初始化状态
bool RobotBrain::IsInitialized()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isInitialized_;
}

// 获取机器人大脑的打开状态
bool RobotBrain::IsOpened()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isOpened_;
}

// 获取机器人大脑的初始化时间戳
uint64_t RobotBrain::GetInitTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return initTimestamp_;
}

// 获取机器人大脑的打开时间戳
uint64_t RobotBrain::GetOpenTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return openTimestamp_;
}

// 初始化机器人大脑
bool RobotBrain::Init()
{
    // 锁定机器人大脑的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - RobotBrain can not be initialized repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置通信串口的初始化状态
    if (serialPort_.IsInitialized())
    {
        serialPort_.Release();
        serialPort_.UnregisterDataReceivedHandler();
    }

    // 读取通信串口参数
    SerialPortParam serialPortParam;
    if (!SerialPortParam::LoadFromYamlFile(param_.SerialPortParamFileName, &serialPortParam))
    {
        log = "[" + param_.Key + "] - RobotBrain was initialized failure because SerialPortParam can't be loaded";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 设置通信串口参数
    if (!serialPort_.SetParam(serialPortParam))
    {
        log = "[" + param_.Key + "] - RobotBrain was initialized failure because SerialPortParam can't be set";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 初始化通信串口
    if (!serialPort_.Init())
    {
        log = "[" + param_.Key + "] - RobotBrain was initialized failure because SerialPort can't be initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }
    else
    {
        serialPort_.RegisterDataReceivedHandler(HandleSerialPortDataReceived, this);
    }

    // 重置所有机器人大脑内核的初始化状态
    for (unsigned int i = 0; i < robotBrainCores_.size(); ++i)
    {
        if (robotBrainCores_[i]->IsInitialized())
        {
            robotBrainCores_[i]->Release();
        }
    }
    robotBrainCores_.clear();

    // 使用机器人大脑内核参数初始化并存储所有机器人大脑内核
    for (unsigned int i = 0; i < param_.RobotBrainCoreParams.size(); ++i)
    {
        // 创建机器人大脑内核
        std::shared_ptr<RobotBrainCore> robotBrainCore(new RobotBrainCore());

        // 设置机器人大脑内核参数
        if (!robotBrainCore->SetParam(param_.RobotBrainCoreParams[i]))
        {
            log = "[" + param_.Key + "] - RobotBrain was initialized failure because "\
                  "RobotBrainCores[" + std::to_string(i)+ "]'s param can't be set";
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }

        // 初始化机器人大脑内核
        if (!robotBrainCore->Init())
        {
            log = "[" + param_.Key + "] - RobotBrain was initialized failure because "\
                  "RobotBrainCores[" + std::to_string(i)+ "] can't be initialized";
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }

        // 记录机器人大脑内核
        robotBrainCores_.emplace_back(robotBrainCore);
    }

    // 设置初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    initTimestamp_ = now.time_since_epoch().count();

    // 设置初始化状态
    isInitialized_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrain was initialized successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回初始化结果
    return true;
}

// 释放机器人大脑占用的系统资源
bool RobotBrain::Release()
{
    // 锁定机器人大脑的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑是否已经打开
    if (isOpened_)
    {
        log = "[" + param_.Key + "] - RobotBrain was released failure because it has been opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断机器人大脑是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - RobotBrain can not be released repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 释放通信串口的系统资源
    if (serialPort_.IsInitialized())
    {
        serialPort_.Release();
        serialPort_.UnregisterDataReceivedHandler();
    }

    // 重置机器人大脑内核集合
    for (unsigned int i = 0; i < robotBrainCores_.size(); ++i)
    {
        if (robotBrainCores_[i]->IsInitialized())
        {
            robotBrainCores_[i]->Release();
        }
    }
    robotBrainCores_.clear();

    // 重置初始化时间戳
    initTimestamp_ = 0;

    // 重置初始化状态
    isInitialized_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrain was released successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回释放结果
    return true;
}

// 打开机器人大脑
bool RobotBrain::Open()
{
    // 锁定机器人大脑的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - RobotBrain was opened failure because it has not been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断机器人大脑是否已经打开
    if (isOpened_)
    {
        log = "[" + param_.Key + "] - RobotBrain can not be opened repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置通信串口的打开状态
    if (serialPort_.IsOpened())
    {
        serialPort_.Close();
    }

    // 打开通信串口
    if (!serialPort_.Open())
    {
        log = "[" + param_.Key + "] - RobotBrain was opened failure because SerialPort can't be opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置所有机器人大脑内核的打开状态
    for (unsigned int i = 0; i < robotBrainCores_.size(); ++i)
    {
        if (robotBrainCores_[i]->IsOpened())
        {
            robotBrainCores_[i]->Close();
        }
    }

    // 打开机器人大脑内核
    for (unsigned int i = 0; i < robotBrainCores_.size(); ++i)
    {
        if (!robotBrainCores_[i]->Open())
        {
            log = "[" + param_.Key + "] - RobotBrain was opened failure because "\
                  "BrainCores[" + std::to_string(i) + "] can't be opened";
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }
    }

    // 设置打开时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    openTimestamp_ = now.time_since_epoch().count();

    // 设置打开状态
    isOpened_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrain was opened successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回打开结果
    return true;
}

// 关闭机器人大脑
bool RobotBrain::Close()
{
    // 锁定机器人大脑的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑是否已经打开
    if (!isOpened_)
    {
        log = "[" + param_.Key + "] - RobotBrain can not be closed repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 关闭通信串口
    if (serialPort_.IsOpened())
    {
        serialPort_.Close();
    }

    // 关闭所有机器人大脑内核
    for (unsigned int i = 0; i < robotBrainCores_.size(); ++i)
    {
        if (robotBrainCores_[i]->IsOpened())
        {
            robotBrainCores_[i]->Close();
        }
    }

    // 重置打开时间戳
    openTimestamp_ = 0;

    // 重置打开状态
    isOpened_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrain was closed successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回关闭结果
    return true;
}

// 启动机器人大脑自检
void RobotBrain::StartScan()
{
    // 锁定机器人大脑的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 启动自检线程
    scanSwitch_ = true;
    scanThread_ = std::thread(&RobotBrain::Scan, this);

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrain's ScanThread was started successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);
}

// 停止机器人大脑自检
void RobotBrain::StopScan()
{
    // 锁定机器人大脑的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 停止自检线程
    scanSwitch_ = false;
    if (scanThread_.joinable())
    {
        scanThread_.join();
    }

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrain's ScanThread was stopped successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);
}

// 获取机器人大脑内核集合
std::vector<std::shared_ptr<RobotBrainCore>> RobotBrain::GetRobotBrainCores()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return robotBrainCores_;
}

// 将数据帧发送给机器人本体
void RobotBrain::TransmitFrame(const unsigned int &size, const unsigned char *frame)
{
//    std::cout<<"TransmitFrame"<<std::endl;
    serialPort_.Write(size, frame);
}

// 处理机器人本体请求
void RobotBrain::HandleRequest(const RobotBodyRequest &request)
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    switch (request.Type)
    {
        // 切换工作模式
        case ERequestType::SwitchWorkMode:
        {
            // 判断请求的数据长度是否合法
            if (request.Datas.size() != 2)
            {
                // 保存日志
                std::string logMessage = "[" + param_.Key + "] - RobotBodyRequest's size is invalid ";
                logger.Save(ELogType::Error, logMessage);

                break;
            }

            // 检索ID匹配的机器人大脑内核
            int robotBrainCoreIndex = -1;
            for (int i = 0; i < robotBrainCores_.size(); ++i)
            {
                if (robotBrainCores_[i]->GetParam().ID == request.Datas[0])
                {
                    robotBrainCoreIndex = i;
                    break;
                }
            }

            // 判断机器人大脑内核是否检索成功
            if (robotBrainCoreIndex == -1)
            {
                // 保存日志
                std::string logMessage = "[" + param_.Key + "] - RobotBodyRequest's ID is invalid ";
                logger.Save(ELogType::Error, logMessage);

                break;
            }

            // 转换机器人本体请求切换的工作模式
            //TODO
            // EWorkMode workMode;
            // if (!SystemConfigurator::ConvertToWorkMode(request.Datas[1], &workMode))
            // {
            //     // 保存日志
            //     std::string logMessage = "[" + param_.Key + "] - RobotBodyRequest's WorkMode is invalid ";
            //     logger.Save(ELogType::Error, logMessage);
            //
            //     break;
            // }

            // 切换机器人大脑内核的工作模式
            // robotBrainCores_[robotBrainCoreIndex]->SwitchWorkMode(workMode);

            break;
        }

        // 切换子弹速度
        case ERequestType::SwitchBulletVelocity:
        {
            // 判断请求的数据长度是否合法
            if (request.Datas.size() != 2)
            {
                // 保存日志
                std::string logMessage = "[" + param_.Key + "] - RobotBodyRequest's size is invalid ";
                logger.Save(ELogType::Error, logMessage);

                break;
            }

            // 检索ID匹配的机器人大脑内核
            int robotBrainCoreIndex = -1;
            for (int i = 0; i < robotBrainCores_.size(); ++i)
            {
                if (robotBrainCores_[i]->GetParam().ID == request.Datas[0])
                {
                    robotBrainCoreIndex = i;
                    break;
                }
            }

            // 判断机器人大脑内核是否检索成功
            if (robotBrainCoreIndex == -1)
            {
                // 保存日志
                std::string logMessage = "[" + param_.Key + "] - RobotBodyRequest's ID is invalid ";
                logger.Save(ELogType::Error, logMessage);

                break;
            }

            // 转换机器人本体请求切换的子弹速度
            //TODO
            // EBulletVelocity bulletVelocity;
            // if (!SystemConfigurator::ConvertToBulletVelocity(request.Datas[1], &bulletVelocity))
            // {
            //     // 保存日志
            //     std::string logMessage = "[" + param_.Key + "] - RobotBodyRequest's BulletVelocity is invalid ";
            //     logger.Save(ELogType::Error, logMessage);
            //
            //     break;
            // }

            // 切换机器人大脑内核的子弹速度
//             robotBrainCores_[robotBrainCoreIndex]->SwitchBulletVelocity(bulletVelocity);

            break;
        }

        // 保存日志
        case ERequestType::SaveLog:
        {
            // 提取请求的机器人内核ID
            std::string id = std::to_string(request.Datas[0]);

            // 提取请求的日志内容
            std::string message;
            message.assign(request.Datas.begin() + 1, request.Datas.end());

            // 保存日志
            std::string logMessage = "[" + param_.Key + "] - RobotBrainCoreID: " + id + "; Robot body log: " + message;
            logger.Save(ELogType::Info, logMessage);

            break;
        }

        // 重启应用程序
        case ERequestType::RebootApplication:
        {
            // 保存日志
            std::string logMessage = "[" + param_.Key + "] - RobotBrain was rebooted";
            logger.Save(ELogType::Info, logMessage);

            // 重置机器人大脑的存活状态
            isAlive_ = false;

            break;
        }

        // 重启计算机
        case ERequestType::RebootComputer:
        {
            // 保存日志
            std::string logMessage = "[" + param_.Key + "] - Computer was rebooted";
            logger.Save(ELogType::Info, logMessage);

            // 重启计算机
            system("reboot");

            break;
        }

        default:
            break;
    }
}

// ******************************  RobotBrain类的私有函数  ******************************

// 构造函数
RobotBrain::RobotBrain():
    param_(),
    isAlive_(true),
    isInitialized_(false),
    isOpened_(false),
    initTimestamp_(0),
    openTimestamp_(0),
    operateMutex_(),
    serialPort_(),
    robotBrainCores_(),
    scanSwitch_(false),
    scanThread_()
{
}

// 机器人大脑自检，扫描通信串口和机器人大脑内核是否正常工作
void RobotBrain::Scan()
{
    // 修改线程名称
    std::string threadName = "scan_brain";
    prctl(PR_SET_NAME, threadName.c_str());

    // 设置线程绑定的CPU内核
    // 参考网址：https://antrn.blog.csdn.net/article/details/114263105?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/qq_34440148/article/details/121603698?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/liaoxiangui/article/details/7905612
    //         https://blog.csdn.net/zxc024000/article/details/79438061
    //         https://www.bbsmax.com/A/q4zVKp9XJK/
    int coreNumber = get_nprocs();
    int coreIndex = param_.ScanCpuCore;
    if ((coreIndex >= 0) && (coreIndex < coreNumber))
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(coreIndex, &mask);
        sched_setaffinity(0, sizeof(mask), &mask);
    }

    // 循环检测机器人大脑
    while (scanSwitch_)
    {
        // 如果机器人大脑没有初始化成功，则初始化机器人大脑
        if (!IsInitialized())
        {
            Init();
        }

        // 如果机器人大脑没有打开成功，则打开机器人大脑
        if (!IsOpened())
        {
            Open();
        }

        // 如果机器人大脑已经打开成功，则扫描通信串口和华睿相机的工作状态
        if (IsOpened())
        {
            // 如果通信串口的工作状态异常，则关闭通信串口
            if (!serialPort_.IsNormal())
            {
                if (serialPort_.IsOpened())
                {
                    serialPort_.Close();
                }
            }

            // 如果通信串口处于关闭状态，则打开通信串口
            if (!serialPort_.IsOpened())
            {
                serialPort_.Open();
            }

            // 判断华睿相机的工作状态异常，则关闭华睿相机
            for (unsigned int i = 0; i < robotBrainCores_.size(); ++i)
            {
                if (!robotBrainCores_[i]->GetCamera().IsNormal())
                {
                    if (robotBrainCores_[i]->GetCamera().IsOpened())
                    {
                        robotBrainCores_[i]->GetCamera().Close();
                    }
                }
            }

            // 如果华睿相机处于关闭状态，则打开华睿相机
            for (unsigned int i = 0; i < robotBrainCores_.size(); ++i)
            {
                if (!robotBrainCores_[i]->GetCamera().IsOpened())
                {
                    robotBrainCores_[i]->GetCamera().Open();
                }
            }
        }

        // 休眠1秒钟
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

// ******************************  RobotBrain类的回调函数  ******************************

/**
 * @brief 串口数据接收回调函数
 * @param[out] bytesToRead 串口接收到的字节数
 * @param[out] userData 回调函数关联的用户数据
 */

void HandleSerialPortDataReceived(const unsigned int &bytesToRead, void* userData) {
    // 获取机器人大脑指针和通信串口引用
    auto robotBrain = (RobotBrain *) userData;
    SerialPort &serialPort = robotBrain->GetSerialPort();

    static std::chrono::time_point<std::chrono::steady_clock> last_receiveTime;
    // 初始化数据帧缓冲区
    unsigned char frame[1000];
    std::vector<PredictorCommand> BodyPredictorCommand;
    // 读取并处理串口缓冲区中的所有数据帧
    while (true) {
        // 从串口缓冲区中读取一帧数据
        unsigned int frameSize = serialPort.ReadLine(frame);

        // 根据读取到的字节长度判断是否读取成功
        if (frameSize == 0) {
            break;
        }
//        std::cout<<"framesize: "<<frameSize<<std::endl;
        // 处理读取到的数据帧
        if ((frameSize == 21) && (frame[0] == 0xAA) && (frame[frameSize - 1] == 0xDD) &&(frame[frameSize - 2] == 0xDD) ){

            // 获取读取数据时间
            std::chrono::time_point<std::chrono::steady_clock> receiveTime = std::chrono::steady_clock::now();
            uint64_t receiveTimestamp = receiveTime.time_since_epoch().count();

            std::chrono::duration<double> gap_time = receiveTime - last_receiveTime;

            //std::cout << "IMU两帧间隔为 : " << gap_time.count() * 1000 << "ms\n";
            last_receiveTime = receiveTime;
            ERequestType type;
            if (frame[1] == 7) {
                type = ERequestType::PredictorCommond;
                // 解析机器人本体请求
                bool result = true;
                RobotBodyRequest request;
                request.Type = type;
                for (unsigned int i = 2; i < frameSize - 2; ++i) {
                    request.Datas.emplace_back(frame[i]);
                }

                // 处理机器人本体请求
                if (request.Type == ERequestType::PredictorCommond) {
                    if (request.Datas.size() != 17) {
                        result = false;
                        std::cout << "size of data is wrong" << std::endl;
                    }
                    if (result) {
                        PredictorCommand predictCommand;
                        if (!PredictorCommand::Parse(request.Datas, &predictCommand)) {
                            std::cout << "PredictorCommond parsed was failed." << std::endl;
                        } else {
                            // 如果内存数据过多则更新
                            if (BodyPredictorCommand.size() > 1000) {
                                // 则刷前半个容器
                                ::BodyPredictorCommand.erase(BodyPredictorCommand.begin(),
                                                             BodyPredictorCommand.begin() + 500);

                                // 更新数据
                                ::BodyPredictorCommand.push_back(predictCommand);

                                std::cout << "erase half of vector" << std::endl;
                            } else {
                                // 更新数据
                                ::BodyPredictorCommand.emplace_back(predictCommand);
//                            std::cout<<"There is new data"<<std::endl;

                            }
                        }
                    }
                }
            }
        }
    }
}