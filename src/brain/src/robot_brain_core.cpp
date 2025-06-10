//
// Created by plutoli on 2021/12/28.
//

#include "robot_brain_core.h"
#include "robot_brain.h"


std::vector<PredictorCommand> BodyPredictorCommand;

// 构造函数
RobotBrainCore::RobotBrainCore():
        param_(),
        huarayCamera_(),
        classicalArmorRecognizer_(),
        classicalWindmillRecognizer_(),
        solver_(),
        isInitialized_(false),
        isOpened_(false),
        isPaused_(false),
        initTimestamp_(0),
        openTimestamp_(0),
        operateMutex_(),
        statistics_(),
        statisticsMutex_(),
        fightData_(),
        fightDataMutex_(),
        buffData_(),
        buffDataMutex_(),
        controlBodySwitch_(false),
        controlBodyThread_(),
        notifyBodySwitch_(false),
        notifyBodyThread_(),
        fittingSwitch_(),
        fittingThread_()
{
}

// 析构函数
RobotBrainCore::~RobotBrainCore()
{
    // 机器人大脑内核恢复工作
    if (IsPaused())
    {
        Resume();
    }

    // 关闭机器人大脑内核
    if (IsOpened())
    {
        Close();
    }

    // 释放机器人大脑内核系统资源
    if (IsInitialized())
    {
        Release();
    }
}

// 获取机器人大脑内核的华睿相机引用
HuarayCamera& RobotBrainCore::GetCamera()
{
    return huarayCamera_;
}

// 获取机器人大脑内核的经典装甲板识别器引用
ClassicalArmorRecognizer& RobotBrainCore::GetClassicalArmorRecognizer()
{
    return classicalArmorRecognizer_;
}

// 获取机器人大脑内核的经典风车识别器引用
ClassicalWindmillRecognizer& RobotBrainCore::GetClassicalWindmillRecognizer()
{
    return classicalWindmillRecognizer_;
}

// 获取机器人大脑内核的目标解算器引用
Solver& RobotBrainCore::GetSolver()
{
    return solver_;
}

// 获取机器人大脑内核参数
RobotBrainCoreParam RobotBrainCore::GetParam()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return param_;
}

// 设置机器人大脑内核参数
bool RobotBrainCore::SetParam(const RobotBrainCoreParam &param)
{
    // 锁定机器人大脑内核的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑内核是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - RobotBrainCoreParam was set failure because RobotBrainCore has been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 记录机器人大脑内核参数
    param_ = param;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrainCoreParam was set successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回设置结果
    return true;
}

// 获取机器人大脑内核的统计信息
RobotBrainCoreStatistics RobotBrainCore::GetStatistics()
{
    std::shared_lock<std::shared_mutex> sharedLock(statisticsMutex_);
    return statistics_;
}

// 获取机器人大脑内核的战斗数据
void RobotBrainCore::GetFightData(RobotFightData *fightData)
{
    std::shared_lock<std::shared_mutex> sharedLock(fightDataMutex_);
    fightData->Type = fightData_.Type;
    fightData_.Image.copyTo(fightData->Image);
}


// 获取机器人大脑内核的初始化状态
bool RobotBrainCore::IsInitialized()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isInitialized_;
}

// 获取机器人大脑内核的打开状态
bool RobotBrainCore::IsOpened()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isOpened_;
}

// 获取机器人大脑内核的暂停状态
bool RobotBrainCore::IsPaused()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isPaused_;
}

// 获取机器人大脑内核的初始化时间戳
uint64_t RobotBrainCore::GetInitTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return initTimestamp_;
}

// 获取机器人大脑内核的打开时间戳
uint64_t RobotBrainCore::GetOpenTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return openTimestamp_;
}

// 初始化机器人大脑内核
bool RobotBrainCore::Init()
{
    // 锁定机器人大脑内核的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑内核是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore can not be initialized repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置机器人大脑内核的华睿相机初始化状态
    if (huarayCamera_.IsInitialized())
    {
        huarayCamera_.Release();
    }

    // 读取华睿相机参数
    HuarayCameraParam huarayCameraParam;
    if (!HuarayCameraParam::LoadFromYamlFile(param_.HuarayCameraParamFileName,
                                             &huarayCameraParam))
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure because HuarayCameraParam can't be loaded";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 设置华睿相机参数
    if (!huarayCamera_.SetParam(huarayCameraParam))
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure because HuarayCameraParam can't be set";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 初始化华睿相机
    if (!huarayCamera_.Init())
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure because HuarayCamera can't be initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置机器人大脑内核中经典装甲板识别器的初始化状态
    if (classicalArmorRecognizer_.IsInitialized())
    {
        classicalArmorRecognizer_.Release();
    }

    // 读取经典装甲板识别器参数
    ClassicalArmorRecognizerParam classicalArmorRecognizerParam;
    if (!ClassicalArmorRecognizerParam::LoadFromYamlFile(param_.ClassicalArmorRecognizerParamFileName,
                                                         &classicalArmorRecognizerParam))
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure "\
              "because ClassicalArmorRecognizerParam can't be loaded";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 设置经典装甲板识别器参数
    if (!classicalArmorRecognizer_.SetParam(classicalArmorRecognizerParam))
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure "\
              "because ClassicalArmorRecognizerParam can't be set";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 初始化经典装甲板识别器
    if (!classicalArmorRecognizer_.Init())
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure "\
              "because ClassicalArmorRecognizer can't be initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置机器人大脑内核中经典风车识别器的初始化状态
    if (classicalWindmillRecognizer_.IsInitialized())
    {
        classicalWindmillRecognizer_.Release();
    }

    // 读取经典风车识别器参数
    ClassicalWindmillRecognizerParam classicalWindmillRecognizerParam;
    if (!ClassicalWindmillRecognizerParam::LoadFromYamlFile(param_.ClassicalWindmillRecognizerParamFileName,
                                                            &classicalWindmillRecognizerParam))
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure "\
              "because ClassicalWindmillRecognizerParam can't be loaded";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 设置经典风车识别器参数
    if (!classicalWindmillRecognizer_.SetParam(classicalWindmillRecognizerParam))
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure "\
              "because ClassicalWindmillRecognizerParam can't be set";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 初始化经典风车识别器
    if (!classicalWindmillRecognizer_.Init())
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure "\
              "because ClassicalWindmillRecognizer can't be initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置机器人大脑内核中目标解算器的初始化状态
    if (solver_.IsInitialized())
    {
        solver_.Release();
    }

    // 读取目标解算器参数
    SolverParam solverParam;
    if (!SolverParam::LoadFromYamlFile(param_.SolverParamFileName, &solverParam))
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure because SolverParam can't be loaded";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 设置目标解算器参数
    if (!solver_.SetParam(solverParam))
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure because SolverParam can't be set";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 初始化目标解算器
    if (!solver_.Init())
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure because Solver can't be initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }
    // 重置机器人大脑内核中目标预测器的初始化状态
    if (predictor_.IsInitialized_())
    {
        predictor_.Release();
    }

    // 读取目标预测器参数
    PredictorParam predictorParam;
    if (!PredictorParam::LoadFromYamlFile(param_.PredictorParamFileName, &predictorParam))
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure because SolverParam can't be loaded";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 设置目标预测器参数
    if (!predictor_.SetParam(predictorParam))
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure because SolverParam can't be set";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 初始化目标预测器
    if (!predictor_.Init())
    {
        log = "[" + param_.Key + "] - RobotBrainCore was initialized failure because Solver can't be initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }
    // 设置初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    initTimestamp_ = now.time_since_epoch().count();

    // 设置初始化状态
    isInitialized_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrainCore was initialized successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回初始化结果
    return true;
}

// 释放机器人大脑内核的系统资源
bool RobotBrainCore::Release()
{
    // 锁定机器人大脑内核的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑内核是否已经打开
    if (isOpened_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore was released failure because it has been opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断机器人大脑内核是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore can not be released repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 释放华睿相机的系统资源
    if (huarayCamera_.IsInitialized())
    {
        huarayCamera_.Release();
    }

    // 释放经典装甲板识别器的系统资源
    if (classicalArmorRecognizer_.IsInitialized())
    {
        classicalArmorRecognizer_.Release();
    }

    // 释放经典风车识别器的系统资源
    if (classicalWindmillRecognizer_.IsInitialized())
    {
        classicalWindmillRecognizer_.Release();
    }

    // 释放目标解算器的系统资源
    if (solver_.IsInitialized())
    {
        solver_.Release();
    }

    // 重置初始化时间戳
    initTimestamp_ = 0;

    // 重置初始化状态
    isInitialized_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrainCore was released successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回释放结果
    return true;
}

// 打开机器人大脑内核
bool RobotBrainCore::Open()
{
//    std::cout<<"robotOpen"<<std::endl;
    // 锁定机器人大脑内核的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑内核是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore was opened failure because it has not been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断机器人大脑内核是否已经打开
    if (isOpened_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore can not be opened repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置华睿相机的打开状态
    if (huarayCamera_.IsOpened())
    {
        huarayCamera_.Close();
    }

    // 打开华睿相机
    if (!huarayCamera_.Open())
    {
        log = "[" + param_.Key + "] - RobotBrainCore was opened failure because HuarayCamera can't be opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 如果华睿相机在线运行，则搜索并切换和机器人大脑内核工作模式匹配的硬件参数
    HuarayCameraParam huarayCameraParam = huarayCamera_.GetParam();
    if (huarayCameraParam.RuntimeParam.IsOnline)
    {
        // 搜索和机器人大脑内核工作模式匹配的华睿相机硬件参数
        // 检查std::vector中是否包含特定元素，详见：https://blog.csdn.net/luoyayun361/article/details/108009585
        int index = -1;
        for (int i = 0; i < huarayCameraParam.HardwareParams.size(); ++i)
        {
            if ((std::find(huarayCameraParam.HardwareParams[i].WorkModes.begin(),
                           huarayCameraParam.HardwareParams[i].WorkModes.end(),
                           param_.WorkMode) != huarayCameraParam.HardwareParams[i].WorkModes.end()))
            {
                index = i;
                break;
            }
        }

        // 如果搜索到匹配的华睿相机硬件参数，则切换华睿相机的硬件参数
        if (index >= 0)
        {
            if (huarayCamera_.SwitchHardwareParam(index))
            {
                log = "[" + param_.Key + "] - The matched HuarayCameraHardwareParam has been switched successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + param_.Key + "] - RobotBrainCore was initialized failure "\
                      "because the matched HuarayCameraHardwareParam can't be switched";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }
        }
        else
        {
            log = "[" + param_.Key + "] - RobotBrainCore was initialized failure "\
                  "because there is no matched HuarayCameraHardwareParam";
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }
    }

    // 初始化机器人大脑内核的统计信息
    statistics_.ReadCameraDataIndex = 0;
    statistics_.ProcessedCameraDataIndex = 0;
    statistics_.FightControlCommandIndex = 0;
    statistics_.BuffControlCommandIndex = 0;
    statistics_.ProcessTime = 0;
    statistics_.Fps = 0;

    // 启动机器人本体控制指令发布线程
    controlBodySwitch_ = true;
    controlBodyThread_ = std::thread(&RobotBrainCore::ControlBody, this);
    log = "[" + param_.Key + "] - RobotBrainCore's ControlBodyThread was started successful";
    logger.Save(ELogType::Info, log);


// 启动机器人本体控制指令发布线程
//    detectionSwitch_ = true;
//    detectionThread_ = std::thread(&RobotBrainCore::DetectArmor, this);
//    log = "[" + param_.Key + "] - RobotBrainCore's DetectArmorThread was started successful";
//    logger.Save(ELogType::Info, log);

// 设置打开时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    openTimestamp_ = now.time_since_epoch().count();

    // 设置打开状态
    isOpened_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrainCore was opened successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回打开结果
    return true;
}

// 关闭机器人大脑内核
bool RobotBrainCore::Close()
{
    // 锁定机器人大脑内核的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑内核是否已经暂停
    if (isPaused_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore was closed failure because it has been paused";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断机器人大脑内核是否已经打开
    if (!isOpened_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore can not be closed repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 等待机器人本体控制指令发布线程结束
    controlBodySwitch_ = false;
    if (controlBodyThread_.joinable())
    {
        controlBodyThread_.join();
        log = "[" + param_.Key + "] - RobotBrainCore's ControlBodyThread was stopped successful";
        logger.Save(ELogType::Info, log);
    }

    // 等待机器人本体提示指令发布线程结束
    notifyBodySwitch_ = false;
    if (notifyBodyThread_.joinable())
    {
        notifyBodyThread_.join();
        log = "[" + param_.Key + "] - RobotBrainCore's NotifyBodyThread was stopped successful";
        logger.Save(ELogType::Info, log);
    }

    // 等待机器人本体提示指令发布线程结束
    fittingSwitch_ = false;
    if (fittingThread_.joinable())
    {
        fittingThread_.join();
        log = "[" + param_.Key + "] - RobotBrainCore's FittingThread was stopped successful";
        logger.Save(ELogType::Info, log);
    }

    // 关闭华睿相机
    if (huarayCamera_.IsOpened())
    {
        huarayCamera_.Close();
    }

    // 重置打开时间戳
    openTimestamp_ = 0;

    // 重置打开状态
    isOpened_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrainCore was closed successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回关闭结果
    return true;
}

// 机器人大脑内核暂停工作
bool RobotBrainCore::Pause()
{
    // 锁定机器人大脑内核的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑内核是否已经打开
    if (!isOpened_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore was paused failure because it has not been opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断机器人大脑内核是否已经暂停
    if (isPaused_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore can not be paused repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 设置暂停状态
    isPaused_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrainCore was paused successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回暂停结果
    return true;
}

// 机器人大脑内核恢复工作
bool RobotBrainCore::Resume()
{
    // 锁定机器人大脑内核的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断机器人大脑内核是否已经打开
    if (!isOpened_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore was resumed failure because it has not been opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断机器人大脑内核是否处于暂停状态
    if (!isPaused_)
    {
        log = "[" + param_.Key + "] - RobotBrainCore can not be resumed repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置暂停状态
    isPaused_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - RobotBrainCore was resumed successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回恢复结果
    return true;
}

// 切换机器人大脑内核的工作模式
bool RobotBrainCore::SwitchWorkMode(const EWorkMode &workMode)
{
    // 锁定机器人大脑内核的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 初始化工作模式字符串
    std::string workModeString = std::to_string(static_cast<int>(workMode));

    // 判断机器人大脑内核是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - WorkMode(" + workModeString + ") was switched failure because "\
              "RobotBrainCore has not been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 如果华睿相机处于在线模式，则切换和机器人大脑内核工作模式匹配的硬件参数
    HuarayCameraParam huarayCameraParam = huarayCamera_.GetParam();
    if (huarayCameraParam.RuntimeParam.IsOnline)
    {
        // 搜索和机器人大脑内核工作模式匹配的华睿相机硬件参数
        // 检查std::vector中是否包含特定元素，详见：https://blog.csdn.net/luoyayun361/article/details/108009585
        int index = -1;
        for (int i = 0; i < huarayCameraParam.HardwareParams.size(); ++i)
        {
            if ((std::find(huarayCameraParam.HardwareParams[i].WorkModes.begin(),
                           huarayCameraParam.HardwareParams[i].WorkModes.end(),
                           workMode) != huarayCameraParam.HardwareParams[i].WorkModes.end()))
            {
                index = i;
                break;
            }
        }

        // 判断是否搜索到和机器人大脑内核工作模式匹配的华睿相机硬件参数
        if (index == -1)
        {
            log = "[" + param_.Key + "] - WorkMode(" + workModeString + ") was switched failure because "\
                  "there is no matched HuarayCameraHardwareParam";
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }

        // 切换华睿相机硬件参数
        if (!huarayCameraParam.HardwareParams[index].IsSelected)
        {
            // 打开华睿相机
            if (!huarayCamera_.IsOpened())
            {
                huarayCamera_.Open();
            }

            // 切换华睿相机硬件参数
            if (!huarayCamera_.SwitchHardwareParam(index))
            {
                log = "[" + param_.Key + "] - WorkMode(" + workModeString + ") was switched failure because "\
                  "the matched HuarayCameraHardwareParam can't be switched";
                logger.Save(ELogType::Error, log);
                log = LOG_END;
                logger.Save(ELogType::Info, log);
                return false;
            }
        }
    }

    // 切换机器人大脑内核的工作模式
    param_.WorkMode = workMode;

    // 记录日志信息
    log = "[" + param_.Key + "] - WorkMode(" + workModeString + ") was switched successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回切换结果
    return true;
}

// 切换机器人大脑内核目标解算器补偿参数适配的子弹速度
bool RobotBrainCore::SwitchBulletVelocity(const EBulletVelocity &bulletVelocity)
{
    // 锁定机器人大脑内核的操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 初始化子弹速度字符串
    std::string bulletVelocityString = std::to_string(static_cast<int>(bulletVelocity));

    // 判断机器人大脑内核是否初始化完毕
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - BulletVelocity(" + bulletVelocityString + ") was switched failure because "\
              "RobotBrainCore has not been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 切换机器人大脑内核目标解算器补偿参数适配的子弹速度
    param_.BulletVelocity = bulletVelocity;

    // 记录日志信息
    log = "[" + param_.Key + "] - BulletVelocity(" + bulletVelocityString + ") was switched successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回更新结果
    return true;
}


// 机器人大脑内核通过经典的OpenCV方法识别以及ceres计算在Buff模式下的击打目标点参数




// ******************************  RobotBrainCore类的私有函数  ******************************
// 新的 机器人大脑内核向机器人本体发布控制指令
void RobotBrainCore::ControlBody()
{
    // 修改线程名称
    std::string threadName = "control_robot_body";
    prctl(PR_SET_NAME, threadName.c_str());

    // 设置线程绑定的CPU内核
    // 参考网址：https://antrn.blog.csdn.net/article/details/114263105?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/qq_34440148/article/details/121603698?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/liaoxiangui/article/details/7905612
    //         https://blog.csdn.net/zxc024000/article/details/79438061
    //         https://www.bbsmax.com/A/q4zVKp9XJK/
    int coreNumber = get_nprocs();
    int coreIndex = param_.ControlBodyCpuCore;
    if ((coreIndex >= 0) && (coreIndex < coreNumber))
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(coreIndex, &mask);
        sched_setaffinity(0, sizeof(mask), &mask);
    }

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 创建上一帧华睿相机数据时间戳
    uint64_t previousCameraDataTimestamp = 0;

    // 创建上一帧战斗指令时间戳
    std::chrono::time_point<std::chrono::steady_clock> previousFightTime = std::chrono::steady_clock::now();
    uint64_t previousFightTimestamp = previousFightTime.time_since_epoch().count();

    // 创建上一帧统计信息时间戳
    std::chrono::time_point<std::chrono::steady_clock> previousStatisticsTime = std::chrono::steady_clock::now();
    uint64_t previousStatisticsTimestamp = previousStatisticsTime.time_since_epoch().count();

    // 初始化机器人大脑内核的统计信息
    RobotBrainCoreStatistics statistics;

    // 记录装甲板信息队列
    std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber>> armorRecord;

    // 记录目标装甲板序列
    std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64>> armorSequence;


    std::chrono::time_point < std::chrono::steady_clock > begaintime = std::chrono::steady_clock::now();
    float  pre_timestamp = begaintime.time_since_epoch().count() / 1000000000.0f;
//    std::cout<<"control: "<<controlBodySwitch_<<std::endl;
    // 循环发布控制指令
    while (controlBodySwitch_)
    {


        // 如果机器人大脑内核处于暂停状态，控制指令发布线程连续休眠
        if (isPaused_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        // 从华睿相机中读取一帧数据
        HuarayCameraData cameraData;
        huarayCamera_.GetData(&cameraData);
        statistics.ReadCameraDataIndex++;

        // 定义打包数据与打包成功标志位
        PredictorTool package;
        bool packageResult = false;

        // 获取决策控制的起始时间戳
        std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
        uint64_t beginTimestamp = beginTime.time_since_epoch().count();
        // 判断读取到的华睿相机数据是否已经处理完毕

        // 当前相机的时间戳是否大于之前的相机时间戳
        if (cameraData.Timestamp > previousCameraDataTimestamp)
        {
            previousCameraDataTimestamp = cameraData.Timestamp;
            statistics.ProcessedCameraDataIndex++;
//            std::cout << "ProcessData: " << statistics.ProcessedCameraDataIndex << std::endl;
            // 记录相机数据处理完毕时计算机设备的时间戳,因为外接相机后相机时间戳来自相机设备
            std::chrono::time_point<std::chrono::steady_clock> frameTime = std::chrono::steady_clock::now();
            uint64_t frameTimeStamp = frameTime.time_since_epoch().count();
            package.frameTimeStamp = frameTimeStamp;
        }
        else
        {
            continue;
        }


        // 如果存在一定下位机数据则对数据进行打包
//         std::cout<<"BodyPredictorCommand.size()  "<<BodyPredictorCommand.size()<<std::endl;
        if(BodyPredictorCommand.size() > 1)
        {
            // 在数据拷贝过程中对数据加锁
            operateMutex_.lock();
            // 相机的时间戳Timestamp是count()得到的
            //uint64_t timeStamp = cameraData.Timestamp;
            uint64_t timeStamp =  package.frameTimeStamp;
            std::vector<PredictorCommand> bodyPredictorCommand;
            bodyPredictorCommand.insert(bodyPredictorCommand.end(), BodyPredictorCommand.begin(),
                                        BodyPredictorCommand.end());
            // 清空被拷贝数据
            BodyPredictorCommand.clear();

            // 解除互斥锁
            operateMutex_.unlock();

            // 对数据进行打包，并更改打包标志位
            if (package.GetPackage(timeStamp, &bodyPredictorCommand))
            {
                // 匹配成功则消除匹配数据与之前的数据
                packageResult = true;


                //看串口数据
                // {
                //     std::cout << "---------------------------------------------------------" << std::endl;
                // std::cout << "getpackage 四元数：" <<
                //           package.Q[0] << "    " << package.Q[1] << "    " << package.Q[2] << "    " << package.Q[3]
                //           << std::endl;
                //     std::cout << "package stand: " << package.stand << std::endl;
                //
                // }

                //陀螺仪
                // {
                //     //原四元数
                Eigen::Quaterniond Q_raw(package.Q[0], package.Q[1], package.Q[2], package.Q[3]);
                //
                //     //原旋转矩阵
                Eigen::Matrix3d nowQMatrix = Q_raw.toRotationMatrix();
//                      std::cout <<"nowQMatrix: " <<nowQMatrix<<std::endl;    //顺时针为正角
                //
                //原四元数的yaw
                 double yaw_Q = atan2(nowQMatrix(1, 0), nowQMatrix(0, 0));
                //原四元数pitch
                 auto pitch_Q = static_cast<float>(atan2(nowQMatrix(2, 1), nowQMatrix(2, 2)));
                //原四元数roll
                 auto roll_Q = static_cast<float>(asin(-nowQMatrix(2, 0)));
//                std::cout << "yaw_Q: " << yaw_Q * 57.3 << "      ";    //顺时针为正角
//                std::cout << "pitch_Q: " << pitch_Q * 57.3 << "     ";
//                 std::cout << "roll_Q: " << roll_Q * 57.3 << std::endl;
                // }


            }
            else
            {
                // 匹配失败则更改标志位,若时间差距过大则清空Vector
                packageResult = false;
                std::cout<<"打包失败 false!!!!!"<<std::endl;
            }
//             std::cout<<"packageResult: "<<packageResult<<std::endl;
        }



        // 获取机器人大脑内核参数
        RobotBrainCoreParam robotBrainCoreParam;
        operateMutex_.lock();
        robotBrainCoreParam = param_;
        operateMutex_.unlock();

        // 初始化计算结果
        bool result = false;
        bool isDistanceCompensated = false;
        bool isVelocityCompensated = false;
        std::pair<float, float> distanceCompensation(0.0, 0.0);
        std::pair<float, float> velocityCompensation(0.0, 0.0);
        float distance = 0.0;
        bool top = false;
        Eigen::Matrix<double, 10, 3> a;
        Eigen::Matrix<double, 10, 1> b;
        double r = 0;

        Eigen::Vector3d  predictCradleHeadCoordinate; //预测打击点在云台坐标系的坐标和在世界坐标系的坐标
        Eigen::Vector3d  predictWorldCoordinate;
        Eigen::Vector3d  predictCradleHeadCoordinate1(0,0,0); //预测打击点在云台坐标系的坐标和在世界坐标系的坐标
        Eigen::Vector3d  predictWorldCoordinate1(0,0,0);

        // 判断机器人大脑内核是否工作在战斗模式
        bool isFightMode = false;
//        package.stand = 1;
        if (package.stand == 0 || package.stand == 1)
        {

            if ( this->param_.WorkMode == EWorkMode::ShootSmallBuff || this->param_.WorkMode == EWorkMode::ShootLargeBuff )
                this->SwitchWorkMode(EWorkMode::AutomaticShoot);

            isFightMode = true;
        }

        // 如果机器人大脑内核工作在战斗模式，则计算战斗模式下的击打目标点参数
        bool isfire = false;
//        std::cout<<"package.stand: "<<package.stand<<"  packageresult: "<<packageResult<<std::endl;
        if (package.stand == 0)
        {

            result = ComputeTargetInFightByYolo_EKF(cameraData,
                                                        robotBrainCoreParam,
                                                        &predictCradleHeadCoordinate,
                                                        &predictWorldCoordinate,
                                                        &predictCradleHeadCoordinate1,
                                                        &predictWorldCoordinate1,
                                                        packageResult,
                                                        isfire,
                                                        &armorRecord,
                                                        &armorSequence,
                                                        &package,
                                                        r);
        }
        else if(package.stand == 1)
        {

            result = ComputeTargetInFightByYolo_outpost(cameraData,
                                                    robotBrainCoreParam,
                                                    &predictCradleHeadCoordinate,
                                                    &predictWorldCoordinate,
                                                    &predictCradleHeadCoordinate1,
                                                    &predictWorldCoordinate1,
                                                    packageResult,
                                                    isfire,
                                                    &armorRecord,
                                                    &armorSequence,
                                                    &package,
                                                    r);
        }

        // 获取当前时间戳，判断是否需要下发控制指令
        bool isIssueCommand = false;
        std::chrono::time_point<std::chrono::steady_clock> nowTime = std::chrono::steady_clock::now();
        uint64_t nowTimestamp = nowTime.time_since_epoch().count();

        // 判断战斗模式下是否需要下发控制指令
        uint64_t fightControlPeriod = robotBrainCoreParam.FightControlPeriod * 1000000;
        if (isFightMode && ((nowTimestamp - previousFightTimestamp) >= fightControlPeriod))
        {
            isIssueCommand = true;
        }

//        std::cout<<"result: "<<result<<"  isIssueCommand: "<<isIssueCommand<<std::endl;
        // 使用击打目标点的世界和相对坐标解算云台yaw轴和pitch轴的偏转角度
        if (result && isIssueCommand)
        {

            // 计算云台yaw轴和pitch轴的偏转角度
            std::pair<float, float> offsetAngle(0.0, 0.0);
            std::pair<float, float> offsetAngle1(0.0, 0.0);
//            std::cout<<predictWorldCoordinate.x()<<"      "<<predictWorldCoordinate1.x()<<std::endl;
            //利用 predictCradleHeadCoordinate和predictWorldCoordinate结算角度
            solver_.Getpitchandrow(predictCradleHeadCoordinate,predictWorldCoordinate,&offsetAngle,solver_);
            solver_.Getpitchandrow(predictCradleHeadCoordinate1,predictWorldCoordinate1,&offsetAngle1,solver_);

            //distance = Solver::ComputeDistance(objectPoints, pixelPoints, huarayCamera_.GetParam().ModelParam).first;
            distance = std::sqrt(predictWorldCoordinate.x() * predictWorldCoordinate.x() + predictWorldCoordinate.y()*predictWorldCoordinate.y()+predictWorldCoordinate.z()*predictWorldCoordinate.z());
//            std::cout << "Distance " << distance << std::endl;

//            std::cout<<predictWorldCoordinate.transpose()<<std::endl;

            RobotBrainControlCommand command;
            command.ID = robotBrainCoreParam.ID;
            command.Yaw = offsetAngle.first;
            command.Pitch = offsetAngle.second;
            command.Distance = distance;
//            command.IsFire = isfire;
            command.IsFire = true;
            command.Yaw1 = offsetAngle1.first;
            std::cout<<"yaw: "<<command.Yaw<<"  pitch:  "<<command.Pitch<<"  IsFire:  "<<command.IsFire<<"  Distance:  "<<command.Distance<<std::endl;
            // 生成控制指令数据帧
            unsigned int commandFrameSize = RobotBrainControlCommand::GetFrameSize();
            unsigned char commandFrame[commandFrameSize];
            command.EncapsulateToFrame(commandFrame);

            // 通过机器人大脑将控制指令数据帧发送给下位机
            RobotBrain& robotBrain = RobotBrain::GetSingleInstance();
            robotBrain.TransmitFrame(commandFrameSize, commandFrame);
            // 更新战斗模式相关数据
            if (isFightMode)
            {
                // 更新战斗模式时间戳和控制指令索引
                previousFightTimestamp = nowTimestamp;
                statistics.FightControlCommandIndex++;

                // 清空装甲板位置缓冲区
                if (isVelocityCompensated)
                {
                    classicalArmorRecognizer_.ClearArmorBuffer();
                }
            }

        }

        // 获取决策控制的截止时间戳
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();
//        std::cout<<"end: "<<endTimestamp<<std::endl;
        // 计算控制决策的处理时间和帧率
        statistics.ProcessTime = endTimestamp - beginTimestamp;
        statistics.Fps = statistics.ProcessedCameraDataIndex * 1000000000 / (endTimestamp - openTimestamp_);

        // 更新机器人大脑内核的统计信息；每秒钟更新1次
        if ((endTimestamp - previousStatisticsTimestamp) > 1000000000)
        {
            // 更新统计时间戳
            previousStatisticsTimestamp = endTimestamp;

            // 更新统计信息
            statisticsMutex_.lock();
            statistics_ = statistics;
            statisticsMutex_.unlock();
        }
    }
}

// 机器人大脑内核向机器人本体发布提示指令
void RobotBrainCore::NotifyBody()
{
    // 修改线程名称
    std::string threadName = "notify_robot_body";
    prctl(PR_SET_NAME, threadName.c_str());

    // 设置线程绑定的CPU内核
    // 参考网址：https://antrn.blog.csdn.net/article/details/114263105?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/qq_34440148/article/details/121603698?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/liaoxiangui/article/details/7905612
    //         https://blog.csdn.net/zxc024000/article/details/79438061
    //         https://www.bbsmax.com/A/q4zVKp9XJK/
    int coreNumber = get_nprocs();
    int coreIndex = param_.NotifyBodyCpuCore;
    if ((coreIndex >= 0) && (coreIndex < coreNumber))
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(coreIndex, &mask);
        sched_setaffinity(0, sizeof(mask), &mask);
    }

    // 循环发布提示指令
    while (notifyBodySwitch_)
    {
        // 如果机器人大脑内核处于暂停状态，提示指令发布线程连续休眠
        if (isPaused_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        // 获取机器人大脑内核参数
        RobotBrainCoreParam robotBrainCoreParam;
        operateMutex_.lock();
        robotBrainCoreParam = param_;
        operateMutex_.unlock();

        // 获取华睿相机的状态信息
        HuarayCameraStatus huarayCameraStatus;
        huarayCamera_.GetStatus(&huarayCameraStatus);

        // 初始化提示指令
        RobotBrainNotifyCommand command;
        command.ID = robotBrainCoreParam.ID;
        command.WorkMode = robotBrainCoreParam.WorkMode;
        command.BulletVelocity = robotBrainCoreParam.BulletVelocity;
        command.IsInitialized = isInitialized_;
        command.IsOpened = isOpened_;
        command.IsCameraConnected = huarayCameraStatus.IsConnected;
        command.IsCameraWorking = huarayCameraStatus.IsWorking;

        // 生成提示指令数据帧
        unsigned int commandFrameSize = RobotBrainNotifyCommand::GetFrameSize();
        unsigned char commandFrame[commandFrameSize];
        command.EncapsulateToFrame(commandFrame);

        // 通过机器人大脑将提示指令数据帧发送给下位机
        RobotBrain& robotBrain = RobotBrain::GetSingleInstance();
        robotBrain.TransmitFrame(commandFrameSize, commandFrame);

        // 休眠100毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void RobotBrainCore::DetectArmor()
{
    std::string threadName = "detect_armor";
    prctl(PR_SET_NAME, threadName.c_str());

    // 设置线程绑定的CPU内核
    // 参考网址：https://antrn.blog.csdn.net/article/details/114263105?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/qq_34440148/article/details/121603698?spm=1001.2014.3001.5502
    //         https://blog.csdn.net/liaoxiangui/article/details/7905612
    //         https://blog.csdn.net/zxc024000/article/details/79438061
    //         https://www.bbsmax.com/A/q4zVKp9XJK/
    int coreNumber = get_nprocs();
    int coreIndex = param_.DetectArmorCpuCore;
    if ((coreIndex >= 0) && (coreIndex < coreNumber))
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(coreIndex, &mask);
        sched_setaffinity(0, sizeof(mask), &mask);
    }

    while (detectionSwitch_)
    {
        HuarayCameraData cameraData;
        huarayCamera_.GetData(&cameraData);

        std::vector<ClassicalArmor> detectedArmors;
        classicalArmorRecognizer_.DetectArmorsByYolo(cameraData.Image, detectedArmors);
//        std::cout << detectedArmors.size() <<std::endl;
//        cv::imshow("raw", cameraData.Image);
        std::unique_lock<std::mutex> lock(detectMutex_);
        detectArmorQueue_.emplace(detectedArmors);
    }
}


//机器人大脑内核通过经典的OpenCV加上卡尔曼滤波方法预测在战斗模式下的击打目标点参数
bool RobotBrainCore::ComputeTargetInFightByYolo_EKF(const HuarayCameraData &huarayCameraData,
                                                        const RobotBrainCoreParam &robotBrainCoreParam,
                                                        Eigen::Vector3d  *predictCradleHeadCoordinate,
                                                        Eigen::Vector3d  *predictWorldCoordinate,
                                                        Eigen::Vector3d  *predictCradleHeadCoordinate1,
                                                        Eigen::Vector3d  *predictWorldCoordinate1,
                                                        bool &packageResult,
                                                        bool &isFire,
                                                        std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber>> *armorRecord,
                                                        std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64>> *armorSequence,
                                                        PredictorTool *package,
                                                        double &r) {
    // 初始化计算结果
    bool result = false;
    double predictTime = 0.0;
    std::vector<std::pair<cv::Point2f, uint64_t>> armorLocationSequence;
    std::vector<cv::Point2f> armorLocations;

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    std::vector<ClassicalArmor> polishedArmors;
     classicalArmorRecognizer_.DetectArmorsByYolo(huarayCameraData.Image, polishedArmors);
//    std::unique_lock<std::mutex> lock(detectMutex_);
//    detectArmorCV_.wait(lock,[&]{return !detectArmorQueue_.empty();});
//    polishedArmors = detectArmorQueue_.back();
//    lock.unlock();
//    std::cout<<"polisgh: "<<polishedArmors.size()<<std::endl;
    std::vector<ClassicalArmor> filteredArmors;
    for (int i = 0; i < polishedArmors.size(); ++i)
    {

        if ((polishedArmors[i].Number != param_.IgnoredArmorNumber)
//        &&((polishedArmors[i].Number == EClassicalArmorNumber::BlueOutpost)
//            || (polishedArmors[i].Number == EClassicalArmorNumber::RedOutpost))
            )
        {

            filteredArmors.emplace_back(polishedArmors[i]);
        }
    }
    // 评估过滤后的精装甲板

    float distance;
    Eigen::Matrix<double, 3, 3> rotationMatrix;

    // 根据评估后的装甲板得分搜索目标装甲板
    ClassicalArmor targetArmor;
    static uint64 targetArmorIndex=0;
    AntiTop AntiTop_ ;
    predictor_.UpdateAntitop(&AntiTop_);
    targetArmor.Number = EClassicalArmorNumber::Invalid;

    std::vector<ClassicalArmor> targetArmorGroup;
    HuarayCameraModelParam modelParam = huarayCamera_.GetParam().ModelParam;

    std::vector<float> yaws;
    std::vector<cv::Point3f> armorCenterLocations;
    if (!filteredArmors.empty())
    {
        if (!AntiTop_.IsLocked)
        {

            AntiTop_.targetNumber = filteredArmors[0].Number;
            AntiTop_.IsLocked = true;

        }
        //从所有识别到的装甲板滤出目标装甲板
        targetArmorGroup = AntiTop_.getTargetArmor(filteredArmors);


        int tarA_size = targetArmorGroup.size();
        // std::cout<<"targetArmor size  "<<tarA_size<<std::endl;

        for (int i = 0; i < tarA_size; i++)
        {
            //获取中心点的三维坐标
            // std::cout<<"第 "<<i<<"块 center.x "<<targetArmorGroup[i].Center.x << "  .y  " <<targetArmorGroup[i].Center.y;
            Eigen::Vector3d CENTERPT = PredictorTool::getArmorCenterLocation(targetArmorGroup[i],modelParam);
            // std::cout<<"centerh "<<targetArmorGroup[i].Center<<std::endl<<std::endl;;
            cv::Point3f cvCenter;
            PredictorTool::eigenVec3d2cvPt3f(CENTERPT,&cvCenter);
            float yaw = Remap::PhiOptimization(cvCenter,modelParam,targetArmorGroup[i],package->stand);
            // std::cout<<"yaw: "<<yaw * 57.3<<std::endl;
            yaws.emplace_back(yaw);
            armorCenterLocations.emplace_back(cvCenter);
        }

        int chooseIndex = 0;

        //容器中无数据，则识别第一个装甲板
        if(!targetArmorGroup.empty())
        {
            targetArmor = targetArmorGroup[0];
        }
        predictor_.setPrivateMember(chooseIndex);



        // 使用相机数据时间戳作为目标装甲板时间戳
        targetArmor.Timestamp = huarayCameraData.Timestamp;
    }
//    std::cout<<"targetNumber: "<<static_cast<int >(targetArmor.Number)<<std::endl;
    cv::Point2f g1,g2,g3,g4,t;
    cv::Point2f targetpixel;
    // 如果找到合法的目标装甲板，则预测击打目标点
    if (targetArmor.Number != EClassicalArmorNumber::Invalid)
    {
        // 设置计算结果
        result = true;

        // 更新原始击打目标点
        targetpixel.x = targetArmor.Center.x;
        targetpixel.y = targetArmor.Center.y;

        // 更新装甲板数据缓冲区，获取目标装甲板的历史位置时序
        classicalArmorRecognizer_.UpdateArmorBuffer(targetArmor);
        classicalArmorRecognizer_.GetArmorLocationSequence(targetArmor.Number,
                                                           &armorLocationSequence);

        // 计算目标装甲板顶点的世界坐标
        std::vector<cv::Point3f> objectPoints;
        ClassicalArmorRecognizerParam recognizerParam = classicalArmorRecognizer_.GetParam();
        if ((targetArmor.Number == EClassicalArmorNumber::RedOne)||
            (targetArmor.Number == EClassicalArmorNumber::BlueOne) ||
            (targetArmor.type == 1)) {
            // 根据大装甲板的宽度和高度，计算目标装甲板顶点的世界坐标
            float width = recognizerParam.LargeArmorPhysicalWidth - 2 * recognizerParam.LightBarPhysicalWidth;
            float height = recognizerParam.LargeArmorPhysicalHeight;
            objectPoints.emplace_back(-width / 2, -height / 2, 0.0);
            objectPoints.emplace_back(width / 2, -height / 2, 0.0);
            objectPoints.emplace_back(width / 2, height / 2, 0.0);
            objectPoints.emplace_back(-width / 2, height / 2, 0.0);
        } else {
            // 根据小装甲板的宽度和高度，计算目标装甲板顶点的世界坐标
            float width = recognizerParam.SmallArmorPhysicalWidth - 2 * recognizerParam.LightBarPhysicalWidth;
            float height = recognizerParam.LargeArmorPhysicalHeight;
            objectPoints.emplace_back(-width / 2, -height / 2, 0.0);
            objectPoints.emplace_back(width / 2, -height / 2, 0.0);
            objectPoints.emplace_back(width / 2, height / 2, 0.0);
            objectPoints.emplace_back(-width / 2, height / 2, 0.0);
        }

        // 计算目标装甲板顶点的像素坐标
        std::vector<cv::Point2f> pixelPoints;


        bool isProject;
        bool isPredict;

        PredictorTool::getArmorCenterLocation(targetArmor,modelParam,&distance,&rotationMatrix);
//        distance = Solver::ComputeDistance(objectPoints, pixelPoints, huarayCamera_.GetParam().ModelParam).first;
//        rotationMatrix = Solver::ComputeDistance(objectPoints, pixelPoints, huarayCamera_.GetParam().ModelParam).second;
        auto bulletVelocity = static_cast<float>(robotBrainCoreParam.BulletVelocity) * 1000;//从米每秒 到毫米每秒
        predictTime = static_cast<float>(distance / bulletVelocity) + robotBrainCoreParam.ShootDelay;
//        std::cout << distance <<std::endl;
//        std::cout<<"predict:"<<predictTime<<std::endl;

        // 如果打包成功则将本次数据记录进队列
        if (packageResult) {
            // 更新有效状态值
            std::tuple<uint64, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber> nowTarget;
            std::get<0>(nowTarget) = package->timeStamp;
            std::get<1>(nowTarget) = distance;
            std::get<2>(nowTarget) = targetArmor.Center;
            std::get<3>(nowTarget) = package->Q;
            std::get<4>(nowTarget) = targetArmor.Number;
            armorRecord->emplace_back(nowTarget);
            if (armorRecord->size() > 10)
            {
                armorRecord->pop_front();//删除容器的第一个数据
            }
            uint64 time_sum ;
            if (armorSequence->size() == 0)
            {
                time_sum = 0;
            }
            else
            {
                time_sum = std::get<2>(armorSequence ->at(armorSequence->size() - 1)) / 1000000000;
            }
            //更新装甲板数据
            uint64 lastTime = 0;
            std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64> armorandyaw;
            std::get<0>(armorandyaw) = targetArmorGroup;
            std::get<1>(armorandyaw) = yaws;
            if (armorRecord->size() > 2)
            {
                lastTime = std::get<0>(armorRecord->at(armorRecord->size() - 2)) / 1000000000;
                std::get<2>(armorandyaw) = time_sum + (package ->timeStamp / 1000000000 - lastTime);
            }
            else
            {
                std::get<2>(armorandyaw) = 0;
            }

            armorSequence -> emplace_back(armorandyaw);
            if (armorSequence->size() > 10)
            {
                armorSequence->pop_front();
            }
        }
        //std::cout<<"target: "<<targetArmor.Center.x<<" "<<targetArmor.Center.y<<std::endl;
        // 打包成功则进行预测
        isPredict = packageResult;
//        std::cout<<"package: "<<packageResult<<std::endl;
        // 如果目标不为空，则进行预测与补偿
        if (targetArmor.Center.x != -1 && targetArmor.Center.y != -1) {
            // 设置重投影标志位
            isProject = true;
            if (packageResult) {
                // 得到预测的二维坐标,如果不预测则得到原有的坐标
                predictor_.GetAntitop(AntiTop_);
                predictor_.AutoaimPredict(isPredict,
                                          predictor_,
                                          modelParam,
                                          armorRecord,
                                          armorSequence,
                                          predictTime,
                                          predictCradleHeadCoordinate,
                                          predictWorldCoordinate,
                                          predictCradleHeadCoordinate1,
                                          predictWorldCoordinate1,
                                          &targetpixel,
                                          rotationMatrix,
                                          g1,g2,
                                          g3,g4,
                                          package->stand,
                                          isFire,
                                          r);

                armorLocations.push_back(targetpixel);


            }
        }
        t.x = predictWorldCoordinate->x()/10;
        t.y = predictWorldCoordinate->y()/10;
    }
    else if (targetArmorGroup.empty() && armorRecord->size() > 2)
    {
        uint64 nowTime = std::get<0>(armorRecord->at(armorRecord->size() - 1));
        uint64 pastTime = std::get<0>(armorRecord->at(armorRecord->size() - 2));
        double gapTime = (nowTime - pastTime) / 1000000;

        uint64 *timeSum = &std::get<2>(armorSequence->at(armorSequence->size() - 1));
        *timeSum += gapTime;
        if(*timeSum > 2000)
        {
            AntiTop_.IsLocked = false;
            AntiTop_.targetNumber = EClassicalArmorNumber::Invalid;
            AntiTop_.Ischoosearmor = false;
            predictor_.setPrivateMember(0,false);
        }

        predictor_.GetAntitop(AntiTop_);
    }


    // 更新战斗数据
    switch (robotBrainCoreParam.CachedFightDataType)
    {
        case EFightDataType::RawImage:
        {
            cv::circle(huarayCameraData.Image,targetpixel,4,cv::Scalar(0,0,255),8);
            fightDataMutex_.lock();
            fightData_.Type = EFightDataType::RawImage;
            huarayCameraData.Image.copyTo(fightData_.Image);
            fightDataMutex_.unlock();

            break;
        }

        default:
            break;
    }
    // 返回计算结果
    return result;
}

bool RobotBrainCore::ComputeTargetInFightByYolo_outpost(const HuarayCameraData &huarayCameraData,
                                                             const RobotBrainCoreParam &robotBrainCoreParam,
                                                             Eigen::Vector3d  *predictCradleHeadCoordinate,
                                                             Eigen::Vector3d  *predictWorldCoordinate,
                                                             Eigen::Vector3d  *predictCradleHeadCoordinate1,
                                                             Eigen::Vector3d  *predictWorldCoordinate1,
                                                             bool &packageResult,
                                                             bool &isFire,
                                                             std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber>> *armorRecord,
                                                             std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64>> *armorSequence,
                                                             PredictorTool *package,
                                                             double &r){
//    std::cout<<"stand: "<<package->stand<<std::endl;
//    std::cout<<"ComputeTargetInFightByYolo_outpost"<<std::endl;
    // 初始化计算结果
    bool result = false;
    double predictTime = 0.0;
    std::vector<std::pair<cv::Point2f, uint64_t>> armorLocationSequence;
    std::vector<cv::Point2f> armorLocations;

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();


    classicalArmorRecognizer_.SetParamColor(package->IsBlue);
    std::vector<ClassicalArmor> polishedArmors;
    classicalArmorRecognizer_.DetectArmorsByYolo(huarayCameraData.Image, polishedArmors);
//     std::cout<<"IsBlue: "<<package->IsBlue<<"  polishSize :  "<<polishedArmors.size()<<std::endl;
    // for (int i = 0; i < polishedArmors.size(); i++)
//    if (!polishedArmors.empty())
//     std::cout<<"polishedArmors: "<<static_cast<int>(polishedArmors.at(0).Number)<<std::endl;
    // std::cout <<polishedArmors.size() <<std::endl;
    // 过滤精装甲板
    std::vector<ClassicalArmor> filteredArmors;
    for (int i = 0; i < polishedArmors.size(); ++i)
    {

        if (
            (polishedArmors[i].Number == EClassicalArmorNumber::BlueOutpost)
            || (polishedArmors[i].Number == EClassicalArmorNumber::RedOutpost))
        {

            filteredArmors.emplace_back(polishedArmors[i]);
        }
    }
    // 评估过滤后的精装甲板

    float distance;
    Eigen::Matrix<double, 3, 3> rotationMatrix;

    // 根据评估后的装甲板得分搜索目标装甲板
    ClassicalArmor targetArmor;
    static uint64 targetArmorIndex=0;
    AntiTop AntiTop_ ;
    predictor_.UpdateAntitop(&AntiTop_);
    targetArmor.Number = EClassicalArmorNumber::Invalid;

    std::vector<ClassicalArmor> targetArmorGroup;
    HuarayCameraModelParam modelParam = huarayCamera_.GetParam().ModelParam;

    std::vector<float> yaws;
    std::vector<cv::Point3f> armorCenterLocations;
    if (!filteredArmors.empty())
    {
        if (!AntiTop_.IsLocked)
        {

            AntiTop_.targetNumber = filteredArmors[0].Number;
            AntiTop_.IsLocked = true;

        }
        //从所有识别到的装甲板滤出目标装甲板
        targetArmorGroup = AntiTop_.getTargetArmor(filteredArmors);


        int tarA_size = targetArmorGroup.size();
//         std::cout<<"targetArmor size  "<<tarA_size<<std::endl;

        for (int i = 0; i < tarA_size; i++)
        {
            //获取中心点的三维坐标
            // std::cout<<"第 "<<i<<"块 center.x "<<targetArmorGroup[i].Center.x << "  .y  " <<targetArmorGroup[i].Center.y;
            Eigen::Vector3d CENTERPT = PredictorTool::getArmorCenterLocation(targetArmorGroup[i],modelParam);
            // std::cout<<"centerh "<<targetArmorGroup[i].Center<<std::endl<<std::endl;;
            cv::Point3f cvCenter;
            PredictorTool::eigenVec3d2cvPt3f(CENTERPT,&cvCenter);
            float yaw = Remap::PhiOptimization(cvCenter,modelParam,targetArmorGroup[i],package->stand);
//             std::cout<<"yaw: "<<yaw * 57.3<<std::endl;
            yaws.emplace_back(yaw);
            armorCenterLocations.emplace_back(cvCenter);
        }

        int chooseIndex = 0;

        //容器中无数据，则识别第一个装甲板
        if(!targetArmorGroup.empty())
        {
            targetArmor = targetArmorGroup[0];
        }
        predictor_.setPrivateMember(chooseIndex);



        // 使用相机数据时间戳作为目标装甲板时间戳
        targetArmor.Timestamp = huarayCameraData.Timestamp;
    }


    cv::Point2f g1,g2,g3,g4,t;
    cv::Point2f targetpixel;
    cv::Point2f targetpixelCenter;
    // 如果找到合法的目标装甲板，则预测击打目标点
    if (targetArmor.Number != EClassicalArmorNumber::Invalid)
    {
        // 设置计算结果
        result = true;

        // 更新原始击打目标点
        targetpixel.x = targetArmor.Center.x;
        targetpixel.y = targetArmor.Center.y;

        // 更新装甲板数据缓冲区，获取目标装甲板的历史位置时序
        classicalArmorRecognizer_.UpdateArmorBuffer(targetArmor);

        classicalArmorRecognizer_.GetArmorLocationSequence(targetArmor.Number,
                                                           &armorLocationSequence);


        bool isProject;
        bool isPredict;

        PredictorTool::getArmorCenterLocation(targetArmor,modelParam,&distance,&rotationMatrix);


        auto bulletVelocity = static_cast<float>(robotBrainCoreParam.BulletVelocity) * 1000;//从米每秒 到毫米每秒
        bulletVelocity = 22500;//临时改的，实在没找到前面哪里给了个3000
        predictTime = static_cast<float>(distance / bulletVelocity) + robotBrainCoreParam.ShootDelay  ;
//         std::cout<<"predictTime:  "<<predictTime<< "velocity: " << bulletVelocity<<" shootDelay: "<<robotBrainCoreParam.ShootDelay<<std::endl;
        // std::cout << distance <<std::endl;
        // std::cout << std::endl;
        HuarayCameraModelParam modelParam = huarayCamera_.GetParam().ModelParam;
        // 如果打包成功则将本次数据记录进队列
        if (packageResult)
        {
            // 更新有效状态值
            std::tuple<uint64, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber> nowTarget;
            std::get<0>(nowTarget) = package->timeStamp;
            std::get<1>(nowTarget) = distance;
            std::get<2>(nowTarget) = targetArmor.Center;
            std::get<3>(nowTarget) = package->Q;
            std::get<4>(nowTarget) = targetArmor.Number;
            armorRecord->emplace_back(nowTarget);
            if (armorRecord->size() > 10)
            {
                armorRecord->pop_front();//删除容器的第一个数据
            }
            uint64 time_sum ;
            if (armorSequence->size() == 0)
            {
                time_sum = 0;
            }
            else
            {
                time_sum = std::get<2>(armorSequence ->at(armorSequence->size() - 1)) / 1000000000;
            }
            //更新装甲板数据
            uint64 lastTime = 0;
            std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64> armorandyaw;
            std::get<0>(armorandyaw) = targetArmorGroup;
            std::get<1>(armorandyaw) = yaws;
            if (armorRecord->size() > 2)
            {
                lastTime = std::get<0>(armorRecord->at(armorRecord->size() - 2)) / 1000000000;
                std::get<2>(armorandyaw) = time_sum + (package ->timeStamp / 1000000000 - lastTime);
            }
            else
            {
                std::get<2>(armorandyaw) = 0;
            }

            armorSequence -> emplace_back(armorandyaw);
            if (armorSequence->size() > 10)
            {
                armorSequence->pop_front();
            }
        }

        // 打包成功则进行预测
        isPredict = packageResult;
        // 如果目标不为空，则进行预测与补偿
        if (targetArmor.Center.x != -1 && targetArmor.Center.y != -1) {
            // 设置重投影标志位
            isProject = true;
            if (packageResult) {
                // 得到预测的二维坐标,如果不预测则得到原有的坐标
                predictor_.GetAntitop(AntiTop_);

                double ArmorWidth = 134.0;
                double dx_pix = targetArmor.RightLower.x - targetArmor.LeftLower.x;
                double w_pix =(3538 * ArmorWidth) /distance;
                bool is_okFire = dx_pix / w_pix > 0.65 ? true : false;

                predictor_.AutoaimPredict_outpost(isPredict,
                                                  predictor_,
                                                  modelParam,
                                                  armorRecord,
                                                  armorSequence,
                                                  predictTime,
                                                  predictCradleHeadCoordinate,
                                                  predictWorldCoordinate,
                                                  predictCradleHeadCoordinate1,
                                                  predictWorldCoordinate1,
                                                  &targetpixel,
                                                  &targetpixelCenter,
                                                  rotationMatrix,
                                                  g1,g2,
                                                  g3,g4,
                                                  is_okFire,
                                                  isFire,
                                                  r);

                armorLocations.push_back(targetpixel);


                if(AntiTop_.IsrotateRight)
                {
//                    if (is_okFire && yaws[0] >  -0.0/57.3)
                    if (yaws[0] >  -0.0/57.3)
                        isFire = true;
                }
                else
                {
//                    if (is_okFire && yaws[0]  <  0.0/57.3)
                    if (yaws[0]  <  0.0/57.3)
                        isFire = true;
                }
//                 std::cout<<"isFire: "<<isFire<<"   Isright: "<<AntiTop_.IsrotateRight<<"   dix/w: "<<(dx_pix / w_pix)<<" yaw: "<<yaws[0]*57.3<<std::endl;
            }
        }
        t.x = predictWorldCoordinate->x()/10;
        t.y = predictWorldCoordinate->y()/10;

    }
    else if (targetArmorGroup.empty() && armorRecord->size() > 2)
    {
        uint64 nowTime = std::get<0>(armorRecord->at(armorRecord->size() - 1));
        uint64 pastTime = std::get<0>(armorRecord->at(armorRecord->size() - 2));
        double gapTime = (nowTime - pastTime) / 1000000;

        uint64 *timeSum = &std::get<2>(armorSequence->at(armorSequence->size() - 1));
        *timeSum += gapTime;
        if(*timeSum > 1000)
        {
            AntiTop_.IsLocked = false;
            AntiTop_.targetNumber = EClassicalArmorNumber::Invalid;
            AntiTop_.Ischoosearmor = false;
            predictor_.setPrivateMember(0,false);
        }
        predictor_.GetAntitop(AntiTop_);
    }


    // 更新战斗数据
    switch (robotBrainCoreParam.CachedFightDataType)
    {

        case EFightDataType::RawImage:
        {

            cv::circle(huarayCameraData.Image,targetpixel,4,cv::Scalar(0,0,255),8);
//            cv::circle(huarayCameraData.Image,targetpixelCenter,4,cv::Scalar(0,0,255),8);
            cv::line(huarayCameraData.Image,targetArmor.Center,targetpixel,cv::Scalar(0,255,0),4);
            fightDataMutex_.lock();
            fightData_.Type = EFightDataType::RawImage;
            huarayCameraData.Image.copyTo(fightData_.Image);
            fightDataMutex_.unlock();

            break;
        }
        default:
            break;
    }
    // 返回计算结果
    return result;
}

cv::Point2f RobotBrainCore::MeanFiltering_LU(cv::Point2f LU) {
    if(lu.size() > 20)
    {
        lu.erase(lu.begin() , lu.begin() + 10);
        lu.push_back(LU);
    } else{
        lu.push_back(LU);
    }
    float sum_x = 0, sum_y = 0;
    for(int i = 0 ; i<lu.size() ; i++)
    {
        sum_x += lu[i].x;
        sum_y += lu[i].y;
    }
    cv::Point2f LU_mean;
    LU_mean.x = sum_x/lu.size();
    LU_mean.y = sum_y/lu.size();
    return LU_mean;
}
cv::Point2f RobotBrainCore::MeanFiltering_LD(cv::Point2f LD) {
    if(ld.size() > 20)
    {
        ld.erase(ld.begin() , ld.begin() + 10);
        ld.push_back(LD);
    } else{
        ld.push_back(LD);
    }
    float sum_x = 0, sum_y = 0;
    for(int i = 0 ; i < ld.size() ; i++)
    {
        sum_x += ld[i].x;
        sum_y += ld[i].y;
    }
    cv::Point2f LD_mean;
    LD_mean.x = sum_x/ld.size();
    LD_mean.y = sum_y/ld.size();
    return LD_mean;
}

cv::Point2f RobotBrainCore::MeanFiltering_RU(cv::Point2f RU) {
    if(ru.size() > 20)
    {
        ru.erase(ru.begin() , ru.begin() + 10);
        ru.push_back(RU);
    } else{
        ru.push_back(RU);
    }
    float sum_x = 0, sum_y = 0;
    for(int i = 0 ; i < ru.size() ; i++)
    {
        sum_x = sum_x + ru[i].x;
        sum_y = sum_y + ru[i].y;
    }
    cv::Point2f RU_mean;
    RU_mean.x = sum_x/ru.size();
    RU_mean.y = sum_y/ru.size();
    return RU_mean;
}

cv::Point2f RobotBrainCore::MeanFiltering_RD(cv::Point2f RD) {
    if(rd.size() > 20)
    {
        rd.erase(rd.begin() , rd.begin() + 10);
        rd.push_back(RD);
    } else{
        rd.push_back(RD);
    }
    float sum_x = 0, sum_y = 0;
    for(int i = 0 ; i < rd.size() ; i++)
    {
        sum_x += rd[i].x;
        sum_y += rd[i].y;
    }
    cv::Point2f RD_mean;
    RD_mean.x = sum_x/rd.size();
    RD_mean.y = sum_y/rd.size();
    return RD_mean;
}