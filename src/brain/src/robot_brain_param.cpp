//
// Created by plutoli on 2021/8/12.
//

#include "robot_brain_param.h"

// ******************************  RobotBrainParam类的公有函数  ******************************

// 构造函数
RobotBrainParam::RobotBrainParam():
    Key(),
    ScanCpuCore(-1),
    SerialPortParamFileName(),
    RobotBrainCoreParams()
{
}

// 从yaml配置文件中加载机器人大脑参数
bool RobotBrainParam::LoadFromYamlFile(const std::string &yamlFileName, RobotBrainParam *robotBrainParam)
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断yaml配置文件是否存在
    if (::access(yamlFileName.c_str(), F_OK) == -1)
    {
        log = "RobotBrainParam was loaded failure because yaml file does not exist";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断yaml配置文件是否可读
    if (::access(yamlFileName.c_str(), R_OK) == -1)
    {
        log = "RobotBrainParam was loaded failure because yaml file can not be read";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        log = "RobotBrainParam was loaded failure because yaml file can not be opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 读取Key参数
    if ((!fileStorage["Key"].isNone()) && (fileStorage["Key"].isString()))
    {
        robotBrainParam->Key = static_cast<std::string>(fileStorage["Key"]);
        log = "[" + robotBrainParam->Key + "] - RobotBrainParam's Key was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + robotBrainParam->Key + "] - RobotBrainParam's Key was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ScanCpuCore参数
    if ((!fileStorage["ScanCpuCore"].isNone()) && (fileStorage["ScanCpuCore"].isInt()))
    {
        robotBrainParam->ScanCpuCore = static_cast<int>(fileStorage["ScanCpuCore"]);
        log = "[" + robotBrainParam->Key + "] - RobotBrainParam's ScanCpuCore was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + robotBrainParam->Key + "] - RobotBrainParam's ScanCpuCore was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取SerialPortParamFileName参数
    if ((!fileStorage["SerialPortParamFileName"].isNone()) && (fileStorage["SerialPortParamFileName"].isString()))
    {
        robotBrainParam->SerialPortParamFileName = static_cast<std::string>(fileStorage["SerialPortParamFileName"]);
        log = "[" + robotBrainParam->Key + "] - RobotBrainParam's SerialPortParamFileName was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + robotBrainParam->Key + "] - RobotBrainParam's SerialPortParamFileName was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取机器人大脑内核参数集合
    cv::FileNode robotBrainCoreParamsNode = fileStorage["RobotBrainCoreParams"];
    if (!robotBrainCoreParamsNode.empty())
    {
        unsigned int index = 0;
        cv::FileNodeIterator iterator = robotBrainCoreParamsNode.begin();
        while (iterator != robotBrainCoreParamsNode.end())
        {
            // 创建机器人大脑内核参数
            RobotBrainCoreParam robotBrainCoreParam;

            // 读取Key参数
            if ((!(*iterator)["Key"].isNone()) && ((*iterator)["Key"].isString()))
            {
                robotBrainCoreParam.Key = static_cast<std::string>((*iterator)["Key"]);
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "Key was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "Key was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取ID参数
            if ((!(*iterator)["ID"].isNone()) && ((*iterator)["ID"].isInt()))
            {
                robotBrainCoreParam.ID = static_cast<int>((*iterator)["ID"]);
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "ID was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "ID was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取HuarayCameraParamFileName参数
            if ((!(*iterator)["HuarayCameraParamFileName"].isNone()) &&
                ((*iterator)["HuarayCameraParamFileName"].isString()))
            {
                auto fileName = static_cast<std::string>((*iterator)["HuarayCameraParamFileName"]);
                robotBrainCoreParam.HuarayCameraParamFileName = fileName;
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "HuarayCameraParamFileName was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "HuarayCameraParamFileName was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取ClassicalArmorRecognizerParamFileName参数
            if ((!(*iterator)["ClassicalArmorRecognizerParamFileName"].isNone()) &&
                ((*iterator)["ClassicalArmorRecognizerParamFileName"].isString()))
            {
                auto fileName = static_cast<std::string>((*iterator)["ClassicalArmorRecognizerParamFileName"]);
                robotBrainCoreParam.ClassicalArmorRecognizerParamFileName = fileName;
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "ClassicalArmorRecognizerParamFileName was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "ClassicalArmorRecognizerParamFileName was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取ClassicalWindmillRecognizerParamFileName参数
            if ((!(*iterator)["ClassicalWindmillRecognizerParamFileName"].isNone()) &&
                ((*iterator)["ClassicalWindmillRecognizerParamFileName"].isString()))
            {
                auto fileName = static_cast<std::string>((*iterator)["ClassicalWindmillRecognizerParamFileName"]);
                robotBrainCoreParam.ClassicalWindmillRecognizerParamFileName = fileName;
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "ClassicalWindmillRecognizerParamFileName was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "ClassicalWindmillRecognizerParamFileName was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取SolverParamFileName参数
            if ((!(*iterator)["SolverParamFileName"].isNone()) && ((*iterator)["SolverParamFileName"].isString()))
            {
                auto fileName = static_cast<std::string>((*iterator)["SolverParamFileName"]);
                robotBrainCoreParam.SolverParamFileName = fileName;
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "SolverParamFileName was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "SolverParamFileName was loaded failure";
                logger.Save(ELogType::Error, log);
            }
            // 读取PredictorParamFileName参数
            if ((!(*iterator)["PredictorParamFileName"].isNone()) && ((*iterator)["PredictorParamFileName"].isString()))
            {
                auto fileName = static_cast<std::string>((*iterator)["PredictorParamFileName"]);
                robotBrainCoreParam.PredictorParamFileName = fileName;
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "PredictorParamFileName was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "PredictorParamFileName was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取WorkMode参数
            if ((!(*iterator)["WorkMode"].isNone()) && ((*iterator)["WorkMode"].isInt()))
            {
                //TODO
                // EWorkMode workMode;
                // if (SystemConfigurator::ConvertToWorkMode(static_cast<int>((*iterator)["WorkMode"]),
                //                                           &workMode))
                // {
                //     robotBrainCoreParam.WorkMode = workMode;
                //     log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                //       "WorkMode was loaded successful";
                //     logger.Save(ELogType::Info, log);
                // }
                // else
                // {
                //     log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                //       "WorkMode was converted failure";
                //     logger.Save(ELogType::Error, log);
                // }
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "WorkMode was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取BulletVelocity参数
            if ((!(*iterator)["BulletVelocity"].isNone()) && ((*iterator)["BulletVelocity"].isInt()))
            {
                //TODO
                // EBulletVelocity bulletVelocity;
                // if (SystemConfigurator::ConvertToBulletVelocity(static_cast<int>((*iterator)["BulletVelocity"]),
                //                                                 &bulletVelocity))
                // {
                //     robotBrainCoreParam.BulletVelocity = bulletVelocity;
                //     log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                //       "BulletVelocity was loaded successful";
                //     logger.Save(ELogType::Info, log);
                // }
                // else
                // {
                //     log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                //       "BulletVelocity was converted failure";
                //     logger.Save(ELogType::Error, log);
                // }
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "BulletVelocity was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取IgnoredArmorNumber参数
            if ((!(*iterator)["IgnoredArmorNumber"].isNone()) && ((*iterator)["IgnoredArmorNumber"].isInt()))
            {
                EClassicalArmorNumber armorNumber;
                if (ClassicalArmor::ConvertToClassicalArmorNumber(static_cast<int>((*iterator)["IgnoredArmorNumber"]),
                                                                  &armorNumber))
                {
                    robotBrainCoreParam.IgnoredArmorNumber = armorNumber;
                    log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "IgnoredArmorNumber was loaded successful";
                    logger.Save(ELogType::Info, log);
                }
                else
                {
                    log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "IgnoredArmorNumber was converted failure";
                    logger.Save(ELogType::Error, log);
                }
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "IgnoredArmorNumber was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取ShootDelay参数
            if ((!(*iterator)["ShootDelay"].isNone())&& ((*iterator)["ShootDelay"].isReal()))
            {
                robotBrainCoreParam.ShootDelay = static_cast<float >((*iterator)["ShootDelay"]);
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "ShootDelay was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "ShootDelay was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取FightControlPeriod参数
            if ((!(*iterator)["FightControlPeriod"].isNone()) && ((*iterator)["FightControlPeriod"].isInt()))
            {
                robotBrainCoreParam.FightControlPeriod = static_cast<int>((*iterator)["FightControlPeriod"]);
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "FightControlPeriod was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "FightControlPeriod was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取BuffControlPeriod参数
            if ((!(*iterator)["BuffControlPeriod"].isNone()) && ((*iterator)["BuffControlPeriod"].isInt()))
            {
                robotBrainCoreParam.BuffControlPeriod = static_cast<int>((*iterator)["BuffControlPeriod"]);
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "BuffControlPeriod was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "BuffControlPeriod was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取CachedFightDataType参数
            if ((!(*iterator)["CachedFightDataType"].isNone()) && ((*iterator)["CachedFightDataType"].isInt()))
            {
                EFightDataType fightDataType;
                if (RobotFightData::ConvertToFightDataType(static_cast<int>((*iterator)["CachedFightDataType"]),
                                                           &fightDataType))
                {
                    robotBrainCoreParam.CachedFightDataType = fightDataType;
                    log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                          "CachedFightDataType was loaded successful";
                    logger.Save(ELogType::Info, log);
                }
                else
                {
                    log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                          "CachedFightDataType was converted failure";
                    logger.Save(ELogType::Error, log);
                }
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "CachedFightDataType was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取CachedBuffDataType参数
            if ((!(*iterator)["CachedBuffDataType"].isNone()) && ((*iterator)["CachedBuffDataType"].isInt()))
            {
                EBuffDataType buffDataType;
                if (RobotBuffData::ConvertToBuffDataType(static_cast<int>((*iterator)["CachedBuffDataType"]),
                                                         &buffDataType))
                {
                    robotBrainCoreParam.CachedBuffDataType = buffDataType;
                    log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                          "CachedBuffDataType was loaded successful";
                    logger.Save(ELogType::Info, log);
                }
                else
                {
                    log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                          "CachedBuffDataType was converted failure";
                    logger.Save(ELogType::Error, log);
                }
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "CachedBuffDataType was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取ControlBodyCpuCore参数
            if ((!(*iterator)["ControlBodyCpuCore"].isNone()) && ((*iterator)["ControlBodyCpuCore"].isInt()))
            {
                robotBrainCoreParam.ControlBodyCpuCore = static_cast<int>((*iterator)["ControlBodyCpuCore"]);
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "ControlBodyCpuCore was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "ControlBodyCpuCore was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取NotifyBodyCpuCore参数
            if ((!(*iterator)["NotifyBodyCpuCore"].isNone()) && ((*iterator)["NotifyBodyCpuCore"].isInt()))
            {
                robotBrainCoreParam.NotifyBodyCpuCore = static_cast<int>((*iterator)["NotifyBodyCpuCore"]);
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "NotifyBodyCpuCore was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "NotifyBodyCpuCore was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取FittingCpuCore参数
            if ((!(*iterator)["FittingCpuCore"].isNone()) && ((*iterator)["FittingCpuCore"].isInt()))
            {
                robotBrainCoreParam.FittingCpuCore = static_cast<int>((*iterator)["FittingCpuCore"]);
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "FittingCpuCore was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams[" + std::to_string(index) + "]'s "\
                      "FittingCpuCore was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 保存大脑内核参数
            robotBrainParam->RobotBrainCoreParams.emplace_back(robotBrainCoreParam);

            // 大脑内核参数索引和迭代器累加
            index++;
            iterator++;
        }
    }
    else
    {
        log = "[" + robotBrainParam->Key + "] - RobotBrainCoreParams was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 关闭文件存储器
    fileStorage.release();

    // 记录日志信息
    log = "[" + robotBrainParam->Key + "] - RobotBrainParam was loaded completely";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回加载结果
    return true;
}