//
// Created by plutoli on 2022/4/5.
//

#include "huaray_camera_param.h"


// ******************************  HuarayCameraParam类的公有函数  ******************************

// 构造函数
HuarayCameraParam::HuarayCameraParam():
    Key(),
    RuntimeParam(),
    ModelParam(),
    HardwareParams()
{
}

// 从yaml配置文件中加载华睿相机参数
bool HuarayCameraParam::LoadFromYamlFile(const std::string &yamlFileName, HuarayCameraParam *huarayCameraParam)
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
        log = "HuarayCameraParam was loaded failure because yaml file does not exist";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断yaml配置文件是否可读
    if (::access(yamlFileName.c_str(), R_OK) == -1)
    {
        log = "HuarayCameraParam was loaded failure because yaml file can not be read";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        log = "HuarayCameraParam was loaded failure because yaml file can not be opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 读取Key参数
    if ((!fileStorage["Key"].isNone()) && (fileStorage["Key"].isString()))
    {
        huarayCameraParam->Key = static_cast<std::string>(fileStorage["Key"]);
        log = "[" + huarayCameraParam->Key + "] - huarayCameraParam's Key was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + huarayCameraParam->Key + "] - huarayCameraParam's Key was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取相机的运行时参数
    cv::FileNode runtimeParamNode = fileStorage["RuntimeParam"];
    if (!runtimeParamNode.empty())
    {
        // 读取IsOnline参数
        if ((!runtimeParamNode["IsOnline"].isNone()) && (runtimeParamNode["IsOnline"].isInt()))
        {
            huarayCameraParam->RuntimeParam.IsOnline = static_cast<int>(runtimeParamNode["IsOnline"]);
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's IsOnline was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's IsOnline was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取IsRecordVideo参数
        if ((!runtimeParamNode["IsRecordVideo"].isNone()) && (runtimeParamNode["IsRecordVideo"].isInt()))
        {
            huarayCameraParam->RuntimeParam.IsRecordVideo = static_cast<int>(runtimeParamNode["IsRecordVideo"]);
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's IsRecordVideo was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's IsRecordVideo was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取UpdateDataCpuCore参数
        if ((!runtimeParamNode["UpdateDataCpuCore"].isNone()) && (runtimeParamNode["UpdateDataCpuCore"].isInt()))
        {
            huarayCameraParam->RuntimeParam.UpdateDataCpuCore = static_cast<int>(runtimeParamNode["UpdateDataCpuCore"]);
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's UpdateDataCpuCore was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's UpdateDataCpuCore was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取RecordVideoCpuCore参数
        if ((!runtimeParamNode["RecordVideoCpuCore"].isNone()) && (runtimeParamNode["RecordVideoCpuCore"].isInt()))
        {
            huarayCameraParam->RuntimeParam.RecordVideoCpuCore = static_cast<int>(runtimeParamNode["RecordVideoCpuCore"]);
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's RecordVideoCpuCore was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's RecordVideoCpuCore was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取RecordVideoFps参数
        if ((!runtimeParamNode["RecordVideoFps"].isNone()) && (runtimeParamNode["RecordVideoFps"].isReal()))
        {
            huarayCameraParam->RuntimeParam.RecordVideoFps = static_cast<double>(runtimeParamNode["RecordVideoFps"]);
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's RecordVideoFps was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's RecordVideoFps was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取OfflineVideoName参数
        if ((!runtimeParamNode["OfflineVideoName"].isNone()) && (runtimeParamNode["OfflineVideoName"].isString()))
        {
            huarayCameraParam->RuntimeParam.OfflineVideoName = static_cast<std::string>(runtimeParamNode["OfflineVideoName"]);
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's OfflineVideoName was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's OfflineVideoName was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取RecordVideoPath参数
        if ((!runtimeParamNode["RecordVideoPath"].isNone()) && (runtimeParamNode["RecordVideoPath"].isString()))
        {
            huarayCameraParam->RuntimeParam.RecordVideoPath = static_cast<std::string>(runtimeParamNode["RecordVideoPath"]);
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's RecordVideoPath was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam's RecordVideoPath was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + huarayCameraParam->Key + "] - HuarayCameraRuntimeParam was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取相机的模型参数
    cv::FileNode modelParamNode = fileStorage["ModelParam"];
    if (!modelParamNode.empty())
    {
        // 读取CvIntrinsics参数
        if ((!modelParamNode["CvIntrinsics"].isNone()) && (modelParamNode["CvIntrinsics"].isMap()))
        {
            modelParamNode["CvIntrinsics"] >> huarayCameraParam->ModelParam.CvIntrinsics;
            huarayCameraParam->ModelParam.EigenIntrinsics.resize(huarayCameraParam->ModelParam.CvIntrinsics.rows,
                                                                 huarayCameraParam->ModelParam.CvIntrinsics.cols);
            cv::cv2eigen(huarayCameraParam->ModelParam.CvIntrinsics, huarayCameraParam->ModelParam.EigenIntrinsics);
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvIntrinsics was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvIntrinsics was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取CvExtrinsics参数
        if ((!modelParamNode["CvExtrinsics"].isNone()) && (modelParamNode["CvExtrinsics"].isMap()))
        {
            modelParamNode["CvExtrinsics"] >> huarayCameraParam->ModelParam.CvExtrinsics;
            cv::cv2eigen(huarayCameraParam->ModelParam.CvExtrinsics,
                         huarayCameraParam->ModelParam.EigenExtrinsics.matrix());
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvExtrinsics was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvExtrinsics was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取CvDistortions参数
        if ((!modelParamNode["CvDistortions"].isNone()) && (modelParamNode["CvDistortions"].isMap()))
        {
            modelParamNode["CvDistortions"] >> huarayCameraParam->ModelParam.CvDistortions;
            cv::cv2eigen(huarayCameraParam->ModelParam.CvDistortions,
                         huarayCameraParam->ModelParam.EigenDistortions);
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvDistortions was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + huarayCameraParam->Key + "] - HuarayCameraModelParam's CvDistortions was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + huarayCameraParam->Key + "] - HuarayCameraModelParam was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取相机的硬件参数
    cv::FileNode hardwareParamsNode = fileStorage["HardwareParams"];
    if (!hardwareParamsNode.empty())
    {
        unsigned int hardwareParamIndex = 0;
        cv::FileNodeIterator hardwareParamIterator = hardwareParamsNode.begin();
        while (hardwareParamIterator != hardwareParamsNode.end())
        {
            // 创建硬件参数
            HuarayCameraHardwareParam hardwareParam;

            // 读取WorkModes参数
            cv::FileNode workModesNode = (*hardwareParamIterator)["WorkModes"];
            if (!workModesNode.empty())
            {
                unsigned int workModeIndex = 0;
                cv::FileNodeIterator workModeIterator = workModesNode.begin();
                while (workModeIterator != workModesNode.end())
                {
                    if ((*workModeIterator).isInt())
                    {
                        EWorkMode workMode;
                        if (SystemConfigurator::ConvertToWorkMode(static_cast<int>(*workModeIterator),
                                                                  &workMode))
                        {
                            hardwareParam.WorkModes.emplace_back(workMode);
                            log = "[" + huarayCameraParam->Key + "] - "\
                                  "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]."\
                                  "WorkModes[" + std::to_string(workModeIndex) + "] was loaded successful";
                            logger.Save(ELogType::Info, log);
                        }
                        else
                        {
                            log = "[" + huarayCameraParam->Key + "] - "\
                                  "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]."\
                                  "WorkModes[" + std::to_string(workModeIndex) + "] was converted failure";
                            logger.Save(ELogType::Error, log);
                        }
                    }
                    else
                    {
                        log = "[" + huarayCameraParam->Key + "] - "\
                              "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]."\
                              "WorkModes[" + std::to_string(workModeIndex) + "] was loaded failure";
                        logger.Save(ELogType::Error, log);
                    }

                    // 工作模式索引和迭代器累加
                    workModeIndex++;
                    workModeIterator++;
                }
            }
            else
            {
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "WorkModes was loaded failure because it is empty";
                logger.Save(ELogType::Error, log);
            }

            // 读取IsBalanceWhiteAuto参数
            if ((!(*hardwareParamIterator)["IsBalanceWhiteAuto"].isNone()) &&
                ((*hardwareParamIterator)["IsBalanceWhiteAuto"].isInt()))
            {
                hardwareParam.IsBalanceWhiteAuto = static_cast<int>((*hardwareParamIterator)["IsBalanceWhiteAuto"]);
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "IsBalanceWhiteAuto was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "IsBalanceWhiteAuto was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取IsExposureAuto参数
            if ((!(*hardwareParamIterator)["IsExposureAuto"].isNone()) &&
                ((*hardwareParamIterator)["IsExposureAuto"].isInt()))
            {
                hardwareParam.IsExposureAuto = static_cast<int>((*hardwareParamIterator)["IsExposureAuto"]);
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "IsExposureAuto was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "IsExposureAuto was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取ExposureTime参数
            if ((!(*hardwareParamIterator)["ExposureTime"].isNone()) &&
                ((*hardwareParamIterator)["ExposureTime"].isReal()))
            {
                hardwareParam.ExposureTime = static_cast<float>((*hardwareParamIterator)["ExposureTime"]);
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "ExposureTime was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "ExposureTime was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取GainRaw参数
            if ((!(*hardwareParamIterator)["GainRaw"].isNone()) && ((*hardwareParamIterator)["GainRaw"].isReal()))
            {
                hardwareParam.GainRaw = static_cast<float>((*hardwareParamIterator)["GainRaw"]);
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "GainRaw was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "GainRaw was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取Gamma参数
            if ((!(*hardwareParamIterator)["Gamma"].isNone()) && ((*hardwareParamIterator)["Gamma"].isReal()))
            {
                hardwareParam.Gamma = static_cast<float>((*hardwareParamIterator)["Gamma"]);
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Gamma was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Gamma was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取Brightness参数
            if ((!(*hardwareParamIterator)["Brightness"].isNone()) && ((*hardwareParamIterator)["Brightness"].isInt()))
            {
                hardwareParam.Brightness = static_cast<int>((*hardwareParamIterator)["Brightness"]);
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Brightness was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "Brightness was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取IsSelected参数
            if ((!(*hardwareParamIterator)["IsSelected"].isNone()) && ((*hardwareParamIterator)["IsSelected"].isInt()))
            {
                hardwareParam.IsSelected = static_cast<int>((*hardwareParamIterator)["IsSelected"]);
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "IsSelected was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + huarayCameraParam->Key + "] - "\
                      "HuarayCameraHardwareParams[" + std::to_string(hardwareParamIndex) + "]'s "\
                      "IsSelected was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 保存硬件参数
            huarayCameraParam->HardwareParams.emplace_back(hardwareParam);

            // 硬件参数索引和迭代器累加
            hardwareParamIndex++;
            hardwareParamIterator++;
        }
    }
    else
    {
        log = "[" + huarayCameraParam->Key + "] - HuarayCameraHardwareParams was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 关闭文件存储器
    fileStorage.release();

    // 记录日志信息
    log = "[" + huarayCameraParam->Key + "] - HuarayCameraParam was loaded completely";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回加载结果
    return true;
}