//
// Created by plutoli on 2021/8/13.
//

#include "solver_param.h"

// ******************************  SolverParam类的公有函数  ******************************

// 构造函数
SolverParam::SolverParam():
    Key("Solver"),
    ArmorDistanceParameters(),
    ArmorVelocityParameters(),
    BuffDistanceParameter(),
    SmallBuffVelocityParameter(),
    LargeBuffVelocityParameter()
{
}

// 从yaml配置文件中加载目标解算器参数
bool SolverParam::LoadFromYamlFile(const std::string &yamlFileName, SolverParam *solverParam)
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
        log = "SolverParam was loaded failure because yaml file does not exist";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断yaml配置文件是否可读
    if (::access(yamlFileName.c_str(), R_OK) == -1)
    {
        log = "SolverParam was loaded failure because yaml file can not be read";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        log = "SolverParam was loaded failure because yaml file can not be opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 读取Key参数
    if ((!fileStorage["Key"].isNone()) && (fileStorage["Key"].isString()))
    {
        solverParam->Key = static_cast<std::string>(fileStorage["Key"]);
        log = "[" + solverParam->Key + "] - SolverParam's Key was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + solverParam->Key + "] - SolverParam's Key was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorDistanceParameters参数
    cv::FileNode armorDistanceParametersNode = fileStorage["ArmorDistanceParameters"];
    if (!armorDistanceParametersNode.empty())
    {
        unsigned int paramIndex = 0;
        cv::FileNodeIterator paramIterator = armorDistanceParametersNode.begin();
        while (paramIterator != armorDistanceParametersNode.end())
        {
            // 创建装甲板距离补偿参数
            ArmorDistanceParam param;

            // 读取WorkModes参数
            cv::FileNode workModesNode = (*paramIterator)["WorkModes"];
            if (!workModesNode.empty())
            {
                unsigned int modeIndex = 0;
                cv::FileNodeIterator modeIterator = workModesNode.begin();
                while (modeIterator != workModesNode.end())
                {
                    if ((*modeIterator).isInt())
                    {
//                        EWorkMode mode;
//                        if (SystemConfigurator::ConvertToWorkMode(static_cast<int>(*modeIterator),
//                                                                  &mode))
//                        {
//                            param.WorkModes.emplace_back(mode);
//                            log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
//                                  "WorkModes[" + std::to_string(modeIndex) + "] was loaded successful";
//                            logger.Save(ELogType::Info, log);
//                        }
//                        else
//                        {
//                            log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
//                                  "WorkModes[" + std::to_string(modeIndex) + "] was converted failure";
//                            logger.Save(ELogType::Error, log);
//                        }
                    }
                    else
                    {
                        log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                              "WorkModes[" + std::to_string(modeIndex) + "] was loaded failure";
                        logger.Save(ELogType::Error, log);
                    }

                    // 工作模式索引和迭代器累加
                    modeIndex++;
                    modeIterator++;
                }
            }
            else
            {
                log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                      "WorkModes was loaded failure because it is empty";
                logger.Save(ELogType::Error, log);
            }

            // 读取BulletVelocity参数
            if ((!(*paramIterator)["BulletVelocity"].empty()) && ((*paramIterator)["BulletVelocity"].isInt()))
            {
                //TODO
                // EBulletVelocity bulletVelocity;
                // if (SystemConfigurator::ConvertToBulletVelocity(static_cast<int>((*paramIterator)["BulletVelocity"]),
                //                                                 &bulletVelocity))
                // {
                //     param.BulletVelocity = bulletVelocity;
                //     log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                //           "BulletVelocity was loaded successful";
                //     logger.Save(ELogType::Info, log);
                // }
                // else
                // {
                //     log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                //           "BulletVelocity was converted failure";
                //     logger.Save(ELogType::Error, log);
                // }
            }
            else
            {
                log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                      "BulletVelocity was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取Offsets参数
            cv::FileNode offsetsNode = (*paramIterator)["Offsets"];
            if (!offsetsNode.empty())
            {
                unsigned int offsetIndex = 0;
                cv::FileNodeIterator offsetIterator = offsetsNode.begin();
                while (offsetIterator != offsetsNode.end())
                {
                    // 创建装甲板距离分段补偿值
                    ArmorDistanceOffset offset;

                    // 读取Lower参数
                    if ((!(*offsetIterator)["Lower"].isNone()) && ((*offsetIterator)["Lower"].isReal()))
                    {
                        offset.Lower = static_cast<float>((*offsetIterator)["Lower"]);
                        log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                              "Offsets[" + std::to_string(offsetIndex) + "]'s Lower was loaded successful";
                        logger.Save(ELogType::Info, log);
                    }
                    else
                    {
                        log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                              "Offsets[" + std::to_string(offsetIndex) + "]'s Lower was loaded failure";
                        logger.Save(ELogType::Error, log);
                    }

                    // 读取Upper参数
                    if ((!(*offsetIterator)["Upper"].isNone()) && ((*offsetIterator)["Upper"].isReal()))
                    {
                        offset.Upper = static_cast<float>((*offsetIterator)["Upper"]);
                        log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                              "Offsets[" + std::to_string(offsetIndex) + "]'s Upper in loaded successful";
                        logger.Save(ELogType::Info, log);
                    }
                    else
                    {
                        log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                              "Offsets[" + std::to_string(offsetIndex) + "]'s Upper was loaded failure";
                        logger.Save(ELogType::Error, log);
                    }

                    // 读取Offset_x参数
                    if ((!(*offsetIterator)["Offset_x"].isNone()) && ((*offsetIterator)["Offset_x"].isReal()))
                    {
                        offset.Offset_x = static_cast<float>((*offsetIterator)["Offset_x"]);
                        log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                              "Offsets[" + std::to_string(offsetIndex) + "]'s Offset_x was loaded successful";
                        logger.Save(ELogType::Info, log);
                    }
                    else
                    {
                        log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                              "Offsets[" + std::to_string(offsetIndex) + "]'s Offset_x was loaded failure";
                        logger.Save(ELogType::Error, log);
                    }

                    // 读取Offset_y参数
                    if ((!(*offsetIterator)["Offset_y"].isNone()) && ((*offsetIterator)["Offset_y"].isReal()))
                    {
                        offset.Offset_y = static_cast<float>((*offsetIterator)["Offset_y"]);
                        log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                              "Offsets[" + std::to_string(offsetIndex) + "]'s Offset_y was loaded successful";
                        logger.Save(ELogType::Info, log);
                    }
                    else
                    {
                        log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                              "Offsets[" + std::to_string(offsetIndex) + "]'s Offset_y was loaded failure";
                        logger.Save(ELogType::Error, log);
                    }

                    // 保存装甲板距离分段补偿值
                    param.Offsets.emplace_back(offset);

                    // 装甲板距离分段补偿值索引和迭代器累加
                    offsetIndex++;
                    offsetIterator++;
                }
            }
            else
            {
                log = "[" + solverParam->Key + "] - ArmorDistanceParameters[" + std::to_string(paramIndex) + "]."\
                      "Offsets was loaded failure because it is empty";
                logger.Save(ELogType::Error, log);
            }

            // 保存装甲板距离补偿参数
            solverParam->ArmorDistanceParameters.emplace_back(param);

            // 装甲板距离补偿参数索引和迭代器累加
            paramIndex++;
            paramIterator++;
        }
    }
    else
    {
        log = "[" + solverParam->Key + "] - ArmorDistanceParameters was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorVelocityParameters参数
    cv::FileNode armorVelocityParametersNode = fileStorage["ArmorVelocityParameters"];
    if (!armorVelocityParametersNode.empty())
    {
        unsigned int paramIndex = 0;
        cv::FileNodeIterator paramIterator = armorVelocityParametersNode.begin();
        while (paramIterator != armorVelocityParametersNode.end())
        {
            // 创建装甲板速度补偿参数
            ArmorVelocityParam param;

            // 读取WorkModes参数
            cv::FileNode workModesNode = (*paramIterator)["WorkModes"];
            if (!workModesNode.empty())
            {
                unsigned int modeIndex = 0;
                cv::FileNodeIterator modeIterator = workModesNode.begin();
                while (modeIterator != workModesNode.end())
                {
                    if ((*modeIterator).isInt())
                    {
//                        EWorkMode mode;
//                        if (SystemConfigurator::ConvertToWorkMode(static_cast<int>(*modeIterator),
//                                                                  &mode))
//                        {
//                            param.WorkModes.emplace_back(mode);
//                            log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]."\
//                                  "WorkModes[" + std::to_string(modeIndex) + "] was loaded successful";
//                            logger.Save(ELogType::Info, log);
//                        }
//                        else
//                        {
//                            log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]."\
//                                  "WorkModes[" + std::to_string(modeIndex) + "] was converted failure";
//                            logger.Save(ELogType::Error, log);
//                        }
                    }
                    else
                    {
                        log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]."\
                              "WorkModes[" + std::to_string(modeIndex) + "] was loaded failure";
                        logger.Save(ELogType::Error, log);
                    }

                    // 工作模式索引和迭代器累加
                    modeIndex++;
                    modeIterator++;
                }
            }
            else
            {
                log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]."\
                      "WorkModes was loaded failure because it is empty";
                logger.Save(ELogType::Error, log);
            }

            // 读取Order参数
            if ((!(*paramIterator)["Order"].isNone()) && ((*paramIterator)["Order"].isInt()))
            {
                //TODO
                // EPolynomialOrder order;
                // if (SystemConfigurator::ConvertToPolynomialOrder(static_cast<int>((*paramIterator)["Order"]),
                //                                                  &order))
                // {
                //     param.Order = order;
                //     log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]'s "\
                //           "Order was loaded successful";
                //     logger.Save(ELogType::Info, log);
                // }
                // else
                // {
                //     log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]'s "\
                //           "Order was converted failure";
                //     logger.Save(ELogType::Error, log);
                // }
            }
            else
            {
                log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]'s "\
                      "Order was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取MaxIterationNumber参数
            if ((!(*paramIterator)["MaxIterationNumber"].isNone()) && ((*paramIterator)["MaxIterationNumber"].isInt()))
            {
                param.MaxIterationNumber = static_cast<int>((*paramIterator)["MaxIterationNumber"]);
                log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]'s "\
                      "MaxIterationNumber was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]'s "\
                      "MaxIterationNumber was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取SampleNumber参数
            if ((!(*paramIterator)["SampleNumber"].isNone()) && ((*paramIterator)["SampleNumber"].isInt()))
            {
                param.SampleNumber = static_cast<int>((*paramIterator)["SampleNumber"]);
                log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]'s "\
                      "SampleNumber was loaded successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + solverParam->Key + "] - ArmorVelocityParameters[" + std::to_string(paramIndex) + "]'s "\
                      "SampleNumber was loaded failure";
                logger.Save(ELogType::Error, log);
            }

            // 保存装甲板速度补偿参数
            solverParam->ArmorVelocityParameters.emplace_back(param);

            // 装甲板速度补偿参数索引和迭代器累加
            paramIndex++;
            paramIterator++;
        }
    }
    else
    {
        log = "[" + solverParam->Key + "] - ArmorVelocityParameters was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取BuffDistanceParameter参数
    cv::FileNode buffDistanceParameterNode = fileStorage["BuffDistanceParameter"];
    if (!buffDistanceParameterNode.empty())
    {
        // 读取Offsets参数
        cv::FileNode offsetsNode = buffDistanceParameterNode["Offsets"];
        if (!offsetsNode.empty())
        {
            unsigned int offsetIndex = 0;
            cv::FileNodeIterator offsetIterator = offsetsNode.begin();
            while (offsetIterator != offsetsNode.end())
            {
                // 创建Buff距离分段补偿值
                BuffDistanceOffset offset;

                // 读取Lower参数
                if ((!(*offsetIterator)["Lower"].isNone()) && ((*offsetIterator)["Lower"].isReal()))
                {
                    offset.Lower = static_cast<float>((*offsetIterator)["Lower"]);
                    log = "[" + solverParam->Key + "] - BuffDistanceParameter."\
                          "Offsets[" + std::to_string(offsetIndex) + "]'s Lower was loaded successful";
                    logger.Save(ELogType::Info, log);
                }
                else
                {
                    log = "[" + solverParam->Key + "] - BuffDistanceParameter."\
                          "Offsets[" + std::to_string(offsetIndex) + "]'s Lower was loaded failure";
                    logger.Save(ELogType::Error, log);
                }

                // 读取Upper参数
                if ((!(*offsetIterator)["Upper"].isNone()) && ((*offsetIterator)["Upper"].isReal()))
                {
                    offset.Upper = static_cast<float>((*offsetIterator)["Upper"]);
                    log = "[" + solverParam->Key + "] - BuffDistanceParameter."\
                          "Offsets[" + std::to_string(offsetIndex) + "]'s Upper in loaded successful";
                    logger.Save(ELogType::Info, log);
                }
                else
                {
                    log = "[" + solverParam->Key + "] - BuffDistanceParameter."\
                          "Offsets[" + std::to_string(offsetIndex) + "]'s Upper was loaded failure";
                    logger.Save(ELogType::Error, log);
                }

                // 读取Offset_x参数
                if ((!(*offsetIterator)["Offset_x"].isNone()) && ((*offsetIterator)["Offset_x"].isReal()))
                {
                    offset.Offset_x = static_cast<float>((*offsetIterator)["Offset_x"]);
                    log = "[" + solverParam->Key + "] - BuffDistanceParameter."\
                          "Offsets[" + std::to_string(offsetIndex) + "]'s Offset_x was loaded successful";
                    logger.Save(ELogType::Info, log);
                }
                else
                {
                    log = "[" + solverParam->Key + "] - BuffDistanceParameter."\
                          "Offsets[" + std::to_string(offsetIndex) + "]'s Offset_x was loaded failure";
                    logger.Save(ELogType::Error, log);
                }

                // 读取Offset_y参数
                if ((!(*offsetIterator)["Offset_y"].isNone()) && ((*offsetIterator)["Offset_y"].isReal()))
                {
                    offset.Offset_y = static_cast<float>((*offsetIterator)["Offset_y"]);
                    log = "[" + solverParam->Key + "] - BuffDistanceParameter."\
                          "Offsets[" + std::to_string(offsetIndex) + "]'s Offset_y was loaded successful";
                    logger.Save(ELogType::Info, log);
                }
                else
                {
                    log = "[" + solverParam->Key + "] - BuffDistanceParameter."\
                          "Offsets[" + std::to_string(offsetIndex) + "]'s Offset_y was loaded failure";
                    logger.Save(ELogType::Error, log);
                }

                // 保存Buff距离分段补偿值
                solverParam->BuffDistanceParameter.Offsets.emplace_back(offset);

                // Buff距离分段补偿值索引和迭代器累加
                offsetIndex++;
                offsetIterator++;
            }
        }
        else
        {
            log = "[" + solverParam->Key + "] - BuffDistanceParameter.Offsets was loaded failure because it is empty";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + solverParam->Key + "] - BuffDistanceParameter was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取SmallBuffVelocityParameter参数
    cv::FileNode smallBuffVelocityParameterNode = fileStorage["SmallBuffVelocityParameter"];
    if (!smallBuffVelocityParameterNode.empty())
    {
        // 读取PredictTime参数
        if ((!smallBuffVelocityParameterNode["PredictTime"].isNone()) &&
            (smallBuffVelocityParameterNode["PredictTime"].isInt()))
        {
            auto predictTime = static_cast<int>(smallBuffVelocityParameterNode["PredictTime"]);
            solverParam->SmallBuffVelocityParameter.PredictTime = predictTime;
            log = "[" + solverParam->Key + "] - SmallBuffVelocityParameter's PredictTime was loaded successful";
            logger.Save(ELogType::Info, log);
        } else
        {
            log = "[" + solverParam->Key + "] - SmallBuffVelocityParameter's PredictTime was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取OffsetRadian参数
        if ((!smallBuffVelocityParameterNode["OffsetRadian"].isNone()) &&
            (smallBuffVelocityParameterNode["OffsetRadian"].isReal()))
        {
            auto offsetRadian = static_cast<float>(smallBuffVelocityParameterNode["OffsetRadian"]);
            solverParam->SmallBuffVelocityParameter.OffsetRadian = offsetRadian;
            log = "[" + solverParam->Key + "] - SmallBuffVelocityParameter's OffsetRadian was loaded successful";
            logger.Save(ELogType::Info, log);
        } else
        {
            log = "[" + solverParam->Key + "] - SmallBuffVelocityParameter's OffsetRadian was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SampleNumber参数
        if ((!smallBuffVelocityParameterNode["SampleNumber"].isNone()) &&
            (smallBuffVelocityParameterNode["SampleNumber"].isInt()))
        {
            auto sampleNumber = static_cast<int>(smallBuffVelocityParameterNode["SampleNumber"]);
            solverParam->SmallBuffVelocityParameter.SampleNumber = sampleNumber;
            log = "[" + solverParam->Key + "] - SmallBuffVelocityParameter's SampleNumber was loaded successful";
            logger.Save(ELogType::Info, log);
        } else
        {
            log = "[" + solverParam->Key + "] - SmallBuffVelocityParameter's SampleNumber was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + solverParam->Key + "] - SmallBuffVelocityParameter was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取LargeBuffVelocityParameter参数
    cv::FileNode largeBuffVelocityParameterNode = fileStorage["LargeBuffVelocityParameter"];
    if (!largeBuffVelocityParameterNode.empty())
    {
        // 读取PredictTime参数
        if ((!largeBuffVelocityParameterNode["PredictTime"].isNone()) &&
            (largeBuffVelocityParameterNode["PredictTime"].isInt()))
        {
            auto predictTime = static_cast<int>(largeBuffVelocityParameterNode["PredictTime"]);
            solverParam->LargeBuffVelocityParameter.PredictTime = predictTime;
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's PredictTime was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's PredictTime was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HighSpeedThreshold参数
        if ((!largeBuffVelocityParameterNode["HighSpeedThreshold"].isNone()) &&
            (largeBuffVelocityParameterNode["HighSpeedThreshold"].isReal()))
        {
            auto highSpeedThreshold = static_cast<float>(largeBuffVelocityParameterNode["HighSpeedThreshold"]);
            solverParam->LargeBuffVelocityParameter.HighSpeedThreshold = highSpeedThreshold;
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's HighSpeedThreshold was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's HighSpeedThreshold was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HighSpeedOffsetRadian参数
        if ((!largeBuffVelocityParameterNode["HighSpeedOffsetRadian"].isNone()) &&
            (largeBuffVelocityParameterNode["HighSpeedOffsetRadian"].isReal()))
        {
            auto highSpeedOffsetRadian = static_cast<float>(largeBuffVelocityParameterNode["HighSpeedOffsetRadian"]);
            solverParam->LargeBuffVelocityParameter.HighSpeedOffsetRadian = highSpeedOffsetRadian;
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's HighSpeedOffsetRadian was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's HighSpeedOffsetRadian was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取LowSpeedThreshold参数
        if ((!largeBuffVelocityParameterNode["LowSpeedThreshold"].isNone()) &&
            (largeBuffVelocityParameterNode["LowSpeedThreshold"].isReal()))
        {
            auto lowSpeedThreshold = static_cast<float>(largeBuffVelocityParameterNode["LowSpeedThreshold"]);
            solverParam->LargeBuffVelocityParameter.LowSpeedThreshold = lowSpeedThreshold;
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's LowSpeedThreshold was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's LowSpeedThreshold was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取LowSpeedOffsetRadian参数
        if ((!largeBuffVelocityParameterNode["LowSpeedOffsetRadian"].isNone()) &&
            (largeBuffVelocityParameterNode["LowSpeedOffsetRadian"].isReal()))
        {
            auto lowSpeedOffsetRadian = static_cast<float>(largeBuffVelocityParameterNode["LowSpeedOffsetRadian"]);
            solverParam->LargeBuffVelocityParameter.LowSpeedOffsetRadian = lowSpeedOffsetRadian;
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's LowSpeedOffsetRadian was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's LowSpeedOffsetRadian was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SampleNumber参数
        if ((!largeBuffVelocityParameterNode["SampleNumber"].isNone()) &&
            (largeBuffVelocityParameterNode["SampleNumber"].isInt()))
        {
            auto sampleNumber = static_cast<int>(largeBuffVelocityParameterNode["SampleNumber"]);
            solverParam->LargeBuffVelocityParameter.SampleNumber = sampleNumber;
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's SampleNumber was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter's SampleNumber was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + solverParam->Key + "] - LargeBuffVelocityParameter was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取AngleDistanceOffsetParameter参数
    cv::FileNode AngleDistanceOffsetParameterNode = fileStorage["AngleDistanceOffsetParameter"];
    if (!AngleDistanceOffsetParameterNode.empty())
    {
        // 读取max_iter参数
        if ((!AngleDistanceOffsetParameterNode["max_iter"].isNone()) &&
            (AngleDistanceOffsetParameterNode["max_iter"].isInt()))
        {
            auto max_iter = static_cast<int>(AngleDistanceOffsetParameterNode["max_iter"]);
            solverParam->AngleDistanceOffsetParameter.max_iter = max_iter;
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter.max_iter's max_iter was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter.max_iter's max_iter was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取R_K_iter参数
        if ((!AngleDistanceOffsetParameterNode["R_K_iter"].isNone()) &&
            (AngleDistanceOffsetParameterNode["R_K_iter"].isInt()))
        {
            auto r_k_iter = static_cast<int>(AngleDistanceOffsetParameterNode["R_K_iter"]);
            solverParam->AngleDistanceOffsetParameter.R_K_iter = r_k_iter;
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter's R_K_iter was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter's R_K_iter was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取stop_error参数
        if ((!AngleDistanceOffsetParameterNode["stop_error"].isNone()) &&
            (AngleDistanceOffsetParameterNode["stop_error"].isReal()))
        {
            auto Stop_error = static_cast<float>(AngleDistanceOffsetParameterNode["stop_error"]);
            solverParam->AngleDistanceOffsetParameter.stop_error = Stop_error;
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter's stop_error was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter's stop_error was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取k参数
        if ((!AngleDistanceOffsetParameterNode["k"].isNone()) &&
            (AngleDistanceOffsetParameterNode["k"].isReal()))
        {
            auto K = static_cast<float>(AngleDistanceOffsetParameterNode["k"]);
            solverParam->AngleDistanceOffsetParameter.k = K;
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter's k was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter's k was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取g参数
        if ((!AngleDistanceOffsetParameterNode["g"].isNone()) &&
            ( AngleDistanceOffsetParameterNode["g"].isReal()))
        {
            auto G = static_cast<float>(AngleDistanceOffsetParameterNode["g"]);
            solverParam->AngleDistanceOffsetParameter.g = G;
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter's g was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter's g was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取bullet_speed参数
        if ((!AngleDistanceOffsetParameterNode["bullet_speed"].isNone()))
        {
            auto Bullet_speed = static_cast<float>(AngleDistanceOffsetParameterNode["bullet_speed"]);
            solverParam->AngleDistanceOffsetParameter.bullet_speed = Bullet_speed;
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter's bullet_speed was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter's bullet_speed was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + solverParam->Key + "] - AngleDistanceOffsetParameter was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }
      // 关闭文件存储器
    fileStorage.release();

    // 记录日志信息
    log = "[" + solverParam->Key + "] - SolverParam was loaded completely";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回加载结果
    return true;
}