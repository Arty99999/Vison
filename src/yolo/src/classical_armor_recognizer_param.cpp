//
// Created by plutoli on 2021/8/2.
//

#include "classical_armor_recognizer_param.h"

// ******************************  ClassicalArmorRecognizerParam类的公有函数  ******************************

// 构造函数
ClassicalArmorRecognizerParam::ClassicalArmorRecognizerParam():
    Key("ArmorYOLO"),
    LightBarColor(EClassicalLightBarColor::Red),
    DownsampleFactor(1.0),
    YoloFaceOpenvinoPath(),
    LightBarPhysicalHeight(1.0),
    LightBarPhysicalWidth(1.0),
    LargeArmorPhysicalHeight(1.0),
    LargeArmorPhysicalWidth(1.0),
    SmallArmorPhysicalHeight(1.0),
    SmallArmorPhysicalWidth(1.0),
    MaxLocationOffset(10.0),
    MaxMemoryLength(100),
    ImageSize(640),
    EvaluationWeights()
{
}

// 从yaml配置文件中加载装甲板识别器参数
bool ClassicalArmorRecognizerParam::LoadFromYamlFile(const std::string &yamlFileName,
                                                     ClassicalArmorRecognizerParam *recognizerParam)
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
        log = "ClassicalArmorRecognizerParam was loaded failure because yaml file does not exist";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断yaml配置文件是否可读
    if (::access(yamlFileName.c_str(), R_OK) == -1)
    {
        log = "ClassicalArmorRecognizerParam was loaded failure because yaml file can not be read";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        log = "ClassicalArmorRecognizerParam was loaded failure because yaml file can not be opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 读取Key参数
    if ((!fileStorage["Key"].isNone()) && (fileStorage["Key"].isString()))
    {
        recognizerParam->Key = static_cast<std::string>(fileStorage["Key"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Key was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Key was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LightBarColor参数
    if ((!fileStorage["LightBarColor"].isNone()) && (fileStorage["LightBarColor"].isInt()))
    {
        EClassicalLightBarColor lightBarColor;
        if (ClassicalLightBar::ConvertToClassicalLightBarColor(static_cast<int>(fileStorage["LightBarColor"]),
                                                               &lightBarColor))
        {
            recognizerParam->LightBarColor = lightBarColor;
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LightBarColor was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LightBarColor was converted failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LightBarColor was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取DownsampleFactor参数
    if ((!fileStorage["DownsampleFactor"].isNone()) && (fileStorage["DownsampleFactor"].isReal()))
    {
        recognizerParam->DownsampleFactor = static_cast<float>(fileStorage["DownsampleFactor"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's DownsampleFactor was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's DownsampleFactor was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取RedYoloFacePath参数
    if ((!fileStorage["YoloFaceOpenvinoPath"].isNone()) && (fileStorage["YoloFaceOpenvinoPath"].isString()))
    {
        recognizerParam->YoloFaceOpenvinoPath = static_cast<std::string>(fileStorage["YoloFaceOpenvinoPath"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's YoloFaceOpenvinoPath was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's YoloFaceOpenvinoPath was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取BoxThreshold参数
    if ((!fileStorage["BoxThreshold"].isNone()) && (fileStorage["BoxThreshold"].isReal()))
    {
        recognizerParam->BoxThreshold = static_cast<float>(fileStorage["BoxThreshold"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's BoxThreshold was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's BoxThreshold was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ConfThreshold参数
    if ((!fileStorage["ConfThreshold"].isNone()) && (fileStorage["ConfThreshold"].isReal()))
    {
        recognizerParam->ConfThreshold = static_cast<float>(fileStorage["ConfThreshold"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ConfThreshold was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ConfThreshold was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取NMSThreshold参数
    if ((!fileStorage["NMSThreshold"].isNone()) && (fileStorage["NMSThreshold"].isReal()))
    {
        recognizerParam->NMSThreshold = static_cast<float>(fileStorage["NMSThreshold"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's NMSThreshold was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's NMSThreshold was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ClassNumber参数
    if ((!fileStorage["ClassNumber"].isNone()) && (fileStorage["ClassNumber"].isInt()))
    {
        recognizerParam->ClassNumber = static_cast<int>(fileStorage["ClassNumber"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ClassNumber was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ClassNumber was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ImageSize参数
    if ((!fileStorage["ImageSize"].isNone()) && (fileStorage["ImageSize"].isInt()))
    {
        recognizerParam->ImageSize = static_cast<int>(fileStorage["ImageSize"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ImageSize was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ImageSize was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LightBarPhysicalHeight参数
    if ((!fileStorage["LightBarPhysicalHeight"].isNone()) && (fileStorage["LightBarPhysicalHeight"].isReal()))
    {
        recognizerParam->LightBarPhysicalHeight = static_cast<float>(fileStorage["LightBarPhysicalHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LightBarPhysicalHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LightBarPhysicalHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LightBarPhysicalWidth参数
    if ((!fileStorage["LightBarPhysicalWidth"].isNone()) && (fileStorage["LightBarPhysicalWidth"].isReal()))
    {
        recognizerParam->LightBarPhysicalWidth = static_cast<float>(fileStorage["LightBarPhysicalWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LightBarPhysicalWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LightBarPhysicalWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LargeArmorPhysicalHeight参数
    if ((!fileStorage["LargeArmorPhysicalHeight"].isNone()) && (fileStorage["LargeArmorPhysicalHeight"].isReal()))
    {
        recognizerParam->LargeArmorPhysicalHeight = static_cast<float>(fileStorage["LargeArmorPhysicalHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LargeArmorPhysicalHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LargeArmorPhysicalHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LargeArmorPhysicalWidth参数
    if ((!fileStorage["LargeArmorPhysicalWidth"].isNone()) && (fileStorage["LargeArmorPhysicalWidth"].isReal()))
    {
        recognizerParam->LargeArmorPhysicalWidth = static_cast<float>(fileStorage["LargeArmorPhysicalWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LargeArmorPhysicalWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's LargeArmorPhysicalWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取SmallArmorPhysicalHeight参数
    if ((!fileStorage["SmallArmorPhysicalHeight"].isNone()) && (fileStorage["SmallArmorPhysicalHeight"].isReal()))
    {
        recognizerParam->SmallArmorPhysicalHeight = static_cast<float>(fileStorage["SmallArmorPhysicalHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SmallArmorPhysicalHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SmallArmorPhysicalHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取SmallArmorPhysicalWidth参数
    if ((!fileStorage["SmallArmorPhysicalWidth"].isNone()) && (fileStorage["SmallArmorPhysicalWidth"].isReal()))
    {
        recognizerParam->SmallArmorPhysicalWidth = static_cast<float>(fileStorage["SmallArmorPhysicalWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SmallArmorPhysicalWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SmallArmorPhysicalWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLocationOffset参数
    if ((!fileStorage["MaxLocationOffset"].isNone()) && (fileStorage["MaxLocationOffset"].isReal()))
    {
        recognizerParam->MaxLocationOffset = static_cast<float>(fileStorage["MaxLocationOffset"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLocationOffset was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLocationOffset was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxMemoryLength参数
    if ((!fileStorage["MaxMemoryLength"].isNone()) && (fileStorage["MaxMemoryLength"].isInt()))
    {
        recognizerParam->MaxMemoryLength = static_cast<int>(fileStorage["MaxMemoryLength"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxMemoryLength was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxMemoryLength was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取EvaluationWeights参数
    cv::FileNode evaluationWeightsNode = fileStorage["EvaluationWeights"];
    if (!evaluationWeightsNode.empty())
    {
        int index = 0;
        cv::FileNodeIterator iterator = evaluationWeightsNode.begin();
        while (iterator != evaluationWeightsNode.end())
        {
            // 创建装甲板评估权值
            ClassicalArmorEvaluationWeight evaluationWeight;

            // 读取ArmorNumber参数
            if ((!(*iterator)["ArmorNumber"].isNone()) && ((*iterator)["ArmorNumber"].isInt()))
            {
                EClassicalArmorNumber armorNumber;
                if (ClassicalArmor::ConvertToClassicalArmorNumber(static_cast<int>((*iterator)["ArmorNumber"]),
                                                                  &armorNumber))
                {
                    evaluationWeight.ArmorNumber = armorNumber;
                    log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's "\
                          "EvaluationWeights[" + std::to_string(index) + "] loaded ArmorNumber successful";
                    logger.Save(ELogType::Info, log);
                }
                else
                {
                    log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's "\
                          "EvaluationWeights[" + std::to_string(index) + "] converted ArmorNumber failure";
                    logger.Save(ELogType::Error, log);
                }
            }
            else
            {
                log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's "\
                      "EvaluationWeights[" + std::to_string(index) + "] loaded ArmorNumber failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取OffsetCoeff参数
            if ((!(*iterator)["OffsetCoeff"].isNone()) && ((*iterator)["OffsetCoeff"].isReal()))
            {
                evaluationWeight.OffsetCoeff = static_cast<float>((*iterator)["OffsetCoeff"]);
                log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's "\
                      "EvaluationWeights[" + std::to_string(index) + "] loaded OffsetCoeff successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's "\
                      "EvaluationWeights[" + std::to_string(index) + "] loaded OffsetCoeff failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取AreaCoeff参数
            if ((!(*iterator)["AreaCoeff"].isNone()) && ((*iterator)["AreaCoeff"].isReal()))
            {
                evaluationWeight.AreaCoeff = static_cast<float>((*iterator)["AreaCoeff"]);
                log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's "\
                      "EvaluationWeights[" + std::to_string(index) + "] loaded AreaCoeff successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's "\
                      "EvaluationWeights[" + std::to_string(index) + "] loaded AreaCoeff failure";
                logger.Save(ELogType::Error, log);
            }

            // 读取ImportanceCoeff参数
            if ((!(*iterator)["ImportanceCoeff"].isNone()) && ((*iterator)["ImportanceCoeff"].isReal()))
            {
                evaluationWeight.ImportanceCoeff = static_cast<float>((*iterator)["ImportanceCoeff"]);
                log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's "\
                      "EvaluationWeights[" + std::to_string(index) + "] loaded ImportanceCoeff successful";
                logger.Save(ELogType::Info, log);
            }
            else
            {
                log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's "\
                      "EvaluationWeights[" + std::to_string(index) + "] loaded ImportanceCoeff failure";
                logger.Save(ELogType::Error, log);
            }

            // 保存装甲板评估权值
            recognizerParam->EvaluationWeights.emplace_back(evaluationWeight);

            // 迭代器和编号累加
            iterator++;
            index++;
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's "\
              "EvaluationWeights was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 关闭文件存储器
    fileStorage.release();

    // 记录日志信息
    log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam was loaded completely";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回加载结果
    return true;
}