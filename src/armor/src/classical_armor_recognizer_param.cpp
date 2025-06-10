//
// Created by plutoli on 2021/8/2.
//

#include "classical_armor_recognizer_param.h"

// ******************************  ClassicalArmorRecognizerParam类的公有函数  ******************************

// 构造函数
ClassicalArmorRecognizerParam::ClassicalArmorRecognizerParam():
    Key("Armor"),
    LightBarColor(EClassicalLightBarColor::Red),
    DownsampleFactor(1.0),
    BGRWeightForBlue(),
    BGRWeightForRed(),
    GrayThresholdForBlue(0),
    GrayThresholdForRed(0),
    HSVThresholdForBlue(),
    HSVThresholdForRed_1(),
    HSVThresholdForRed_2(),
    MaxLightBarArea(0.0),
    MinLightBarArea(0.0),
    MaxLightBarHeight(0.0),
    MinLightBarHeight(0.0),
    MaxLightBarAspectRatio(0.0),
    MaxLightBarAngle(0.0),
    MinLightBarAngle(0.0),
    MaxLightBarsDistanceRatio(0.0),
    MinLightBarsDistanceRatio(0.0),
    MaxLightBarsAngleOffset(0.0),
    MaxLightBarsHeightRatio(1.0),
    ArmorHogWindowWidth(40),
    ArmorHogWindowHeight(40),
    ArmorHogBlockWidth(16),
    ArmorHogBlockHeight(16),
    ArmorHogCellWidth(8),
    ArmorHogCellHeight(8),
    ArmorHogStrideWidth(8),
    ArmorHogStrideHeight(8),
    ArmorHogBins(9),
    ArmorHogSvmFileName(),
    LightBarPhysicalHeight(1.0),
    LightBarPhysicalWidth(1.0),
    LargeArmorPhysicalHeight(1.0),
    LargeArmorPhysicalWidth(1.0),
    SmallArmorPhysicalHeight(1.0),
    SmallArmorPhysicalWidth(1.0),
    MaxLocationOffset(10.0),
    MaxMemoryLength(100),
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

    // 读取BGRWeightForBlue参数
    cv::FileNode bgrWeightForBlueNode = fileStorage["BGRWeightForBlue"];
    if (!bgrWeightForBlueNode.empty())
    {
        // 读取Blue参数
        if ((!bgrWeightForBlueNode["Blue"].isNone()) && (bgrWeightForBlueNode["Blue"].isReal()))
        {
            recognizerParam->BGRWeightForBlue.Blue = static_cast<float>(bgrWeightForBlueNode["Blue"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Blue weight "\
                  "in BGRWeightForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Blue weight "\
                  "in BGRWeightForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取Green参数
        if ((!bgrWeightForBlueNode["Green"].isNone()) && (bgrWeightForBlueNode["Green"].isReal()))
        {
            recognizerParam->BGRWeightForBlue.Green = static_cast<float>(bgrWeightForBlueNode["Green"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Green weight "\
                  "in BGRWeightForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Green weight "\
                  "in BGRWeightForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取Red参数
        if ((!bgrWeightForBlueNode["Red"].isNone()) && (bgrWeightForBlueNode["Red"].isReal()))
        {
            recognizerParam->BGRWeightForBlue.Red = static_cast<float>(bgrWeightForBlueNode["Red"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Red weight "\
                  "in BGRWeightForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Red weight "\
                  "in BGRWeightForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's BGRWeightForBlue was loaded failure "\
              "because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取BGRWeightForRed参数
    cv::FileNode bgrWeightForRedNode = fileStorage["BGRWeightForRed"];
    if (!bgrWeightForRedNode.empty())
    {
        // 读取Blue参数
        if ((!bgrWeightForRedNode["Blue"].isNone()) && (bgrWeightForRedNode["Blue"].isReal()))
        {
            recognizerParam->BGRWeightForRed.Blue = static_cast<float>(bgrWeightForRedNode["Blue"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Blue weight "\
                  "in BGRWeightForRed was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Blue weight "\
                  "in BGRWeightForRed was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取Green参数
        if ((!bgrWeightForRedNode["Green"].isNone()) && (bgrWeightForRedNode["Green"].isReal()))
        {
            recognizerParam->BGRWeightForRed.Green = static_cast<float>(bgrWeightForRedNode["Green"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Green weight "\
                  "in BGRWeightForRed was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Green weight "\
                  "in BGRWeightForRed was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取Red参数
        if ((!bgrWeightForRedNode["Red"].isNone()) && (bgrWeightForRedNode["Red"].isReal()))
        {
            recognizerParam->BGRWeightForRed.Red = static_cast<float>(bgrWeightForRedNode["Red"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Red weight "\
                  "in BGRWeightForRed was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's Red weight "\
                  "in BGRWeightForRed was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's BGRWeightForRed was loaded failure "\
              "because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取GrayThresholdForBlue参数
    if ((!fileStorage["GrayThresholdForBlue"].isNone()) && (fileStorage["GrayThresholdForBlue"].isInt()))
    {
        recognizerParam->GrayThresholdForBlue = static_cast<int>(fileStorage["GrayThresholdForBlue"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's GrayThresholdForBlue was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's GrayThresholdForBlue was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取GrayThresholdForRed参数
    if ((!fileStorage["GrayThresholdForRed"].isNone()) && (fileStorage["GrayThresholdForRed"].isInt()))
    {
        recognizerParam->GrayThresholdForRed = static_cast<int>(fileStorage["GrayThresholdForRed"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's GrayThresholdForRed was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's GrayThresholdForRed was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取HSVThresholdForBlue参数
    cv::FileNode hsvThresholdForBlueNode = fileStorage["HSVThresholdForBlue"];
    if (!hsvThresholdForBlueNode.empty())
    {
        // 读取HueLower参数
        if ((!hsvThresholdForBlueNode["HueLower"].isNone()) && (hsvThresholdForBlueNode["HueLower"].isInt()))
        {
            recognizerParam->HSVThresholdForBlue.HueLower = static_cast<int>(hsvThresholdForBlueNode["HueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueLower "\
                  "in HSVThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueLower "\
                  "in HSVThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HueUpper参数
        if ((!hsvThresholdForBlueNode["HueUpper"].isNone()) && (hsvThresholdForBlueNode["HueUpper"].isInt()))
        {
            recognizerParam->HSVThresholdForBlue.HueUpper = static_cast<int>(hsvThresholdForBlueNode["HueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueUpper "\
                  "in HSVThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueUpper "\
                  "in HSVThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationLower参数
        if ((!hsvThresholdForBlueNode["SaturationLower"].isNone()) && (hsvThresholdForBlueNode["SaturationLower"].isInt()))
        {
            recognizerParam->HSVThresholdForBlue.SaturationLower = static_cast<int>(hsvThresholdForBlueNode["SaturationLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationLower "\
                  "in HSVThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationLower "\
                  "in HSVThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationUpper参数
        if ((!hsvThresholdForBlueNode["SaturationUpper"].isNone()) && (hsvThresholdForBlueNode["SaturationUpper"].isInt()))
        {
            recognizerParam->HSVThresholdForBlue.SaturationUpper = static_cast<int>(hsvThresholdForBlueNode["SaturationUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationUpper "\
                  "in HSVThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationUpper "\
                  "in HSVThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueLower参数
        if ((!hsvThresholdForBlueNode["ValueLower"].isNone()) && (hsvThresholdForBlueNode["ValueLower"].isInt()))
        {
            recognizerParam->HSVThresholdForBlue.ValueLower = static_cast<int>(hsvThresholdForBlueNode["ValueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueLower "\
                  "in HSVThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueLower "\
                  "in HSVThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueUpper参数
        if ((!hsvThresholdForBlueNode["ValueUpper"].isNone()) && (hsvThresholdForBlueNode["ValueUpper"].isInt()))
        {
            recognizerParam->HSVThresholdForBlue.ValueUpper = static_cast<int>(hsvThresholdForBlueNode["ValueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueUpper "\
                  "in HSVThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueUpper "\
                  "in HSVThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HSVThresholdForBlue "\
              "was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取HSVThresholdForRed_1参数
    cv::FileNode hsvThresholdForRedNode_1 = fileStorage["HSVThresholdForRed_1"];
    if (!hsvThresholdForRedNode_1.empty())
    {
        // 读取HueLower参数
        if ((!hsvThresholdForRedNode_1["HueLower"].isNone()) && (hsvThresholdForRedNode_1["HueLower"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_1.HueLower = static_cast<int>(hsvThresholdForRedNode_1["HueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueLower "\
                  "in HSVThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueLower "\
                  "in HSVThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HueUpper参数
        if ((!hsvThresholdForRedNode_1["HueUpper"].isNone()) && (hsvThresholdForRedNode_1["HueUpper"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_1.HueUpper = static_cast<int>(hsvThresholdForRedNode_1["HueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueUpper "\
                  "in HSVThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueUpper "\
                  "in HSVThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationLower参数
        if ((!hsvThresholdForRedNode_1["SaturationLower"].isNone()) && (hsvThresholdForRedNode_1["SaturationLower"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_1.SaturationLower = static_cast<int>(hsvThresholdForRedNode_1["SaturationLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationLower "\
                  "in HSVThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationLower "\
                  "in HSVThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationUpper参数
        if ((!hsvThresholdForRedNode_1["SaturationUpper"].isNone()) && (hsvThresholdForRedNode_1["SaturationUpper"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_1.SaturationUpper = static_cast<int>(hsvThresholdForRedNode_1["SaturationUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationUpper "\
                  "in HSVThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationUpper "\
                  "in HSVThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueLower参数
        if ((!hsvThresholdForRedNode_1["ValueLower"].isNone()) && (hsvThresholdForRedNode_1["ValueLower"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_1.ValueLower = static_cast<int>(hsvThresholdForRedNode_1["ValueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueLower "\
                  "in HSVThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueLower "\
                  "in HSVThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueUpper参数
        if ((!hsvThresholdForRedNode_1["ValueUpper"].isNone()) && (hsvThresholdForRedNode_1["ValueUpper"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_1.ValueUpper = static_cast<int>(hsvThresholdForRedNode_1["ValueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueUpper "\
                  "in HSVThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueUpper "\
                  "in HSVThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HSVThresholdForRed_1 "\
              "was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取HSVThresholdForRed_2参数
    cv::FileNode hsvThresholdForRedNode_2 = fileStorage["HSVThresholdForRed_2"];
    if (!hsvThresholdForRedNode_2.empty())
    {
        // 读取HueLower参数
        if ((!hsvThresholdForRedNode_2["HueLower"].isNone()) && (hsvThresholdForRedNode_2["HueLower"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_2.HueLower = static_cast<int>(hsvThresholdForRedNode_2["HueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueLower "\
                  "in HSVThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueLower "\
                  "in HSVThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HueUpper参数
        if ((!hsvThresholdForRedNode_2["HueUpper"].isNone()) && (hsvThresholdForRedNode_2["HueUpper"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_2.HueUpper = static_cast<int>(hsvThresholdForRedNode_2["HueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueUpper "\
                  "in HSVThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HueUpper "\
                  "in HSVThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationLower参数
        if ((!hsvThresholdForRedNode_2["SaturationLower"].isNone()) && (hsvThresholdForRedNode_2["SaturationLower"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_2.SaturationLower = static_cast<int>(hsvThresholdForRedNode_2["SaturationLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationLower "\
                  "in HSVThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationLower "\
                  "in HSVThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationUpper参数
        if ((!hsvThresholdForRedNode_2["SaturationUpper"].isNone()) && (hsvThresholdForRedNode_2["SaturationUpper"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_2.SaturationUpper = static_cast<int>(hsvThresholdForRedNode_2["SaturationUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationUpper "\
                  "in HSVThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's SaturationUpper "\
                  "in HSVThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueLower参数
        if ((!hsvThresholdForRedNode_2["ValueLower"].isNone()) && (hsvThresholdForRedNode_2["ValueLower"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_2.ValueLower = static_cast<int>(hsvThresholdForRedNode_2["ValueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueLower "\
                  "in HSVThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueLower "\
                  "in HSVThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueUpper参数
        if ((!hsvThresholdForRedNode_2["ValueUpper"].isNone()) && (hsvThresholdForRedNode_2["ValueUpper"].isInt()))
        {
            recognizerParam->HSVThresholdForRed_2.ValueUpper = static_cast<int>(hsvThresholdForRedNode_2["ValueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueUpper "\
                  "in HSVThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ValueUpper "\
                  "in HSVThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's HSVThresholdForRed_2 "\
              "was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLightBarArea参数
    if ((!fileStorage["MaxLightBarArea"].isNone()) && (fileStorage["MaxLightBarArea"].isReal()))
    {
        recognizerParam->MaxLightBarArea = static_cast<float>(fileStorage["MaxLightBarArea"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarArea was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarArea was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinLightBarArea参数
    if ((!fileStorage["MinLightBarArea"].isNone()) && (fileStorage["MinLightBarArea"].isReal()))
    {
        recognizerParam->MinLightBarArea = static_cast<float>(fileStorage["MinLightBarArea"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MinLightBarArea was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MinLightBarArea was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLightBarHeight参数
    if ((!fileStorage["MaxLightBarHeight"].isNone()) && (fileStorage["MaxLightBarHeight"].isReal()))
    {
        recognizerParam->MaxLightBarHeight = static_cast<float>(fileStorage["MaxLightBarHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinLightBarHeight参数
    if ((!fileStorage["MinLightBarHeight"].isNone()) && (fileStorage["MinLightBarHeight"].isReal()))
    {
        recognizerParam->MinLightBarHeight = static_cast<float>(fileStorage["MinLightBarHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MinLightBarHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MinLightBarHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLightBarAspectRatio参数
    if ((!fileStorage["MaxLightBarAspectRatio"].isNone()) && (fileStorage["MaxLightBarAspectRatio"].isReal()))
    {
        recognizerParam->MaxLightBarAspectRatio = static_cast<float>(fileStorage["MaxLightBarAspectRatio"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarAspectRatio was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarAspectRatio was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLightBarAngle参数
    if ((!fileStorage["MaxLightBarAngle"].isNone()) && (fileStorage["MaxLightBarAngle"].isReal()))
    {
        recognizerParam->MaxLightBarAngle = static_cast<float>(fileStorage["MaxLightBarAngle"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarAngle was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarAngle was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinLightBarAngle参数
    if ((!fileStorage["MinLightBarAngle"].isNone()) && (fileStorage["MinLightBarAngle"].isReal()))
    {
        recognizerParam->MinLightBarAngle = static_cast<float>(fileStorage["MinLightBarAngle"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MinLightBarAngle was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MinLightBarAngle was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLightBarsDistanceRatio参数
    if ((!fileStorage["MaxLightBarsDistanceRatio"].isNone()) && (fileStorage["MaxLightBarsDistanceRatio"].isReal()))
    {
        recognizerParam->MaxLightBarsDistanceRatio = static_cast<float>(fileStorage["MaxLightBarsDistanceRatio"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarsDistanceRatio was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarsDistanceRatio was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinLightBarsDistanceRatio参数
    if ((!fileStorage["MinLightBarsDistanceRatio"].isNone()) && (fileStorage["MinLightBarsDistanceRatio"].isReal()))
    {
        recognizerParam->MinLightBarsDistanceRatio = static_cast<float>(fileStorage["MinLightBarsDistanceRatio"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MinLightBarsDistanceRatio was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MinLightBarsDistanceRatio was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLightBarsAngleOffset参数
    if ((!fileStorage["MaxLightBarsAngleOffset"].isNone()) && (fileStorage["MaxLightBarsAngleOffset"].isReal()))
    {
        recognizerParam->MaxLightBarsAngleOffset = static_cast<float>(fileStorage["MaxLightBarsAngleOffset"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarsAngleOffset was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarsAngleOffset was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLightBarsHeightRatio参数
    if ((!fileStorage["MaxLightBarsHeightRatio"].isNone()) && (fileStorage["MaxLightBarsHeightRatio"].isReal()))
    {
        recognizerParam->MaxLightBarsHeightRatio = static_cast<float>(fileStorage["MaxLightBarsHeightRatio"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarsHeightRatio was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's MaxLightBarsHeightRatio was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorHogWindowWidth参数
    if ((!fileStorage["ArmorHogWindowWidth"].isNone()) && (fileStorage["ArmorHogWindowWidth"].isInt()))
    {
        recognizerParam->ArmorHogWindowWidth = static_cast<int>(fileStorage["ArmorHogWindowWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogWindowWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogWindowWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorHogWindowHeight参数
    if ((!fileStorage["ArmorHogWindowHeight"].isNone()) && (fileStorage["ArmorHogWindowHeight"].isInt()))
    {
        recognizerParam->ArmorHogWindowHeight = static_cast<int>(fileStorage["ArmorHogWindowHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogWindowHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogWindowHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorHogBlockWidth参数
    if ((!fileStorage["ArmorHogBlockWidth"].isNone()) && (fileStorage["ArmorHogBlockWidth"].isInt()))
    {
        recognizerParam->ArmorHogBlockWidth = static_cast<int>(fileStorage["ArmorHogBlockWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogBlockWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogBlockWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorHogBlockHeight参数
    if ((!fileStorage["ArmorHogBlockHeight"].isNone()) && (fileStorage["ArmorHogBlockHeight"].isInt()))
    {
        recognizerParam->ArmorHogBlockHeight = static_cast<int>(fileStorage["ArmorHogBlockHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogBlockHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogBlockHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorHogCellWidth参数
    if ((!fileStorage["ArmorHogCellWidth"].isNone()) && (fileStorage["ArmorHogCellWidth"].isInt()))
    {
        recognizerParam->ArmorHogCellWidth = static_cast<int>(fileStorage["ArmorHogCellWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogCellWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogCellWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorHogCellHeight参数
    if ((!fileStorage["ArmorHogCellHeight"].isNone()) && (fileStorage["ArmorHogCellHeight"].isInt()))
    {
        recognizerParam->ArmorHogCellHeight = static_cast<int>(fileStorage["ArmorHogCellHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogCellHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogCellHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorHogStrideWidth参数
    if ((!fileStorage["ArmorHogStrideWidth"].isNone()) && (fileStorage["ArmorHogStrideWidth"].isInt()))
    {
        recognizerParam->ArmorHogStrideWidth = static_cast<int>(fileStorage["ArmorHogStrideWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogStrideWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogStrideWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorHogStrideHeight参数
    if ((!fileStorage["ArmorHogStrideHeight"].isNone()) && (fileStorage["ArmorHogStrideHeight"].isInt()))
    {
        recognizerParam->ArmorHogStrideHeight = static_cast<int>(fileStorage["ArmorHogStrideHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogStrideHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogStrideHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorHogBins参数
    if ((!fileStorage["ArmorHogBins"].isNone()) && (fileStorage["ArmorHogBins"].isInt()))
    {
        recognizerParam->ArmorHogBins = static_cast<int>(fileStorage["ArmorHogBins"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogBins was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogBins was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取ArmorHogSvmFileName参数
    if ((!fileStorage["ArmorHogSvmFileName"].isNone()) && (fileStorage["ArmorHogSvmFileName"].isString()))
    {
        recognizerParam->ArmorHogSvmFileName = static_cast<std::string>(fileStorage["ArmorHogSvmFileName"]);
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogSvmFileName was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalArmorRecognizerParam's ArmorHogSvmFileName was loaded failure";
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