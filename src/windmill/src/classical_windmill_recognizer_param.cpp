//
// Created by plutoli on 2022/1/23.
//

#include "classical_windmill_recognizer_param.h"

// ******************************  ClassicalWindmillRecognizerParam类的公有函数  ******************************

// 构造函数
ClassicalWindmillRecognizerParam::ClassicalWindmillRecognizerParam():
    Key("Windmill"),
    WindmillColor(EClassicalWindmillColor::Red),
    DownsampleFactor(1.0),
    BGRWeightForBlue(),
    BGRWeightForRed(),
    GrayThresholdForBlue(120),
    GrayThresholdForRed(120),
    FanHSVThresholdForBlue(),
    FanHSVThresholdForRed_1(),
    FanHSVThresholdForRed_2(),
    TargetHSVThresholdForBlue(),
    TargetHSVThresholdForRed_1(),
    TargetHSVThresholdForRed_2(),
    CloseElementWidth(5),
    CloseElementHeight(5),
    MaxLogoContourArea(0.0),
    MinLogoContourArea(40.0),
    MaxLogoAspectRatio(0.0),
    MinLogoAspectRatio(0.0),
    LogoHogWindowWidth(40),
    LogoHogWindowHeight(40),
    LogoHogBlockWidth(16),
    LogoHogBlockHeight(16),
    LogoHogCellWidth(8),
    LogoHogCellHeight(8),
    LogoHogStrideWidth(8),
    LogoHogStrideHeight(8),
    LogoHogBins(9),
    LogoHogSvmFileName(),
    MaxFanContourArea(0.0),
    MinFanContourArea(0.0),
    MaxFanAspectRatio(0.0),
    MinFanAspectRatio(0.0),
    MaxTargetContourArea(0.0),
    MinTargetContourArea(0.0),
    MaxTargetAspectRatio(0.0),
    MinTargetAspectRatio(0.0),
    MaxFanTargetAngle(0.0),
    MinFanTargetAngle(0.0),
    TargetPhysicalHeight(0.0),
    TargetPhysicalWidth(0.0),
    LogoPhysicalHeight(0.0),
    LogoPhysicalWidth(0.0),
    MaxLogoLocationOffset(0.0),
    MaxFanLocationOffset(0.0),
    MaxMemoryLength(0)
{
}

// 转换风车颜色
bool ClassicalWindmillRecognizerParam::ConvertToWindmillColor(const int &input, EClassicalWindmillColor *output)
{
    // 初始化转换结果
    bool result = true;

    // 转换风车颜色
    switch (input)
    {
        case 1:
            *output = EClassicalWindmillColor::Red;
            break;

        case 2:
            *output = EClassicalWindmillColor::Blue;
            break;

        default:
            result = false;
            break;
    }

    // 返回转换结果
    return result;
}

// 从yaml配置文件中加载风车识别器参数
bool ClassicalWindmillRecognizerParam::LoadFromYamlFile(const std::string &yamlFileName,
                                                        ClassicalWindmillRecognizerParam *recognizerParam)
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
        log = "ClassicalWindmillRecognizerParam was loaded failure because yaml file does not exist";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断yaml配置文件是否可读
    if (::access(yamlFileName.c_str(), R_OK) == -1)
    {
        log = "ClassicalWindmillRecognizerParam was loaded failure because yaml file can not be read";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        log = "ClassicalWindmillRecognizerParam was loaded failure because yaml file can not be opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 读取Key参数
    if ((!fileStorage["Key"].isNone()) && (fileStorage["Key"].isString()))
    {
        recognizerParam->Key = static_cast<std::string>(fileStorage["Key"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Key was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Key was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取WindmillColor参数
    if ((!fileStorage["WindmillColor"].isNone()) && (fileStorage["WindmillColor"].isInt()))
    {
        EClassicalWindmillColor windmillColor;
        if (ClassicalWindmillRecognizerParam::ConvertToWindmillColor(static_cast<int>(fileStorage["WindmillColor"]),
                                                                     &windmillColor))
        {
            recognizerParam->WindmillColor = windmillColor;
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's WindmillColor was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's WindmillColor was converted failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's WindmillColor was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取DownsampleFactor参数
    if ((!fileStorage["DownsampleFactor"].isNone()) && (fileStorage["DownsampleFactor"].isReal()))
    {
        recognizerParam->DownsampleFactor = static_cast<float>(fileStorage["DownsampleFactor"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's DownsampleFactor was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's DownsampleFactor was loaded failure";
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
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Blue weight "\
                  "in BGRWeightForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Blue weight "\
                  "in BGRWeightForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取Green参数
        if ((!bgrWeightForBlueNode["Green"].isNone()) && (bgrWeightForBlueNode["Green"].isReal()))
        {
            recognizerParam->BGRWeightForBlue.Green = static_cast<float>(bgrWeightForBlueNode["Green"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Green weight "\
                  "in BGRWeightForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Green weight "\
                  "in BGRWeightForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取Red参数
        if ((!bgrWeightForBlueNode["Red"].isNone()) && (bgrWeightForBlueNode["Red"].isReal()))
        {
            recognizerParam->BGRWeightForBlue.Red = static_cast<float>(bgrWeightForBlueNode["Red"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Red weight "\
                  "in BGRWeightForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Red weight "\
                  "in BGRWeightForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's BGRWeightForBlue was loaded failure "\
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
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Blue weight "\
                  "in BGRWeightForRed was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Blue weight "\
                  "in BGRWeightForRed was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取Green参数
        if ((!bgrWeightForRedNode["Green"].isNone()) && (bgrWeightForRedNode["Green"].isReal()))
        {
            recognizerParam->BGRWeightForRed.Green = static_cast<float>(bgrWeightForRedNode["Green"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Green weight "\
                  "in BGRWeightForRed was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Green weight "\
                  "in BGRWeightForRed was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取Red参数
        if ((!bgrWeightForRedNode["Red"].isNone()) && (bgrWeightForRedNode["Red"].isReal()))
        {
            recognizerParam->BGRWeightForRed.Red = static_cast<float>(bgrWeightForRedNode["Red"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Red weight "\
                  "in BGRWeightForRed was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's Red weight "\
                  "in BGRWeightForRed was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's BGRWeightForRed was loaded failure "\
              "because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取GrayThresholdForBlue参数
    if ((!fileStorage["GrayThresholdForBlue"].isNone()) && (fileStorage["GrayThresholdForBlue"].isInt()))
    {
        recognizerParam->GrayThresholdForBlue = static_cast<int>(fileStorage["GrayThresholdForBlue"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's GrayThresholdForBlue was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's GrayThresholdForBlue was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取GrayThresholdForRed参数
    if ((!fileStorage["GrayThresholdForRed"].isNone()) && (fileStorage["GrayThresholdForRed"].isInt()))
    {
        recognizerParam->GrayThresholdForRed = static_cast<int>(fileStorage["GrayThresholdForRed"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's GrayThresholdForRed was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's GrayThresholdForRed was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取FanHSVThresholdForBlue参数
    cv::FileNode hsvFanThresholdForBlueNode = fileStorage["FanHSVThresholdForBlue"];
    if (!hsvFanThresholdForBlueNode.empty())
    {
        // 读取HueLower参数
        if ((!hsvFanThresholdForBlueNode["HueLower"].isNone()) && (hsvFanThresholdForBlueNode["HueLower"].isInt()))
        {
            recognizerParam->FanHSVThresholdForBlue.HueLower = static_cast<int>(hsvFanThresholdForBlueNode["HueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVFanThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVFanThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HueUpper参数
        if ((!hsvFanThresholdForBlueNode["HueUpper"].isNone()) && (hsvFanThresholdForBlueNode["HueUpper"].isInt()))
        {
            recognizerParam->FanHSVThresholdForBlue.HueUpper = static_cast<int>(hsvFanThresholdForBlueNode["HueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVFanThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVFanThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationLower参数
        if ((!hsvFanThresholdForBlueNode["SaturationLower"].isNone()) && (hsvFanThresholdForBlueNode["SaturationLower"].isInt()))
        {
            recognizerParam->FanHSVThresholdForBlue.SaturationLower = static_cast<int>(hsvFanThresholdForBlueNode["SaturationLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVFanThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVFanThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationUpper参数
        if ((!hsvFanThresholdForBlueNode["SaturationUpper"].isNone()) && (hsvFanThresholdForBlueNode["SaturationUpper"].isInt()))
        {
            recognizerParam->FanHSVThresholdForBlue.SaturationUpper = static_cast<int>(hsvFanThresholdForBlueNode["SaturationUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVFanThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVFanThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueLower参数
        if ((!hsvFanThresholdForBlueNode["ValueLower"].isNone()) && (hsvFanThresholdForBlueNode["ValueLower"].isInt()))
        {
            recognizerParam->FanHSVThresholdForBlue.ValueLower = static_cast<int>(hsvFanThresholdForBlueNode["ValueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVFanThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVFanThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueUpper参数
        if ((!hsvFanThresholdForBlueNode["ValueUpper"].isNone()) && (hsvFanThresholdForBlueNode["ValueUpper"].isInt()))
        {
            recognizerParam->FanHSVThresholdForBlue.ValueUpper = static_cast<int>(hsvFanThresholdForBlueNode["ValueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVFanThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVFanThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HSVFanThresholdForBlue "\
              "was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取FanHSVThresholdForRed_1参数
    cv::FileNode hsvFanThresholdForRedNode_1 = fileStorage["FanHSVThresholdForRed_1"];
    if (!hsvFanThresholdForRedNode_1.empty())
    {
        // 读取HueLower参数
        if ((!hsvFanThresholdForRedNode_1["HueLower"].isNone()) && (hsvFanThresholdForRedNode_1["HueLower"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_1.HueLower = static_cast<int>(hsvFanThresholdForRedNode_1["HueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVFanThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVFanThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HueUpper参数
        if ((!hsvFanThresholdForRedNode_1["HueUpper"].isNone()) && (hsvFanThresholdForRedNode_1["HueUpper"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_1.HueUpper = static_cast<int>(hsvFanThresholdForRedNode_1["HueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVFanThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVFanThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationLower参数
        if ((!hsvFanThresholdForRedNode_1["SaturationLower"].isNone()) && (hsvFanThresholdForRedNode_1["SaturationLower"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_1.SaturationLower = static_cast<int>(hsvFanThresholdForRedNode_1["SaturationLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVFanThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVFanThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationUpper参数
        if ((!hsvFanThresholdForRedNode_1["SaturationUpper"].isNone()) && (hsvFanThresholdForRedNode_1["SaturationUpper"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_1.SaturationUpper = static_cast<int>(hsvFanThresholdForRedNode_1["SaturationUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVFanThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVFanThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueLower参数
        if ((!hsvFanThresholdForRedNode_1["ValueLower"].isNone()) && (hsvFanThresholdForRedNode_1["ValueLower"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_1.ValueLower = static_cast<int>(hsvFanThresholdForRedNode_1["ValueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVFanThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVFanThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueUpper参数
        if ((!hsvFanThresholdForRedNode_1["ValueUpper"].isNone()) && (hsvFanThresholdForRedNode_1["ValueUpper"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_1.ValueUpper = static_cast<int>(hsvFanThresholdForRedNode_1["ValueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVFanThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVFanThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HSVFanThresholdForRed_1 "\
              "was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取FanHSVThresholdForRed_2参数
    cv::FileNode hsvFanThresholdForRedNode_2 = fileStorage["FanHSVThresholdForRed_2"];
    if (!hsvFanThresholdForRedNode_2.empty())
    {
        // 读取HueLower参数
        if ((!hsvFanThresholdForRedNode_2["HueLower"].isNone()) && (hsvFanThresholdForRedNode_2["HueLower"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_2.HueLower = static_cast<int>(hsvFanThresholdForRedNode_2["HueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVFanThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVFanThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HueUpper参数
        if ((!hsvFanThresholdForRedNode_2["HueUpper"].isNone()) && (hsvFanThresholdForRedNode_2["HueUpper"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_2.HueUpper = static_cast<int>(hsvFanThresholdForRedNode_2["HueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVFanThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVFanThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationLower参数
        if ((!hsvFanThresholdForRedNode_2["SaturationLower"].isNone()) && (hsvFanThresholdForRedNode_2["SaturationLower"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_2.SaturationLower = static_cast<int>(hsvFanThresholdForRedNode_2["SaturationLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVFanThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVFanThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationUpper参数
        if ((!hsvFanThresholdForRedNode_2["SaturationUpper"].isNone()) && (hsvFanThresholdForRedNode_2["SaturationUpper"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_2.SaturationUpper = static_cast<int>(hsvFanThresholdForRedNode_2["SaturationUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVFanThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVFanThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueLower参数
        if ((!hsvFanThresholdForRedNode_2["ValueLower"].isNone()) && (hsvFanThresholdForRedNode_2["ValueLower"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_2.ValueLower = static_cast<int>(hsvFanThresholdForRedNode_2["ValueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVFanThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVFanThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueUpper参数
        if ((!hsvFanThresholdForRedNode_2["ValueUpper"].isNone()) && (hsvFanThresholdForRedNode_2["ValueUpper"].isInt()))
        {
            recognizerParam->FanHSVThresholdForRed_2.ValueUpper = static_cast<int>(hsvFanThresholdForRedNode_2["ValueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVFanThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVFanThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HSVFanThresholdForRed_2 "\
              "was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取TargetHSVThresholdForBlue参数
    cv::FileNode hsvTargetThresholdForBlueNode = fileStorage["TargetHSVThresholdForBlue"];
    if (!hsvTargetThresholdForBlueNode.empty())
    {
        // 读取HueLower参数
        if ((!hsvTargetThresholdForBlueNode["HueLower"].isNone()) && (hsvTargetThresholdForBlueNode["HueLower"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForBlue.HueLower = static_cast<int>(hsvTargetThresholdForBlueNode["HueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVTargetThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVTargetThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HueUpper参数
        if ((!hsvTargetThresholdForBlueNode["HueUpper"].isNone()) && (hsvTargetThresholdForBlueNode["HueUpper"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForBlue.HueUpper = static_cast<int>(hsvTargetThresholdForBlueNode["HueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVTargetThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVTargetThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationLower参数
        if ((!hsvTargetThresholdForBlueNode["SaturationLower"].isNone()) && (hsvTargetThresholdForBlueNode["SaturationLower"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForBlue.SaturationLower = static_cast<int>(hsvTargetThresholdForBlueNode["SaturationLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVTargetThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVTargetThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationUpper参数
        if ((!hsvTargetThresholdForBlueNode["SaturationUpper"].isNone()) && (hsvTargetThresholdForBlueNode["SaturationUpper"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForBlue.SaturationUpper = static_cast<int>(hsvTargetThresholdForBlueNode["SaturationUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVTargetThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVTargetThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueLower参数
        if ((!hsvTargetThresholdForBlueNode["ValueLower"].isNone()) && (hsvTargetThresholdForBlueNode["ValueLower"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForBlue.ValueLower = static_cast<int>(hsvTargetThresholdForBlueNode["ValueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVTargetThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVTargetThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueUpper参数
        if ((!hsvTargetThresholdForBlueNode["ValueUpper"].isNone()) && (hsvTargetThresholdForBlueNode["ValueUpper"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForBlue.ValueUpper = static_cast<int>(hsvTargetThresholdForBlueNode["ValueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVTargetThresholdForBlue was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVTargetThresholdForBlue was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HSVTargetThresholdForBlue "\
              "was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取TargetHSVThresholdForRed_1参数
    cv::FileNode hsvTargetThresholdForRedNode_1 = fileStorage["TargetHSVThresholdForRed_1"];
    if (!hsvTargetThresholdForRedNode_1.empty())
    {
        // 读取HueLower参数
        if ((!hsvTargetThresholdForRedNode_1["HueLower"].isNone()) && (hsvTargetThresholdForRedNode_1["HueLower"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_1.HueLower = static_cast<int>(hsvTargetThresholdForRedNode_1["HueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVTargetThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVTargetThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HueUpper参数
        if ((!hsvTargetThresholdForRedNode_1["HueUpper"].isNone()) && (hsvTargetThresholdForRedNode_1["HueUpper"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_1.HueUpper = static_cast<int>(hsvTargetThresholdForRedNode_1["HueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVTargetThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVTargetThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationLower参数
        if ((!hsvTargetThresholdForRedNode_1["SaturationLower"].isNone()) && (hsvTargetThresholdForRedNode_1["SaturationLower"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_1.SaturationLower = static_cast<int>(hsvTargetThresholdForRedNode_1["SaturationLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVTargetThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVTargetThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationUpper参数
        if ((!hsvTargetThresholdForRedNode_1["SaturationUpper"].isNone()) && (hsvTargetThresholdForRedNode_1["SaturationUpper"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_1.SaturationUpper = static_cast<int>(hsvTargetThresholdForRedNode_1["SaturationUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVTargetThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVTargetThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueLower参数
        if ((!hsvTargetThresholdForRedNode_1["ValueLower"].isNone()) && (hsvTargetThresholdForRedNode_1["ValueLower"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_1.ValueLower = static_cast<int>(hsvTargetThresholdForRedNode_1["ValueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVTargetThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVTargetThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueUpper参数
        if ((!hsvTargetThresholdForRedNode_1["ValueUpper"].isNone()) && (hsvTargetThresholdForRedNode_1["ValueUpper"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_1.ValueUpper = static_cast<int>(hsvTargetThresholdForRedNode_1["ValueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVTargetThresholdForRed_1 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVTargetThresholdForRed_1 was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HSVTargetThresholdForRed_1 "\
              "was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取TargetHSVThresholdForRed_2参数
    cv::FileNode hsvTargetThresholdForRedNode_2 = fileStorage["TargetHSVThresholdForRed_2"];
    if (!hsvTargetThresholdForRedNode_2.empty())
    {
        // 读取HueLower参数
        if ((!hsvTargetThresholdForRedNode_2["HueLower"].isNone()) && (hsvTargetThresholdForRedNode_2["HueLower"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_2.HueLower = static_cast<int>(hsvTargetThresholdForRedNode_2["HueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVTargetThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueLower "\
                  "in HSVTargetThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取HueUpper参数
        if ((!hsvTargetThresholdForRedNode_2["HueUpper"].isNone()) && (hsvTargetThresholdForRedNode_2["HueUpper"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_2.HueUpper = static_cast<int>(hsvTargetThresholdForRedNode_2["HueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVTargetThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HueUpper "\
                  "in HSVTargetThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationLower参数
        if ((!hsvTargetThresholdForRedNode_2["SaturationLower"].isNone()) && (hsvTargetThresholdForRedNode_2["SaturationLower"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_2.SaturationLower = static_cast<int>(hsvTargetThresholdForRedNode_2["SaturationLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVTargetThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationLower "\
                  "in HSVTargetThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取SaturationUpper参数
        if ((!hsvTargetThresholdForRedNode_2["SaturationUpper"].isNone()) && (hsvTargetThresholdForRedNode_2["SaturationUpper"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_2.SaturationUpper = static_cast<int>(hsvTargetThresholdForRedNode_2["SaturationUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVTargetThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's SaturationUpper "\
                  "in HSVTargetThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueLower参数
        if ((!hsvTargetThresholdForRedNode_2["ValueLower"].isNone()) && (hsvTargetThresholdForRedNode_2["ValueLower"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_2.ValueLower = static_cast<int>(hsvTargetThresholdForRedNode_2["ValueLower"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVTargetThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueLower "\
                  "in HSVTargetThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }

        // 读取ValueUpper参数
        if ((!hsvTargetThresholdForRedNode_2["ValueUpper"].isNone()) && (hsvTargetThresholdForRedNode_2["ValueUpper"].isInt()))
        {
            recognizerParam->TargetHSVThresholdForRed_2.ValueUpper = static_cast<int>(hsvTargetThresholdForRedNode_2["ValueUpper"]);
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVTargetThresholdForRed_2 was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's ValueUpper "\
                  "in HSVTargetThresholdForRed_2 was loaded failure";
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's HSVTargetThresholdForRed_2 "\
              "was loaded failure because it is empty";
        logger.Save(ELogType::Error, log);
    }

    // 读取CloseElementWidth参数
    if ((!fileStorage["CloseElementWidth"].isNone()) && (fileStorage["CloseElementWidth"].isInt()))
    {
        recognizerParam->CloseElementWidth = static_cast<int>(fileStorage["CloseElementWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's CloseElementWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's CloseElementWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取CloseElementHeight参数
    if ((!fileStorage["CloseElementHeight"].isNone()) && (fileStorage["CloseElementHeight"].isInt()))
    {
        recognizerParam->CloseElementHeight = static_cast<int>(fileStorage["CloseElementHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's CloseElementHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's CloseElementHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLogoContourArea参数
    if ((!fileStorage["MaxLogoContourArea"].isNone()) && (fileStorage["MaxLogoContourArea"].isReal()))
    {
        recognizerParam->MaxLogoContourArea = static_cast<float>(fileStorage["MaxLogoContourArea"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxLogoContourArea was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxLogoContourArea was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinLogoContourArea参数
    if ((!fileStorage["MinLogoContourArea"].isNone()) && (fileStorage["MinLogoContourArea"].isReal()))
    {
        recognizerParam->MinLogoContourArea = static_cast<float>(fileStorage["MinLogoContourArea"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinLogoContourArea was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinLogoContourArea was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLogoAspectRatio参数
    if ((!fileStorage["MaxLogoAspectRatio"].isNone()) && (fileStorage["MaxLogoAspectRatio"].isReal()))
    {
        recognizerParam->MaxLogoAspectRatio = static_cast<float>(fileStorage["MaxLogoAspectRatio"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxLogoAspectRatio was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxLogoAspectRatio was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinLogoAspectRatio参数
    if ((!fileStorage["MinLogoAspectRatio"].isNone()) && (fileStorage["MinLogoAspectRatio"].isReal()))
    {
        recognizerParam->MinLogoAspectRatio = static_cast<float>(fileStorage["MinLogoAspectRatio"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinLogoAspectRatio was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinLogoAspectRatio was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoHogWindowWidth参数
    if ((!fileStorage["LogoHogWindowWidth"].isNone()) && (fileStorage["LogoHogWindowWidth"].isInt()))
    {
        recognizerParam->LogoHogWindowWidth = static_cast<int>(fileStorage["LogoHogWindowWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogWindowWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogWindowWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoHogWindowHeight参数
    if ((!fileStorage["LogoHogWindowHeight"].isNone()) && (fileStorage["LogoHogWindowHeight"].isInt()))
    {
        recognizerParam->LogoHogWindowHeight = static_cast<int>(fileStorage["LogoHogWindowHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogWindowHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogWindowHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoHogBlockWidth参数
    if ((!fileStorage["LogoHogBlockWidth"].isNone()) && (fileStorage["LogoHogBlockWidth"].isInt()))
    {
        recognizerParam->LogoHogBlockWidth = static_cast<int>(fileStorage["LogoHogBlockWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogBlockWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogBlockWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoHogBlockHeight参数
    if ((!fileStorage["LogoHogBlockHeight"].isNone()) && (fileStorage["LogoHogBlockHeight"].isInt()))
    {
        recognizerParam->LogoHogBlockHeight = static_cast<int>(fileStorage["LogoHogBlockHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogBlockHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogBlockHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoHogCellWidth参数
    if ((!fileStorage["LogoHogCellWidth"].isNone()) && (fileStorage["LogoHogCellWidth"].isInt()))
    {
        recognizerParam->LogoHogCellWidth = static_cast<int>(fileStorage["LogoHogCellWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogCellWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogCellWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoHogCellHeight参数
    if ((!fileStorage["LogoHogCellHeight"].isNone()) && (fileStorage["LogoHogCellHeight"].isInt()))
    {
        recognizerParam->LogoHogCellHeight = static_cast<int>(fileStorage["LogoHogCellHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogCellHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogCellHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoHogStrideWidth参数
    if ((!fileStorage["LogoHogStrideWidth"].isNone()) && (fileStorage["LogoHogStrideWidth"].isInt()))
    {
        recognizerParam->LogoHogStrideWidth = static_cast<int>(fileStorage["LogoHogStrideWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogStrideWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogStrideWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoHogStrideHeight参数
    if ((!fileStorage["LogoHogStrideHeight"].isNone()) && (fileStorage["LogoHogStrideHeight"].isInt()))
    {
        recognizerParam->LogoHogStrideHeight = static_cast<int>(fileStorage["LogoHogStrideHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogStrideHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogStrideHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoHogBins参数
    if ((!fileStorage["LogoHogBins"].isNone()) && (fileStorage["LogoHogBins"].isInt()))
    {
        recognizerParam->LogoHogBins = static_cast<int>(fileStorage["LogoHogBins"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogBins was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogBins was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoHogSvmFileName参数
    if ((!fileStorage["LogoHogSvmFileName"].isNone()) && (fileStorage["LogoHogSvmFileName"].isString()))
    {
        recognizerParam->LogoHogSvmFileName = static_cast<std::string>(fileStorage["LogoHogSvmFileName"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogSvmFileName was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoHogSvmFileName was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxFanContourArea参数
    if ((!fileStorage["MaxFanContourArea"].isNone()) && (fileStorage["MaxFanContourArea"].isReal()))
    {
        recognizerParam->MaxFanContourArea = static_cast<float>(fileStorage["MaxFanContourArea"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxFanContourArea was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxFanContourArea was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinFanContourArea参数
    if ((!fileStorage["MinFanContourArea"].isNone()) && (fileStorage["MinFanContourArea"].isReal()))
    {
        recognizerParam->MinFanContourArea = static_cast<float>(fileStorage["MinFanContourArea"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinFanContourArea was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinFanContourArea was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxFanAspectRatio参数
    if ((!fileStorage["MaxFanAspectRatio"].isNone()) && (fileStorage["MaxFanAspectRatio"].isReal()))
    {
        recognizerParam->MaxFanAspectRatio = static_cast<float>(fileStorage["MaxFanAspectRatio"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxFanAspectRatio was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxFanAspectRatio was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinFanAspectRatio参数
    if ((!fileStorage["MinFanAspectRatio"].isNone()) && (fileStorage["MinFanAspectRatio"].isReal()))
    {
        recognizerParam->MinFanAspectRatio = static_cast<float>(fileStorage["MinFanAspectRatio"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinFanAspectRatio was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinFanAspectRatio was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxTargetContourArea参数
    if ((!fileStorage["MaxTargetContourArea"].isNone()) && (fileStorage["MaxTargetContourArea"].isReal()))
    {
        recognizerParam->MaxTargetContourArea = static_cast<float>(fileStorage["MaxTargetContourArea"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxTargetContourArea was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxTargetContourArea was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinTargetContourArea参数
    if ((!fileStorage["MinTargetContourArea"].isNone()) && (fileStorage["MinTargetContourArea"].isReal()))
    {
        recognizerParam->MinTargetContourArea = static_cast<float>(fileStorage["MinTargetContourArea"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinTargetContourArea was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinTargetContourArea was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxTargetAspectRatio参数
    if ((!fileStorage["MaxTargetAspectRatio"].isNone()) && (fileStorage["MaxTargetAspectRatio"].isReal()))
    {
        recognizerParam->MaxTargetAspectRatio = static_cast<float>(fileStorage["MaxTargetAspectRatio"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxTargetAspectRatio was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxTargetAspectRatio was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinTargetAspectRatio参数
    if ((!fileStorage["MinTargetAspectRatio"].isNone()) && (fileStorage["MinTargetAspectRatio"].isReal()))
    {
        recognizerParam->MinTargetAspectRatio = static_cast<float>(fileStorage["MinTargetAspectRatio"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinTargetAspectRatio was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinTargetAspectRatio was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxFanTargetAngle参数
    if ((!fileStorage["MaxFanTargetAngle"].isNone()) && (fileStorage["MaxFanTargetAngle"].isReal()))
    {
        recognizerParam->MaxFanTargetAngle = static_cast<float>(fileStorage["MaxFanTargetAngle"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxFanTargetAngle was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxFanTargetAngle was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MinFanTargetAngle参数
    if ((!fileStorage["MinFanTargetAngle"].isNone()) && (fileStorage["MinFanTargetAngle"].isReal()))
    {
        recognizerParam->MinFanTargetAngle = static_cast<float>(fileStorage["MinFanTargetAngle"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinFanTargetAngle was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MinFanTargetAngle was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取TargetPhysicalHeight参数
    if ((!fileStorage["TargetPhysicalHeight"].isNone()) && (fileStorage["TargetPhysicalHeight"].isReal()))
    {
        recognizerParam->TargetPhysicalHeight = static_cast<float>(fileStorage["TargetPhysicalHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's TargetPhysicalHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's TargetPhysicalHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取TargetPhysicalWidth参数
    if ((!fileStorage["TargetPhysicalWidth"].isNone()) && (fileStorage["TargetPhysicalWidth"].isReal()))
    {
        recognizerParam->TargetPhysicalWidth = static_cast<float>(fileStorage["TargetPhysicalWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's TargetPhysicalWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's TargetPhysicalWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoPhysicalHeight参数
    if ((!fileStorage["LogoPhysicalHeight"].isNone()) && (fileStorage["LogoPhysicalHeight"].isReal()))
    {
        recognizerParam->LogoPhysicalHeight = static_cast<float>(fileStorage["LogoPhysicalHeight"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoPhysicalHeight was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoPhysicalHeight was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取LogoPhysicalWidth参数
    if ((!fileStorage["LogoPhysicalWidth"].isNone()) && (fileStorage["LogoPhysicalWidth"].isReal()))
    {
        recognizerParam->LogoPhysicalWidth = static_cast<float>(fileStorage["LogoPhysicalWidth"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoPhysicalWidth was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's LogoPhysicalWidth was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxLogoLocationOffset参数
    if ((!fileStorage["MaxLogoLocationOffset"].isNone()) && (fileStorage["MaxLogoLocationOffset"].isReal()))
    {
        recognizerParam->MaxLogoLocationOffset = static_cast<float>(fileStorage["MaxLogoLocationOffset"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxLogoLocationOffset was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxLogoLocationOffset was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxFanLocationOffset参数
    if ((!fileStorage["MaxFanLocationOffset"].isNone()) && (fileStorage["MaxFanLocationOffset"].isReal()))
    {
        recognizerParam->MaxFanLocationOffset = static_cast<float>(fileStorage["MaxFanLocationOffset"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxFanLocationOffset was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxFanLocationOffset was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取MaxMemoryLength参数
    if ((!fileStorage["MaxMemoryLength"].isNone()) && (fileStorage["MaxMemoryLength"].isInt()))
    {
        recognizerParam->MaxMemoryLength = static_cast<int>(fileStorage["MaxMemoryLength"]);
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxMemoryLength was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam's MaxMemoryLength was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 关闭文件存储器
    fileStorage.release();

    // 记录日志信息
    log = "[" + recognizerParam->Key + "] - ClassicalWindmillRecognizerParam was loaded completely";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回加载结果
    return true;
}