//
// Created by tony on 2022/4/23.
//

#include"predictor_param.h"

// 构造函数
PredictorParam::PredictorParam():Q(Eigen::Matrix<double, 9, 9>::Identity()),
                                 R(Eigen::Matrix<double, 4, 4> ::Identity()),
                                 Q00(1),Q11(1),Q22(1),Q33(1),Q44(1),Q55(1),Q66(1),Q77(1),Q88(1),
                                 R00(1),R11(1),R22(1),R33(1){}

// 从yaml文件中加载预测器参数
bool PredictorParam::LoadFromYamlFile(const std::string &yamlFileName, PredictorParam *predictParam){

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断yaml配置文件是否存在
    if (::access(yamlFileName.c_str(), F_OK) == -1)
    {
        log = "PredictorParam was loaded failure because yaml file does not exist";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 判断yaml配置文件是否可读
    if (::access(yamlFileName.c_str(), R_OK) == -1)
    {
        log = "PredictorParam was loaded failure because yaml file can not be read";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 创建并打开文件存储器
    cv::FileStorage fileStorage;
    if (!fileStorage.open(yamlFileName, cv::FileStorage::READ))
    {
        log = "PredictorParam was loaded failure because yaml file can not be opened";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 读取Key参数
    if ((!fileStorage["Key"].isNone()) && (fileStorage["Key"].isString()))
    {
        predictParam->Key = static_cast<std::string>(fileStorage["Key"]);
        log = "[" + predictParam->Key + "] - PredictorParam's Key was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's Key was loaded failure";
        logger.Save(ELogType::Error, log);
    }



    // 读取GapIndex参数
    if ((!fileStorage["GapIndex"].isNone()) && (fileStorage["GapIndex"].isInt()))
    {
        predictParam->GapIndex = static_cast<int>(fileStorage["GapIndex"]);
        log = "[" + predictParam->Key + "] - PredictorParam's GapIndex was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's GapIndex was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Q00参数
    if ((!fileStorage["Q00"].isNone()))
    {
        predictParam->Q00 = static_cast<double>(fileStorage["Q00"]);
        predictParam->Q(0,0)=predictParam->Q00;
        log = "[" + predictParam->Key + "] - PredictorParam's Q00 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's Q00 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Q11参数
    if ((!fileStorage["Q11"].isNone()))
    {
        predictParam->Q11 = static_cast<double>(fileStorage["Q11"]);
        predictParam->Q(1,1)=predictParam->Q11;
        log = "[" + predictParam->Key + "] - PredictorParam's Q11 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's Q11 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Q22参数
    if ((!fileStorage["Q22"].isNone()))
    {
        predictParam->Q22 = static_cast<double>(fileStorage["Q22"]);
        predictParam->Q(2,2)=predictParam->Q22;
        log = "[" + predictParam->Key + "] - PredictorParam's Q22 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's Q22 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Q33参数
    if ((!fileStorage["Q33"].isNone()))
    {
        predictParam->Q33 = static_cast<double>(fileStorage["Q33"]);
        predictParam->Q(3,3)=predictParam->Q33;
        log = "[" + predictParam->Key + "] - PredictorParam's Q33 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's Q33 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Q44参数
    if ((!fileStorage["Q44"].isNone()))
    {
        predictParam->Q44 = static_cast<double>(fileStorage["Q44"]);
        predictParam->Q(4,4)=predictParam->Q44;
        log = "[" + predictParam->Key + "] - PredictorParam's Q44 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's Q44 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Q55参数
    if ((!fileStorage["Q55"].isNone()))
    {
        predictParam->Q55 = static_cast<double>(fileStorage["Q55"]);
        predictParam->Q(5,5)=predictParam->Q55;
        log = "[" + predictParam->Key + "] - PredictorParam's Q55 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's Q55 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Q66参数
    if ((!fileStorage["Q66"].isNone()))
    {
        predictParam->Q66 = static_cast<double>(fileStorage["Q66"]);
        predictParam->Q(6,6)=predictParam->Q66;
        log = "[" + predictParam->Key + "] - PredictorParam's Q66 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's Q66 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Q77参数
    if ((!fileStorage["Q77"].isNone()))
    {
        predictParam->Q77 = static_cast<double>(fileStorage["Q77"]);
        predictParam->Q(7,7)=predictParam->Q77;
        log = "[" + predictParam->Key + "] - PredictorParam's Q77 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's Q77 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取Q88参数
    if ((!fileStorage["Q88"].isNone()))
    {
        predictParam->Q88 = static_cast<double>(fileStorage["Q88"]);
        predictParam->Q(8,8)=predictParam->Q88;
        log = "[" + predictParam->Key + "] - PredictorParam's Q88 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's Q88 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取R00参数
    if ((!fileStorage["R00"].isNone()))
    {
        predictParam->R00= static_cast<double>(fileStorage["R00"]);
        predictParam->R(0,0)=predictParam->R00;
        log = "[" + predictParam->Key + "] - PredictorParam's R00 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's R00 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取R11参数
    if ((!fileStorage["R11"].isNone()) )
    {
        predictParam->R11= static_cast<double>(fileStorage["R11"]);
        predictParam->R(1,1)=predictParam->R11;
        log = "[" + predictParam->Key + "] - PredictorParam's R11 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's R11 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取R22参数
    if ((!fileStorage["R22"].isNone()) )
    {
        predictParam->R22= static_cast<double>(fileStorage["R22"]);
        predictParam->R(2,2)=predictParam->R22;
        log = "[" + predictParam->Key + "] - PredictorParam's R22 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's R22 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 读取R33参数
    if ((!fileStorage["R33"].isNone()) )
    {
        predictParam->R33= static_cast<double>(fileStorage["R33"]);
        predictParam->R(3,3)=predictParam->R33;
        log = "[" + predictParam->Key + "] - PredictorParam's R33 was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's R33 was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    if ((!fileStorage["sigma_X"].isNone()))
    {
        predictParam->sigma_X = static_cast<double>(fileStorage["sigma_X"]);
        log = "[" + predictParam->Key + "] - PredictorParam's sigma_X was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's sigma_x was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    if ((!fileStorage["sigma_Y"].isNone()))
    {
        predictParam->sigma_Y = static_cast<double>(fileStorage["sigma_Y"]);
        log = "[" + predictParam->Key + "] - PredictorParam's sigma_y was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's sigma_Y was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    if ((!fileStorage["sigma_R"].isNone()))
    {
        predictParam->sigma_X = static_cast<double>(fileStorage["sigma_R"]);
        log = "[" + predictParam->Key + "] - PredictorParam's sigma_R was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's sigma_R was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    if ((!fileStorage["sigma_W"].isNone()))
    {
        predictParam->sigma_Y = static_cast<double>(fileStorage["sigma_W"]);
        log = "[" + predictParam->Key + "] - PredictorParam's sigma_W was loaded successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "[" + predictParam->Key + "] - PredictorParam's sigma_W was loaded failure";
        logger.Save(ELogType::Error, log);
    }

    // 关闭文件存储器
    fileStorage.release();

    // 记录日志信息
    log = "[" + predictParam->Key + "] - PredictorParam was loaded completely";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回加载结果
    return true;

}