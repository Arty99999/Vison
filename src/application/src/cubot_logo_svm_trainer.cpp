//
// Created by plutoli on 2022/2/6.
//

#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "classical_windmill_recognizer.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 初始化日志信息
    std::string log;

    // 初始化终止条件
    cv::TermCriteria termCriteria(cv::TermCriteria::EPS,1000, 1e-6);

    // 初始化模型存储路径
    std::string modelFileName="/home/plutoli/data/logo_hog_svm.xml";

    // 创建风车识别器
    ClassicalWindmillRecognizer windmillRecognizer;

    // 读取风车识别器的参数
    ClassicalWindmillRecognizerParam recognizerParam;
    std::string windmillRecognizerYaml = "config/infantry_3/basement/classical_windmill_recognizer_param.yaml";
    if (!ClassicalWindmillRecognizerParam::LoadFromYamlFile(windmillRecognizerYaml,
                                                            &recognizerParam))
    {
        return -1;
    }

    // 设置风车识别器参数
    if (!windmillRecognizer.SetParam(recognizerParam))
    {
        return -1;
    }

    // 初始化风车识别器
    if (!windmillRecognizer.Init())
    {
        return -1;
    }

    // 保存日志信息
    log = "LogoHogSvm model are being trained......";
    logger.Save(ELogType::Info, log);

    // 训练模型
    std::string positivePath = "/home/plutoli/development/cubot_brain/data/sample/logo/2022-05-23/positive/*.bmp";
    std::string negativePath = "/home/plutoli/development/cubot_brain/data/sample/logo/2022-05-23/negative/*.bmp";
    if (windmillRecognizer.TrainLogoHogSvm(positivePath, negativePath, termCriteria, modelFileName))
    {
        log = "LogoHogSvm model has been trained successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "LogoHogSvm model has been trained failure";
        logger.Save(ELogType::Error, log);
    }

    // 释放风车识别器系统资源
    windmillRecognizer.Release();

    return 0;
}