//
// Created by plutoli on 2021/9/7.
//

#include <vector>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "classical_armor_recognizer.h"
#include "classical_armor_recognizer_param.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 初始化日志信息
    std::string log;

    // 初始化训练集
    std::vector<std::pair<std::string, EClassicalArmorNumber>> trainingSet;
    trainingSet.emplace_back("/home/plutoli/development/cubot_brain/data/sample/armor/2021-09-05/train/Invalid/*.bmp",
                             EClassicalArmorNumber::Invalid);
    trainingSet.emplace_back("/home/plutoli/development/cubot_brain/data/sample/armor/2021-09-05/train/One/*.bmp",
                             EClassicalArmorNumber::One);
    trainingSet.emplace_back("/home/plutoli/development/cubot_brain/data/sample/armor/2021-09-05/train/Two/*.bmp",
                             EClassicalArmorNumber::Two);
    trainingSet.emplace_back("/home/plutoli/development/cubot_brain/data/sample/armor/2021-09-05/train/Three/*.bmp",
                             EClassicalArmorNumber::Three);
    trainingSet.emplace_back("/home/plutoli/development/cubot_brain/data/sample/armor/2021-09-05/train/Four/*.bmp",
                             EClassicalArmorNumber::Four);
    trainingSet.emplace_back("/home/plutoli/development/cubot_brain/data/sample/armor/2021-09-05/train/Five/*.bmp",
                             EClassicalArmorNumber::Five);

    // 初始化终止条件
    cv::TermCriteria termCriteria(cv::TermCriteria::EPS,1000, 1e-6);

    // 初始化模型存储路径
    std::string modelFileName="/home/plutoli/data/armor_hog_svm.xml";

    // 创建装甲板识别器
    ClassicalArmorRecognizer armorRecognizer;

    // 读取装甲板识别器的参数
    std::string armorRecognizerYaml = "config/infantry_1/2022-05-10/classical_armor_recognizer_param.yaml";
    ClassicalArmorRecognizerParam armorRecognizerParam;
    if (!ClassicalArmorRecognizerParam::LoadFromYamlFile(armorRecognizerYaml,
                                                         &armorRecognizerParam))
    {
        return -1;
    }

    // 设置装甲板识别器参数
    if (!armorRecognizer.SetParam(armorRecognizerParam))
    {
        return -1;
    }

    // 初始化装甲板识别器
    if (!armorRecognizer.Init())
    {
        return -1;
    }

    // 保存日志信息
    log = "ArmorHogSvm model are being trained......";
    logger.Save(ELogType::Info, log);

    // 训练模型
    if (armorRecognizer.TrainArmorHogSvm(trainingSet, termCriteria, modelFileName))
    {
        log = "ArmorHogSvm model was trained successful";
        logger.Save(ELogType::Info, log);
    }
    else
    {
        log = "ArmorHogSvm model was trained failure";
        logger.Save(ELogType::Error, log);
    }

    // 释放装甲板识别器系统资源
    armorRecognizer.Release();

    return 0;
}