//
// Created by plutoli on 2021/8/12.
//

#include "easy_logger.h"
#include "classical_armor_recognizer_param.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 加载装甲板识别器参数
    ClassicalArmorRecognizerParam param;
    std::string yamlFileName = "config/infantry_3/basement/classical_armor_recognizer_param.yaml";
    bool result = ClassicalArmorRecognizerParam::LoadFromYamlFile(yamlFileName, &param);
}