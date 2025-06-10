//
// Created by plutoli on 2022/1/23.
//

#include "easy_logger.h"
#include "classical_windmill_recognizer_param.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 加载装甲板识别器参数
    std::string yamlFileName = "config/infantry_3/basement/classical_windmill_recognizer_param.yaml";
    ClassicalWindmillRecognizerParam param;
    bool result = ClassicalWindmillRecognizerParam::LoadFromYamlFile(yamlFileName, &param);
}