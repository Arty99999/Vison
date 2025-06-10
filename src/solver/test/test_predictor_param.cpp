//
// Created by cubot on 2022/5/15.
//

#include "easy_logger.h"
#include "predictor_param.h"

int main(){

    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 加载机器人大脑参数
    std::string yamlFileName = "config/infantry_3/basement/autoaim_predictor_param.yaml";
    PredictorParam param;
    bool result = PredictorParam::LoadFromYamlFile(yamlFileName, &param);

    return 0;
}
