//
// Created by plutoli on 2022/1/17.
//

#include "solver_param.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 读取目标解算器参数
    SolverParam solverParam;
    SolverParam::LoadFromYamlFile("config/infantry_3/basement/solver_param.yaml",
                                  &solverParam);

    return 0;
}