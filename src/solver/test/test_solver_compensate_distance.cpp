//
// Created by plutoli on 2021/10/3.
//

#include "solver.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 创建目标解算器
    Solver solver;

    // 读取目标解算器参数
    SolverParam solverParam;
    std::string solverYaml = "config/infantry_3/basement/solver_param.yaml";
    if (!SolverParam::LoadFromYamlFile(solverYaml, &solverParam))
    {
        return -1;
    }

    // 设置目标解算器参数
    if (!solver.SetParam(solverParam))
    {
        return -1;
    }

    // 初始化目标解算器
    if (!solver.Init())
    {
        return -1;
    }

    // 补偿距离
    float distance = 2.0;
    std::pair<float, float> offsetPixel;
    solver.CompensateArmorDistance(EWorkMode::AutomaticShoot,
                                   EBulletVelocity::MPS_15,
                                   distance,
                                   &offsetPixel);

    // 输出补偿结果
    std::cout << "offsetPixel_x = " << offsetPixel.first << std::endl;
    std::cout << "offsetPixel_y = " << offsetPixel.second << std::endl;
}