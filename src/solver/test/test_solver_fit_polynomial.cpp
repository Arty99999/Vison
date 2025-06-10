//
// Created by plutoli on 2022/1/18.
//

#include "easy_logger.h"
#include "solver.h"

// 对 y = 1.1 * t^4 + 2.2 * t^3 + 3.3 * t^2 + 4.4 * t 进行采样和拟合
int main(int argc, char *argv[])
{
    // 初始化函数的系数
    double a0 = 1.1;
    double a1 = 2.2;
    double a2 = 3.3;
    double a3 = 4.4;

    // 生成样本数据
    std::vector<std::pair<float, float>> datas;
    for (unsigned i = 0; i < 100; ++i)
    {
        auto t = static_cast<float>(0.1 * i);
        auto y = static_cast<float>(a0 * std::pow(t, 4) + a1 * std::pow(t, 3) + a2 * t * t + a3 * t);
        std::pair<float, float> data;
        data.first = t;
        data.second = y;
        datas.emplace_back(data);
    }

    // 记录起始时间戳
    std::chrono::time_point<std::chrono::steady_clock> begin = std::chrono::steady_clock::now();
    uint64_t beginTimestamp = begin.time_since_epoch().count();

    // 创建初始系数
    std::vector<float> initialCoeffs(4, 0.0);

    // 对采样数据进行多项式拟合
    std::vector<float> fittedCoeffs;
    Solver::FitPolynomial(datas,
                          EPolynomialOrder::Four,
                          10,
                          initialCoeffs,
                          &fittedCoeffs);

    // 记录终止时间戳
    std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::steady_clock::now();
    uint64_t endTimestamp = end.time_since_epoch().count();

    // 输出拟合结果
    std::cout << "process time: " << (endTimestamp - beginTimestamp) / 1000 << "us" << std::endl;
    std::cout << "a0 = " << fittedCoeffs[0] << std::endl;
    std::cout << "a1 = " << fittedCoeffs[1] << std::endl;
    std::cout << "a2 = " << fittedCoeffs[2] << std::endl;
    std::cout << "a3 = " << fittedCoeffs[3] << std::endl;

    return 0;
}