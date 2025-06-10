//
// Created by plutoli on 2022/2/7.
//

#include "easy_logger.h"
#include "solver.h"

// 对 y = a * sin(w * t + fai) + 2.09 - a 进行采样和拟合
int main(int argc, char *argv[])
{
    // 初始化函数的系数
    double a = 0.89;
    double w = 1.95;
    double fai = 1.57;

    // 生成样本数据
    std::vector<std::pair<float, float>> datas;
    for (unsigned i = 0; i < 500; ++i)
    {
        auto t = static_cast<float>(0.01 * i);
        auto y = static_cast<float>(a * sin(w * t + fai) + 2.09 - a);
        std::pair<float, float> data;
        data.first = t;
        data.second = y;
        datas.emplace_back(data);
    }

    // 记录起始时间戳
    std::chrono::time_point<std::chrono::steady_clock> begin = std::chrono::steady_clock::now();
    uint64_t beginTimestamp = begin.time_since_epoch().count();

    // 创建初始系数
    std::vector<float> initialCoeffs(3);
    initialCoeffs[0] = 1.0;
    initialCoeffs[1] = 1.6;
    initialCoeffs[2] = 1.2;

    // 对采样数据进行正弦函数拟合
    std::vector<float> fittedCoeffs;
    Solver::FitSine(datas, 50, initialCoeffs, &fittedCoeffs);

    // 记录终止时间戳
    std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::steady_clock::now();
    uint64_t endTimestamp = end.time_since_epoch().count();

    // 输出拟合结果
    std::cout << "process time: " << (endTimestamp - beginTimestamp) / 1000 << "us" << std::endl;
    std::cout << "real a = " << a << std::endl;
    std::cout << "real w = " << w << std::endl;
    std::cout << "real fai = " << fai << std::endl;
    std::cout << "fitted a = " << fittedCoeffs[0] << std::endl;
    std::cout << "fitted w = " << fittedCoeffs[1] << std::endl;
    std::cout << "fitted fai = " << fittedCoeffs[2] << std::endl;

    return 0;
}