//
// Created by plutoli on 2022/3/16.
//

#include "solver.h"

int main(int argc, char *argv[])
{
    // 初始化x轴的多项式拟合样本集合
    // 集合中的每一个元素由PNP算法解算出来的装甲板到枪口的距离和x轴补偿值组成，需要根据实际测试数据进行填充
    std::vector<std::pair<float, float>> x_samples;
    x_samples.emplace_back(0.0, 0.0);

    // 初始化y轴的多项式拟合样本集合
    // 集合中的每一个元素由PNP算法解算出来的装甲板到枪口的距离和x轴补偿值组成，需要根据实际测试数据进行填充
    std::vector<std::pair<float, float>> y_samples;
    y_samples.emplace_back(0.0, 0.0);

    // 创建初始系数；因为采用4阶多项式进行拟合，所以有5个系数
    std::vector<float> initialCoeffs(5, 0.0);

    // 拟合x轴的补偿多项式
    std::vector<float> x_fittedCoeffs;
    Solver::FitPolynomial(x_samples,
                          EPolynomialOrder::Four,
                          10,
                          initialCoeffs,
                          &x_fittedCoeffs);

    // 拟合y轴的补偿多项式
    std::vector<float> y_fittedCoeffs;
    Solver::FitPolynomial(y_samples,
                          EPolynomialOrder::Four,
                          10,
                          initialCoeffs,
                          &y_fittedCoeffs);

    // 输出x轴补偿多项式的拟合结果
    std::cout << "x_fittedCoeffs: " << std::endl;
    for (unsigned int i = 0; i < x_fittedCoeffs.size(); ++i)
    {
        std::cout << "x_fittedCoeffs[" << std::to_string(i) << "] = " << std::to_string(x_fittedCoeffs[i]) << std::endl;
    }

    // 输出y轴补偿多项式的拟合结果
    std::cout << std::endl << "y_fittedCoeffs: " << std::endl;
    for (unsigned int i = 0; i < y_fittedCoeffs.size(); ++i)
    {
        std::cout << "y_fittedCoeffs[" << std::to_string(i) << "] = " << std::to_string(y_fittedCoeffs[i]) << std::endl;
    }
}