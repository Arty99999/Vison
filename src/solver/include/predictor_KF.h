//
// Created by tony on 2022/4/13.
//

#ifndef CUBOT_BRAIN_PREDICTOR_KF_H
#define CUBOT_BRAIN_PREDICTOR_KF_H

#include <Eigen/Dense>

// 卡尔曼滤波实现，未使用
class Kalman
        {
public:

    Kalman() = default;

    Kalman(Eigen::Matrix<double, 3, 3> A,
           Eigen::Matrix<double, 1, 3> H,
           Eigen::Matrix<double, 3, 3> R,
           Eigen::Matrix<double, 1, 1> Q,
           Eigen::Matrix<double, 3, 1> init,
           double t);

    void reset(Eigen::Matrix<double, 3, 3> A,
                       Eigen::Matrix<double, 1, 3> H,
                       Eigen::Matrix<double, 3, 3> R,
                       Eigen::Matrix<double, 1, 1> Q,
                       Eigen::Matrix<double, 3, 1> init,
                       double t);


    void reset(Eigen::Matrix<double, 3, 1> init,
               double t);

    void reset(double x,
               double t);

    Eigen::Matrix<double, 3, 1> update(Eigen::Matrix<double, 1, 1> z_k,
                                       double t);

private:
    Eigen::Matrix<double, 3, 1> x_k1;   // k-1时刻的滤波值，即是k-1时刻的值
    Eigen::Matrix<double, 3, 1> K;      // Kalman增益
    Eigen::Matrix<double, 3, 3> A;      // 转移矩阵
    Eigen::Matrix<double, 1, 3> H;      // 观测矩阵
    Eigen::Matrix<double, 3, 3> R;      // 预测过程噪声偏差的方差
    Eigen::Matrix<double, 1, 1> Q;      // 测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
    Eigen::Matrix<double, 3, 3> P;      // 估计误差协方差
    double last_t{0};
};

#endif //CUBOT_BRAIN_PREDICTOR_KF_H
