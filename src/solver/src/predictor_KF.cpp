//
// Created by tony on 2022/4/13.
//

#include"predictor_KF.h"

Kalman::Kalman(Eigen::Matrix<double, 3, 3> A,
               Eigen::Matrix<double, 1, 3> H,
               Eigen::Matrix<double, 3, 3> R,
               Eigen::Matrix<double, 1, 1> Q,
               Eigen::Matrix<double, 3, 1> init,
               double t){

reset(A, H, R, Q, init, t);

}

void Kalman::reset(Eigen::Matrix<double, 3, 3> A,
                   Eigen::Matrix<double, 1, 3> H,
                   Eigen::Matrix<double, 3, 3> R,
                   Eigen::Matrix<double, 1, 1> Q,
                   Eigen::Matrix<double, 3, 1> init,
                   double t) {

    this->A = A;
    this->H = H;
    this->P = Eigen::Matrix<double, 3, 3>::Zero();
    this->R = R;
    this->Q = Q;
    x_k1 = init;
    last_t = t;

}

void Kalman::reset(Eigen::Matrix<double, 3, 1> init,
                   double t)
{

x_k1 = init;
last_t = t;

}

void Kalman::reset(double x,
                   double t){
        x_k1(0,0) = x;
        last_t = t;
    }

Eigen::Matrix<double, 3, 1> Kalman::update(Eigen::Matrix<double, 1, 1> z_k,
                          double t){
    // 设置转移矩阵中的时间项
    for (int i = 1; i < 3; i++) {
        A(i - 1, i) = t - last_t;
    }
    last_t = t;

    // 预测下一时刻的值
    Eigen::Matrix<double, 3, 1> p_x_k = A * x_k1;   // x 的先验估计由上一个时间点的后验估计值和输入信息给出

    // 求协方差
    P = A * P * A.transpose() + R;  // 计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q

    // 计算kalman增益
    K = P * H.transpose() * (H * P * H.transpose() + Q).inverse();  // Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)

    // 修正结果，即计算滤波值
    x_k1 = p_x_k +K * (z_k - H * p_x_k);  // 利用残余的信息改善对 x(t) 的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))

    // 更新后验估计
    P = (Eigen::Matrix<double, 3, 3>::Identity() - K * H) * P;  // 计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]

    return x_k1;
}