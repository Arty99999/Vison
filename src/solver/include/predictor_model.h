//
// Created by tony on 2022/4/22.
//

#ifndef CUBOT_BRAIN_PREDICTOR_MODEL_H
#define CUBOT_BRAIN_PREDICTOR_MODEL_H

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <ceres/ceres.h>

/**
 * @brief EKF假设运动模型
 */
class PredictorModel {
public:

    /**
     * @brief 构造函数
     */
    PredictorModel();

    /**
     * @brief 析构函数
     */
    ~PredictorModel();

    /**
     * @brief 预测方程求雅克比
     * @note 匀速运动模型
     * @note 本来是没有必要的，但是为了运算方便和非线性一起做了
     */
    void JetUniformStraight(ceres::Jet<double, 9> x0[9],
                            ceres::Jet<double, 9> x1[9]) const;

    void JetAccelerator(ceres::Jet<double, 9> x0[9],
                            ceres::Jet<double, 9> x1[9]) const;

    /**
     * @brief 预测方程
     * @note 匀速运动模型
     */
    void UniformStraight(Eigen::Matrix<double, 9, 1> &x0,
                         Eigen::Matrix<double, 9, 1> &x1) const;

    void Accelerator(Eigen::Matrix<double, 9, 1> &x0,
                         Eigen::Matrix<double, 9, 1> &x1) const;

    /**
     * @brief 反陀螺预测模型
     * @note 匀速陀螺运动模型
     */
    void JetAntiTop(ceres::Jet<double, 5> x0[5],
                    ceres::Jet<double, 5> x1[5]) const;

    /**
     * @brief 反陀螺预测模型
     * @note 匀速陀螺运动模型
     */
    void AntiTop(Eigen::Matrix<double, 5, 1> x0,
                 Eigen::Matrix<double, 5, 1> x1) const;

    /**
     * @brief 击打前哨站模型求导数时候调用
     */
    void JetOutPost(ceres::Jet<double, 5> x0[5],
                    ceres::Jet<double, 5> x1[5]) const;

    /**
     * @brief 击打前哨站模型
     */
    void OutPost(Eigen::Matrix<double, 5, 1> x0,
                 Eigen::Matrix<double, 5, 1> x1) const;

    /**
     * @brief Singer模型
     * @note 机动追踪算法,是改进过的singer算法
     */
    void Singer();

    /**
     * @brief 小符运动模型
     */
    void SmallWindmillModel(Eigen::Matrix<double, 5, 1> x0,
                            Eigen::Matrix<double, 5, 1> x1);

    /**
     * @brief 大符运动模型
     */
    void BigWindmillModel(Eigen::Matrix<double, 5, 1> x0,
                          Eigen::Matrix<double, 5, 1> x1);

    double delta_t = 0.1;                               ///< 预测时间（等于发弹延迟加子弹飞行时间）

};

/**
 * @brief 输入观测值
 */
class Measure{

public:

    /**
     * @brief 构造函数
     */
    Measure()=default;

    /**
     * @brief 析构函数
     */
    ~Measure()=default;

    /**
     * @brief 观测方程求雅克比
     */
    static void JetMeasure(ceres::Jet<double, 9> WorldCoordinate[9],
                           ceres::Jet<double, 9> Posture[4]);

    /**
     * @brief 观测方程
     * @note 由于观测值是pitch,yaw角度，预测值是xyz坐标，
     * @note 其转化是非线性的，所以要用EKF
     */
    static void measure(Eigen::Matrix<double, 9, 1>  &WorldCoordinate,
                        Eigen::Matrix<double, 4, 1> &Posture);
    /**
    * @brief 观测方程
    * @note 由于观测值是xyz坐标，
    * @note 其转化是线性的
     */
    static void measure_linear(Eigen::Matrix<double, 4, 1>  &WorldCoordinate,
                        Eigen::Matrix<double, 4, 1> &Posture);

    /**
    * @brief 观测方程求雅克比
    */
    static void JetMeasure_linear(ceres::Jet<double, 9> WorldCoordinate[9],
                           ceres::Jet<double, 9> Posture[4]);
};
#endif //CUBOT_BRAIN_PREDICTOR_MODEL_H
