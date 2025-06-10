//
// Created by tony on 2022/4/22.
//

#include "predictor_model.h"

// 构造函数
PredictorModel::PredictorModel():
    delta_t(0)
{
}

// 析构函数
PredictorModel::~PredictorModel()
{
    delta_t = 0;
}

// EKF中调用的求导过的匀速直线运动方程
void PredictorModel::JetUniformStraight(ceres::Jet<double, 9> x0[9], ceres::Jet<double, 9> x1[9]) const
{
    x1[0] = x0[0] + delta_t * x0[1];
    x1[1] = x0[1] ;
    x1[2] = x0[2] + delta_t * x0[3];
    x1[3] = x0[3] ;
    x1[4] = x0[4] + delta_t * x0[5];
    x1[5] = x0[5];
    x1[6] = x0[6] + delta_t * x0[7];
    x1[7] = x0[7];
    x1[8] = x0[8];


}

//EKF中调用的求导过的匀jia速直线运动方程
void PredictorModel::JetAccelerator(ceres::Jet<double, 9> x0[9], ceres::Jet<double, 9> x1[9]) const
{
    x1[0] = x0[0] + delta_t * x0[1] + 0.5 * delta_t * delta_t * x0[2];
    x1[1] = x0[1] + delta_t * x0[2];
    x1[2] = x0[2];
    x1[3] = x0[3] + delta_t * x0[4] + 0.5 + delta_t * delta_t * x0[5];
    x1[4] = x0[4] + delta_t * x0[5];
    x1[5] = x0[5];
    x1[6] = x0[6];
    x1[7] = x0[7];
    x1[8] = x0[8];
}

// 预测方程，通过匀速直线模型进行移动预测
void PredictorModel::UniformStraight(Eigen::Matrix<double, 9, 1> &x0, Eigen::Matrix<double, 9, 1> &x1)const
{
    x1[0] = x0[0] + delta_t * x0[1];
    x1[1] = x0[1];
    x1[2] = x0[2] + delta_t * x0[3];
    x1[3] = x0[3];
    x1[4] = x0[4];


}

// 预测方程，通过匀jia速直线模型进行移动预测
void PredictorModel::Accelerator(Eigen::Matrix<double, 9, 1> &x0, Eigen::Matrix<double, 9, 1> &x1)const
{
    x1[0] = x0[0] + delta_t * x0[1] + 0.5 * delta_t * delta_t * x0[2];
    x1[1] = x0[1] + delta_t * x0[2];
    x1[2] = x0[2];
    x1[3] = x0[3] + delta_t * x0[4] + 0.5 + delta_t * delta_t * x0[5];
    x1[4] = x0[4] + delta_t * x0[5];
    x1[5] = x0[5];
    x1[6] = x0[6];
    x1[7] = x0[7];
    x1[8] = x0[8];
}



// 匀速反陀螺模型
void PredictorModel::AntiTop(Eigen::Matrix<double, 5, 1> x0, Eigen::Matrix<double, 5, 1> x1) const
{
}

// 击打前哨站求导调用
void PredictorModel::JetOutPost(ceres::Jet<double, 5> *x0, ceres::Jet<double, 5> *x1) const
{
    x1[0] = x0[0] + 553.0 * ceres::sin(x0[1] + 0.4 * delta_t) - 553.0 * ceres::sin(x0[1]);
    x1[1] = x0[1] + 0.4 * delta_t;
    x1[2] = x0[2] - 553.0 * ceres::cos(x0[3] + 0.4 * delta_t) + 553.0 * ceres::cos(x0[3]);
    x1[3] = x0[3] + 0.4 * delta_t;
    x1[4] = x0[4];
}

// 击打前哨站调用模型
void PredictorModel::OutPost(Eigen::Matrix<double, 5, 1> x0, Eigen::Matrix<double, 5, 1> x1) const
{
    x1[0] = x0[0] + 553.0 * std::sin(x0[1] + 0.4 * delta_t) - 553.0 * std::sin(x0[1]);
    x1[1] = x0[1] + 0.4 * delta_t;
    x1[2] = x0[2] - 553.0 * std::cos(x0[3] + 0.4 * delta_t) + 553.0 * std::cos(x0[3]);
    x1[3] = x0[3] + 0.4 * delta_t;
    x1[4] = x0[4];
}

// Singer模型实现
void PredictorModel::Singer()
{
    /// 最终移动预测模型。代替原有EKF中的匀速直线模型。
}

// 小符预测方程
void PredictorModel::SmallWindmillModel(Eigen::Matrix<double, 5, 1> x0, Eigen::Matrix<double, 5, 1> x1)
{
    /// 小符预测
}

// 大符预测方程
void PredictorModel::BigWindmillModel(Eigen::Matrix<double, 5, 1> x0, Eigen::Matrix<double, 5, 1> x1)
{
    /// 大符预测
}

// EKF内调用的求导过的观测方程
void Measure::JetMeasure(ceres::Jet<double, 9> WorldCoordinate[9], ceres::Jet<double, 9> Posture[4])
{
    //将xyz转化为pitch,yaw轴角度与距离  (0 2 4 -> 0 1 2)
    Posture[0] = ceres::atan2(WorldCoordinate[0],
                              WorldCoordinate[4]);
    Posture[1] = ceres::atan2(WorldCoordinate[0],
                              WorldCoordinate[2]);
    Posture[2] = ceres::sqrt(WorldCoordinate[0] * WorldCoordinate[0] + WorldCoordinate[2] * WorldCoordinate[2] + WorldCoordinate[4] * WorldCoordinate[4]);
}

// 观测方程
void Measure::measure(Eigen::Matrix<double, 9, 1> &WorldCoordinate, Eigen::Matrix<double, 4, 1> &Posture)
{
    WorldCoordinate[0] = WorldCoordinate[0] - WorldCoordinate[8] * sin(WorldCoordinate[3]/57.3);
    WorldCoordinate[1] = WorldCoordinate[1] - WorldCoordinate[8] * cos(WorldCoordinate[3]/57.3);
    WorldCoordinate[2] = WorldCoordinate[2];
    WorldCoordinate[3] = WorldCoordinate[3];
}


void Measure::measure_linear(Eigen::Matrix<double, 4, 1>  &WorldCoordinate,
                           Eigen::Matrix<double, 4, 1> &Posture)
{

    Posture[0] = WorldCoordinate[0];
    Posture[1] = WorldCoordinate[1];
    Posture[2] = WorldCoordinate[2];
    Posture[3] = atan2(WorldCoordinate[0], WorldCoordinate[1]);
}

void Measure::JetMeasure_linear(ceres::Jet<double, 9> WorldCoordinate[9],
                                ceres::Jet<double, 9> Posture[4])
{
    Posture[0] = WorldCoordinate[0];
    Posture[1] = WorldCoordinate[2];
    Posture[2] = WorldCoordinate[4];
    Posture[3] = WorldCoordinate[6];
}