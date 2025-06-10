//
// Created by cubot on 2022/5/15.
//

#include "easy_logger.h"
#include "predictor_model.h"

int main(){

    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 初始化参数
    PredictorModel model;
    model.delta_t = 1.0;
//    Eigen::Matrix<double, 5, 1> x0;
//    Eigen::Matrix<double, 5, 1> x1;
//    x1 = Eigen::Matrix<double, 5, 1>::Zero();
//    x0<<10,11,1,1,0;
//
//    // 检测匀速直线模型
//    model.UniformStraight(x0,x1);
//    for(int j=0;j<5;j++){
//        std::cout<<x1[j];
//    }

    ceres::Jet<double, 5> x0[5];
    ceres::Jet<double, 5> x1[5];
    x0[0].a = 10;
    x0[1].a = 1;
    x0[2].a = 19;
    x0[3].a = 2;
    x0[4].a = 5 ;


    model.JetUniformStraight(x0,x1);

//    for(int i =0;i<5;i++){
//        std::cout<<x1[i]<<std::endl;
//    }

    ceres::Jet<double, 5> y1[3];
    Measure::JetMeasure(x0,y1);

    for(int i=0;i<3;i++){
        std::cout<<y1[i]<<std::endl;
    }



}
