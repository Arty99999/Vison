//
// Created by ly on 24-2-16.
//

#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include <iostream>
#include "classical_armor.h"
#include "classical_armor_recognizer.h"
#include "predictor_model.h"
#include "predictor_param.h"
#include "predictor_tool.h"
#include "predictor_EKF.h"
#include "predictor_AntiTop.h"

int main(int argc, char *argv[])
{
    AutoaimEKF EKF;
    ClassicalArmor armor;

    Eigen::Vector3d nwCoordinate = {0,0,0};
    double euler;

    PredictorModel model;

    float i = 0;
    float j = 0;
    float k = 0;
    float r = 20;
    int o = 0;



    std::string predictorYaml = "config/infantry_3/basement/autoaim_predictor_param.yaml";
    PredictorParam param;
    PredictorParam::LoadFromYamlFile(predictorYaml,&param);
    PredictorModel Model;
    Model.delta_t = 0.1;

    EKF.Q =param.Q;
    EKF.R = param.R;
//    std::cout << EKF.Q << std::endl;
//    std::cout << EKF.R << std::endl;

    Eigen::Matrix<double,9,1> xr;
    xr << 0,0,0,0,0,0,0,0,40;
    EKF.init(xr,Model);

    Eigen::Matrix<double , 9 , 1> xe;
    Eigen::Matrix<double , 9 , 1> xp;
    while(1)
    {
        EKF.StatePredict(nwCoordinate,euler,Model);
//        EKF.StateUpdate(nwCoordinate,euler);

//

        nwCoordinate << 25 *( sin(i) + cos(i)), 27 * (sin(j) + cos(j)) , k;
//        std::cout << nwCoordinate[0]<<std::endl;
        euler += (2.1*i + 2.7*j) /57.3;
        EKF.yaw_sum = euler;
//        std::cout << EKF.H << "\n" << EKF.F << std::endl;



//        std::cout << EKF.Xe[0] << "\n" << EKF.Xe[2] << "\n" << EKF.Xe[4] << "\n" << std::endl;
        std::cout << EKF.StateUpdate(nwCoordinate,euler).transpose()[0] << std::endl;
        i += 1.2;  j += 0.9;

        o ++;
        if (o == 1000)
            break;
    }


}