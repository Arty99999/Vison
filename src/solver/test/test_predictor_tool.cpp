//
// Created by tony on 22-6-1.
//

#include "predictor_tool.h"
#include "solver.h"

int main()
{
//    std::array<float, 4> quaterniond;
//    quaterniond[0] = 0.5;
//    quaterniond[1] = -0.5;
//    quaterniond[2] = 0.5;
//    quaterniond[3] = -0.5;
//    Eigen::Quaterniond Q_raw(quaterniond[0],quaterniond[1],quaterniond[2],quaterniond[3]);
//    Eigen::Quaterniond Q(Q_raw.matrix().transpose());
//    std::cout<<Q_raw.w()<<" "<<Q_raw.x()<<"  "<<Q_raw.y()<<"  "<<Q_raw.z()<<std::endl;
//    std::cout<<Q.w()<<" "<<Q.x()<<"  "<<Q.y()<<"  "<<Q.z()<<std::endl;
//    Eigen::Matrix3d RotationMatrix = Q.matrix().cast<double>();
//    std::cout<<Q_raw.matrix()<<std::endl;
//    std::cout<<"************************************************"<<std::endl;
//    std::cout<<Q_raw.matrix().transpose()<<std::endl;
//    std::cout<<"************************************************"<<std::endl;
//    std::cout<<RotationMatrix<<std::endl;
//      Eigen::Matrix<double, 3, 3> one;
//      one(0,0) = 1;
//      one(0,1) = 0;
//      one(0, 2) = 0;
//      one(1,0) = 0;
//      one(1, 1) = 0;
//      one(1, 2) = 1;
//      one(2,0) = 0;
//      one()
//      one<<1,0,0,0,0,1,0,-1,0;
//    Eigen::Quaterniond Q_1(one);
//    std::cout<<"Q_1 :"<<Q_1.w()<<" "<<Q_1.x()<<" "<<Q_1.y()<<" "<<Q_1.z()<<std::endl;
//    Eigen::Matrix3d two;
//    two = one.inverse();
//    std::cout<<"two: "<<std::endl<<one<<std::endl;

//    Eigen::Vector3d eulerAngle(0,90,0);
//
//    Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitX()));
//    Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
//    Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitZ()));
//
//    Eigen::Matrix3d rotation_matrix;
//    rotation_matrix=yawAngle*pitchAngle*rollAngle;
//    std::cout<<std::endl<<rotation_matrix;

//    Eigen::Vector3d nowCameraCoordinate;
//    Eigen::Matrix3d nowCamera2World;
//    Eigen::Isometry3d imuCamera;
//    Eigen::Vector3d nowWorldCoordinate;
//    std::array<float, 4> quaterniond21;
//    quaterniond21[0] = 1.0;
//    quaterniond21[1] = 0;
//    quaterniond21[2] = 0;
//    quaterniond21[3] = 0;
//    imuCamera.matrix() << -0.448074 ,      0,    0.893997,   0,
//            0,      1 ,          0,    0,
//            -0.893997,       0,    -0.448074,   0,
//            0,      0,            0,   1;
//    imuCamera.matrix()<<-1, 0,  0,  0,
//                        0,  0, -1, 0,
//                        0  ,-1  ,0,  0,
//                        0,  0,  0,  1;
////    PredictorTool::GetRotationMatrix( quaterniond21, imuCamera,nowCamera2World);
//    Eigen::Vector3d eulerAngle = imuCamera.rotation().eulerAngles(0,1,2);
//    std::cout<<"Euler: "<<eulerAngle *180/3.1415<<std::endl;
//
//    cv::Point2f target;
//    target.x = 100;
//    target.y = 100;
//    float distance = 1000;

    float W;
    int16_t w;
    unsigned char quaternionW[2];
    quaternionW[0] = 0x01;
    quaternionW[1] = 0x08;
    ::memcpy(&w, quaternionW, 2);
    w = *(int16_t *)(quaternionW);

    std::cout<<"w: "<<w<<std::endl;

//    uint32_t timeStamp[4];
//    timeStamp[0] = 0x01;
//    timeStamp[1] = 0x00;
//    timeStamp[2] = 0x00;
//    timeStamp[3] = 0x00;
//    uint32_t predictTimeStamp = timeStamp[0]+(timeStamp[1]<<8)+(timeStamp[2]<<16)+(timeStamp[3]<<24);
//
//    std::cout<<"predictCOmmond: "<<predictTimeStamp<<std::endl;
}