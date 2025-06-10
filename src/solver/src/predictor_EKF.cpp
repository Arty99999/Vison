//
// Created by tony on 2022/4/21.
//

#include "predictor_EKF.h"

// 构造函数
AutoaimEKF::AutoaimEKF():
          Xe(Eigen::Matrix<double, 9, 1>::Zero()),
          P(Eigen::Matrix<double, 9, 9>::Identity()),
          Q(Eigen::Matrix<double, 9, 9>::Identity()),
          R(Eigen::Matrix<double, 4, 4>::Identity()),
          F(Eigen::Matrix<double, 9, 9>::Identity()),
          H(Eigen::Matrix<double, 4, 9>::Identity()),
          yaw_sum(0),yaw_last(0),yaw_change(std::pair<bool,int>(false,0)),
          angle_position(false),
          Q_coe(Eigen::Matrix<double,4,1> (32,5,1,0.001)){}

// 初始化函数
void AutoaimEKF::init(Eigen::Matrix<double, 9, 1> &X0,PredictorModel &PredictFunc)
{
    // 初始化Xe与P阵
    Xe = X0;
    Xp = X0;
    P = Eigen::Matrix<double, 9, 9>::Identity();
    Dk = Eigen::Matrix<double, 4, 4>::Identity();
    ek = Eigen::Matrix<double, 4, 1>::Identity();
    F  = Eigen::Matrix<double, 9, 9>::Identity();
    rk = 0;
}

// 释放滤波器函数
void AutoaimEKF::Release() {

    // 对滤波器内部参数进行释放
    Xe = Eigen::Matrix<double, 9, 1>::Zero();
    P = Eigen::Matrix<double, 9, 9>::Identity();
    Q = Eigen::Matrix<double, 9, 9>::Identity();
    R = Eigen::Matrix<double, 4, 4>::Identity();
}

// 判断当前滤波器是否为空,为空则初始化滤波器
bool AutoaimEKF::IsEmpty(AutoaimEKF &ekf)
{
    bool result = false;

    if(ekf.Xp == Eigen::Matrix<double, 9, 1>::Zero())
    {
        result = true;
    }

    return result;
}

// 预测部分。AutoDiff使用DualNumber计算雅克比。
// http://t.zoukankan.com/JingeTU-p-11706351.html /n
Eigen::Matrix<double, 9, 1> AutoaimEKF::LinearPredict(PredictorModel &PredictFunc)
{
    // 定义此时刻与预测时刻的求雅克比值
    ceres::Jet<double, 9> Xe_auto_jet[9];
    ceres::Jet<double, 9> Xp_auto_jet[9];

    // 进行jet求导的初始化
    for (int i = 0; i < 9; i++)
    {
        // a为大量，v为小量
        Xe_auto_jet[i].a = Xe[i];
        Xe_auto_jet[i].v[i] = 1;
    }
    // 匀速直线模型预测，并求预测值的雅克比
    PredictFunc.JetUniformStraight(Xe_auto_jet, Xp_auto_jet);
    for (int i = 0; i < 9; i++)
    {
        // 将xp的大值赋值给xp
        Xp[i] = Xp_auto_jet[i].a;
        // 将xp的小值的转置赋值给F的子矩阵
        F.block(i, 0, 1, 9) = Xp_auto_jet[i].v.transpose();
    }

    // 得到状态协方差
    P = F * P * F.transpose() + Q;

    return Xp;
}

// 更新部分
Eigen::Matrix<double, 9, 1> AutoaimEKF::LinearUpdate(Measure &Measure, const  Eigen::Matrix<double, 4, 1> &Y)
{
    // 定义预测值X与观测值Y
    ceres::Jet<double, 9> Xp_auto_jet[9];
    ceres::Jet<double, 9> Yp_auto_jet[4];

    // 对预测值进行初始化
    for (int i = 0; i < 9; i++)
    {
        Xp_auto_jet[i].a = Xp[i];
        Xp_auto_jet[i].v[i] = 1;
    }

    // 将预测值转化为观测值的形式，并求雅克比
    Measure::JetMeasure_linear(Xp_auto_jet, Yp_auto_jet);

    for (int i = 0; i < 4; i++)
    {
        Yp[i] = Yp_auto_jet[i].a;
        H.block(i, 0, 1, 9) = Yp_auto_jet[i].v.transpose();
    }

    // 得到kalman gain
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    // 得到目标位置的最优估计
    Xe = Xp + K * (Y - Yp);
    ek = Y - Yp;
    Dk = H * P * H.transpose() + R;
    rk = ek.transpose() * Dk.inverse() * ek;
    // 更新状态协方差
    P = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P;

    return Xe;
}

Eigen::Matrix<double, 9, 1> AutoaimEKF::StatePredict(Eigen::Matrix<double, 3, 1> nowWorldCoordinate,
                                                       double euler,
                                                        PredictorModel &PredictFunc) {
//    F <<    1,   0,   0,   0,   PredictFunc.delta_t, 0,   0,   0,   0,
//            0,   1,   0,   0,   0,   PredictFunc.delta_t, 0,   0,   0,
//            0,   0,   1,   0,   0,   0,   PredictFunc.delta_t, 0,   0,
//            0,   0,   0,   1,   0,   0,   0,   PredictFunc.delta_t, 0,
//            0,   0,   0,   0,   1,   0,   0,   0,   0,
//            0,   0,   0,   0,   0,   1,   0,   0,   0,
//            0,   0,   0,   0,   0,   0,   1,   0,   0,
//            0,   0,   0,   0,   0,   0,   0,   1,   0,
//            0,   0,   0,   0,   0,   0,   0,   0,   1;

    ceres::Jet<double, 9> Xe_auto_jet[9];
    ceres::Jet<double, 9> Xp_auto_jet[9];
    // 进行jet求导的初始化
    for (int i = 0; i < 9; i++)
    {
        // a为大量，v为小量
        Xe_auto_jet[i].a = Xe[i];
        Xe_auto_jet[i].v[i] = 1;
    }
    double last_yaw = Xe[6];
    PredictFunc.JetUniformStraight(Xe_auto_jet, Xp_auto_jet);
    for(int i = 0; i < 9; i++)
    {
        Xp[i] = Xp_auto_jet[i].a;
        F.block(i, 0, 1, 9) = Xp_auto_jet[i].v.transpose();
    }
    if (abs(Xp[6] * 57.3) > 150)
    {
//        std::cout <<Xp[7] * 57.3 <<std::endl;
//        std::cout <<"last: "<< last_yaw * 57.3<<std::endl;
    }
//    std::cout << " w: " << Xp[7] << " sita: " <<Xp[6] <<std::endl;
    P = F * P * F.transpose() + Q;
    //std::cout<<Xp.transpose()<<std::endl;
    return Xp;
}

Eigen::Matrix<double, 9, 1> AutoaimEKF::StateUpdate(Eigen::Matrix<double, 3, 1> nowWorldCoordinate,double euler) {

    Eigen::Matrix<double, 4 ,1> zp;
    Eigen::Matrix<double, 4, 1> Y;
    //std::cout<<Xe.transpose()<<std::endl;
    ceres::Jet<double, 9> Xp_auto_jet[9];
    ceres::Jet<double, 9> Yp_auto_jet[4];

    // 对预测值进行初始化
    for (int i = 0; i < 9; i++)
    {
        Xp_auto_jet[i].a = Xp[i];
        Xp_auto_jet[i].v[i] = 1;
    }
    Yp_auto_jet[0] = Xp_auto_jet[0] + Xp_auto_jet[8] * ceres::sin(euler);
    Yp_auto_jet[1] = Xp_auto_jet[2] - Xp_auto_jet[8] * ceres::cos(euler);
    Yp_auto_jet[2] = Xp_auto_jet[4] ;
    Yp_auto_jet[3] = Xp_auto_jet[6] ;

    for (int i = 0; i < 4; i++)
    {
        Yp[i] = Yp_auto_jet[i].a;
        H.block(i, 0, 1, 9) = Yp_auto_jet[i].v.transpose();
    }
//    std::cout << H <<std::endl;
    //H <<  1,   0,   0,   Xp[8] * cos(Xp[3] / 57.3),   0,   0,   0,   0,   -sin(Xp[3] / 57.3),
    //        0,   1,   0,   -Xp[8] * sin(Xp[3] / 57.3),  0,   0,   0,   0,   -cos(Xp[3] / 57.3),
    //        0,   0,   1,   0,            0,   0,   0,   0,   0,
    //        0,   0,   0,   1,            0,   0,   0,   0,   0;
//    H << 1,     0,     0,     0,     0,     0,     Xp[8] * cos(Xe[6]),     0,     sin(Xe[6]),
//         0,     0,     1,     0,     0,     0,     Xp[8] * sin(Xe[6]),     0,     -cos(Xe[6]),
//         0,     0,     0,     0,     1,     0,                         0,    0,                  0,
//         0,     0,     0,     0,     0,     0,                         1,    0,                  0;
    Y << nowWorldCoordinate[0],nowWorldCoordinate[1],nowWorldCoordinate[2],euler;
//    std::cout << yaw_sum <<std::endl;
    // 得到kalman gain
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    //std::cout<<K<<std::endl;
    // 得到目标位置的最优估计
    Xe = Xp + K * (Y - Yp);
//    std::cout <<Xp[6] * 57.3 << "    " <<(K*Y)[6] * 57.3 <<"     " <<(K*Yp)[6] * 57.3 <<std::endl;
//    std::cout << Xe[6] *57.3 << "   " <<Xe[7] * 57.3 <<std::endl;
//    std::cout <<(K*Y)[6]*57.3 <<"   "<<(K*Y)[7] * 57.3 <<"    YP:" <<(K*Yp)[6] * 57.3 <<" " <<(K*Yp)[7] * 57.3 <<std::endl;
    // 更新状态协方差
    P = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P;

    return Xe;
}

double AutoaimEKF::handleYawJump(double distance, double &yaw) {
    double dyaw = yaw - yaw_last;
    //std::cout<<dyaw * 57.3<<std::endl;
    if (dyaw * 57.3 > 30) dyaw = yaw - CV_PI/2 - yaw_last;
    if (dyaw * 57.3 < -30) dyaw = yaw +CV_PI/2 -yaw_last;
//    if(dyaw > 180) dyaw = 180 - yaw + yaw_last + 180;
//    if(dyaw < -180) dyaw = yaw_last - 180 + yaw - 180;
    //std::cout<<"dyaw1: "<<dyaw * 57.3<<std::endl;
    if(abs(dyaw * 57.3) < 2)
    {
        yaw_d = dyaw;
        yaw_sum = yaw_sum + yaw_d;
    }
    else
    {
        yaw_sum = yaw_sum + yaw_d;
    }
    yaw_last = yaw;
    return yaw_sum;
}
std::pair<double, double> AutoaimEKF::GetYawNow(double sum) {
    double dyaw = sum - yaw_sum;
    return {yaw_last + dyaw, dyaw};
}

std::pair<bool,double> AutoaimEKF::isTuoLuo(double yaw,double time) {
    double d = abs(yaw - yaw_last);
    //std::cout<<yaw<<"       "<<yaw_last<<std::endl;
    if(d_time.size() > 50)
    {
        d_time.erase(d_time.begin() , d_time.begin() + 10);
        d_time.push_back(time);
    } else{
        d_time.push_back(time);
    }
    //std::cout<<"yaw: "<<yaw<<"   "<<yaw_last<<std::endl;
    if(d_yaw.size() > 50)
    {
        d_yaw.erase(d_yaw.begin() , d_yaw.begin() + 10);
        d_yaw.push_back(d);
    } else{
        d_yaw.push_back(d);
    }
    double  sumValue = std::accumulate(std::begin(d_yaw), std::end(d_yaw), 0.0);
    //std::cout<<sumValue * 57.3<<std::endl;
    double  sumTime = std::accumulate(std::begin(d_time), std::end(d_time), 0.0);
    double mean = sumValue / sumTime;
    //std::cout<<"mean: "<<mean * 57.5<<std::endl;
    yaw_last  = yaw;
    if((mean * 57.3) > 2 && (mean * 57.3) < 100)
    {
        return {true,mean};
    }
    else{
        return {false,mean};
    }
}

Eigen::Matrix<double, 4, 4> AutoaimEKF::SetVMatrix(Eigen::Matrix<double, 3, 1> Location) {

    Eigen::Matrix<double, 4, 4> V;
    double phi = atan(Location.x() / Location.y());
    double psi = atan(Location.z() / sqrt(pow(Location.x(),2) + pow(Location.y(),2)));
    double rou = sqrt(pow(Location.x() , 2) + pow(Location.y() , 2) + pow(Location.z() , 2));

    V << -rou * cos(psi) * sin(phi) , -rou * sin(psi) * cos(phi) , cos(psi) * sin(phi) , 0,
         rou * cos(psi) * sin(phi)  , -rou * sin(psi) * sin(phi) , cos(psi) * sin(phi) , 0,
         0                               ,  rou * cos(psi)               , sin(psi)              , 0,
         0                               , 0                               , 0                        , 1;
    return V;

}

Eigen::Matrix<double, 9, 9> AutoaimEKF::SetQMatrix(double t) {
    Eigen::Matrix<double ,9 ,9> Q;
    double t4 = pow(t,3) / 3;
    double t3 = pow(t,2) / 2;
    double t2 = pow(t,1) / 1;
    Q << t4*Q_coe[0] ,t3*Q_coe[0] ,0           ,0           ,0        ,0       ,0           ,0           ,0,
         t3*Q_coe[0] ,t2*Q_coe[0] ,0           ,0           ,0        ,0       ,0           ,0           ,0,
         0           ,0           ,t4*Q_coe[0] ,t3*Q_coe[0] ,0        ,0       ,0           ,0           ,0,
         0           ,0           ,t3*Q_coe[0] ,t2*Q_coe[0] ,0        ,0       ,0           ,0           ,0,
         0           ,0           ,0           ,0           ,Q_coe[1] ,0       ,0           ,0           ,0,
         0           ,0           ,0           ,0           ,0        ,Q_coe[1],0           ,0           ,0,
         0           ,0           ,0           ,0           ,0        ,0       ,t4*Q_coe[2] ,t3*Q_coe[2] ,0,
         0           ,0           ,0           ,0           ,0        ,0       ,t3*Q_coe[2] ,t2*Q_coe[2] ,0,
         0           ,0           ,0           ,0           ,0        ,0       ,0           ,0           ,Q_coe[3];

    return Q;

}

double AutoaimEKF::HandleJump(double yaw) {

    int position_coe = 0;
    auto judge_ori = [&](double delta_yaw) ->bool {
        bool ori = delta_yaw > 0;
        yaw_change.second += ori ? 1: -1;
        //同或的定义
        position_coe = !((!angle_position && ori) || (angle_position && !ori))? 1 : -1 ;
//        std::cout << ori << std::endl;
        return ori;
    };

    double delta_yaw = yaw - yaw_last;
    if (yaw_last == 0)
    {
        angle_position = delta_yaw > 0;
    }
    if (delta_yaw > 35/57.3 && yaw_last != 0)
    {
        delta_yaw = yaw - yaw_last - CV_PI / 2;
        jump_flag = true;

    }
    else if (delta_yaw < -35/57.3 && yaw_last != 0)
    {
        delta_yaw = yaw - yaw_last + CV_PI / 2;
        jump_flag = true;
    }
    else
        jump_flag = false;

    yaw_change.first = judge_ori(delta_yaw);

    yaw_sum += position_coe * delta_yaw;
//    std::cout << jump_flag <<std::endl;
    if (yaw_change.first && yaw_sum > (180 / 360) * 2 * CV_PI)
    {
        angle_position = false;
        position_coe = -1;
        yaw_sum = 2*CV_PI - yaw_sum;
    }
    else if (yaw_change.first && yaw_sum < 0)
    {
        angle_position = true;
        position_coe = 1;
        yaw_sum = -yaw_sum;
    }

    if (!yaw_change.first && yaw_sum < -((180/360) * 2*CV_PI))
    {
        angle_position = true;
        position_coe = -1;
        yaw_sum = -2*CV_PI + yaw_sum;
    }
    else if (!yaw_change.first && yaw_sum > 0)
    {
        angle_position = false;
        position_coe = 1;
        yaw_sum = -yaw_sum;
    }
    yaw_last = yaw;
    return yaw_sum;

}

double AutoaimEKF::HandleYawSum(double yaw) {
    double nowYaw;

    if (angle_position && yaw_change.first )
    {
        if ( yaw>0 )
            nowYaw = yaw_sum > 90 / 57.3 ? yaw_sum - CV_PI/2 : yaw_sum;
        else
            nowYaw = yaw_sum > 90 / 57.3 ? yaw_sum - CV_PI: yaw_sum - CV_PI/ 2;
    }
    if (!angle_position && yaw_change.first )
    {
        if (yaw > 0)
            nowYaw = yaw_sum > 90/57.3? CV_PI - yaw_sum:  CV_PI/2 - yaw_sum;
        else
            nowYaw = yaw_sum > 90/57.3? CV_PI / 2 - yaw_sum: -yaw_sum;
    }

    if (angle_position && !yaw_change.first)
    {
        if (yaw > 0)
            nowYaw = yaw_sum < -90/57.3? -CV_PI / 2 - yaw_sum: -yaw_sum;
        else
            nowYaw = yaw_sum < -90/57.3? -CV_PI - yaw_sum : -CV_PI / 2-yaw_sum;
    }

    if (!angle_position && !yaw_change.first)
    {
        if (yaw> 0)
            nowYaw = yaw_sum < -90/57.3? yaw_sum + CV_PI : yaw_sum + CV_PI/2;
        else
            nowYaw = yaw_sum < -90/57.3? yaw_sum + CV_PI/2 : yaw_sum;
    }


    return  nowYaw;

}


//暂未使用
cv::Point2f Remap::imu2camera(double distance,
                              Eigen::Matrix<double, 3, 1> &imucoordinate,
                              std::pair<Eigen::Matrix<double, 3, 3>,Eigen::Matrix<double, 3, 1>> RandTvec)
{
    //
    Eigen::Matrix<double,3,1> cameraCoordinate;
    cameraCoordinate = RandTvec.first * imucoordinate + RandTvec.second;
    cv::Point2f armorCenterPixelCoordinate;

    HuarayCamera camera;
    HuarayCameraModelParam modelParam = camera.GetParam().ModelParam;

    PredictorTool::GetPixelLocation(distance,&armorCenterPixelCoordinate,modelParam,cameraCoordinate);
    return armorCenterPixelCoordinate;
}

//比较重投影装甲板和捕捉装甲板
//coordinate应该为转换后的中心点像素坐标
//在使用手持装甲板测试时，一定要将装甲板倾斜一定角度（15度左右），不然yaw值效果会比较差
std::pair<double,std::vector<cv::Point2f>> Remap::Compare(const float yaw,
                                                          HuarayCameraModelParam &modelParam,
                                                          cv::Point3f Coordinate,
                                                          ClassicalArmor armor,
                                                          bool stand){

    //
    std::vector<Eigen::Matrix<double, 3, 1>> remapPoints;

    double abs_cos_yaw_2 = abs(cos(yaw)) * 0.5;
    double sin_yaw_2 = sin(yaw) * 0.5;
    //大板
    float laWidth = 240.0 - 10.0;
    float laHeight = 55.0;

    //小板
    float loWidth = 134.0 - 10;
    float loHeight = 52.0;

    double sin_15 = sin( 15.0 / 57.3);
    if (stand)
        sin_15 = sin( - 15.0 / 57.3);

    double cos_15_2 = cos(15.0 / 57.3) * 0.5;



    //大装甲版
    if (armor.Number == EClassicalArmorNumber::BlueBaseBig || armor.Number == EClassicalArmorNumber::RedBaseBig ||
        armor.Number == EClassicalArmorNumber::RedOne || armor.Number == EClassicalArmorNumber::BlueOne){

        remapPoints.emplace_back(Eigen::Matrix<double, 3, 1>(
                Coordinate.x - laWidth * abs_cos_yaw_2 - laHeight * sin_15 * sin_yaw_2 ,
                Coordinate.y - laWidth * sin_yaw_2 + laHeight * sin_15 * abs_cos_yaw_2 ,
                Coordinate.z - laHeight * cos_15_2 ));
        remapPoints.emplace_back(Eigen::Matrix<double, 3, 1>(
                Coordinate.x + laWidth * abs_cos_yaw_2 - laHeight * sin_15 * sin_yaw_2,
                Coordinate.y + laWidth * sin_yaw_2 + laHeight * sin_15 * abs_cos_yaw_2 ,
                Coordinate.z - laHeight * cos_15_2));
        remapPoints.emplace_back(Eigen::Matrix<double, 3, 1>(
                Coordinate.x + laWidth * abs_cos_yaw_2 + laHeight * sin_15 * sin_yaw_2,
                Coordinate.y + laWidth * sin_yaw_2 - laHeight * sin_15 * abs_cos_yaw_2,
                Coordinate.z + laHeight * cos_15_2));
        remapPoints.emplace_back(Eigen::Matrix<double, 3, 1>(
                Coordinate.x - laWidth * abs_cos_yaw_2 + laHeight * sin_15 * sin_yaw_2,
                Coordinate.y - laWidth * sin_yaw_2 - laHeight * sin_15 * abs_cos_yaw_2 ,
                Coordinate.z + laHeight * cos_15_2));
    }
    else if (armor.Number == EClassicalArmorNumber::Invalid)
    {
        remapPoints.emplace_back(0,0,0);
        remapPoints.emplace_back(0,0,0);
        remapPoints.emplace_back(0,0,0);
        remapPoints.emplace_back(0,0,0);
    }
        //小装甲版
    else {
        remapPoints.emplace_back(Eigen::Matrix<double, 3, 1>(
                Coordinate.x - loWidth * abs_cos_yaw_2  - laHeight * sin_15 * sin_yaw_2 ,
                Coordinate.y - loWidth * sin_yaw_2 + laHeight * sin_15 * abs_cos_yaw_2 ,
                Coordinate.z - loHeight * cos_15_2 ));
        remapPoints.emplace_back(Eigen::Matrix<double, 3, 1>(
                Coordinate.x + loWidth * abs_cos_yaw_2 - laHeight * sin_15 * sin_yaw_2,
                Coordinate.y + loWidth * sin_yaw_2 + laHeight * sin_15 * abs_cos_yaw_2,
                Coordinate.z - loHeight * cos_15_2));
        remapPoints.emplace_back(Eigen::Matrix<double, 3, 1>(
                Coordinate.x + loWidth * abs_cos_yaw_2 + laHeight * sin_15 * sin_yaw_2,
                Coordinate.y + loWidth * sin_yaw_2 - laHeight *sin_15 * abs_cos_yaw_2,
                Coordinate.z + loHeight * cos_15_2));
        remapPoints.emplace_back(Eigen::Matrix<double, 3, 1>(
                Coordinate.x - loWidth * abs_cos_yaw_2 + laHeight * sin_15 * sin_yaw_2,
                Coordinate.y - loWidth * sin_yaw_2 - laHeight * sin_15 * abs_cos_yaw_2,
                Coordinate.z + loHeight * cos_15_2));
    }

    std::vector<cv::Point2f> remapPixelPoints;
    std::vector<cv::Point2f> capturePoints;
    cv::Point2f tempPoint(0,0);
    float compare_d = 0;
    double distance;
    capturePoints.emplace_back(armor.LeftUpper);
    capturePoints.emplace_back(armor.RightUpper);
    capturePoints.emplace_back(armor.RightLower);
    capturePoints.emplace_back(armor.LeftLower);


    //三维坐标二维化
    for (int i = 0; i < 4; ++i) {
        //正式使用时用
        distance = sqrt(remapPoints[i](0, 0) * remapPoints[i](0, 0) +
                        remapPoints[i](1, 0) * remapPoints[i](1, 0) +
                        remapPoints[i](2, 0) * remapPoints[i](2, 0));
        PredictorTool::GetPixelLocation(distance,&tempPoint, modelParam, remapPoints[i]);
//        使用相机调试时用
//        PredictorTool::CameraGetPixelLocation(&tempPoint, modelParam, remapPoints[i]);
        remapPixelPoints.emplace_back(tempPoint);
    }

    //重投影装甲板和相机捕捉装甲板比较
    //注意：drawandComputeIOU函数两个输入向量都必须是cv::Point2i型
    std::vector<cv::Point2i> IOUremapP;
    std::vector<cv::Point2i> IOUcaptureP;
    for (int i = 0; i < 4; ++i)
    {
        IOUremapP.emplace_back(static_cast<cv::Point2i>(remapPixelPoints[i]));
        IOUcaptureP.emplace_back(static_cast<cv::Point2i>(capturePoints[i]));
    }

    //3  用相似度比较
//    compare_d = std::sqrt(similarity(capturePoints,remapPixelPoints,yaw));
    //2  用IOU比较
    compare_d = drawAndComputeIOU(IOUremapP,IOUcaptureP);
//    Remap::Img1 = Remap::toolImg.clone();
//    Remap::Img2 = Remap::toolImg.clone();
    Remap::Img3 = Remap::toolImg.clone();
    //1  用角点距离比较
//    for (int j = 0; j < 4; ++j) {
//        compare_d += sqrt(pow((remapPixelPoints[j].x  - capturePoints[j].x ) , 2) +
//                          pow((remapPixelPoints[j].y  - capturePoints[j].y ) , 2));
//    }

    std::pair<double,std::vector<cv::Point2f>> compare_a;
    compare_a.first = compare_d;
    compare_a.second = remapPixelPoints;
    return  compare_a;

}

/**
 *   TODO  目前是遍历所有点，速度非常慢，认为时间开销大的是Compare函数中的四个角点的重投影和IOU计算，可以优化这个时间开销以及优化算法减少计算的点
 *  目前的改进想法是通过捕捉装甲板上面两个点的y值判断偏向，即设定一个阈值，如右上和左上点的y值偏差，绝对值大于阈值则通过偏差值的正负来判断偏向，小于
 *  阈值则认为在曲线的中间段。这样就将曲线分成三段，再通过在段中的导数值来判断距离目标点的远近，如导数较大且为正，则往右偏若干个点舍弃左边的点，从而
 *  一步步减少需要遍历的点，最后当导数较小时，再在仅剩的区域遍历，从而大大减少需要遍历的点。
 *  考虑到目标点可能会落在三个区域的边界，所以可以通过长宽比来确定区域，从而使得区域随每一次装甲板长宽比的变化而变化
 */
//在使用手持装甲板测试时，一定要将装甲板倾斜一定角度（15度左右），不然yaw值效果会比较差
float Remap::findAccurateYaw(cv::Point3f Coordinate,
                             HuarayCameraModelParam &modelParam,
                             ClassicalArmor armor,
                             bool stand){

    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
    uint64_t initTimestamp = initTime.time_since_epoch().count();

//    int ori = 0;
//    ori = armor.LeftUpper.y + armor.LeftLower.y - armor.RightLower.y - armor.RightLower.y > 0?  1 : -1;

//    Remap::accurateyaw = ori * -60/57.3;
//    std::vector<cv::Point3f> Pts;
//    PredictorTool::pnpArray(modelParam, armor, &Pts);

    std::vector<cv::Point2f> pixelPts;
    pixelPts.emplace_back(armor.LeftUpper);
    pixelPts.emplace_back(armor.RightUpper);
    pixelPts.emplace_back(armor.RightLower);
    pixelPts.emplace_back(armor.LeftLower);

    std::vector<Eigen::Vector2f> lines;
    for (int i = 0; i < pixelPts.size(); ++i) {
        int j = (i + 1) % 4;
        lines.emplace_back(pixelPts[j].x - pixelPts[i].x, pixelPts[j].y - pixelPts[i].y);
    }

    float angle1 = (90.0 / 57.3 - atan(lines[0].y() / lines[0].x()) - atan(lines[3].x() / lines[3].y())) * 57.3;
    float angle2 = (90.0 / 57.3 - atan(lines[1].x() / lines[1].y()) - atan(lines[2].y() / lines[2].x())) * 57.3;

    float AVangle = (angle1 + angle2) / 2;
//    std::cout << angle1 << "       " << angle2 << std::endl;

//    float leftCenter, rightCenter, deltaZ;
//
//    leftCenter = (Pts[0].z + Pts[3].z) / 2;
//    rightCenter = (Pts[1].z + Pts[2].z) / 2;
//
//    deltaZ = leftCenter - rightCenter;


    float last_yaw;
    float right_iou;
    float left_iou;
    float purpose_yaw = FLT_MIN;
    float purpose_iou = FLT_MIN;
    float tempiou;
    std::vector<float> ious2yaw;
    std::vector<float> iou_left;
    std::vector<float> iou_right;

//    Remap::accurateyaw = deltaZ > 0 ? -60 / 57.3 : 60 / 57.3;
    if (AVangle > 91.5)
    {
        Remap::accurateyaw = -60 / 57.3;
        while (Remap::accurateyaw < 0) {
            tempiou = Remap::Compare(Remap::accurateyaw, modelParam, Coordinate, armor,stand).first;
            if (tempiou > purpose_iou)
            {
                purpose_iou = tempiou;
                purpose_yaw = Remap::accurateyaw;
            }
            Remap::accurateyaw += 1.5 / 57.3;
//            std::cout <<tempiou << std::endl;
        }
//        std::cout << "1" << std::endl;
    }
    else if (AVangle < 88.5)
    {
        Remap::accurateyaw = 60 / 57.3;
        while (Remap::accurateyaw > 0)
        {
            tempiou = Remap::Compare(Remap::accurateyaw, modelParam, Coordinate, armor,stand).first;
            if (tempiou > purpose_iou)
            {
                purpose_iou = tempiou;
                purpose_yaw = Remap::accurateyaw;
            }
            Remap::accurateyaw -= 1.5 / 57.3;
//            std::cout <<tempiou << std::endl;
        }
//        std::cout << "2" << std::endl;
    }
    else
    {
        Remap::accurateyaw = -30 / 57.3;
        while(Remap::accurateyaw <30 / 57.3)
        {
            tempiou = Remap::Compare(Remap::accurateyaw,modelParam,Coordinate,armor,stand).first;
            if (tempiou > purpose_iou)
            {
                purpose_iou = tempiou;
                purpose_yaw = Remap::accurateyaw;
            }
            Remap::accurateyaw += 1.5 / 57.3;
        }
//        std::cout << "3" <<std::endl;
    }



//    if (deltaZ > 0)
//    {
//        while (Remap::accurateyaw < 0) {
//            tempiou = Remap::Compare(Remap::accurateyaw, modelParam, Coordinate, armor).first;
//            if (tempiou > purpose_iou)
//            {
//                purpose_iou = tempiou;
//                purpose_yaw = Remap::accurateyaw;
//            }
//            Remap::accurateyaw += 1.6 / 57.3;
//            std::cout <<tempiou << std::endl;
//        }
//        std::cout << "1" << std::endl;
//    }
//    else
//    {
//        while (Remap::accurateyaw > 0)
//        {
//            tempiou = Remap::Compare(Remap::accurateyaw, modelParam, Coordinate, armor).first;
//            if (tempiou > purpose_iou)
//            {
//                purpose_iou = tempiou;
//                purpose_yaw = Remap::accurateyaw;
//            }
//            Remap::accurateyaw -= 1.6 / 57.3;
//            std::cout <<tempiou << std::endl;
//        }
//        std::cout << "2" << std::endl;
//    }

//    for (int i = 0; i < 4; ++i)
//    {
//        iou_left.emplace_back(Compare(-10 -5*i/57.3,modelParam,Coordinate,armor).first);
//        iou_right.emplace_back(Compare(10 +5*i/57.3,modelParam,Coordinate,armor).first);
//    }
//
//    left_iou = std::accumulate(iou_left.begin(), iou_left.end(),0.0);
//    right_iou = std::accumulate(iou_right.begin(),iou_right.end(),0.0);
//    int RorL=0;
//    if (left_iou > right_iou)
//    {
//        RorL = -1;
//    }
//    else
//    {
//        RorL = 1;
//    }
//
//    Remap::accurateyaw = RorL * 60/57.3;
//    while ( Remap::accurateyaw < 60/ 57.3)
//    {
//        tempiou = Compare(Remap::accurateyaw,modelParam,Coordinate,armor).first;
//
//        if (tempiou > purpose_iou)
//        {
//            purpose_iou = tempiou;
//            purpose_yaw = Remap::accurateyaw;
//        }
//        Remap::accurateyaw +=  1.5 / 57.3;
//    }

//    if (abs(Remap::accurateyaw) > 10 / 57.3)
//    {
//        std::vector<cv::Point2f> Pts;
//        Pts.emplace_back(armor.LeftUpper);
//        Pts.emplace_back(armor.RightLower);
//        Pts.emplace_back(armor.RightUpper);
//        Pts.emplace_back(armor.LeftLower);
//
//        float curve_peak= similarity(Pts, Compare(Remap::accurateyaw,modelParam,Coordinate,armor).second,Remap::accurateyaw);
//        float contrary_curve = similarity(Pts, Compare(-Remap::accurateyaw,modelParam,Coordinate,armor).second,-Remap::accurateyaw);
//
////        std::cout << "p: "<< curve_peak <<"\nc: " << contrary_curve<<std::endl;
//        Remap::accurateyaw = curve_peak > contrary_curve ? -Remap::accurateyaw: Remap::accurateyaw;

//    }
//    ious2yaw.emplace_back(Compare(0,modelParam,Coordinate,armor).first);
//    ious2yaw.emplace_back(Compare(  -16/57.3 , modelParam, Coordinate, armor).first);
//    ious2yaw.emplace_back(Compare(  -32/57.3 , modelParam, Coordinate, armor).first);
//    ious2yaw.emplace_back(Compare(  -48/57.3 , modelParam, Coordinate, armor).first);
//
//    ious2yaw.emplace_back(Compare(   16/57.3 , modelParam, Coordinate, armor).first);
//    ious2yaw.emplace_back(Compare(   32/57.3 , modelParam, Coordinate, armor).first);
//    ious2yaw.emplace_back(Compare(   48/57.3 , modelParam, Coordinate, armor).first);
//
//
//    double iou_left  = 0;
//    iou_left  += (  ious2yaw[1]
//                    + ious2yaw[2]
//                    + ious2yaw[3]);
//
//    double iou_right  = 0;
//    iou_right  += (  ious2yaw[4]
//                     + ious2yaw[5]
//                     + ious2yaw[6]);
//
//    value = (iou_left - iou_right);
//
//
//    if ( value > 0 ) {
//
//        ious2yaw.erase(ious2yaw.begin()+4,ious2yaw.end());
//        ious2yaw.emplace_back(Compare(  -40/57.3 , modelParam, Coordinate, armor).first);
//        ious2yaw.emplace_back(Compare(  -50/57.3 , modelParam, Coordinate, armor).first);
//
//        float maxiou = FLT_MIN, maxyaw = 0;
//        for ( int j = 0 ; j < ious2yaw.size() ; j ++ ){
//            if (ious2yaw[j] > maxiou) {
//                maxiou = ious2yaw[j];
//                maxyaw = -j * 16 / 57.3;
//            }
//        }
//        std::cout << maxyaw <<std::endl;
//        std::cout <<ious2yaw[2] - ious2yaw[1] << std::endl;
//
//        maxiou =FLT_MIN;
//        for (int i = -5 ; i <= 5 ; i++){
//
//            float tempyaw = maxyaw + ( 1.6*i / 57.3 );
//            float tempiou = Compare(  tempyaw , modelParam, Coordinate, armor).first;
//
//            if ( tempiou > maxiou ){
//                maxiou = tempiou;
//                purpose_yaw = tempyaw;
//            }
//        }
//        if(maxiou == ious2yaw[2])
//        {
//
//            std::vector<float> ious3yaw;
//            ious3yaw.emplace_back(ious2yaw[1]);
//            ious3yaw.emplace_back(Compare(-24/57.3,modelParam, Coordinate, armor).first);
//            ious3yaw.emplace_back(ious2yaw[2]);
//            ious3yaw.emplace_back(Compare(-40/57.3 , modelParam, Coordinate, armor).first);
//
//            auto MaxIOUandYAW = [&](float border_longth) -> std::pair<float,float>{
//                float longth = border_longth;
//                while(longth > 2 ) {
//                    float L_YAW = maxyaw - border_longth / (4 * 57.3);
//                    float R_YAW = maxyaw + border_longth / (4 * 57.3);
//
//                    float L_IOU = Compare(L_YAW, modelParam, Coordinate, armor).first;
//                      float R_IOU = Compare(R_YAW, modelParam, Coordinate, armor).first;
//
//                    maxiou, maxyaw = R_IOU > maxiou ? R_IOU, R_YAW : maxiou, maxyaw;
//                    maxiou, maxyaw = L_IOU > maxiou ? L_IOU, L_YAW : maxiou, maxyaw;
//
//                    longth /= 2;
//                }
//
//            };
//
//             int maxindex =  *max_element(ious3yaw.begin(),ious3yaw.end());
//             maxiou = ious3yaw[maxindex];
//            auto maxindex = max_element();
//
//            double l_iou = Compare((-16-maxindex * 8 - 4)/57.3,modelParam, Coordinate, armor).first;
//            double r_iou = Compare((-16-maxindex * 8 + 4)/57.3,modelParam, Coordinate, armor).first;
//
//            if (l_iou > maxiou)
//            {
//                maxiou = l_iou;
//                maxyaw = (-16-maxindex * 8 - 4)/57.3;
//            }
//            if (r_iou > maxiou)
//            {
//                maxiou = r_iou;
//                maxyaw = (-16-maxindex * 8 + 4)/57.3;
//            }
//
//
//
//            std::pair(maxiou,maxyaw) = MaxIOUandYAW(16);
//        }
//        else {
//            for (int i = 0; i < 4; ++i) {
//                float tempyaw = maxyaw - 16 / 57.3 + (i + 1) * 1.6 * 4 / 57.3;
//                float tempiou = Compare(tempyaw, modelParam, Coordinate, armor).first;
//
//                if (tempiou > maxiou) {
//                    maxiou = tempiou;
//                    purpose_yaw = tempyaw;
//                }
//            }
//
//            for (int k = 0; k < 3; ++k) {
//                float tempyaw_l = purpose_yaw - (k + 1) * 1.6 / 57.3;
//                float tempyaw_r = purpose_yaw + (k + 1) * 1.6 / 57.3;
//
//                float tempiou_l = Compare(tempyaw_l, modelParam, Coordinate, armor).first;
//                float tempiou_r = Compare(tempiou_r, modelParam, Coordinate, armor).first;
//
//                if (tempiou_l > maxiou) {
//                    maxiou = tempiou_l;
//                    maxyaw = tempyaw_l;
//                }
//
//                if (tempiou_r > maxiou) {
//                    maxiou = tempiou_r;
//                    maxyaw = tempyaw_r;
//                }
//
//                purpose_yaw = maxyaw;
//
//            }
//        }
//    }else{
//
//        ious2yaw.erase(ious2yaw.begin()+1 ,ious2yaw.end() - 3);
//        ious2yaw.emplace_back(Compare(  40/57.3 , modelParam, Coordinate, armor).first);
//        ious2yaw.emplace_back(Compare(  50/57.3 , modelParam, Coordinate, armor).first);
//
//        float maxiou = FLT_MIN, maxyaw = 0;
//        for ( int j = 0 ; j < ious2yaw.size() ; j ++ ){
//            if (ious2yaw[j] > maxiou) {
//                maxiou = ious2yaw[j];
//                maxyaw = j * 16 / 57.3;
//            }
//        }
//        std::cout << maxyaw <<std::endl;
    //
//        maxiou =FLT_MIN;
//        for (int i = -5 ; i <= 5 ; i++){
//
//            float tempyaw = maxyaw + ( 1.6*i / 57.3 );
//            float tempiou = Compare(  tempyaw , modelParam, Coordinate, armor).first;
//
//            if ( tempiou > maxiou ){
//
//                maxiou = tempiou;
//                purpose_yaw = tempyaw;
//
//            }
//        }
//
//
//    }
//
//    std::cout << " value = " << value << std::endl;
    std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
    uint64_t endTimestamp = endTime.time_since_epoch().count();
//
//    std::cout << "time_cost" << (endTimestamp - initTimestamp) / 1000000 << " ms"<<std::endl;
    return purpose_yaw;
}

//使用Phi优选法计算yaw值
float Remap::PhiOptimization(cv::Point3f &Coordinate,
                             HuarayCameraModelParam &modelParam,
                             ClassicalArmor &armor,
                             bool stand)
{
    auto diff = [&](float yaw) ->float {
        double diff;
        double fx = Compare(yaw,modelParam,Coordinate,armor,stand).first;
        double fxP1 = Compare(yaw + 1e-6,modelParam,Coordinate,armor,stand).first;

        diff = (fxP1 - fx) / 1e-6;
        return diff;
    };


    std::vector<cv::Point2f> pixelPts;
    pixelPts.emplace_back(armor.LeftUpper);
    pixelPts.emplace_back(armor.RightUpper);
    pixelPts.emplace_back(armor.RightLower);
    pixelPts.emplace_back(armor.LeftLower);

    std::vector<Eigen::Vector2f> lines;
    for (int i = 0; i < pixelPts.size(); ++i) {
        int j = (i + 1) % 4;
        lines.emplace_back(pixelPts[j].x - pixelPts[i].x, pixelPts[j].y - pixelPts[i].y);
    }

    //计算识别到的装甲板四边形的左上和右下角点角
    float coe = 180 / CV_PI;
    float angle1 = (90.0 / coe - atan(lines[0].y() / lines[0].x()) - atan(lines[3].x() / lines[3].y())) * coe;
    float angle2 = (90.0 / coe - atan(lines[1].x() / lines[1].y()) - atan(lines[2].y() / lines[2].x())) * coe;
    float angle3 = (90 / coe + atan(lines[0].y()/lines[0].x()) + atan(lines[1].x() / lines[1].y())) * coe;
    float angle4 = (90 / coe + atan(lines[2].y()/lines[2].x()) + atan(lines[3].x() / lines[3].y())) * coe;

    float inclined;
    inclined = atan(lines[0].y() / lines[0].x()) + atan(lines[2].y() / lines[2].x());
    // std::cout <<"inclined: " <<inclined * 57.3 << std::endl;

    float AVangle = (angle1 + angle2) / 2;
    float AVangle_all = (angle1 + angle2 + (180 - angle3) + (180 - angle4)) / 4;


    float a,b;
    float l_phi,r_phi;
    float alpha,beta;
    if (inclined < 0u)
    {
        a = 0/57.3;
        b = 60/57.3;
        float accuracy = b-a;
        //精度控制在1度
        while (accuracy > 1 / 57.3)
        {
            l_phi = a + 0.382 * accuracy;
            r_phi = a + 0.618 * accuracy;
//            l_diff = diff(l_phi);
//            r_diff = diff(r_phi);
            alpha = Compare(l_phi,modelParam,Coordinate,armor,stand).first;
            beta = Compare(r_phi,modelParam,Coordinate,armor,stand).first;

            if (alpha > beta)
            {
                a = a;
                b = r_phi;
            }
            else
            {
                a = l_phi;
                b = b;
            }
            accuracy = b - a;

        }
    }
    else
    {
        a = -60/57.3;
        b = 0/57.3;
        float accuracy = b-a;
        //精度控制在1度
        while (accuracy > 1 / 57.3)
        {
            l_phi = a + 0.382 * accuracy;
            r_phi = a + 0.618 * accuracy;

            alpha = Compare(l_phi,modelParam,Coordinate,armor,stand).first;
            beta = Compare(r_phi,modelParam,Coordinate,armor,stand).first;

            if (alpha > beta)
            {
                a = a;
                b = r_phi;
            }
            else
            {
                a = l_phi;
                b = b;
            }
            accuracy = b - a;

        }
    }
    float remap_yaw = (a+b) / 2;
    float angle_yaw = (90 - AVangle_all) * 5;
    // float yaw = (remap_yaw + angle_yaw / coe) / 2;
    float yaw = remap_yaw;
//    std::cout<<"yaw: "<<yaw*57.3<<"   inclined:   "<<inclined*57.3<<std::endl;
    // std::cout << "remap: "<< yaw * 57.3 << "  "<< "mix: "<<(remap_yaw * sin(inclined* 5) * sin(inclined*5) + angle_yaw / coe * cos(inclined*5) * cos(inclined*5)) * 57.3 <<std::endl;
    return yaw;

}


//计算IOU
//在使用手持装甲板测试时，一定要将装甲板倾斜一定角度（15度左右），不然yaw值效果会比较差
float Remap::drawAndComputeIOU(std::vector<cv::Point2i> Points1,
                               std::vector<cv::Point2i> Points2) {

//    std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
//    uint64_t startTimestamp = startTime.time_since_epoch().count();

    std::vector<std::vector<cv::Point2i>> p1;
    std::vector<std::vector<cv::Point2i>> p2;
    std::vector<std::vector<cv::Point2i>> p3;
    std::vector<cv::Vec4i> hir1;
    std::vector<cv::Vec4i> hir2;
    std::vector<cv::Vec4i> hir3;

    //截取roi
    auto get_roi = [&](std::vector<cv::Point2i> Point1,
                       cv::Mat image,
                       bool flag = false,
                       std::vector<cv::Point2i> *roi = NULL) -> cv::Mat{

        int LU_P_x = INT_MIN,LU_P_y = INT_MIN;
        int hei = INT_MIN,wid = INT_MIN;

        //对于两矩形的roi截取
        if (flag){
            LU_P_x = std::min(std::min(std::min(Points1[0].x,Points1[1].x),Points1[3].x),
                              std::min(std::min(Points2[0].x,Points2[1].x),Points2[3].x));
            LU_P_y = std::min(std::min(std::min(Points1[0].y,Points1[1].y),Points1[3].y),
                              std::min(std::min(Points2[0].y,Points2[1].y),Points2[3].y));

            wid  = std::max(std::max(std::max(Points1[1].x,Points1[2].x),Points1[3].x) ,
                            std::max(std::max(Points2[1].x,Points2[2].x),Points2[3].x));
            hei  = std::max(std::max(std::max(Points1[1].y,Points1[2].y),Points1[3].y) ,
                            std::max(std::max(Points2[1].y,Points2[2].y),Points2[3].y));

        }

            //对于单矩形的roi截取

        else {
            for (int i = 0; i < Point1.size(); ++i) {
                if (-Point1[i].x > LU_P_x)
                    LU_P_x = Point1[i].x;
                if (-Point1[i].y > LU_P_y)
                    LU_P_y = Point1[i].y;
                if (Point1[i].x > wid)
                    wid = Point1[i].x;
                if (Point1[i].y > hei)
                    hei = Point1[i].y;
            }
        }
        //矩形在image边界时会出现内存越界，故做出了部分限制
        wid = wid >1200?  1200 : wid;
        hei = hei >1200?  1200 : hei;
        wid = wid - LU_P_x < 0? LU_P_x + 1: wid;
        hei = hei -LU_P_y < 0? LU_P_y + 1 : hei;
        LU_P_x = LU_P_x <=0 ? 1 : LU_P_x;
        LU_P_y = LU_P_y <=0 ? 1 : LU_P_y;
        cv::Rect roi_rect(std::abs(LU_P_x),std::abs(LU_P_y),wid - LU_P_x +1 ,hei - LU_P_y +1 );
        if (roi != NULL) {
            roi->emplace_back(LU_P_x, LU_P_y);
            roi->emplace_back(wid, hei);
        }
        cv::Mat img = image(roi_rect);
        return img;

    };

    //计算边的叉乘
    auto crossProduct = [](const cv::Point& A, const cv::Point& B, const cv::Point& C) {
        return (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
    };
    //计算三角形的面积
    auto areaOfTriangle = [&crossProduct](const cv::Point& A, const cv::Point& B, const cv::Point& C) {
        return 0.5 * std::abs(crossProduct(A, B, C));
    };

//    for (int i = 0 ; i < 4; i++)
//    {
//        int j = i+1;
//        i = 3 ? :j = 0;
//        cv::line(img1,Points1[i],Points1[j],cv::Scalar(255,0,0));
//        cv::line(Img2,Points2[i],Points2[j],cv::Scalar(0,255,0));
//        cv::line(Img3,Points1[i],Points1[j],cv::Scalar(255,0,0));
//        cv::line(Img3,Points2[i],Points2[j],cv::Scalar(0,255,0));
//    }

//    cv::fillConvexPoly(Remap::Img1,Points1,cv::Scalar (255,255,255),cv::LINE_8);
//    cv::fillConvexPoly(Remap::Img2,Points2,cv::Scalar (255,255,255));
    cv::fillConvexPoly(Remap::Img3,Points1,cv::Scalar (255,255,255));
    cv::fillConvexPoly(Remap::Img3,Points2,cv::Scalar (255,255,255));
//    std::chrono::time_point<std::chrono::steady_clock> mid1Time = std::chrono::steady_clock::now();
//    uint64_t mid1Timestamp = mid1Time.time_since_epoch().count();
//    std::cout << Points1 << std::endl;


//    cv::findContours(get_roi(Points1,Remap::Img1),p1,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE,cv::Point(0, 0));
//    cv::findContours(get_roi(Points2,Remap::Img2),p2,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE,cv::Point(0, 0));
    cv::findContours(get_roi(Points1,Remap::Img3,true),p3,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE,cv::Point(0, 0));

//    std::chrono::time_point<std::chrono::steady_clock> mid2Time = std::chrono::steady_clock::now();
//    uint64_t mid2Timestamp = mid2Time.time_since_epoch().count();
//    std::cout << p1[0] << std::endl;
    float area1 = areaOfTriangle(Points1[0],Points1[1],Points1[2]) + areaOfTriangle(Points1[0],Points1[2],Points1[3]);
    float area2 = areaOfTriangle(Points2[0],Points2[1],Points2[2]) + areaOfTriangle(Points2[0],Points2[2],Points2[3]);
    float uni;
    if (p3.empty())
        uni = 100000;
    else
        uni   = static_cast<float>(cv::contourArea(p3[0]));

//    std::cout << area1 << std::endl;
//    std::cout << area2 << std::endl;
//    std::cout << uni << std::endl;

//    std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
//    uint64_t endTimestamp = endTime.time_since_epoch().count();

    //耗时输出
//    std::cout << "-------------------------------------------------------" <<std::endl;
//    std::cout << (mid1Timestamp - startTimestamp) / 1000 << "ns" << std::endl;
//    std::cout << (mid2Timestamp - mid1Timestamp) / 1000 << "ns" << std::endl;
//    std::cout << (endTimestamp - mid2Timestamp) /1000 << "ns" << std::endl;
//    std::cout << "-------------------------------------------------------" <<std::endl;

    float IOU = (area1 + area2 - uni) / uni;
    return IOU;
}

//对交爷拙劣的模仿罢了
//在我们的识别框架下此方法效果奇差，可能是因对于识别精度要求比较高
float Remap::similarity(const std::vector<cv::Point2f>& capPts,
                        const std::vector<cv::Point2f>& remPts,
                        const float& yaw) {

    std::vector<Eigen::Vector2d> eigen_capPoints;
    std::vector<Eigen::Vector2d> eigen_remapPoints;

    for (int i = 0; i < 4; ++i)
    {
        eigen_capPoints.emplace_back(capPts[i].x,capPts[i].y);
        eigen_remapPoints.emplace_back(remPts[i].x,remPts[i].y);
    }
    float cost = FLT_MIN;
    //
    for(int j =0 ; j < eigen_remapPoints.size() ; j++ )
    {
        //
        Eigen::Vector2d diffvalue_capP;
        Eigen::Vector2d diffvalue_remP;
        int u = (j + 1) % 4;

        diffvalue_capP = eigen_capPoints[j] - eigen_capPoints[u];
        diffvalue_remP = eigen_remapPoints[j] - eigen_remapPoints[u];

        double diffV_rect = (0.5 * ((eigen_remapPoints[j] - eigen_capPoints[j]).norm() + (eigen_remapPoints[u] - eigen_capPoints[u]).norm()) +
                             std::abs(diffvalue_remP.norm() - diffvalue_capP.norm())) / diffvalue_remP.norm();
        double cost_r,cost_a;
        if (!(j % 2))
        {
            double diffV_angle = PredictorTool::getAbsAngle(diffvalue_remP, diffvalue_capP);

//            cost_r += diffV_rect;
//            cost_a += diff_angle;
            cost += diffV_angle;


        }
//        cost += sqrt(cost_r) + sqrt(cost_a) * 3;
//        float cost_ = std::pow(diffV_rect * cos(yaw),2) + std::pow(diffV_angle * sin(yaw)  ,2) * 2;
//        float cost_ = std::pow(diffV_rect,2);


    }

    return cost;
}

void Remap::yawFilter(std::vector<float> yawSequence) {
    int yaw_size = yawSequence.size();
    float nowyaw = yawSequence[yaw_size - 1];



}

cv::Mat Remap::Img1(cv::Mat::zeros(1400,1400,CV_8UC1));
cv::Mat Remap::Img2(cv::Mat::zeros(1400,1400,CV_8UC1));
cv::Mat Remap::Img3(cv::Mat::zeros(1400,1400,CV_8UC1));
cv::Mat Remap::toolImg(cv::Mat::zeros(1400,1400,CV_8UC1));
float Remap::accurateyaw(0.1);
bool Remap::IsSuitable(false);






