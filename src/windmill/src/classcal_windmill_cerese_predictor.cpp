//
// Created by gusha on 23-12-3.
//

#include "classcal_windmill_cerese_predictor.h"


    Fitcoordinate::Fitcoordinate ( RunMode mode , uint64_t delayT , bool isClockwise ):

               IsTargetReady  (false),
               IsLogoReady (false),
               IsParam_ok (false),
               IsClockwise (isClockwise),
               delaytime(delayT),
               Startime(0),
               Mode(mode),
               ValidLogo(),
               ValidTarget(),
               time_gap(){}


bool Fitcoordinate::_3DComputeLogoHOP ( ClassicalWindmillLogo Logo , std::vector <ClassicalWindmillLogo> &ValidLogos , Eigen::Matrix3d QMatrix , const HuarayCameraModelParam &modelParam){

    if ( ValidLogos.size() > 100 ) ValidLogos.erase(ValidLogos.begin(),ValidLogos.begin() + 30),ValidLogos.erase(ValidLogos.end() - 30,ValidLogos.end());

    Eigen::Vector3d LogoCoordinate(0,0,0);

    PredictorTool::GetRelativeCorrdinate(7420,
                                         Logo.ContourCenter,
                                         false,
                                         modelParam,
                                         &LogoCoordinate);


    Logo.CradleHeadCoordinate = LogoCoordinate;
    LogoCoordinate = QMatrix * LogoCoordinate;
    Logo.WorldCoordinate = LogoCoordinate;

//    std::cout << " Logo : [" << LogoCoordinate.x() << " , " << LogoCoordinate.y() << " , " << LogoCoordinate.z()  << " ] " << std::endl;

    int value = 600;
    auto _3DComputeDistance = [&] (int i , int j)-> bool {
        return (ValidLogos[j].WorldCoordinate - ValidLogos[i].WorldCoordinate).norm() < value;
    };

    ValidLogos.emplace_back(Logo);

    //插入排序
    for (int i = 0 ; i < ValidLogos.size() ; i ++)
        if ( _3DComputeDistance (ValidLogos.size() - 1,i) ){
            for (int j = ValidLogos.size()-1 ; j > i ; j -- )  std::swap(ValidLogos[j] , ValidLogos[j - 1]);
            break;
        }

    //计算logos众数

    int MaxCount = 0;
    ClassicalWindmillLogo Templogo;

    for (int i = 0 ; i < ValidLogos.size();)
    {
        int count = 1,j = i;

        for (; j <= ValidLogos.size() - 2; j ++ ){

            if (  _3DComputeDistance (j , j + 1) )  count++;
            else  break;
        }

        if (MaxCount < count)
        {
            MaxCount = count;
            Templogo = ValidLogos[j];
        }
        ++j;
        i = j;
    }

//    std::cout <<  LogoCoordinate << "   " << Templogo.WorldCoordinate  << std::endl;
    //判断 logo 是否与logos的众数差距太大
    return ( ( LogoCoordinate - Templogo.WorldCoordinate ).norm() < value );

}



bool Fitcoordinate::_2DComputeLogoHOP ( ClassicalWindmillLogo Logo ,std::vector <ClassicalWindmillLogo> &ValidLogos ) {

        if ( ValidLogos.size() > 100 ) ValidLogos.erase(ValidLogos.begin(),ValidLogos.begin() + 30),ValidLogos.erase(ValidLogos.end() - 30,ValidLogos.end());

        ValidLogos.emplace_back(Logo);

//        std::cout << " now =  " << Logo.ContourCenter << std::endl;

        //插入排序

        int value = 150;

        for (int i = 0 ; i < ValidLogos.size() ; i ++)
            if ( ComputeDistance(ValidLogos.back().ContourCenter,ValidLogos[i].ContourCenter) < value ){
                for (int j = ValidLogos.size()-1 ; j > i ; j -- )  std::swap(ValidLogos[j] , ValidLogos[j - 1]);
                break;
            }

        //计算logos众数

        int MaxCount = 0;
        ClassicalWindmillLogo Templogo;

        for (int i = 0 ; i < ValidLogos.size();)
        {
            int count = 1,j = i;

            for (; j <= ValidLogos.size() - 2; j ++ ){

                if (ComputeDistance(ValidLogos[j].ContourCenter,ValidLogos[j + 1].ContourCenter) < value )  count++;
                else  break;
            }

            if (MaxCount < count)
            {
                MaxCount = count;
                Templogo = ValidLogos[j];
            }
            ++j;
            i = j;
        }


//        std::cout << " z =  " << Templogo.ContourCenter << std::endl;

        //判断 logo 是否与logos的众数差距太大
        return (ComputeDistance( Logo.ContourCenter,Templogo.ContourCenter) < value );

    }


    float Fitcoordinate::ComputeDistance(cv::Point2f a, cv::Point2f b) {

        return sqrt( pow( a.x - b.x,2) + pow(a.y - b.y,2) );

    }


    bool Fitcoordinate::Fit_SmallWindmill( cv::Point2f &calpoint ) {

        if ( Mode != SMALL_RUN ) return false;

        if ( !IsTargetReady || !IsLogoReady) return false;


            float roll_angle =   M_PI * (delaytime / 1000.0f) / 3.0f;    // 小符旋转速度恒定 1/3 * M_PI 每秒

            float tempangle;
            float R = Fitcoordinate::ComputeDistance( ValidLogo.ContourCenter , ValidTarget.targetCenter );

            ClassicalWindmillRecognizer::ComputeWindmillFanAngle(ValidLogo.ContourCenter,
                                                                 ValidTarget.targetCenter,
                                                                 &tempangle);

            tempangle /= 57.3;

            float de_x , de_y;

            if (IsClockwise){

                de_x =  R * cos (  tempangle - roll_angle );
                de_y =  R * sin (  tempangle - roll_angle );

                calpoint.x = ValidLogo.ContourCenter.x + de_x;
                calpoint.y = ValidLogo.ContourCenter.y - de_y;

            }else{

                de_x =  R * cos (  tempangle + roll_angle );
                de_y =  R * sin (  tempangle + roll_angle );

                calpoint.x = ValidLogo.ContourCenter.x + de_x;
                calpoint.y = ValidLogo.ContourCenter.y - de_y;

            }

        return true;

    }

bool Fitcoordinate::GetIsClockwise ( float angleChange ){

    static std::vector<float> angleChanges;

    static bool isclockwise = true;

    if (angleChanges.size() > 20 ) {

        isclockwiseOk = 1;
        return isclockwise;
    }

    angleChanges.push_back(angleChange);

    if (angleChanges.size() != 20 ) return isclockwise;

    double clockwise = 0;
    double anticlockwise = 0;

    for( auto item : angleChanges )     item < 0 ? clockwise++ : anticlockwise++;

    isclockwise = clockwise > anticlockwise ;

    return isclockwise;

}


    bool Fitcoordinate::GetSpeed_Data( std::vector<float> &angles , std::vector<uint64_t> &w_Timestamps , std::vector<W_Time_data> &sin_datas ) {

            if ( !IsTargetReady || !IsLogoReady)  return false;

            static int times = 0;

            float temp;

            ClassicalWindmillRecognizer::ComputeWindmillFanAngle(ValidLogo.ContourCenter,
                                                                 ValidTarget.targetCenter,
                                                                 &temp);

            angles.emplace_back(temp);
            std::chrono::time_point<std::chrono::steady_clock> w_Time = std::chrono::steady_clock::now();
            w_Timestamps.emplace_back(w_Time.time_since_epoch().count());

            Startime = *w_Timestamps.begin();

//        std::cout << angles.size() << std::endl;

            if (angles.size() >= 2  && abs(angles.back() - *(angles.end() - 2)) > 50  && abs(angles.back() - *(angles.end() - 2)) < 300 ){
//
//                std::chrono::time_point<std::chrono::steady_clock> now_jtime = std::chrono::steady_clock::now();
//                uint64_t _jtime = now_jtime.time_since_epoch().count();
//
//
//                std::cout << " ---------------------------跳变---------------------------- " << std::endl;     //🤑🤑🤑🤑🤑🤑🤑🤑🤑🤑🤑🤑🤑🤑🤑
//                times++;
//                uint64_t  pretime = times == 1 ? 0 : pretime = _jtime;
            }
            else if (angles.size() >= 2 ) {

                float angles_change = angles.back() - *(angles.end() - 2);

                IsClockwise = GetIsClockwise( angles_change );

                angles_change = abs( angles_change );

                float times_change = ( w_Timestamps.back() - *(w_Timestamps.end() - 2)) / 1000000000.0f ;

                float womiga = angles_change > 340 ? (360 - angles_change) / times_change : angles_change / times_change;

                sin_datas.emplace_back(womiga / 57.3 , ( w_Timestamps.back() - *w_Timestamps.begin() ) / 1000000000.0f );

                if (sin_datas.size() > 500 ) sin_datas.erase(sin_datas.begin(),sin_datas.begin() + 200);
//                    puts(" ");
//                    std::cout << " <---- ValidLogoCenter ----> " << ValidLogo.ContourCenter << "  <-------ValidTargetCenter------> " << ValidTarget.targetCenter <<std::endl;
//                    std::cout << " <---- angle_before ----> ----" << *(angle.end() - 2)  << " <-------angle------>  " <<angle.back() << "  " <<std::endl;
//                    std::cout << " <---- womiga ----> " << womiga  / 57.3 << " rad/s  <-------times_change------> " << times_change  <<"s  <---------pos--------->  " << sin_datas.size() <<std::endl;
//                    puts(" ");

            }

        return true;
    }


    void Fitcoordinate::Avg_weighted (std::vector< W_Time_data > datas, std::vector< W_Time_data > &callbake_datas , float B ) {

        if ( B > 1 || B < 0) { std::cout << " <------- Avg_weighted WA  ------->" << std::endl; return;}

        callbake_datas.push_back({1,0});

        for ( auto data : datas){

            float temp = B * callbake_datas.back().omiga + (1 - B) * data.omiga;
            callbake_datas.push_back( {temp,data.time} );

        }

        callbake_datas.erase(callbake_datas.begin());

        return ;
    }


    bool Fitcoordinate::Fit_Data2Param ( std::vector<W_Time_data> sin_datas ,double param[3] , bool IsShow ,RunMode Mode ){

        if ( Mode != BIG_RUN || sin_datas.size() < 50 ) return false;

        if ( (sin_datas.size() % 50) ) return true;

        if ( sin_datas.size() > 1e3 ) sin_datas.erase( sin_datas.begin() ,sin_datas.begin() + 500 );

            std::vector< W_Time_data > fit_datas;
            Fitcoordinate::Avg_weighted (sin_datas , fit_datas , 0.5);   //指数加权平均

///*          //输出 data
//            int pos = 1;
//            for (auto fit : fit_datas)
////        std::cout << " [womiga,time] " <<  "  [ " <<fit.first <<" , " << fit.second <<  " ]      <---------pos--------->  "  <<  pos++ << std::endl;
//                std::cout << fit.omiga << std::endl;
//            puts(" >----------------------omiga----------------------------<");
//            for (auto fit : fit_datas)
//                std::cout << fit.time << std::endl;
//            puts(" >-----------------------time---------------------------<");
//            std::cout << fit_datas.size() << std::endl;
//            puts(" >--------------------------------------------------<");

//            int idex = 0;
//            for (auto i : fit_datas) {
//
//                if (++idex == 20) {
//                    std::cout << std::endl;
//                    idex = 0;
//                }
//                std::cout << i.time << ",";
//            }
//*/

            //初始化参数

            double max_womiga = -1.0;
            double min_womiga = 99999.99;

            for(auto data : fit_datas)
            {
                if(data.omiga > max_womiga)
                {
                    max_womiga = data.omiga;
                }
                if(data.omiga < min_womiga)
                {
                    min_womiga = data.omiga;
                }
            }

            double p0 = (max_womiga - min_womiga)/2.0;
            double p1 = 2 * M_PI / fit_datas.back().time ;

            param[0] = p0;
            param[1] = p1;
            param[2] = M_PI ;

            ceres::Problem problem;
            for (int i = 0; i < sin_datas.size(); i+=5) {

                ceres::CostFunction *costfun = new ceres::AutoDiffCostFunction<sincostfunction, 1, 3>(
                        new sincostfunction(sin_datas[i].omiga, sin_datas[i].time));

                problem.AddResidualBlock(
                        costfun,
                        new ceres::CauchyLoss(0.5),
                        param
                );
            }

            // 设置优化参数范围
            problem.SetParameterLowerBound(param, 0, 0.780);
            problem.SetParameterUpperBound(param, 0, 1.045);
            problem.SetParameterLowerBound(param, 1, 1.884);
            problem.SetParameterUpperBound(param, 1, 2);
            problem.SetParameterLowerBound(param, 2, 0);
            problem.SetParameterUpperBound(param, 2, 2*CV_PI);

            ceres::Solver::Options options;
            options.max_num_iterations = 25;
            options.linear_solver_type = ceres::DENSE_QR;
            options.minimizer_progress_to_stdout = IsShow;

            ceres::Solver::Summary summary;
            Solve(options, &problem, &summary);

            if ( IsShow ){
                std::cout << " sin_A = " << param[0] << std::endl;
                std::cout << " sin_w = " << param[1] << std::endl;
                std::cout << " sin_t0 = " << param[2] << std::endl;
                std::cout << " sin_b = " << 2.090 - param[0] << std::endl;
                std::cout << " sin_datas_size = " << sin_datas.size() << std::endl;
            }

        return true;
    }


    bool Fitcoordinate::Fit_BigWindmill(cv::Point2f &calpoint ,std::vector<uint64_t> w_Timestamps , double param[3]) {

        if ( !IsTargetReady || !IsLogoReady )  return false;

            float roll_angle =
                    call_param2roll_angle(param, (w_Timestamps.back() + ( delaytime * 1000000 ) - Startime) / 1000000000.0f)
                    - call_param2roll_angle(param, (w_Timestamps.back() - Startime) / 1000000000.0f);

//            float D_1 = -2 * ValidLogo.ContourCenter.x , E_1 = -2 * ValidLogo.ContourCenter.y ;
//            float F_1 = pow(ValidLogo.ContourCenter.x,2)
//                        + pow(ValidLogo.ContourCenter.y,2)
//                        + pow( ClassicalWindmillRecognizer::ComputeDistance( ValidLogo.ContourCenter , ValidTarget.targetCenter ) ,2);
//
//            float D_2 = -2 * ValidTarget.targetCenter.x , E_2 = -2 * ValidTarget.targetCenter.y ;
//            float F_2 = pow(ValidTarget.targetCenter.x,2)
//                        + pow(ValidTarget.targetCenter.y,2)
//                        + 2 * pow( ClassicalWindmillRecognizer::ComputeDistance( ValidLogo.ContourCenter , ValidTarget.targetCenter ) ,2) * ( 1 - cos ( roll_angle ));
//
//            float a = ( E_2 - E_1 ) / ( D_1 -D_2 ) , b = ( F_2 - F_1 ) / ( D_1 - D_2 );
//            float A = pow( a , 2 ) + 1 , B = 2 * a * b + D_1 * a + E_1 , C = pow(b,2) + b * D_1 + F_1;
//
//            std::cout << " A = " << A << std::endl;
//            std::cout << " B = " << B << std::endl;
//            std::cout << " C = " << C << std::endl;
//            std::cout << " pow(B,2) - 4 * A * C ) = " << pow(B,2) - 4 * A * C  << std::endl;
//
//            cv::Point2f TargetPoint;
//
//            TargetPoint.y = ( -B + sqrt(pow(B,2) - 4 * A * C ) ) / ( 2 * A );
//            TargetPoint.x = a * TargetPoint.y + b;
//            ps : 我是小丑🤡，白推了

            float temp;
            float R = ClassicalWindmillRecognizer::calDistance ( ValidLogo.ContourCenter , ValidTarget.targetCenter );

            ClassicalWindmillRecognizer::ComputeWindmillFanAngle(ValidLogo.ContourCenter,
                                                                 ValidTarget.targetCenter,
                                                                 &temp);

            temp /= 57.3;

            float de_x , de_y;

            if (IsClockwise){

                de_x =  R * cos (  temp - roll_angle );
                de_y =  R * sin (  temp - roll_angle );

                calpoint.x = ValidLogo.ContourCenter.x + de_x;
                calpoint.y = ValidLogo.ContourCenter.y - de_y;

            }else{

                de_x =  R * cos (  temp + roll_angle );
                de_y =  R * sin (  temp + roll_angle );

                calpoint.x = ValidLogo.ContourCenter.x + de_x;
                calpoint.y = ValidLogo.ContourCenter.y - de_y;
            }

        return true;

    }

    void Fitcoordinate::Run( std::vector<float> &angles , std::vector<uint64_t> &w_Timestamps ,std::vector<W_Time_data> &sin_datas , double *param ,HuarayCameraModelParam modelParam , cv::Point2f &Pre_point ,Eigen::Vector3d &Pre_Coordinate ) {

        GetSpeed_Data( angles , w_Timestamps ,sin_datas );

        //      小能量机关

        if ( Mode == SMALL_RUN )   {

            Fit_SmallWindmill( Pre_point );

            double distanceTarget = 7420;
            PredictorTool::GetRelativeCorrdinate(distanceTarget,Pre_point,false,modelParam,&Pre_Coordinate);
        }


        //      大能量机关
        if ( Mode == BIG_RUN ){

            IsParam_ok =  Fitcoordinate::Fit_Data2Param ( sin_datas , param , 0 , Mode);
            if ( IsParam_ok ) {

                Fit_BigWindmill ( Pre_point , w_Timestamps , param);

                double distanceTarget = 7420;
                PredictorTool::GetRelativeCorrdinate(distanceTarget,Pre_point,false,modelParam,&Pre_Coordinate);

            }

        }


    }



