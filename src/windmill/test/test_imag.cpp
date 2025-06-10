//
// Created by gusha on 23-10-31.
//
//
// Created by gusha on 23-10-28.
//

//#include <opencv2/opencv.hpp>
//#include <chrono>
//#include "easy_logger.h"
//#include "huaray_camera.h"
//#include "classical_windmill_recognizer.h"
//#include "solver.h"
//
//using namespace std;
//
//uint64_t temp1,temp2;
//int times = 1e3;
//
//
//void fun (){
//
//    std::chrono::time_point<std::chrono::steady_clock> funbegainTime = std::chrono::steady_clock::now();
//    uint64_t funbegainTimestamp = funbegainTime.time_since_epoch().count();
//
//   {
//        sleep(5);
//        std::chrono::time_point<std::chrono::steady_clock> funTime = std::chrono::steady_clock::now();
//        uint64_t funTimestamp = funTime.time_since_epoch().count();
//
//        temp1 = funTimestamp - funbegainTimestamp ;
//
//    }
//
//}
//
//void Timefun (){
//
//    std::chrono::time_point<std::chrono::steady_clock> b_video_timePoint = std::chrono::steady_clock::now();
//    uint64_t b_Videostamp = b_video_timePoint.time_since_epoch().count();
//
//    bool idex[100000];
//
//    while (1){
//
//        if ( Isvalidrun ) break;
//
//        std::chrono::time_point<std::chrono::steady_clock> e_video_timePoint = std::chrono::steady_clock::now();
//        uint64_t e_Videostamp = e_video_timePoint.time_since_epoch().count();
//
//        uint64_t temp_times = (e_Videostamp - b_Videostamp) / 10000000;
//
//        if ( !idex[temp_times] )  {
//            video >> cameraData.Image;
//            idex[temp_times] = true;
//        }
//
//    }
//
//
//}

//#include "ceres/ceres.h"
//#include "classcal_windmill_cerese_predictor.h"
//#include "glog/logging.h"
//#include<iostream>
//#include<thread>
////#include<string>
//using namespace std;
//
//void myfunc_work() {
//    cout << "myfunc_work ....." << endl;
//    // do something 5s
//    sleep(7);
//}

//int main() {
//    std::thread t1(myfunc_work);
//    // 阻塞当前main主线程，待子线程执行完毕后，自己恢复主线程逻辑
//    t1.join();
//    cout << "main thread ....." << endl;
//    sleep(5);
//
//}

//
#include <opencv4/opencv2/opencv.hpp>
#include "classcal_windmill_cerese_predictor.h"

int main (){

    // 记录初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
    uint64_t initTimestamp = initTime.time_since_epoch().count();


    int frameIndex = 0;

    /*  程序
     *
     *   每读取一下图片  frameIndex ++
     *
     * */

    Fitcoordinate Fit (SMALL_RUN,0);

    // 记录截止时间戳
    std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
    uint64_t endTimestamp = endTime.time_since_epoch().count();


    std::cout << "fps: " << (frameIndex * 1000000000) / (endTimestamp - initTimestamp) << std::endl;
}


//int main (){
//
//        Fitcoordinate::test.push_back(10);
//
//        Fitcoordinate::ttest();
//
//
//}





//using ceres::AutoDiffCostFunction;
//using ceres::CostFunction;
//using ceres::Problem;
//using ceres::Solve;
//using ceres::Solver;

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.

// value =  yaw^2 + 4*yaw + 5
//double testfun (float yaw ){
//
//    return 2 * pow(yaw,2) + 4 * yaw + 5;
//}
//
//
//struct CostFunctor {
//    template <typename T>
//    bool operator()(const T* const x, T* residual) const {
//
//        double value = testfun( x[0] );
//        residual[0] = value;
//
//        std::cout << value << std::endl;
//
//        return true;
//    }
//};
//
//int main() {
//
//    double x = 0.5;
//    const double initial_x = x;
//
//    Problem problem;
//
//    CostFunction* cost_function =
//            new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 1, 1>(new CostFunctor);
//    problem.AddResidualBlock(cost_function, nullptr, &x);
//
//    Solver::Options options;
//    options.minimizer_progress_to_stdout = true;
//    options.max_num_iterations = 15;
//    options.gradient_check_numeric_derivative_relative_step_size = 1e-1;
//    Solver::Summary summary;
//    Solve(options, &problem, &summary);
//
//    std::cout << summary.BriefReport() << "\n";
//    std::cout << " min_x : "<< x << "\n";
//    return 0;
//}


//int main (){
//
//    Eigen::Quaterniond Q_raw(cos (M_PI / 2),0,sin ( M_PI / 2 ) * 1,0);
//    Eigen::Matrix3d QMatrix = Q_raw.toRotationMatrix();
//
//    std::cout << QMatrix << std::endl;
//
//    Eigen::Vector3d euler_t = Q_raw.toRotationMatrix().eulerAngles(2,1,0);
//
//    std::cout<<" YAW: "<<euler_t(0)*57.3<<" Pitch: "<<euler_t(1)*57.3<<" Roll: "<<euler_t(2)*57.3<<std::endl;
//
//    return 0;
//}
//


//int main (){
//
//        std::thread t1(fun);
//
//        std::chrono::time_point<std::chrono::steady_clock> mainbegainTime = std::chrono::steady_clock::now();
//        uint64_t mainbegainTimestamp = mainbegainTime.time_since_epoch().count();
//
//        {
//            sleep(5);
//            std::chrono::time_point<std::chrono::steady_clock> mainTime = std::chrono::steady_clock::now();
//            uint64_t mainTimestamp = mainTime.time_since_epoch().count();
//
//            temp2 = mainTimestamp - mainbegainTimestamp ;
//
//        }
//
//    std::cout << " <------temp1----->" << temp1 << "  <-----temp2----> " << temp2 << std::endl;
//
//    return 0;
//
//}


//#include <iostream>
//#include <chrono>
//
//using namespace std;
//
//int main (){
//
//    chrono::time_point < chrono::steady_clock > begaintime = chrono::steady_clock::now();
//    float  pre_timestamp = begaintime.time_since_epoch().count() / 1000000000.0f;
//    bool isfire = false;
//    float  gap = 0.8;
//
//    while (!false){
//
//    chrono::time_point < chrono::steady_clock > nowtime = chrono::steady_clock::now();
//    float now_timestamp = nowtime.time_since_epoch().count() / 1000000000.0f;
//
//    float nowgap = now_timestamp - pre_timestamp;
//    if ( nowgap > gap ){
//
//        isfire = true;
//        pre_timestamp = now_timestamp;
//    }else isfire = false;
//
//        if (isfire) puts (" ---------- fire ! ----------");
//    }
//
//    return  0;
//}


//int main () {
//
//    ClassicalWindmillLogo temp1; temp1.ContourCenter = {560.5 , 559.5};
//    ClassicalWindmillLogo temp2; temp2.ContourCenter = {560 , 559.5};
//    ClassicalWindmillLogo temp3; temp3.ContourCenter = {1560.5 , 559};
//    ClassicalWindmillLogo temp4; temp4.ContourCenter = {560 , 559};
//    ClassicalWindmillLogo temp5; temp5.ContourCenter = {1060.5 , 559.5};
//
//
//    std::vector<ClassicalWindmillLogo> temps = {temp1};
//
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp5 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp3 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp5 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp2 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp2 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp3 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp5 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp5 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp2 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp3 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp3 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp4 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp4 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp2 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//    ClassicalWindmillRecognizer::ComputeLogoHOP ( temp1 , temps);
//
//
//    for ( auto it : temps ) std::cout<<"<------cont----->   "<<it.ContourCenter<<std::endl;
//
//
//    if (  ClassicalWindmillRecognizer::ComputeLogoHOP ( temp4 , temps) ) std::cout << " Yes!!!!!!!!! " <<std::endl;
//    else std::cout << " NO!!!!!!!!! " <<std::endl;
//
//    return 0;
//}



//采样拟合函数
//           sqd = A * sin ( w * t + t0) + b
//           param[0] is A  ||  param[1] is w  ||param[2] is t0
//class sincostfunction{
//public:
//
//    sincostfunction(double _spd,double _t): spd(_spd),t(_t) {};
//
//    template <class T>
//    bool operator()(const T* const param, T* residual) const {
//
//        residual[0] = spd - (
//
//                param[0] * sin ( param[1] * t + param[2]) + 2.090 - param[0]
//
//        );
//        return true;
//    }
//
//private:
//    const double spd;
//    const double t;
//};

//采样拟合函数
//           roll_Angle = -(a/w) * cos(w * t + t0) + (2.09 -a) * t + (a/w) * cos(t0)   #上面函数的积分用计算一定时间内中心点转过的角度
//           param[0] is A  ||  param[1] is w  ||param[2] is t0
//class integral_sincostfunction{
//public:
//
//    integral_sincostfunction(double _roll_Angle,double _t): roll_Angle(_roll_Angle),t(_t) {};
//
//    template <class T>
//    bool operator()(const T* const param, T* residual) const {
//
//        residual[0] = roll_Angle - (
//
//                -(param[0] / param[1]) * cos(param[1] * t + param[2]) + (2.09 - param[0]) * t+ (param[0] / param[1]) * cos(param[2])
//        );
//        return true;
//    }
//
//private:
//    const double roll_Angle;
//    const double t;
//};

//
//bool sleep (uint  m){
//
//
//
//
//}


// 按ESC键，退出系统
// 按Enter键，暂停系统
// 按任意键，继续处理
//int main(int argc, char *argv[])
//{
//    // 初始化日志记录器
//    EasyLogger &logger = EasyLogger::GetSingleInstance();
//    logger.Init();
//
//    // 创建相机
//    HuarayCamera camera;
//
//    // 读取相机参数
//    HuarayCameraParam cameraParam;
//    std::string cameraYaml = "config/infantry_3/basement/huaray_camera_param.yaml";
//    if (!HuarayCameraParam::LoadFromYamlFile(cameraYaml, &cameraParam))
//    {
//        return -1;
//    }
//
//    // 设置相机参数
//    if (!camera.SetParam(cameraParam))
//    {
//        return -1;
//    }
//
////     初始化相机
//    if (!camera.Init())
//    {
//        return -1;
//    }
//
////     打开相机
//    if (!camera.Open())
//    {
//        return -1;
//    }
//
////    // 创建风车识别器
////    ClassicalWindmillRecognizer recognizer;
////
////    // 加载风车识别器参数
////    ClassicalWindmillRecognizerParam recognizerParam;
////    std::string recognizerYaml = "config/infantry_3/basement/classical_windmill_recognizer_param.yaml";
////    if (!ClassicalWindmillRecognizerParam::LoadFromYamlFile(recognizerYaml, &recognizerParam))
////    {
////        return -1;
////    }
////
////    // 设置风车识别器参数
////    if (!recognizer.SetParam(recognizerParam))
////    {
////        return -1;
////    }
////
////    // 初始化风车识别器
////    if (!recognizer.Init())
////    {
////        return -1;
////    }
//
////     创建图像显示窗口
//    cv::namedWindow("rawimage", cv::WINDOW_NORMAL);
//    cv::resizeWindow("rawimage", 800, 600);
////    cv::namedWindow("image", cv::WINDOW_NORMAL);
////    cv::resizeWindow("image", 800, 600);
//
//    // 记录初始化时间戳
//    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
//    uint64_t initTimestamp = initTime.time_since_epoch().count();
//
//    // 初始化数据帧索引和风车扇叶索引
//    uint64_t frameIndex = 0;
//
//    std::vector< std::pair<float, float> > sin_datas;
//    std::vector<float> angle;
//
//    // 获取相机数据
//    HuarayCameraData cameraData;
//    camera.GetData(&cameraData);
//
//    // 循环处理相机数据
//    while (true)
//    {
//        // 记录起始时间戳
//        std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
//        uint64_t beginTimestamp = beginTime.time_since_epoch().count();
//
//
//
////        cv::imshow("rawimage",cameraData.Image);
//
//        // 累加数据帧索引
//        frameIndex++;
//
////        // 原始图像预处理
////        cv::Mat binaryImageFan;
////        cv::Mat binaryImageTarget;
////        recognizer.Preprocess(cameraData.Image, &binaryImageFan, &binaryImageTarget);
////
//////        cv::imshow("binaryImageFan",binaryImageFan);
//////        cv::imshow("binaryImageTarget",binaryImageTarget);
////
////        // 检测风车扇叶集合
////        ClassicalWindmillFan windmillFan;
////        std::vector<ClassicalWindmillFan> targetContours;
////        recognizer.DetectWindmillFans(binaryImageFan, binaryImageTarget, cameraData.Image, &windmillFan, &targetContours);
////
//////        std::cout << " ---------------targetContours_size--------------- " << targetContours.size() << std::endl;
////
////        cv::Mat src(cameraData.Image);
////        // 检索二值图像中的所有轮廓，并填充轮廓的索引结构
////        // cv::findContours()函数使用说明详见：https://www.cnblogs.com/wojianxin/p/12602490.html
////        std::vector<cv::Vec4i> hierarchyFan;
////        std::vector<std::vector<cv::Point>> contoursFan;
////        cv::findContours(binaryImageFan,
////                         contoursFan,
////                         hierarchyFan,
////                         cv::RETR_TREE,
////                         cv::CHAIN_APPROX_NONE,
////                         cv::Point(0, 0));
////
////        cv::Mat tempimag(src.rows,src.cols,CV_8UC1,cv::Scalar(0));
////
//////        for ( auto i : contoursFan )
//////            for ( auto j : i ){
//////
//////                tempimag.at<uchar>(j.y,j.x) = 255;
//////            }
////
////            int i = 5;
////            for (int j = 0 ; j < contoursFan[i].size() ; j ++){
////
////                tempimag.at<uchar>(contoursFan[i][j].y,contoursFan[i][j].x) = 255;
////            }
////
////        cv::imshow(" test_findContours ",tempimag);
////
////
////        // 读取按键的ASCII码；注意：cv::waitKey()返回的是按键的ASCII码
////        int keyValue = cv::waitKey(0);
////
////        // 如果按下ESC键，退出系统
////        if (keyValue == 27)
////        {
////            break;
////        }
////
//////         如果按下Enter键，暂停系统
////        if (keyValue == 13)
////        {
////            cv::waitKey(0);
////        }
//
//        cv::imshow ("rawimage",cameraData.Image);
//
//        int keyValue = cv::waitKey(0);
//
//        // 如果按下ESC键，退出系统
//        if (keyValue == 27)
//        {
//            break;
//        }
//
//    }
//
//    // 关闭相机
//    camera.Close();
//
//    // 释放相机资源
//    camera.Release();
//
//    // 释放风车识别器资源
////    recognizer.Release();
//
//    std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
//    uint64_t endTimestamp = endTime.time_since_epoch().count();
//
//    std::cout << "  --------------fdex--------------  " << frameIndex * 1000000000.0f / (endTimestamp - initTimestamp);
//
//    return 0;
//
//}