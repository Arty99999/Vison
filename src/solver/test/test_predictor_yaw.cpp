//
// Created by ly on 23-7-15.
//

#include  <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "solver.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"
#include "predictor_tool.h"
#include "predictor_EKF.h"



int main()
{
    //初始化日志记录
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    //创建相机
    HuarayCamera Camera;

    //
//    cv::VideoCapture capture("/home/ly/video0/video_3.avi");

    //读取相机参数
    HuarayCameraParam CameraParam;
    std::string yamlFile = "config/infantry_3/basement/huaray_camera_param.yaml";
    if (!CameraParam.LoadFromYamlFile(yamlFile, &CameraParam))
    {
        return -1;
    }

    //设置相机参数
    if(!Camera.SetParam(CameraParam))
    {
        return -1;
    }
//
    //相机初始化
    if(!Camera.Init())
    {
        return -1;
    }

    //开启相机
    if(!Camera.Open())
    {
        return -1 ;
    }

    //创建装甲板识别器
    ClassicalArmorRecognizer recognizer;

    //读取识别器参数
    ClassicalArmorRecognizerParam recognizerParam;
    std::string recognizeryaml = "config/infantry_3/basement/classical_armor_yolo_openvino.yaml";
    if(!recognizerParam.LoadFromYamlFile(recognizeryaml,&recognizerParam))
    {
        return -1;
    }

    //设置识别器参数
    if(!recognizer.SetParam(recognizerParam))
    {
        return -1;
    }

    //初始化识别器
    if(!recognizer.Init())
    {
        return -1;
    }

    //创建solver
    Solver solver;

    //读取solver参数
    SolverParam solverParam;
    std::string solveryaml = "config/infantry_3/basement/huaray_camera_param.yaml";
    if(!SolverParam::LoadFromYamlFile(solveryaml,&solverParam))
    {
        return -1;
    }

    //设置solver参数
    if(!solver.SetParam(solverParam))
    {
        return -1;
    }

    //初始化solver
    if(!solver.Init())
    {
        return -1;
    }


    //创建图像窗口
    cv::namedWindow("rawImage",cv::WINDOW_NORMAL);
    cv::resizeWindow("rawImage",800,600);
    cv::namedWindow("polishedImage",cv::WINDOW_NORMAL);
    cv::resizeWindow("polishedImage",800,600);


    //记录初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
    uint64_t initTimestamp = initTime.time_since_epoch().count();

    //初始化数据帧索引
    uint64_t frametime = 0;

    std::vector<cv::Point2f> fourp;
    double consequence;
    bool flag = false;

    while(1)
    {
        //记录开始时间戳
        std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
        uint64_t startTimestamp = startTime.time_since_epoch().count();

        //获取相机数据
        HuarayCameraData CameraData;
        Camera.GetData(&CameraData);
//        cv::Mat frame;
//        capture >> frame;

        //数据帧累加
        frametime++;

        //图像处理
        std::vector<ClassicalArmor> Armors;
        recognizer.DetectArmorsByYolo(CameraData.Image, Armors);

        //判断是否存在有效装甲板
        bool armor = 0;
        ClassicalArmor validArmor;
        for (int i = 0; i < Armors.size(); i++)
        {
            if(Armors[i].Number != EClassicalArmorNumber::Invalid)
            {
                armor = true;
                flag = true;
                validArmor = Armors[i];
                break;
            }
        }


        //装甲板世界坐标
        std::vector<cv::Point3f> objectPoints;

        if(armor)
        {
            if(validArmor.Center.x > 200 && validArmor.Center.x < 1000 ||
               validArmor.Center.y > 200 && validArmor.Center.y < 900)
            {
                Remap::IsSuitable = true;
            }
            //大装甲板
//            if(validArmor.Number == EClassicalArmorNumber:: BlueSentry or validArmor.Number == EClassicalArmorNumber:: RedSentry
//            or validArmor.Number == EClassicalArmorNumber:: BlueOne    or validArmor.Number == EClassicalArmorNumber:: RedOne)
            if( validArmor.Number == EClassicalArmorNumber:: BlueSentry or validArmor.Number == EClassicalArmorNumber::RedSentry||
                validArmor.Number == EClassicalArmorNumber::RedTwo or validArmor.Number == EClassicalArmorNumber::BlueTwo)
            {
                float laWidth = recognizerParam.LargeArmorPhysicalWidth -  recognizerParam.LightBarPhysicalWidth;
                float laHeight = 55.0;
                //返回四角点世界坐标(顺序：左上，右上，右下，左下)
                objectPoints.emplace_back( -0.5*laWidth, 0.5*laHeight, 0);
                objectPoints.emplace_back( 0.5*laWidth, 0.5*laHeight, 0);
                objectPoints.emplace_back( 0.5*laWidth, -0.5*laHeight, 0);
                objectPoints.emplace_back( -0.5*laWidth, -0.5*laHeight, 0);
            }
                //小装甲板
            else
            {
                float loWidth =  recognizerParam.SmallArmorPhysicalWidth - recognizerParam.LightBarPhysicalWidth;
                float loHeight = 50.0;
                //返回四角点世界坐标(顺序：左上，右上，右下，左下)
                objectPoints.emplace_back( -0.5*loWidth, 0.5*loHeight, 0);
                objectPoints.emplace_back( 0.5*loWidth, 0.5*loHeight, 0);
                objectPoints.emplace_back( 0.5*loWidth, -0.5*loHeight, 0);
                objectPoints.emplace_back( -0.5*loWidth, -0.5*loHeight, 0);
            }
            //像素坐标
            std::vector<cv::Point2f> pixelPoints;
            //返回四角点像素坐标(顺序：左上，右上，右下，左下)
            pixelPoints.emplace_back(validArmor.LeftUpper);
            pixelPoints.emplace_back(validArmor.RightUpper);
            pixelPoints.emplace_back(validArmor.RightLower);
            pixelPoints.emplace_back(validArmor.LeftLower);


            //返回装甲板距离距离
            cv::Mat rotationMat;
            cv::Mat translationMat;
            float Distance = Solver::ComputeDistance( objectPoints,pixelPoints,CameraParam.ModelParam).first;
            cv::solvePnP(objectPoints,
                         pixelPoints,
                         CameraParam.ModelParam.CvIntrinsics,
                         CameraParam.ModelParam.CvDistortions,
                         rotationMat,
                         translationMat);
            std::vector<cv::Point3f> armorLocation;
            PredictorTool::pnpArray(CameraParam.ModelParam,Armors[0],&armorLocation);
            cv::Point3f LC,RC;

            LC = (armorLocation[0] + armorLocation[3]) / 2;     RC = (armorLocation[1] + armorLocation[2]) / 2;

            float arrayYaw = atan((LC.z - RC.z) / (LC.x - RC.x));

//            std::cout << translationMat << std::endl;
//            std::cout << pixelPoints <<std::endl;
//            世界坐标类型转换(向量->矩阵)
            cv::Mat matObjectPoints;
            matObjectPoints = cv::Mat(objectPoints);

//            坐标转换为相机坐标ua
            std::vector<cv::Point3f> cameraPoints;
//            cameraPoints = cv::Mat_<cv::Point3f>(rotationMat * matObjectPoints + translationMat);

            cv::Point3f p1;
//            std::cout << translationMat << std::endl;

            //
            p1.x = translationMat.at<double>(0,0);
            p1.y = translationMat.at<double>(1,0);
            p1.z = translationMat.at<double>(2,0);
            float yaw  = 0;
            if(Remap::IsSuitable) {
//                yaw = Remap::gradientdescent(p1, CameraParam.ModelParam, Armors[0]);
//                yaw = Remap::test_gd(p1, CameraParam.ModelParam, Armors[0]);

//                std::cout << "-------------------  " << yaw * 57.3 <<std::endl;

//                yaw = Remap::findAccurateYaw(p1,CameraParam.ModelParam,Armors[0]);
                yaw = Remap::PhiOptimization(p1,CameraParam.ModelParam,Armors[0]);
//                Remap::Compare(yaw,CameraParam.ModelParam,p1,Armors[0]);
            }



            if(flag) {
                fourp = Remap::Compare(yaw, CameraParam.ModelParam,  p1, Armors[0]).second;
                consequence = yaw;
            }

            std::vector<cv::Point2i> pixelP;
            std::vector<cv::Point2i> fourP;
            for(int  o =0;o<4;o++)
            {
                pixelP.emplace_back(static_cast<cv::Point2i>(pixelPoints[o]));
                fourP.emplace_back(static_cast<cv::Point2i>(fourp[o]));

            }
//            float IOU =ekf.drawAndComputeIOU(pixelP,fourP);

            float pnpyaw;
            pnpyaw = rotationMat.at<double>(2,0);
//            pnpyaw2 = std::atan2(-rotationMat.at<float>(1,0),-rotationMat.at<float>(0,0));
            float coe ;
            float com_yaw =  0.3 * arrayYaw + 0.7  * yaw;


//            std::cout << rotationMat.at<double>(2,0) << std::endl;

            //记录结束时间戳CameraParam.ModelParam,
            std::chrono::time_point<std::chrono::steady_clock> endTime =std::chrono::steady_clock::now();
            uint64_t endTimestamp = endTime.time_since_epoch().count();

            //输出结果
            std::cout << "**************************************************"<< std::endl;
//            std::cout << pixelPoints << std::endl;
//            std::cout <<  sqrt(pow(pixelPoints[0].x - pixelPoints[1].x , 2) + pow((pixelPoints[0].y - pixelPoints[1].y) , 2)) / sqrt(pow(pixelPoints[1].x - pixelPoints[2].x,2) + pow(pixelPoints[1].y - pixelPoints[2].y,2))<<std::endl;
//            std::cout << sqrt(pow(pixelPoints[1].x - pixelPoints[2].x,2) + pow(pixelPoints[1].y - pixelPoints[2].y,2)) /45 *(pixelPoints[0].y - pixelPoints[1].y)<< std::endl;
//            std::cout << "IOU: " << IOU <<std::endl;
//            std::cout <<"weight: " << abs(polishedLightBars[0].Center.x - polishedLightBars[1].Center.x) << std::endl;
//            std::cout <<"fourp:  " <<fourp << std::endl;
//            std::cout << "gyaw" << yaw * 57.3 << std::endl;
            std::cout << "pnpyaw" << pnpyaw * 57.3 << std::endl;
            std::cout <<"arrayaw: " << arrayYaw * 57.3 <<std::endl;
            std::cout << "complex: " << com_yaw*57.3 <<std::endl;
//            std::cout << "frame Index:"<< frametime <<std::endl;
//            std::cout << "fps:" << (frametime * 1000000000) / (endTimestamp - initTimestamp) <<std::endl;
//            std::cout << "d
//            istance of target armor:" << Distance << std::endl;
//            std::cout << "camera coordinate system points:" << cameraPoints <<std::endl;
            //std::cout << "process time:" << (endTimestamp - startTimestamp) * 1000000 <<std::endl;
            std::cout << "**************************************************"<< std::endl;

        }
        //创建普通装甲板图像
        cv::Mat polishedImage;
        ClassicalArmorRecognizer::CreateCommonArmorsImage(Armors,CameraData.Image,&polishedImage);
        if(!fourp.empty())
        {
            for (int i = 0; i < 3; ++i) {
                cv::line(polishedImage, static_cast<cv::Point2i>(fourp[i]), static_cast<cv::Point2i>(fourp[i + 1]),
                         cv::Scalar(255, 0, 0),8);
            }
            cv::line(polishedImage, static_cast<cv::Point2i>(fourp[3]), static_cast<cv::Point2i>(fourp[0]),
                     cv::Scalar(255, 0, 0),8);
        }
        //显示图像
        cv::imshow("rawImage",CameraData.Image);
        cv::imshow("polishedImage",polishedImage);

        int KeyValue = cv::waitKey(10);

        if(KeyValue == 27)
        {
            break;
        }

        if(KeyValue == 13)
        {
            cv::waitKey(0);
        }

    }
    //关闭相机
    Camera.Close();
    //释放相机资源
    Camera.Release();
    //释放识别器资源
    recognizer.Release();
    //释放solver资源
    solver.Release();
    //释放ekf资源
    return 0;
}
