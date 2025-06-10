//
// Created by ly on 24-1-22.
//

#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include <iostream>
#include "solver.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"
#include "predictor_tool.h"
#include "predictor_EKF.h"
#include "predictor_AntiTop.h"

int main(int argc, char *argv[])
{
    cv::VideoCapture capture("/home/ly/video0/video_1.avi");


//    if (!capture.isOpened()) {
//        std::cout <<"1" <<std::endl;
//        std::cout << "Error opening video file!" << std::endl;
//        return -1;
//    }

    HuarayCameraParam cameraParam;
    std::string cameraYaml = "config/infantry_3/basement/huaray_camera_param.yaml";
    if (!HuarayCameraParam::LoadFromYamlFile(cameraYaml, &cameraParam))
    {
        return -1;
    }

    HuarayCameraModelParam modelParam = cameraParam.ModelParam;

    ClassicalArmorRecognizer recognizer;

    // 读取装甲板识别器参数
    ClassicalArmorRecognizerParam recognizerParam;
    std::string recognizerYaml = "config/infantry_3/basement/classical_armor_yolo.yaml";
    if (!ClassicalArmorRecognizerParam::LoadFromYamlFile(recognizerYaml, &recognizerParam))
    {
        return -1;
    }

    // 设置装甲板识别器参数
    if (!recognizer.SetParam(recognizerParam))
    {
        return -1;
    }

    if(!recognizer.Init())
    {
        return -1;
    }


    cv::namedWindow("img",cv::WINDOW_NORMAL);
    cv::resizeWindow("img",cv::Size(800,600));

    //由于读取较慢，所以采用抽帧的方法来加快视频
    int frameStep = 4;

    AntiTop antiTop;

    while (true) {
        // 读取视频的一帧
        cv::Mat frame;
        capture >> frame;


        // 检查是否成功读取帧
        if (frame.empty()) {
            std::cout << "End of video!" << std::endl;
            break;
        }

        std::vector<ClassicalArmor> Armors;
        recognizer.DetectArmorsByYolo(frame, Armors);

        std::vector<ClassicalArmor> armorGroup;
        EClassicalArmorNumber number = EClassicalArmorNumber::BlueThree;
        antiTop.targetNumber = number;

        armorGroup = antiTop.getTargetArmor(Armors);
        int armorGroupSize = armorGroup.size();

        std::vector<Eigen::Vector3d> centerPts;
        std::vector<float> yaws;
        for (int i = 0; i < armorGroupSize; ++i) {
            cv::Point3f center;
            centerPts.emplace_back( PredictorTool::getArmorCenterLocation(armorGroup[i],modelParam));
            PredictorTool::eigenVec3d2cvPt3f(centerPts[i],&center);
            std::cout << "Pts"<< center << std::endl;
            yaws.emplace_back(Remap::findAccurateYaw(center,modelParam,armorGroup[i]));
        }

        ClassicalArmor targetArmor ;
        if(armorGroupSize == 2)
            targetArmor = antiTop.chooseShootArmor(armorGroup,yaws[0],yaws[1]);
        else if(armorGroupSize == 1)
            targetArmor = antiTop.chooseShootArmor(armorGroup,yaws[0]);

//        std::cout << "Location: " << targetArmor.Center <<std::endl;
//        std::cout << "Pts"<< centerPts[0] << std::endl;
        cv::circle(frame,targetArmor.Center,4,cv::Scalar (0,255,255),8);



//        PredictorTool::getArmorCenterLocation();
//        Remap::findAccurateYaw();

// 跳过指定的帧数
        for (int i = 0; i < frameStep - 1; ++i) {
            // 跳过帧
            capture.grab();
        }

        cv::imshow("img",frame);

        int KeyValue = cv::waitKey(10);

        if(KeyValue == 27)
        {
            break;
        }

        if(KeyValue == 13)
        {
            cv::waitKey(0);
        }



        // 检查视频是否结束
        if (!capture.isOpened() && capture.get(cv::CAP_PROP_POS_FRAMES) == capture.get(cv::CAP_PROP_FRAME_COUNT)) {
            std::cout << "Video playback completed." << std::endl;
        }

    }

    capture.release();
    return 0;


}