#include <iostream>
#include "serial_port_param.h"
#include "serial_port.h"
#include "predictor_command.h"
#include "predictor_EKF.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"
// 串口数据接收缓冲区
std::deque<PredictorCommand> BodyPredictorCommand;
typedef struct {
    float q[4];
    std::vector<ClassicalArmor> Armor;
    cv::Point3f Armor_Center;
    float yaw;
}Armor_Infor;

std::deque<Armor_Infor> armorInfor;

int cnt,FPS;
// 串口数据接收回调函数
void HandleDataReceived(const unsigned int &bytesToRead, void* userData)
{
    unsigned char frame[200], frameSize;
    auto *serialPort = (SerialPort *)userData;
    while (frameSize = serialPort->ReadLine(frame)) {
        if ((frameSize == 21) && (frame[0] == 0xAA)) {
            cnt++;
            PredictorCommand predictCommand;
            if (PredictorCommand::Parse(&frame[2], &predictCommand)) {
                if (BodyPredictorCommand.size() >=20) BodyPredictorCommand.pop_front();
                BodyPredictorCommand.push_back(predictCommand);
            } else std::cout << "PredictorCommond parsed was failed." << std::endl;
        } else std::cout << "frameSize was failed." << std::endl;
    }


}

//// ******************************  RobotBrainCore类的私有函数  ******************************
std::vector<cv::Point3f> GetArmorCenterLocation(Armor_Infor& armorInfor)
{
    float laWidth = 134 - 10, laHeight = 48;
//134-10
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> pixelPoints;
    cv::Mat rotation(3, 1, CV_32F),translation(3, 1, CV_32F);
    objectPoints.emplace_back(-0.5 * laWidth, 0.5 * laHeight, 0);
    objectPoints.emplace_back(0.5 * laWidth, 0.5 * laHeight, 0);
    objectPoints.emplace_back(0.5 * laWidth, -0.5 * laHeight, 0);
    objectPoints.emplace_back(-0.5 * laWidth, -0.5 * laHeight, 0);
    pixelPoints.emplace_back(armorInfor.Armor.back().LeftUpper);
    pixelPoints.emplace_back(armorInfor.Armor.back().RightUpper);
    pixelPoints.emplace_back(armorInfor.Armor.back().RightLower);
    pixelPoints.emplace_back(armorInfor.Armor.back().LeftLower);
    cv::Mat CvIntrinsics = (cv::Mat_<float>(3, 3) << 3538.4, 0.000000000, 630.1781,
            0.000000000, 3526.7, 525.5381,
            0.000000000, 0.000000000, 1.000000000 );
    //huarayCamera_.GetParam().ModelParam
    cv::Mat CvDistortions =  (cv::Mat_<float>(1, 5)<< -0.08688139531, -0.07194527501, -0.001206271983, 0.003265832671, 0.7037082012);
    // cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 1000, 1e-8);
    cv::solvePnP(objectPoints,
                 pixelPoints,
                 CvIntrinsics,
                 CvDistortions,
                 rotation,
                 translation,
                 false,
                 cv::SOLVEPNP_IPPE);
    cv::Mat R;
    cv::Mat R1 = (cv::Mat_<float>(3, 3) << 1, 0, 0,
            0, 0, 1,
            0, -1, 0);
    //
    cv::Rodrigues(rotation, R); // 内部自动计算 θ 和 n

    //  cv::Point3f Center;
    // std::cout << "Rotation Vector (rvec): " << rotation.t() << std::endl;
    // std::cout << "Translation Vector (tvec): " << translation.t() << std::endl;
    // cv::Mat center = R.t()*(-translation) ;
    // cv::Mat center = translation;
    // Center.x = center.at<float>(0);
    // Center.y = center.at<float>(1);
    // Center.z = center.at<float>(2);
// Assuming armorInfor.q is a quaternion in the form [w, x, y, z]
    float w = armorInfor.q[0];
    float x = armorInfor.q[1];
    float y = armorInfor.q[2];
    float z = armorInfor.q[3];

    cv::Mat Q = (cv::Mat_<float>(3, 3) <<
                                       1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w,
            2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w,
            2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y);
    std::vector<cv::Point3f> cameraCorners;
    for (const auto &pt : objectPoints)
    {
        cv::Mat pt_obj = (cv::Mat_<float>(3, 1) << pt.x, pt.y, pt.z);
        cv::Mat pt_cam = R * pt_obj + translation;
        pt_cam = R1 * pt_cam;
        //pt_cam = Q* pt_cam;
        cameraCorners.emplace_back(pt_cam.at<float>(0), pt_cam.at<float>(1), pt_cam.at<float>(2));
    }
    cv::Point3f a=cameraCorners[2]-cameraCorners[3];
   //std::cout<<atan2(a.y,a.x)*57.3<<std::endl;
   //std::cout<<cameraCorners[2].y<<"   "<<cameraCorners[3].y<<"  "<<a.y<<std::endl;
   // std::cout<<a.x<<"   "<<a.y<<"   "<<a.z<<std::endl;
    translation=R1* translation;
   // translation=Q* translation;
    cv::Point3f Center(translation.at<float>(0),translation.at<float>(1),translation.at<float>(2));
    // R = R1 * R;
    armorInfor.Armor_Center=Center;
    return cameraCorners;
}

// Linux命令行列出所有的串口设备：dmesg | grep ttyS*
int main(int, char* [])
{
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();// 初始化日志记录器
    SerialPort serialPort; // 创建串口
    SerialPortParam serialPortParam;// 加载串口参数
    std::string yameFile = "config/infantry_3/basement/serial_port_param.yaml";
    if (!SerialPortParam::LoadFromYamlFile(yameFile, &serialPortParam)) return -1;
    if (!serialPort.SetParam(serialPortParam))return -1;// 设置串口参数
    serialPort.RegisterDataReceivedHandler(HandleDataReceived, &serialPort);// 注册串口的数据接收回调函数
    if (!serialPort.Init()) return -1;// 初始化串口
    if (!serialPort.Open()) return -1;// 打开串口

    HuarayCamera camera;
    HuarayCameraParam cameraParam; // 读取相机参数
    std::string cameraYaml = "config/infantry_3/basement/huaray_camera_param.yaml";

    if (!HuarayCameraParam::LoadFromYamlFile(cameraYaml, &cameraParam))    return -1;
    if (!camera.SetParam(cameraParam)) return -1; // 设置相机参数
    if (!camera.Init())return -1;// 初始化相机
    if (!camera.Open()) return -1;// 打开相机
    ClassicalArmorRecognizer recognizer; // 创建装甲板识别器
    ClassicalArmorRecognizerParam recognizerParam;    // 读取装甲板识别器参数
    std::string recognizerYaml = "config/infantry_3/basement/classical_armor_yolo_openvino.yaml";
    if (!ClassicalArmorRecognizerParam::LoadFromYamlFile(recognizerYaml, &recognizerParam))return -1;

    if (!recognizer.SetParam(recognizerParam))return -1; // 设置装甲板识别器参数
    if (!recognizer.Init()) return -1;    // 初始化装甲板识别器

    cv::namedWindow("rawImage",cv::WINDOW_NORMAL); // 创建图像显示窗口
    cv::resizeWindow("rawImage", 800, 600);


    // 记录初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
    uint64_t initTimestamp = initTime.time_since_epoch().count();

    // 初始化数据帧索引
    uint64_t frameIndex = 0;

    while (1)
    {
        std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
        uint64_t beginTimestamp = beginTime.time_since_epoch().count();
        // 获取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);
        Armor_Infor armor_infor;
        for (int i = 0; i < 4; i++)
            armor_infor.q[i]=BodyPredictorCommand.back().Q[i];

        // 累加数据帧索引
        frameIndex++;
//        std::vector<ClassicalArmor> Armors;
        recognizer.DetectArmorsByYolo(cameraData.Image, armor_infor.Armor);
       // if (armor_infor.Armor.size()>=1) std::cout<<armor_infor.Armor.back().RightLower.y<<std::endl;
        if (armor_infor.Armor.size()>=1)  {
            GetArmorCenterLocation(armor_infor);

        armor_infor.yaw = Remap::PhiOptimization(armor_infor.Armor_Center,cameraParam.ModelParam,armor_infor.Armor.back(),1);
            std::cout<<armor_infor.yaw*57.3<<std::endl;
           }
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();

//
        // 输出处理结果
//        std::cout << "**************************************************" << std::endl;
//        std::cout << "frame index: " << frameIndex << std::endl;
//        std::cout << "fps: " << (frameIndex * 1000000000) / (endTimestamp - initTimestamp) << std::endl;
//        std::cout << "process time: " << (endTimestamp - beginTimestamp) / 1000000 << "ms" << std::endl;
//        std::cout << "**************************************************" << std::endl << std::endl;

        // 显示图像
        cv::imshow("rawImage", cameraData.Image);


//        FPS=cnt;
//        cnt=0;
//        std::cout<<" FPS: "<<FPS<<std::endl;
        cv::waitKey(1) ;


    }

    // 关闭串口
    serialPort.Close();

    // 释放串口资源
    serialPort.Release();

    return 0;
}