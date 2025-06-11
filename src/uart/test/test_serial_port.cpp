#include <iostream>
#include "serial_port_param.h"
#include "serial_port.h"
#include "predictor_command.h"
#include "predictor_EKF.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"
// 串口数据接收缓冲区
std::deque<PredictorCommand> BodyPredictorCommand;

struct
{
    ClassicalArmor Armor;
    cv::Point3f Armor_Center;
    float yaw;
} Armor_Data;
struct
{
    float q[4];
    std::vector<Armor_Data> armors;
} Armor_Infor;

std::deque<Armor_Infor> armorInfor;

int cnt, FPS;
// 串口数据接收回调函数
void HandleDataReceived(const unsigned int &bytesToRead, void *userData)
{
    unsigned char frame[200], frameSize;
    auto *serialPort = (SerialPort *)userData;
    while (frameSize = serialPort->ReadLine(frame))
    {
        if ((frameSize == 21) && (frame[0] == 0xAA))
        {
            cnt++;
            PredictorCommand predictCommand;
            if (PredictorCommand::Parse(&frame[2], &predictCommand))
            {
                if (BodyPredictorCommand.size() >= 20)
                    BodyPredictorCommand.pop_front();
                BodyPredictorCommand.push_back(predictCommand);
            }
            else
                std::cout << "PredictorCommond parsed was failed." << std::endl;
        }
        else
            std::cout << "frameSize was failed." << std::endl;
    }
}

//// ******************************  RobotBrainCore类的私有函数  ******************************
void GetArmorCenterLocation(Armor_Infor &armorInfor, const HuarayCameraModelParam &modelParam)
{
    float laWidth, laHeight = 48;
    for (const auto &armor : armorInfor.armors)
    {
        if (armor.Number == EClassicalArmorNumber::BlueOne || armor.Number == EClassicalArmorNumber::RedOne)
            laWidth = 240 - 10;
        else
            laWidth = 134 - 10;

        std::vector<cv::Point3f> objectPoints;
        objectPoints.emplace_back(-0.5 * laWidth, 0.5 * laHeight, 0);
        objectPoints.emplace_back(0.5 * laWidth, 0.5 * laHeight, 0);
        objectPoints.emplace_back(0.5 * laWidth, -0.5 * laHeight, 0);
        objectPoints.emplace_back(-0.5 * laWidth, -0.5 * laHeight, 0);

        std::vector<cv:armor : Point2f> pixelPoints;
        pixelPoints.emplace_back(armor.LeftUpper);
        pixelPoints.emplace_back(armor.RightUpper);
        pixelPoints.emplace_back(armor.RightLower);
        pixelPoints.emplace_back(armor.LeftLower);

        cv::Mat rotation(3, 1, CV_32F), translation(3, 1, CV_32F);
        cv::solvePnP(objectPoints, pixelPoints,
                     modelParam.CvIntrinsics, modelParam.CvDistortions,
                     rotation, translation,
                     false,
                     cv::SOLVEPNP_IPPE);

        cv::Mat Change_matrix = (cv::Mat_<float>(3, 3) << 1, 0, 0,
                                 0, 0, 1,
                                 0, -1, 0),
                rotation_matrix;
        cv::Rodrigues(rotation, rotation_matrix); // 内部自动计算 θ 和 n

        float w = armorInfor.q[0], x = armorInfor.q[1], y = armorInfor.q[2], z = armorInfor.q[3];
        cv::Mat Q = (cv::Mat_<float>(3, 3) << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w,
                     2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w,
                     2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y);

        std::vector<cv::Point3f> cameraCorners;
        for (const auto &pt : objectPoints)
        {
            cv::Mat pt_obj = (cv::Mat_<float>(3, 1) << pt.x, pt.y, pt.z);
            cv::Mat pt_cam = R * pt_obj + translation;
            pt_cam = Q * Change_matrix * pt_cam;
            cameraCorners.emplace_back(pt_cam.at<float>(0), pt_cam.at<float>(1), pt_cam.at<float>(2));
        }

        cv::Mat pt_Center = Q * Change_matrix * translation;
        cv::Point3f Center(pt_Center.at<float>(0), pt_Center.at<float>(1), pt_Center.at<float>(2));

        armor.Armor_Center = Center;
    }
}

// Linux命令行列出所有的串口设备：dmesg | grep ttyS*
int main(int, char *[])
{
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init(); // 初始化日志记录器

    SerialPort serialPort;
    if (!serialPort.init(serialPort, "config/infantry_3/basement/serial_port_param.yaml", HandleDataReceived))
        return -1;

    HuarayCameraParam cameraParam; // 读取相机参数
    if (!HuarayCameraParam::LoadFromYamlFile("config/infantry_3/basement/huaray_camera_param.yaml", &cameraParam))
        return -1;
    HuarayCamera camera;
    if (!camera.SetParam(cameraParam))
        return -1; // 设置相机参数
    if (!camera.Init())
        return -1; // 初始化相机
    if (!camera.Open())
        return -1; // 打开相机

    ClassicalArmorRecognizer recognizer;           // 创建装甲板识别器
    ClassicalArmorRecognizerParam recognizerParam; // 读取装甲板识别器参数
    std::string recognizerYaml = "config/infantry_3/basement/classical_armor_yolo_openvino.yaml";
    if (!ClassicalArmorRecognizerParam::LoadFromYamlFile(recognizerYaml, &recognizerParam))
        return -1;
    if (!recognizer.SetParam(recognizerParam))
        return -1; // 设置装甲板识别器参数
    if (!recognizer.Init())
        return -1; // 初始化装甲板识别器

    cv::namedWindow("rawImage", cv::WINDOW_NORMAL); // 创建图像显示窗口
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
            armor_infor.q[i] = BodyPredictorCommand.back().Q[i];

        // 累加数据帧索引
        frameIndex++;

        recognizer.DetectArmorsByYolo(cameraData.Image, armor_infor.Armor);
        GetArmorCenterLocation(armor_infor, cameraParam.ModelParam);

        //  if (armor_infor.Armor.size()>=1)  {

        armor_infor.yaw = Remap::PhiOptimization(armor_infor.Armor_Center, cameraParam.ModelParam, armor_infor.Armor.back(), 1);
        //     std::cout<<armor_infor.yaw*57.3<<std::endl;
        //    }
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
        cv::waitKey(1);
    }

    // 关闭串口
    serialPort.Close();

    // 释放串口资源
    serialPort.Release();

    return 0;
}

// #include <ceres/ceres.h>

// struct PnPCostFunctor
// {
//     const cv::Point2f pixel_point;
//     const cv::Point3f object_point;
//     const cv::Mat &camera_matrix;
//     const cv::Mat &dist_coeffs;
//     const cv::Mat &R_x; // Roll=75°的固定旋转矩阵

//     PnPCostFunctor(const cv::Point2f &pixel, const cv::Point3f &obj,
//                    const cv::Mat &cam, const cv::Mat &dist, const cv::Mat &Rx)
//         : pixel_point(pixel), object_point(obj), camera_matrix(cam), dist_coeffs(dist), R_x(Rx) {}

//     template <typename T>
//     bool operator()(const T *const yaw, const T *const tvec, T *residuals) const
//     {
//         // 构造旋转矩阵 R = R_z(yaw) * R_x(75°)
//         T R_z[9] = {
//             cos(yaw[0]), -sin(yaw[0]), T(0),
//             sin(yaw[0]), cos(yaw[0]), T(0),
//             T(0), T(0), T(1)};
//         T R[9];
//         ceres::MatrixAdapter<T, 3, 3>(R).assign(
//             ceres::MatrixAdapter<T, 3, 3>(R_z).data() *
//             ceres::MatrixAdapter<T, 3, 3>(R_x).data());

//         // 投影模型点到图像平面
//         T point[3] = {T(object_point.x), T(object_point.y), T(object_point.z)};
//         T projected[2];
//         ceres::ProjectPointToCamera(point, R, tvec,
//                                     camera_matrix.ptr<double>(), dist_coeffs.ptr<double>(),
//                                     projected);

//         // 计算残差
//         residuals[0] = projected[0] - T(pixel_point.x);
//         residuals[1] = projected[1] - T(pixel_point.y);
//         return true;
//     }
// };

// void optimizePoseWithFixedRoll(
//     const std::vector<cv::Point3f> &object_points,
//     const std::vector<cv::Point2f> &pixel_points,
//     const cv::Mat &camera_matrix,
//     const cv::Mat &dist_coeffs,
//     double &optimized_yaw,
//     cv::Mat &optimized_tvec)
// {
//     // 固定 Roll=75° 的旋转矩阵
//     cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0,
//                    0, cos(75 * CV_PI / 180), -sin(75 * CV_PI / 180),
//                    0, sin(75 * CV_PI / 180), cos(75 * CV_PI / 180));

//     // 初始猜测
//     double yaw = 0;
//     double tvec[3] = {0, 0, 0};

//     // Ceres 优化问题
//     ceres::Problem problem;
//     for (size_t i = 0; i < object_points.size(); ++i)
//     {
//         ceres::CostFunction *cost =
//             new ceres::AutoDiffCostFunction<PnPCostFunctor, 2, 1, 3>(
//                 new PnPCostFunctor(pixel_points[i], object_points[i],
//                                    camera_matrix, dist_coeffs, R_x));
//         problem.AddResidualBlock(cost, nullptr, &yaw, tvec);
//     }

//     // 求解
//     ceres::Solver::Options options;
//     options.linear_solver_type = ceres::DENSE_QR;
//     ceres::Solver::Summary summary;
//     ceres::Solve(options, &problem, &summary);

//     // 输出结果
//     optimized_yaw = yaw;
//     optimized_tvec = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]);
// }

// std::vector<cv::Point3f> object_points = /* 4个共面3D点 */;
// std::vector<cv::Point2f> pixel_points = /* 对应的2D像素点 */;
// cv::Mat camera_matrix = /* 相机内参 */;
// cv::Mat dist_coeffs = /* 畸变系数 */;

// double yaw;
// cv::Mat tvec;
// optimizePoseWithFixedRoll(object_points, pixel_points, camera_matrix, dist_coeffs, yaw, tvec);

// // 最终旋转矩阵 = R_z(yaw) * R_x(75°)
// cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0,
//                sin(yaw), cos(yaw), 0,
//                0, 0, 1);
// cv::Mat R_x = /* Roll=75°的矩阵 */;
// cv::Mat final_rotation = R_z * R_x;