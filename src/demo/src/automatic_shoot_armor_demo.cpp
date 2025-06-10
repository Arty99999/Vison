//
// Created by plutoli on 2022-05-16.
//

#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"
#include "solver.h"
#include "serial_port.h"
#include "robot_brain_control_command.h"
#include "predictor.h"
#include "predictor_command.h"
#include "robot_brain.h"
#include <shared_mutex>
#include <fstream>

// 串口数据接收缓冲区
unsigned char IMU_frame[30000];
unsigned char frame[524280];  // 524280
unsigned char IMU_data_buffer_length = 13;
// 大脑预测指令缓存区
std::vector<PredictorCommand> BodyPredictorCommand;
// IMU连接串口接收的回调函数 读取角度
void HandleIMUDataReceived(const unsigned int &bytesToRead, void* userData)
{
    // 读取请求数据帧
    auto serialPort = (SerialPort *) userData;
    static int8_t frameHeadMarker = 0;
    int8_t noFrameHeadCnt = 0;
    unsigned int frameSize = 0;
    // 只有在缓存区数据大于13时才读取缓存区。
    if(bytesToRead >= IMU_data_buffer_length)
    {
        if(frameHeadMarker == 0)
        {
            for(int cnt = 0; cnt < IMU_data_buffer_length; cnt++) IMU_frame[cnt]=0;
            frameSize = serialPort->Read(IMU_data_buffer_length, IMU_frame);
        }
        else
        {
            for(int cnt = 0; cnt < IMU_data_buffer_length; cnt++) IMU_frame[cnt]=0;
            frameSize = serialPort->Read(frameHeadMarker, IMU_frame);
        }
        for(int j = 0; j < IMU_data_buffer_length; j++)
        {
            // 当0x55出现在1-12个字节(索引0-11)里面，且后一个字节也是0x55,两个0x55,。
            if(IMU_frame[j] == 0x55 && IMU_frame[j+1] == 0x55)
                frameHeadMarker = j;
                // 当0字节是第二位时，
            else if((j == 0 && IMU_frame[j] == 0x55 && IMU_frame[j+1] != 0x55)||(j == 12 && IMU_frame[j] == 0x55) && IMU_frame[j-1] != 0x55)
                frameHeadMarker = 12;
            else
                noFrameHeadCnt++;
        }
        if(noFrameHeadCnt == 13)
            frameHeadMarker = 0;
        else
            frameHeadMarker = frameHeadMarker;
        // 解析并处理机器人本体的请求
        if ((frameSize == IMU_data_buffer_length) && (IMU_frame[0] == 85) && (IMU_frame[1] == 85))
        {
            float W = (float) ((int16_t) (IMU_frame[5] << 8) | IMU_frame[4]) / 32768;
            float X = (float) ((int16_t) (IMU_frame[7] << 8) | IMU_frame[6]) / 32768;
            float Y = (float) ((int16_t) (IMU_frame[9] << 8) | IMU_frame[8]) / 32768;
            float Z = (float) ((int16_t) (IMU_frame[11] << 8) | IMU_frame[10]) / 32768;
            Eigen::Quaterniond q(W, X, Y, Z);

            //经过测试，是很标准的右手系
            Eigen::Vector3d euler_t = q.toRotationMatrix().eulerAngles(2, 1, 0);
            //std::cout<<" YAW: "<<euler_t(0)*57.3<<" Pitch: "<<euler_t(1)*57.3<<" Roll: "<<euler_t(2)*57.3<<std::endl;

            // 创建预测器指令结构体
            PredictorCommand predictCommand;
            if (!PredictorCommand::Parse(q, &predictCommand))
            {
                std::cout << "PredictorCommond parsed was failed." << std::endl;
            }
            else
            {
                // 如果内存数据过多则更新
                if (BodyPredictorCommand.size() > 1000) {
                    // 则刷前半个容器
                    ::BodyPredictorCommand.erase(BodyPredictorCommand.begin(), BodyPredictorCommand.begin() + 500);

                    // 更新数据
                    ::BodyPredictorCommand.push_back(predictCommand);

                    std::cout << "erase half of vector" << std::endl;
                } else {
                    // 更新数据
                    ::BodyPredictorCommand.emplace_back(predictCommand);
//                            std::cout<<"There is new data"<<std::endl;

                }
            }
        }
    }
}


void HandleSerialPortDataReceived(const unsigned int &bytesToRead, void* userData) {
    // 获取机器人大脑指针和通信串口引用
    auto robotBrain = (RobotBrain *) userData;
    SerialPort &serialPort = robotBrain->GetSerialPort();

    static std::chrono::time_point<std::chrono::steady_clock> last_receiveTime;
    // 初始化数据帧缓冲区
    unsigned char frame[1000];

    std::vector<PredictorCommand> BodyPredictorCommand;
    // 读取并处理串口缓冲区中的所有数据帧
    while (true) {
        // 从串口缓冲区中读取一帧数据
        unsigned int frameSize = serialPort.ReadLine(frame);

        // 根据读取到的字节长度判断是否读取成功
        if (frameSize == 0) {
            break;
        }

        // 处理读取到的数据帧
        if ((frameSize == 21) && (frame[0] == 0xAA) && (frame[frameSize - 1] == 0xDD)) {

            // 获取读取数据时间
            std::chrono::time_point<std::chrono::steady_clock> receiveTime = std::chrono::steady_clock::now();
            uint64_t receiveTimestamp = receiveTime.time_since_epoch().count();

            std::chrono::duration<double> gap_time = receiveTime - last_receiveTime;

            //std::cout << "IMU两帧间隔为 : " << gap_time.count() * 1000 << "ms\n";
            last_receiveTime = receiveTime;
            ERequestType type;
            if (SystemConfigurator::ConvertToRequestType(frame[1], &type)) {
                // 解析机器人本体请求
                bool result = true;
                RobotBodyRequest request;
                request.Type = type;
                for (unsigned int i = 2; i < frameSize - 1; ++i) {
                    request.Datas.emplace_back(frame[i]);
                }

                // 处理机器人本体请求
                if (request.Type == ERequestType::PredictorCommond) {
                    if (request.Datas.size() != 18) {
                        result = false;
                        std::cout << "size of data is wrong" << std::endl;
                    }
                    if (result) {
                        PredictorCommand predictCommand;
                        if (!PredictorCommand::Parse(request.Datas, &predictCommand)) {
                            std::cout << "PredictorCommond parsed was failed." << std::endl;
                        } else {
                            // 如果内存数据过多则更新
                            if (BodyPredictorCommand.size() > 1000) {
                                // 则刷前半个容器
                                ::BodyPredictorCommand.erase(BodyPredictorCommand.begin(),
                                                             BodyPredictorCommand.begin() + 500);

                                // 更新数据
                                ::BodyPredictorCommand.push_back(predictCommand);

                                std::cout << "erase half of vector" << std::endl;
                            } else {
                                // 更新数据
                                ::BodyPredictorCommand.emplace_back(predictCommand);
//                            std::cout<<"There is new data"<<std::endl;

                            }
                        }
                    }
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    // 创建一个记录数据的txt文件
    //std::ofstream outfile("/home/xcx/cubot_brain_predictor-xcx/coord.txt");
    std::cout.precision(3);//限制输出精度 小数点后三位
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 创建相机
    HuarayCamera camera;

    // 读取相机参数
    HuarayCameraParam cameraParam;
    std::string cameraYaml = "config/infantry_3/basement/huaray_camera_param.yaml";
    if (!HuarayCameraParam::LoadFromYamlFile(cameraYaml, &cameraParam))
    {
        return -1;
    }

    // 设置相机参数
    if (!camera.SetParam(cameraParam))
    {
        return -1;
    }

    // 初始化相机
    if (!camera.Init())
    {
        return -1;
    }

    // 打开相机
    if (!camera.Open())
    {
        return -1;
    }

    // 创建装甲板识别器
    ClassicalArmorRecognizer armorRecognizer;

    // 读取装甲板识别器参数
    ClassicalArmorRecognizerParam armorRecognizerParam;
    std::string armorRecognizerYaml = "config/infantry_3/basement/classical_armor_recognizer_param.yaml";
    if (!ClassicalArmorRecognizerParam::LoadFromYamlFile(armorRecognizerYaml,
                                                         &armorRecognizerParam))
    {
        return -1;
    }

    // 设置装甲板识别器参数
    if (!armorRecognizer.SetParam(armorRecognizerParam))
    {
        return -1;
    }

    // 初始化装甲板识别器
    if (!armorRecognizer.Init())
    {
        return -1;
    }

    // 创建目标解算器
    Solver solver;

    // 读取目标解算器参数
    SolverParam solverParam;
    std::string solverYaml = "config/infantry_3/basement/solver_param.yaml";
    if (!SolverParam::LoadFromYamlFile(solverYaml, &solverParam))
    {
        return -1;
    }

    // 设置目标解算器参数
    if (!solver.SetParam(solverParam))
    {
        return -1;
    }

    // 初始化目标解算器
    if (!solver.Init())
    {
        return -1;
    }

    // 创建预测器
    Predictor predictor;

    // 读取预测器参数
    PredictorParam predictorParam;
    std::string predictorYaml = "config/infantry_3/basement/autoaim_predictor_param.yaml";
    if(!PredictorParam::LoadFromYamlFile(predictorYaml,&predictorParam))
    {
        return -1;
    }

    // 设置预测器参数
    if(!predictor.SetParam(predictorParam))
    {
        return -1;
    }

    // 初始化预测器
    if(!predictor.Init())
    {
        return -1;
    }

    //// 创建通信串口
    //SerialPort serialPort;
//
    //// 读取通信串口参数
    //SerialPortParam serialPortParam;
    //std::string serialPortYaml = "config/infantry_3/basement/serial_port_param.yaml";
    //if (!SerialPortParam::LoadFromYamlFile(serialPortYaml, &serialPortParam))
    //{
    //    return -1;
    //}
//
    //// 设置通信串口参数
    //if (!serialPort.SetParam(serialPortParam))
    //{
    //    return -1;
    //}
//
    //// 注册串口的数据接收回调函数
    //serialPort.RegisterDataReceivedHandler(HandleSerialPortDataReceived, &serialPort);
    //// 初始化通信串口
    //if (!serialPort.Init())
    //{
    //    return -1;
    //}
//
    //// 打开通信串口
    //if (!serialPort.Open())
    //{
    //    return -1;
    //}
    // 记录初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> initTime = std::chrono::steady_clock::now();
    uint64_t initTimestamp = initTime.time_since_epoch().count();

    // 初始化数据帧索引和上一帧相机数据时间戳
    unsigned int frameIndex = 0;
    uint64_t previousTimestamp = 0;
    bool isPredict = false;

    // 预测指令缓存的互斥锁
    std::mutex operateMutex_;

    // 创建装甲板记忆队列 和 当前帧下的装甲板
    std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber>> armorRecord;
    std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber> nowTarget;
    //像素坐标系下的打击点坐标  求出相机坐标系下的打击点的坐标 根据imu的四元数 直接求出角度
    int count1=0;
    // 循环处理相机数据
    while (true) {
        // 记录起始时间戳
        std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
        uint64_t beginTimestamp = beginTime.time_since_epoch().count();

        // 获取相机数据
        HuarayCameraData cameraData;
        camera.GetData(&cameraData);

        // 定义打包数据与打包成功标志位
        PredictorTool package;
        bool packageResult = false;


        // 判断读取到的相机数据是否已经处理完毕
        if (cameraData.Timestamp > previousTimestamp) {
            previousTimestamp = cameraData.Timestamp;
            frameIndex++;
            // 记录相机数据处理完毕时计算机设备的时间戳,因为外接相机后相机时间戳来自相机设备
            std::chrono::time_point<std::chrono::steady_clock> frameTime = std::chrono::steady_clock::now();
            uint64_t frameTimeStamp = frameTime.time_since_epoch().count();
            package.frameTimeStamp = frameTimeStamp;
            //std::cout<<"camera's timestamp :"<<package.frameTimeStamp <<std::endl;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 如果存在一定下位机数据则对数据进行打包
        if (BodyPredictorCommand.size() > 1) {
            // 在数据拷贝过程中对数据加锁
            operateMutex_.lock();
            // 相机的时间戳Timestamp是count()得到的
            //uint64_t timeStamp = cameraData.Timestamp;
            uint64_t timeStamp = package.frameTimeStamp;
            //打印相机时间戳
            //std::cout<<"Camera Timestamp: "<<timeStamp/1000000<<std::endl;

            std::vector<PredictorCommand> bodyPredictorCommand;
            bodyPredictorCommand.insert(bodyPredictorCommand.end(), BodyPredictorCommand.begin(),
                                        BodyPredictorCommand.end());
            // 清空被拷贝数据
            BodyPredictorCommand.clear();
            // 解除互斥锁
            operateMutex_.unlock();

            // 对数据进行打包，并更改打包标志位 timeStamp 相机的时间戳
            if (package.GetPackage(timeStamp, &bodyPredictorCommand)) {
                // 匹配成功则消除匹配数据与之前的数据
                packageResult = true;
            } else {
                // 匹配失败则更改标志位,若时间差距过大则清空Vector
                packageResult = false;
            }
        }
        // 数据帧索引累加
        frameIndex++;

        // 原始图像预处理
        cv::Mat binaryImage;
        armorRecognizer.Preprocess(cameraData.Image, &binaryImage);

        // 检测粗灯条
        std::vector<ClassicalLightBar> rawLightBars;
        armorRecognizer.DetectRawLightBars(binaryImage, &rawLightBars);

        // 检测精灯条
        std::vector<ClassicalLightBar> polishedLightBars;
        armorRecognizer.DetectPolishedLightBars(rawLightBars, &polishedLightBars);

        // 检测灯条对
        std::vector<std::pair<ClassicalLightBar, ClassicalLightBar>> lightBarPairs;
        armorRecognizer.DetectLightBarPairs(polishedLightBars, &lightBarPairs);

        // 检测粗装甲板
        std::vector<ClassicalArmor> rawArmors;
        armorRecognizer.DetectRawArmors(lightBarPairs, cameraData.Image, &rawArmors);

        // 检测精装甲板
        std::vector<ClassicalArmor> polishedArmors;
        armorRecognizer.DetectPolishedArmors(rawArmors, &polishedArmors);

        // 创建灯条图像
        cv::Mat lightBarImage;
        ClassicalArmorRecognizer::CreateLightBarsImage(polishedLightBars, cameraData.Image, &lightBarImage);

        // 绘制灯条图像
        cv::Mat lightBarPairsImage;
        ClassicalArmorRecognizer::CreateLightBarPairsImage(lightBarPairs, cameraData.Image, &lightBarPairsImage);

        // 创建精装甲板图像
        cv::Mat polishedArmorsImage;
        ClassicalArmorRecognizer::CreateCommonArmorsImage(polishedArmors,
                                                          cameraData.Image,
                                                          &polishedArmorsImage);

        // 显示当前二值化图像
        //cv::imshow("binaryImage", polishedArmorsImage);
        //cv::waitKey(1);

        // 检索目标装甲板
        int targetArmorIndex = -1;
        for (int i = 0; i < polishedArmors.size(); ++i) {
            if (polishedArmors[i].Number != EClassicalArmorNumber::Invalid) {
                targetArmorIndex = i;
                break;
            }
        }

        // 判断是否检索到目标装甲板
        if (targetArmorIndex < 0) {
            continue;
        }

        // 创建目标装甲板
        ClassicalArmor targetArmor;
        targetArmor.Center.x = -1;
        targetArmor.Center.y = -1;
        targetArmor = polishedArmors[targetArmorIndex];

        // 创建预测数据
        double predictTime = 0;
        std::pair<float, float> distanceCompensation(0.0, 0.0);         // 击打目标点的距离补偿

        // 计算目标装甲板顶点的世界坐标
        std::vector<cv::Point3f> objectPoints;
        float width = armorRecognizerParam.SmallArmorPhysicalWidth - 2 * armorRecognizerParam.LightBarPhysicalWidth;
        float height = armorRecognizerParam.SmallArmorPhysicalHeight;
        objectPoints.emplace_back(-width / 2, -height / 2, 0.0);
        objectPoints.emplace_back(width / 2, -height / 2, 0.0);
        objectPoints.emplace_back(width / 2, height / 2, 0.0);
        objectPoints.emplace_back(-width / 2, height / 2, 0.0);

        // 计算目标装甲板顶点的像素坐标
        std::vector<cv::Point2f> pixelPoints;
        pixelPoints.emplace_back(targetArmor.LeftUpper);
        pixelPoints.emplace_back(targetArmor.RightUpper);
        pixelPoints.emplace_back(targetArmor.RightLower);
        pixelPoints.emplace_back(targetArmor.LeftLower);

        // 计算目标装甲板到相机的距离
        float yaw;
        float pitch;
        float distance = Solver::ComputeDistance(objectPoints, pixelPoints, camera.GetParam().ModelParam);
        double ShootDelay = 0.3;
        auto bulletVelocity = 30.0;
        predictTime = distance / (1000 * bulletVelocity) + ShootDelay;

        // 如果打包成功则将本次数据记录进队列
        if (packageResult) {
            // 更新有效状态值
//            count1++;
//            std::cout<<count1<<std::endl;
            std::get<0>(nowTarget) = package.timeStamp;
            std::get<1>(nowTarget) = distance;
            std::get<2>(nowTarget) = targetArmor.Center;
            std::get<3>(nowTarget) = package.Q;
            std::get<4>(nowTarget) = targetArmor.Number;
            armorRecord.emplace_back();
            if (armorRecord.size() > 10) {
                armorRecord.pop_front();//删除容器的第一个数据
            }
        }
//
//        // 打包成功则进行预测
        isPredict = packageResult;
//
//        // 初始化相对预测坐标`
        Eigen::Vector3d predictrelativeCoordinate;
        Eigen::Vector3d predictworldCoordinate;
        Eigen::Vector3d nowworldCoordinate;
        cv::Point2f predictTarget;
        // 如果目标不为空，则进行预测与补偿
        if (targetArmor.Center.x != -1 && targetArmor.Center.y != -1) {

            if (1) {
                //predictor.AutoGetCoord(cameraParam.ModelPara
                //
                // m,nowTarget,&nowworldCoordinate,&QMatrix);
                // 若装甲板记忆队列大小大于2
                // 得到预测的二维坐标,如果不预测则得到原有的坐标
                predictor.AutoaimPredict(isPredict,
                                         predictor,
                                         cameraParam.ModelParam,
                                         &armorRecord,
                                         predictTime,
                                         &predictrelativeCoordinate,
                                         &predictworldCoordinate,
                                         &predictTarget);

                cv::Mat predictImage;


                std::pair<float, float> offsetAngle;
                solver.Getpitchandrow(predictrelativeCoordinate, predictworldCoordinate, &offsetAngle,solver);
                // 得到当前云台偏角
                yaw = offsetAngle.first;
                pitch = offsetAngle.second;
            }
            else
            {
                // 若装甲板记忆过短则直接解算,或者本次打包失败
                std::pair<float, float> offsetAngle;
                Solver::Solve(distance,
                              targetArmor.Center,
                              false,
                              cameraParam.ModelParam,
                              &offsetAngle);
                // 得到当前云台偏角
                yaw = offsetAngle.first;
                pitch = offsetAngle.second;

            }
        }


        // 初始化控制指令
        RobotBrainControlCommand command;
        command.ID = 1;
        command.Yaw = 0;
        command.Pitch = 0;
        command.Distance = 0;

        // 生成控制指令数据帧
        unsigned int commandFrameSize = RobotBrainControlCommand::GetFrameSize();
        unsigned char commandFrame[commandFrameSize];
        command.EncapsulateToFrame(commandFrame);

        // 将指令数据帧发送给下位机
        //serialPort.Write(commandFrameSize, commandFrame);

        // 记录截止时间戳
        std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
        uint64_t endTimestamp = endTime.time_since_epoch().count();

        // 输出处理帧率与当前是否在预测
        double processTime = static_cast<double>(endTimestamp - beginTimestamp) / 1000000;
        int FPS = static_cast<int>(1000 / processTime);


        //std::cout<<"predictState: "<<isPredict<<" "<<"FPS: "<<FPS<<std::endl;
//        std::cout << "predictworldCoordinate: " << predictworldCoordinate.transpose() << std::endl;
        //outfile << yaw << "\t" << pitch << std::endl;
//        std::cout<<"yaw: "<<yaw<<" "<<"pitch: "<<pitch<<std::endl;
        std::cout<<"fps: "<<FPS<<std::endl;

    }
    // 关闭相机
    camera.Close();
    camera.Release();

    // 关闭装甲板识别器
    armorRecognizer.Release();

    // 关闭解算器
    solver.Release();

    // 释放预测器
    predictor.Release();

    // 关闭并且释放串口
    //serialPort.Close();
    //serialPort.Release();


    //outfile.close();
}