//
// Created by cubot on 2022/5/2.
//

#include  "predictor_command.h"

// 构造函数
PredictorCommand::PredictorCommand():ID(), Q(), TimeStamp(), index(), nowTime()
{
}

// 解析机器人预测指令
bool PredictorCommand::Parse(unsigned char *robotBodyRequest,
                             PredictorCommand *predictorCommand)
{
    // 初始化解析结果
    bool result = true;

    // 解析机器人预测命令索引
    int16_t predictIndex;
    unsigned char index[2];
    index[0] = robotBodyRequest[2];
    index[1] = robotBodyRequest[1];
    ::memcpy(&predictIndex, index, 2);
    predictIndex = *(int16_t*)(index);


    // 判断索引数目是否合理
   if(predictIndex == 0)
    {
        result = false;
        std::cout<<"the index is out of range"<<std::endl;
    }
    else{
        predictorCommand->index = predictIndex;
    }
    //std::cout<<"index: "<<predictIndex<<std::endl;
    // 解析机器人串口发来的时间戳
    uint32_t timeStamp[4];
    timeStamp[0] = robotBodyRequest[6];
    timeStamp[1] = robotBodyRequest[5];
    timeStamp[2] = robotBodyRequest[4];
    timeStamp[3] = robotBodyRequest[3];
    uint32_t predictTimeStamp = timeStamp[0]+(timeStamp[1]<<8)+(timeStamp[2]<<16)+(timeStamp[3]<<24);

    // 判断发来时间戳是否合理
    if(predictTimeStamp <= 0)
    {
        result = false;
        std::cout<<"timestamp is wrong"<<std::endl;
    }
    else
    {
        // 将下位机在
        predictorCommand->TimeStamp = predictTimeStamp;
    }


    //std::cout<<"timestamp: "<<predictTimeStamp<<std::endl;
    // 解析机器人当前是否在打击状态
    unsigned char IsBlue = robotBodyRequest[7];
    bool color = static_cast<bool>(IsBlue);
    predictorCommand->IsBlue = color;
    if (color!=0 && color!=1)  std::cout<<"color is error"<<std::endl;
 //   std::cout<<"color: "<<color<<std::endl;

    // 解析机器人当前四元数的W
    float W;
    int16_t w;
    unsigned char quaternionW[2];
    quaternionW[0] = robotBodyRequest[8];
    quaternionW[1] = robotBodyRequest[9];
    ::memcpy(&w, quaternionW, 2);
    w = *(int16_t *)(quaternionW);
    W = static_cast<float>(w)/30000;

    // 判断四元数数值是否在要求范围内
    if(W<-1 || W>1)
    {
        result = false;
        std::cout<<"W is out range"<<std::endl;
    }
    else
    {
        // std::cout<<"W: "<<std::endl;
        predictorCommand->Q[0] = W;
    }

    // 解析机器人当前四元数的X
    float X;
    int16_t x;
    unsigned char quaternionX[2];
    quaternionX[0] = robotBodyRequest[10];
    quaternionX[1] = robotBodyRequest[11];
    ::memcpy(&x, quaternionX, 2);
    x = *(int16_t *)(quaternionX);
    X = static_cast<float>(x) / 30000;

    // 判断四元数数值是否在要求范围内
    if(X<-1 || X>1)
    {
        result = false;
        std::cout<<"X is out range"<<std::endl;
    }
    else
    {
        predictorCommand->Q[1] = X;
    }

    // 解析机器人当前四元数的Y
    float Y;
    int16_t y;
    unsigned char quaternionY[2];
    quaternionY[0] = robotBodyRequest[12];
    quaternionY[1] = robotBodyRequest[13];
    ::memcpy(&y, quaternionY, 2);
    y = *(int16_t*)(quaternionY);
    Y = static_cast<float>(y)/30000;

    // 判断四元数数值是否在要求范围内
    if(Y<-1 || Y>1)
    {
        result = false;
        std::cout<<"Y is out range"<<std::endl;
    }
    else
    {
        predictorCommand->Q[2] = Y;
    }

    // 解析机器人当前四元数的Y
    float Z;
    int16_t z;
    unsigned char quaternionZ[2];
    quaternionZ[0] = robotBodyRequest[14];
    quaternionZ[1] = robotBodyRequest[15];
    ::memcpy(&z, quaternionZ, 2);
    z = *(int16_t *)(quaternionZ);
    Z = static_cast<float>(z)/30000;

    // 判断四元数数值是否在要求范围内
    if(Z<-1 || Z>1)
    {
        result = false;
        std::cout<<"Z is out range"<<std::endl;
    }
    else
    {
        predictorCommand->Q[3] = Z;
    }



    int Stand;
    unsigned  char st;
    st = robotBodyRequest[16];
    Stand = static_cast<int>(st);

    if(Stand != 0 && Stand != 1)
    {
        result = false;
        std::cout<<"Stand is wrong!"<<std::endl;
    }
    else
    {
        predictorCommand->stand = Stand;
    }
//    std::cout<<"stand: "<<Stand<<std::endl;

    // 得到收到数据的时间戳
    if(result)
    {
        // 获取读取数据时间
        std::chrono::time_point<std::chrono::steady_clock> receiveTime = std::chrono::steady_clock::now();
        uint64_t receiveTimestamp = receiveTime.time_since_epoch().count();

        // 得到接受数据时间
        predictorCommand->nowTime = receiveTimestamp;

        // 计算出机器人当前欧拉角
        Eigen::Quaterniond q(W, X, Y, Z);
        Eigen::Matrix3d mate = q.toRotationMatrix();
        Eigen::Vector3d euler_t = q.toRotationMatrix().eulerAngles(2, 1, 0);

        float yaw = atan2 ( mate(1,0) ,mate (0,0) );
        float  roll = atan2( -mate(2,0 ) , sqrt(pow(mate(2,1),2) + pow(mate(2,2),2)));
        float pitch = atan2( mate(2,1) ,mate (2,2) );

//        std::cout<<" YAW: "<<yaw*57.3<<" Pitch: "<<pitch*57.3<<" Roll: "<<roll*57.3<<std::endl;
//        std::cout<<"Q: "<<W<<" "<<X<<" "<<" "<<Y<<" "<<Z<<" "<<std::endl;
        //std::cout<<" nowTime: "<<predictorCommand->nowTime/1000000<<" timeStamp: "<<predictorCommand->TimeStamp<<std::endl;

    }
    // 返回解析结果
    return result;
}

// 解析机器人预测指令
bool PredictorCommand::Parse(Eigen::Quaterniond quaterniond, PredictorCommand *predictorCommand)
{
    // 初始化解析结果
    bool result = true;
    static std::chrono::time_point<std::chrono::steady_clock> last_receiveTime;
    // 得到收到数据的时间戳
    if(result)
    {
        // 获取读取数据时间
        std::chrono::time_point<std::chrono::steady_clock> receiveTime = std::chrono::steady_clock::now();
        uint64_t receiveTimestamp = receiveTime.time_since_epoch().count();

        std::chrono::duration<double> gap_time = receiveTime - last_receiveTime;
//        std::cout << "IMU两帧间隔为 : " << gap_time.count() * 1000<< "ms\n";

        // IMU接收数据进入Parse函数后获取一次时间戳，需要跟相机获取图像的时间戳对比
        predictorCommand->TimeStamp = receiveTimestamp;
//        std::cout<<"IMU Timestamp： "<< receiveTimestamp<<std::endl;

        // 得到接受数据时间
        predictorCommand->nowTime = receiveTimestamp;

        // 得到机器人当前位姿
        predictorCommand->Q[0] = (float)quaterniond.w();
        predictorCommand->Q[1] = (float)quaterniond.x();
        predictorCommand->Q[2] = (float)quaterniond.y();
        predictorCommand->Q[3] = (float)quaterniond.z();

        last_receiveTime = receiveTime;
    }

    // 返回解析结果
    return result;
}