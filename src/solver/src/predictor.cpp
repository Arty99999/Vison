//
// Created by tony on 2022/4/23.
//

#include "predictor.h"
static int Count=0;
static int SwitchCount = 0;

// 构造函数
Predictor::Predictor() :param_(),
                        operateMutex_(),
                        bufferMutex_(),
                        isInitialized_(false),
                        initTimestamp_(),
                        predictorBuffer_(),
                        AutoaimEKF_(),
                        armorHop(false),
                        armornumchange(false),
                        stateCount(0)
{
}

// 初始化预测器
bool Predictor::Init() {

    // 锁定目标解算器操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断目标解算器是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - Predictor can not be released repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }
    else
    {

        // 重置初始化状态
        isInitialized_ = true;

        predictorBuffer_.clear();

        if(!AutoaimEKF::IsEmpty(AutoaimEKF_))
        {
            AutoaimEKF_.Release();
        }

        return true;
    }
}

// 设置预测器参数
bool Predictor::SetParam(PredictorParam &predictorParam)
{
    // 锁定目标解算器操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 把参数复制给预测器参数
    param_ = predictorParam;

    // 记录日志信息
    log = "[" + param_.Key + "] - PredictorParam was set successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回设置结果
    return true;
}

// 得到预测器参数
PredictorParam Predictor::GetParam()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return param_;
}

// 预测器释放函数
bool Predictor::Release() {

    // 锁定目标解算器操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断目标解算器是否已经初始化
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - Solver can not be released repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 重置初始化时间戳
    initTimestamp_ = 0;

    // 重置初始化状态
    isInitialized_ = false;

    // 对预测器缓冲区进行释放
    predictorBuffer_.clear();

    // 对滤波器内参数进行释放
    if(!AutoaimEKF::IsEmpty(AutoaimEKF_))
    {
        AutoaimEKF_.Release();
    }
}

// 得到预测的器的初始化参数
bool Predictor::IsInitialized_() {

    // 预测器的互斥锁
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    // 得到当前预测器初始化
    return isInitialized_;
}


// 自瞄移动预测器
void Predictor::AutoaimPredict(bool &isPredict,
                               Predictor &predictor,
                               HuarayCameraModelParam &modelParam,
                               std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber>> *armorRecord,
                               std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64>> *ArmorsAndYaws,
                               double &predict_time,
                               Eigen::Vector3d *predictCradleHeadCoordinate,
                               Eigen::Vector3d *predictWorldCoordinate,
                               Eigen::Vector3d *predictCradleHeadCoordinate1,
                               Eigen::Vector3d *predictWorldCoordinate1,
                               cv::Point2f *predictpixel,
                               const Eigen::Matrix<double, 3, 3>& rotationMatrix,
                               cv::Point2f &Get1, cv::Point2f &Get2,
                               cv::Point2f &Get3, cv::Point2f &Get4,
                               int stand,
                               bool &isFire,
                               double &r) {
    //
    stand = 0;
//    stand = 1;
    // 预测器的互斥锁r
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    // 若预测则进行下一步

    if ((armorRecord->size() > 3) && isPredict)
    {
        // 初始化坐标
        Eigen::Vector3d nowCradleHeadCoordinate;//云台坐标系
        Eigen::Vector3d nowWorldCoordinate;//世界坐标系(重力加速度竖直向下)
        Eigen::Vector3d pastCradleHeadCoordinate;
        Eigen::Vector3d pastWorldCoordinate;
        // 得到装甲板记忆队列的大小，来进行读取操作
        unsigned long recordSize;
        unsigned long sequenceSize;
        recordSize = armorRecord->size();
        sequenceSize = ArmorsAndYaws->size();

        // 读取当前坐标缓存数据
        uint64_t nowTime = std::get<0>(armorRecord->at(recordSize - 1));
        float nowDistance = std::get<1>(armorRecord->at(recordSize - 1));
        cv::Point2f nowLocation = std::get<2>(armorRecord->at(recordSize - 1));
        std::array<float, 4> nowQuaterniond = std::get<3>(armorRecord->at(recordSize - 1));
        EClassicalArmorNumber nowNumber = std::get<4>(armorRecord->at(recordSize-1));
        // 得到云台坐标系与世界坐标系的旋转矩阵
//        std::cout<<"nowLocation: "<<nowLocation.x<<" "<<nowLocation.y<<std::endl;
        Eigen::Quaterniond Q_raw(nowQuaterniond[0],nowQuaterniond[1],nowQuaterniond[2],nowQuaterniond[3]);
        // 四元数得到的旋转矩阵
        Eigen::Vector3d euler = Q_raw.toRotationMatrix().eulerAngles(2, 1, 0);
        Eigen::Matrix3d nowQMatrix = Q_raw.toRotationMatrix();
        Eigen::Matrix<double,3,3> R= nowQMatrix * rotationMatrix;
        double yaw_armor1 = atan2( rotationMatrix( 1, 0) ,rotationMatrix(0 ,0));
        std::vector<ClassicalArmor> nowArmors = std::get<0>(ArmorsAndYaws -> at(sequenceSize - 1));
        std::vector<float> nowYaws = std::get<1>(ArmorsAndYaws -> at(sequenceSize - 1));
        unsigned long yaw_size = nowYaws.size();
        double IMU_yaw = atan2(nowQMatrix(1,0),nowQMatrix(0,0));
        double yaw_armor = nowYaws[armorIndex];

//        std::cout<<"yaw_armor: "<<yaw_armor * 57.3<<"       "<<yaw_armor1 * 57.3<<std::endl;
        double yaw_sum = AutoaimEKF_.HandleJump( yaw_armor);
        //std::cout<<"yaw_sum: "<<yaw_sum * 57.3<<std::endl;
        //std::cout<<nowDistance<<std::endl;

        std::vector<ClassicalArmor> lastArmors = std::get<0>(ArmorsAndYaws->at(sequenceSize - 2));
        // 使用了相机外参，将坐标系从相机光心变换到转动中心
        // 图像到相机再到云台坐标系
        
        PredictorTool::GetRelativeCorrdinate(nowDistance,
                                             nowLocation,
                                             false,
                                             modelParam,
                                             &nowCradleHeadCoordinate);
        // 云台到惯性坐标系是imu的旋转矩阵
        nowWorldCoordinate = nowQMatrix * nowCradleHeadCoordinate;

//        std::cout << " worldcoordinste =  [ " <<  nowWorldCoordinate.x() << " , " << nowWorldCoordinate.y() << " , " << nowWorldCoordinate.z()  << " ]" << std::endl;
//        std::cout << "nowArmor: " << nowWorldCoordinate.transpose() <<std::endl;
//        std::cout << "nowcenter: " << nowWorldCoordinate.x() - 240*sin(yaw_armor) << "  " << nowWorldCoordinate.y() + 240*cos(yaw_armor) << "  " << nowWorldCoordinate.z() << std::endl;
        *predictWorldCoordinate1 = nowWorldCoordinate;
        *predictCradleHeadCoordinate1 = nowCradleHeadCoordinate;
        //std::cout<<"YAW: "<<euler(0)*57.3<<" Pitch: "<<euler(1)*57.3<<" Roll: "<<euler(2)*57.3<<std::endl;
        //std::cout<<"now: "<<nowWorldCoordinate.transpose()<<std::endl;
//        std::cout<<"cradle: "<<nowCradleHeadCoordinate.transpose()<<std::endl;
        // 得到历史目标二维坐标,四元数和距离
        uint64_t pastTime = std::get<0>(armorRecord->at(recordSize - 2));
        float pastDistance = std::get<1>(armorRecord->at(recordSize - 2));
        cv::Point2f pastLocation = std::get<2>(armorRecord->at(recordSize - 2));
        std::array<float, 4> pastQuaterniond = std::get<3>(armorRecord->at(recordSize - 2));
        EClassicalArmorNumber pastNumber = std::get<4>(armorRecord->at(recordSize-2));
        // 得到历史坐标的旋转矩阵与欧拉角
        Eigen::Quaterniond QPastRaw(pastQuaterniond[0], pastQuaterniond[1], pastQuaterniond[2], pastQuaterniond[3]);
        Eigen::Matrix3d pastQMatrix = QPastRaw.toRotationMatrix();
        Eigen::Vector3d euler_t_p = QPastRaw.toRotationMatrix().eulerAngles(2, 1, 0);
//        Eigen::Matrix3d check_p;
//        check_p<<cos(euler_t_p(2)),0,-sin(euler_t_p(2)),0,1,0,sin(euler_t_p(2)),0,cos(euler_t_p(2));
//        Eigen::Matrix3d pastQMatrix =  check_p * pastQMatrix_c ;
        // 得到历史目标的云台坐标系坐标
        PredictorTool::GetRelativeCorrdinate(pastDistance,
                                             pastLocation,
                                             false,
                                             modelParam,
                                             &pastCradleHeadCoordinate);

        
        // 得到历史目标的世界坐标系坐标
        pastWorldCoordinate = pastQMatrix * pastCradleHeadCoordinate;

        // 得到历史目标的世界坐标系坐标
//        std::cout << nowCradleHeadCoordinate.transpose() << "    "  << nowWorldCoordinate.transpose()  << std::endl;
//        std::cout<<"nowwrold: "<<nowWorldCoordinate.transpose()<<" pastwrold: "<<pastWorldCoordinate.transpose()<<std::endl;
        //std::cout<<"nowlocation: "<<nowLocation.x<<" "<<nowLocation.y<<" pastlocation: "<<pastLocation.x<<" "<<pastLocation.y<<std::endl;
        // 得到和上次预测差距的时间
        double gapTime = static_cast<double>(nowTime - pastTime) / 1000000000;
        // 对当前目标的预测器进行处理
       // bool isEmpty = AutoaimEKF::IsEmpty(AutoaimEKF_);
//        AutoaimEKF_.Q = AutoaimEKF_.SetQMatrix(gapTime);

        bool isEmpty = AutoaimEKF::IsEmpty(AutoaimEKF_);
        PredictorModel Model;
        double PastDistance = std::sqrt(pastWorldCoordinate.x() * pastWorldCoordinate.x() + pastWorldCoordinate.y() * pastWorldCoordinate.y() + pastWorldCoordinate.z() * pastWorldCoordinate.z());
        double Distance = std::sqrt(nowWorldCoordinate.x() * nowWorldCoordinate.x() + nowWorldCoordinate.y() * nowWorldCoordinate.y() + nowWorldCoordinate.z() * nowWorldCoordinate.z());
        if(AutoaimEKF_.Xe[8] == 0)
        {
            Eigen::Matrix<double, 9, 1> Xr;
            Xr<<nowWorldCoordinate.x() - 260 * sin(yaw_armor), 0, nowWorldCoordinate.y() + 260 * cos(yaw_armor), 0, nowWorldCoordinate.z(), 0, yaw_sum, 0, 260;
            AutoaimEKF_.init(Xr, Model);
        }
        double last_r = AutoaimEKF_.Xe[8];
        double last_w = AutoaimEKF_.Xe[7];
        double last_yaw = AutoaimEKF_.Xe[6];
        double last_vx = AutoaimEKF_.Xe[1];
        double last_vy = AutoaimEKF_.Xe[3];

        std::vector<int> numIndex;
        std::vector<double> Radi;

        //std::cout<<"eeeeeee:   "<<yaw_armor<<std::endl;
        // 判断是否为同一块装甲板
        if((!isEmpty) && (gapTime < 2) && (nowNumber == pastNumber))
        {
            double targetYaw = nowYaws[armorIndex];
            nowWorldCoordinate = Eigen::Matrix<double, 3, 1>(nowWorldCoordinate.x()  ,
                                                             nowWorldCoordinate.y()  ,
                                                             nowWorldCoordinate.z());
            double predictTime = predict_time ;
            //std::cout<<"now: "<<nowWorldCoordinate.transpose()<<std::endl;
            if (AutoaimEKF_.jump_flag)
            {
                AutoaimEKF_.Xe[6] = targetYaw;
                last_z = nowWorldCoordinate.z();
                AutoaimEKF_.Xe[4] = nowWorldCoordinate.z();
                isFire = false;
            }

            if (abs(yaw_armor *57.3) > 100)
            {
                AutoaimEKF_.Xe[6] = last_yaw;
                AutoaimEKF_.Xe[7] = last_w;
            }
//            std::cout << gapTime << "   " << predictTime<< std::endl;
            predictor.FightLinearTarget(gapTime ,
                                        Model,
                                        nowWorldCoordinate,
                                        yaw_armor);
//            std::cout << AutoaimEKF_.Xe[6]<< std::endl;
            double dyaw = AutoaimEKF_.GetYawNow(AutoaimEKF_.Xe[6]).second;
            //std::cout<<dyaw * 57.3<<std::endl;
//            std::cout<<"gaptime: "<<gapTime<<std::endl;
            //std::cout<<"yawsum: "<<yaw_sum<<std::endl;
            //std::cout<<"pose: "<<AutoaimEKF_.Xe[6] * 57.3<<"    "<<AutoaimEKF_.Xe[7] * 57.3<<std::endl;
            AutoaimEKF_.Xe[8] = 240;
            if (abs(AutoaimEKF_.Xe[7]* 57.3) > 500)
            {
                AutoaimEKF_.Xe[7] = last_w;
                AutoaimEKF_.Xe[6] = yaw_armor;
            }

            if(stand == 0) {

                predictWorldCoordinate->x() = nowWorldCoordinate.x();
                predictWorldCoordinate->y() = nowWorldCoordinate.y();
                predictWorldCoordinate->z() = nowWorldCoordinate.z();
                Eigen::Vector3d predictW;
                predictW << predictWorldCoordinate->x(),predictWorldCoordinate->y(),predictWorldCoordinate->z();
                *predictCradleHeadCoordinate = nowQMatrix.inverse() * predictW;
                *predictpixel = nowLocation;
            }
            if(stand == 1) {
                std::vector<float> Distances = {0, 0};
                for (int i = 0; i < nowArmors.size(); ++i) {
                    PredictorTool::getArmorCenterLocation(nowArmors[i], modelParam, &Distances[i]);
                }

                cv::Point2d ctP;
                AntiTop(modelParam, ArmorsAndYaws, Distances, &ctP, &numIndex, &Radi);

//                if (AutoaimEKF_.Xe[6] < -90/57.3)
                double nowYaw;
                if (AutoaimEKF_.yaw_sum != 0) {
                    nowYaw = yaw_armor + AutoaimEKF_.Xe[7]  * predictTime;
                }
                if (abs(nowYaw) > 70 / 57.3)
                {
                    int e = AutoaimEKF_.Xe[7] >0? 1 :-1;
                    nowYaw -= CV_PI / 2 * e;
                    AutoaimEKF_.Xe[4] = last_z;
                    isFire = false;
                }
                nowWorldCoordinate << AutoaimEKF_.Xe[0] + AutoaimEKF_.Xe[1] * (predict_time+0.08) + AutoaimEKF_.Xe[8] * ceres::sin(nowYaw),
                                      AutoaimEKF_.Xe[2] + AutoaimEKF_.Xe[3] * (predict_time+0.02) - AutoaimEKF_.Xe[8] * ceres::cos(nowYaw),
                                      AutoaimEKF_.Xe[4];

//                std::cout <<"predict: " <<AutoaimEKF_.Xe[0] << "  " << AutoaimEKF_.Xe[2] <<"  " <<AutoaimEKF_.Xe[4] << std::endl;
//                std::cout <<AutoaimEKF_.Xe[8]<<std::endl;
//                PredictorTool::AVF(nowWorldCoordinate,recordSize,a,count);
//                PredictorTool::AVF1(AutoaimEKF_.Xe[8],recordSize,b,count1);
//                std::cout <<predictTime <<std::endl;
                r = AutoaimEKF_.Xe[8];
//                std::cout << yaw_armor * 57.3 <<std::endl;
//                std::cout << AutoaimEKF_.Xe[6] * 57.3 << "     " <<AutoaimEKF_.Xe[7] * 57.3 << std::endl;
//                std::cout <<yaw_armor * 57.3<<std::endl;
//                std::cout <<"yaw: " << yaw_armor * 57.3 <<"     " <<" now: " <<nowYaw* 57.3 <<std::endl;
//                std::cout <<" R: " <<AutoaimEKF_.Xe[8] <<std::endl;
                //若识别发生跳变
//                if (armorHop)
//                {
//                    std::cout << "111111111111111" << std::endl;
//                    ClassicalArmor L_armor,R_armor,virtualArmor;
//                    Eigen::Vector3d HopLocation;
//                    for (int i = 0; i < nowArmors.size(); ++i)
//                    {
//                        nowArmors[i].Center.x < 600 ? L_armor = nowArmors[i] : R_armor = nowArmors[i];
//                    }
//
//                    if (AntiTop_.IsrotateRight && R_armor.Number != EClassicalArmorNumber::Invalid)
//                    {
//                        AntiTop_.lockedShootArmor = R_armor;
//                        PredictorTool::GetRelativeCorrdinate(nowDistance,R_armor.Center, false,modelParam,&HopLocation);
//
//                    }
//                    else if (!AntiTop_.IsrotateRight && L_armor.Number != EClassicalArmorNumber::Invalid)
//                    {
//                        AntiTop_.lockedShootArmor = L_armor;
//                        PredictorTool::GetRelativeCorrdinate(nowDistance,L_armor.Center, false,modelParam,&HopLocation);
//                    }
//                    else
//                    {
//                        HopLocation = pastCradleHeadCoordinate;
//                    }
//                    HopLocation = nowQMatrix * HopLocation;
//                    nowWorldCoordinate = HopLocation;
//
//                }
//                nowWorldCoordinate = nowQMatrix.inverse() * nowWorldCoordinate;
//                nowWorldCoordinate << nowWorldCoordinate(0,0) - AutoaimEKF_.Xe[8] * sin(yaw_armor1), nowWorldCoordinate(1,0) - AutoaimEKF_.Xe[8] * cos(yaw_armor1), nowWorldCoordinate(2,0);
//                nowWorldCoordinate = nowQMatrix * nowWorldCoordinate;
                predictWorldCoordinate->x() = nowWorldCoordinate.x() ;
                predictWorldCoordinate->y() = nowWorldCoordinate.y() ;
                predictWorldCoordinate->z() = nowWorldCoordinate.z() ;
//                std::cout <<"predict: " <<nowWorldCoordinate.transpose()<< " \n" <<nowCradleHeadCoordinate.transpose() <<std::endl;
                Eigen::Vector3d predictW;
                predictW << predictWorldCoordinate->x(),predictWorldCoordinate->y(),predictWorldCoordinate->z();
                *predictCradleHeadCoordinate = nowQMatrix.inverse() * predictW;
                PredictorTool::GetPixelLocation(1,predictpixel,modelParam,*predictCradleHeadCoordinate);
//                *predictpixel = nowLocation;
            }
        }
            // 对滤波器进行初始化 将当前坐标传出
        else
        {
            Eigen::Matrix<double, 9, 1> Xr;
            Xr<<nowWorldCoordinate.x() - 260 * sin(yaw_armor), 0, nowWorldCoordinate.y() + 260 * cos(yaw_armor), 0, nowWorldCoordinate.z(), 0, yaw_sum, 0, 260;
            AutoaimEKF_.init(Xr,Model);
            *predictWorldCoordinate = nowWorldCoordinate;
            *predictCradleHeadCoordinate = nowCradleHeadCoordinate;
            *predictpixel = nowLocation;
        }
        Eigen::Vector3d pre;
        pre<<AutoaimEKF_.Xe[0],AutoaimEKF_.Xe[2],AutoaimEKF_.Xe[4];
        PredictorTool::GetAll(pre,yaw_armor,AutoaimEKF_.Xe[8],nowQMatrix,Get1,Get2,Get3,Get4,modelParam);
    }
    else
    {
        // 初始化坐标
        Eigen::Vector3d nowCradleHeadCoordinate;//云台坐标系
        Eigen::Vector3d nowWorldCoordinate;//世界坐标系(重力加速度竖直向下)
        unsigned long recordSize;
        recordSize = armorRecord->size();

        // 读取当前坐标缓存数据
        uint64_t nowTime = std::get<0>(armorRecord->at(0));
        float nowDistance = std::get<1>(armorRecord->at(0));
        cv::Point2f nowLocation = std::get<2>(armorRecord->at(0));
        std::array<float, 4> nowQuaterniond = std::get<3>(armorRecord->at(0));
        EClassicalArmorNumber nowNumber = std::get<4>(armorRecord->at(0));
        // 得到云台坐标系与世界坐标系的旋 转矩阵
        Eigen::Quaterniond Q_raw(nowQuaterniond[0],nowQuaterniond[1],nowQuaterniond[3],nowQuaterniond[2]);
        // 四元数得到的旋转矩阵
        Eigen::Matrix3d nowQMatrix = Q_raw.toRotationMatrix();
        // 使用了相机外参，将坐标系从相机光心变换到转动中心
        // 图像到相机再到云台坐标系
        PredictorTool::GetRelativeCorrdinate(nowDistance,
                                             nowLocation,
                                             false,
                                             modelParam,
                                             &nowCradleHeadCoordinate);
        // 云台到惯性坐标系是imu的旋转矩阵
        nowWorldCoordinate = nowQMatrix * nowCradleHeadCoordinate;

        //1.0测试专用 先将结算的世界坐标输出
        *predictWorldCoordinate = nowWorldCoordinate;
        *predictWorldCoordinate1 = nowWorldCoordinate;
        *predictCradleHeadCoordinate = nowCradleHeadCoordinate;
        *predictCradleHeadCoordinate1 = nowCradleHeadCoordinate;
        *predictpixel = nowLocation;
    }
}

void Predictor::AutoaimPredict_outpost(bool &isPredict,
                                       Predictor &predictor,
                                       HuarayCameraModelParam &modelParam,
                                       std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber>> *armorRecord,
                                       std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64>> *ArmorsAndYaws,
                                       double &predict_time,
                                       Eigen::Vector3d *predictCradleHeadCoordinate,
                                       Eigen::Vector3d *predictWorldCoordinate,
                                       Eigen::Vector3d *predictCradleHeadCoordinate1,
                                       Eigen::Vector3d *predictWorldCoordinate1,
                                       cv::Point2f *predictpixel,
                                       cv::Point2f *predictpixelCenter,
                                       const Eigen::Matrix<double, 3, 3>& rotationMatrix,
                                       cv::Point2f &Get1, cv::Point2f &Get2,
                                       cv::Point2f &Get3, cv::Point2f &Get4,
                                       bool is_okFire,
                                       bool &isFire,
                                       double &r) {
//    std::cout<<"Preatime: "<<predict_time<<std::endl;
//    std::cout<<"AutoAim_outpost"<<std::endl;
    int stand = 1;
//     int stand = 0;
    // 预测器的互斥锁r
    std::lock_guard <std::mutex> lockGuard(operateMutex_);
    // 若预测则进行下一步
    if ((armorRecord->size() > 3) && isPredict)
    {
        // 初始化坐标
        Eigen::Vector3d nowCradleHeadCoordinate;//云台坐标系
        Eigen::Vector3d nowWorldCoordinate;//世界坐标系(重力加速度竖直向下)
        Eigen::Vector3d pastCradleHeadCoordinate;
        Eigen::Vector3d pastWorldCoordinate;

        // 得到装甲板记忆队列的大小，来进行读取操作
        unsigned long recordSize;
        unsigned long sequenceSize;
        recordSize = armorRecord->size();
        sequenceSize = ArmorsAndYaws->size();

        // 读取当前坐标缓存数据
        uint64_t nowTime = std::get<0>(armorRecord->at(recordSize - 1));
        float nowDistance = std::get<1>(armorRecord->at(recordSize - 1));
        cv::Point2f nowLocation = std::get<2>(armorRecord->at(recordSize - 1));
        std::array<float, 4> nowQuaterniond = std::get<3>(armorRecord->at(recordSize - 1));
        EClassicalArmorNumber nowNumber = std::get<4>(armorRecord->at(recordSize-1));
        // 得到云台坐标系与世界坐标系的旋转矩阵
        // std::cout<<"nowLocation: "<<nowLocation.x<<" "<<nowLocation.y<<std::endl;
        Eigen::Quaterniond Q_raw(nowQuaterniond[0],nowQuaterniond[1],nowQuaterniond[2],nowQuaterniond[3]);
        // 四元数得到的旋转矩阵
        Eigen::Matrix3d nowQMatrix = Q_raw.toRotationMatrix();


        Eigen::Matrix<double,3,3> R= nowQMatrix * rotationMatrix;
        std::vector<ClassicalArmor> nowArmors = std::get<0>(ArmorsAndYaws -> at(sequenceSize - 1));
        std::vector<float> nowYaws = std::get<1>(ArmorsAndYaws -> at(sequenceSize - 1));

        double IMU_yaw = atan2(nowQMatrix(1,0),nowQMatrix(0,0));
        double yaw_armor = nowYaws[armorIndex];
        double  world_yaw = yaw_armor + IMU_yaw;

        double yaw_sum = AutoaimEKF_.HandleJump( yaw_armor);

        std::vector<ClassicalArmor> lastArmors = std::get<0>(ArmorsAndYaws->at(sequenceSize - 2));


        // 使用了相机外参，将坐标系从相机光心变换到转动中心
        // 图像到相机再到云台坐标系

        PredictorTool::GetRelativeCorrdinate(nowDistance,
                                             nowLocation,
                                             false,
                                             modelParam,
                                             &nowCradleHeadCoordinate);
        // 云台到惯性坐标系是imu的旋转矩阵
        nowWorldCoordinate = nowQMatrix * nowCradleHeadCoordinate;


        *predictWorldCoordinate1 = nowWorldCoordinate;
        *predictCradleHeadCoordinate1 = nowCradleHeadCoordinate;

        // 得到历史目标二维坐标,四元数和距离
        uint64_t pastTime = std::get<0>(armorRecord->at(recordSize - 2));
        float pastDistance = std::get<1>(armorRecord->at(recordSize - 2));
        cv::Point2f pastLocation = std::get<2>(armorRecord->at(recordSize - 2));
        std::array<float, 4> pastQuaterniond = std::get<3>(armorRecord->at(recordSize - 2));
        EClassicalArmorNumber pastNumber = std::get<4>(armorRecord->at(recordSize-2));
        // 得到历史坐标的旋转矩阵与欧拉角
        Eigen::Quaterniond QPastRaw(pastQuaterniond[0], pastQuaterniond[1], pastQuaterniond[2], pastQuaterniond[3]);
        Eigen::Matrix3d pastQMatrix = QPastRaw.toRotationMatrix();

        // 得到历史目标的云台坐标系坐标
        PredictorTool::GetRelativeCorrdinate(pastDistance,
                                             pastLocation,
                                             false,
                                             modelParam ,
                                             &pastCradleHeadCoordinate);


        // 得到历史目标的世界坐标系坐标
        pastWorldCoordinate = pastQMatrix * pastCradleHeadCoordinate;
        double now_Isright = false;
        double nowRotionYaw = atan(nowWorldCoordinate.y() / nowWorldCoordinate.x());
        double pastRotionYaw = atan(pastWorldCoordinate.y() / pastWorldCoordinate.x());
        if (nowRotionYaw - pastRotionYaw > 0)
            Count += 1;
        else
            Count -= 1;
        if (Count > 100)
            Count = 100;
        if (Count < -100)
            Count = -100;
        if (Count < 0)
            now_Isright = true;
        else now_Isright = false;


        if (now_Isright){
            if (yaw_armor > 25./57.3)  SwitchCount ++;
            else if (yaw_armor < 0) SwitchCount =0;
        } else{
            if (yaw_armor < -25./57.3)   SwitchCount --;
            else if (yaw_armor > 0) SwitchCount =0;
        }
        if (SwitchCount > 2) SwitchCount = 2;
        if (SwitchCount < -2) SwitchCount = -2;
        std::cout<<"yaw: "<<yaw_armor*57.3<<"  NowIsRight:  "<<now_Isright<<"  SwitchCount: "<<SwitchCount<<std::endl;
//        std::cout<<"diff : "<<nowRotionYaw - pastRotionYaw<<"  Count:  "<<Count<<"  IsRight: "<<now_Isright<<std::endl;
        // 得到和上次预测差距的时间
        double gapTime = static_cast<double>(nowTime - pastTime) / 1000000000;


        AntiTop_.IsrotateRight = now_Isright;
//        if (abs(diff) > 1)
//        {
//            std::cout<<"IsRitght: "<<now_Isright<<" Count: "<<Count<<"  Isright: "<<IsRightCount;
//            std::cout<<" diff : "<<diff<<" past: "<<pastLocation.x<<"  "<<"  now: "<<nowLocation.x<<std::endl;;
//        }

        bool isEmpty = AutoaimEKF::IsEmpty(AutoaimEKF_);
        PredictorModel Model;
        double PastDistance = std::sqrt(pastWorldCoordinate.x() * pastWorldCoordinate.x() + pastWorldCoordinate.y() * pastWorldCoordinate.y() + pastWorldCoordinate.z() * pastWorldCoordinate.z());
        double Distance = std::sqrt(nowWorldCoordinate.x() * nowWorldCoordinate.x() + nowWorldCoordinate.y() * nowWorldCoordinate.y() + nowWorldCoordinate.z() * nowWorldCoordinate.z());
        if(AutoaimEKF_.Xe[8] == 0)
        {   //用车体中心初始化
            // std::cout<<"init"<<std::endl;
            Eigen::Matrix<double, 9, 1> Xr;
            Xr<<nowWorldCoordinate.x() - 276 * sin(yaw_armor), 0, nowWorldCoordinate.y() + 276 * cos(yaw_armor), 0, nowWorldCoordinate.z(), 0, yaw_sum, 0 , 276;
            AutoaimEKF_.init(Xr, Model);
        }
        double last_z = AutoaimEKF_.Xe[4];
        double last_w = AutoaimEKF_.Xe[7];
        double last_yaw = AutoaimEKF_.Xe[6];


        std::vector<int> numIndex;
        std::vector<double> Radi;


        // 判断是否为同一块装甲板
        if((!isEmpty) && (gapTime < 2) && (nowNumber == pastNumber))
        {
            double targetYaw = nowYaws[armorIndex];
            nowWorldCoordinate = Eigen::Matrix<double, 3, 1>(nowWorldCoordinate.x()  ,
                                                             nowWorldCoordinate.y()  ,
                                                             nowWorldCoordinate.z());
            double predictTime = predict_time ;
            // std::cout<<"now: "<<nowWorldCoordinate.transpose()<<std::endl;




            AutoaimEKF_.Xe[8] = 276;
            if (now_Isright)
                AutoaimEKF_.Xe[7] = 0.8 * CV_PI;
            else
                AutoaimEKF_.Xe[7] = -0.8 * CV_PI;
            if(stand == 0)
            {
                predictWorldCoordinate->x() = nowWorldCoordinate.x();
                predictWorldCoordinate->y() = nowWorldCoordinate.y();
                predictWorldCoordinate->z() = nowWorldCoordinate.z();
                Eigen::Vector3d predictW;
                predictW << predictWorldCoordinate->x(),predictWorldCoordinate->y(),predictWorldCoordinate->z();
                *predictCradleHeadCoordinate = nowQMatrix.inverse() * predictW;
                *predictpixel = nowLocation;
            }
            if(stand == 1)
            {
                std::vector<float> Distances = {0, 0};
                for (int i = 0; i < nowArmors.size(); ++i)
                {
                    PredictorTool::getArmorCenterLocation(nowArmors[i], modelParam, &Distances[i]);
                }
                cv::Point2d ctP;
                AntiTop(modelParam, ArmorsAndYaws, Distances, &ctP, &numIndex, &Radi);

                int e = AntiTop_.IsrotateRight ? -1 : 1;
                double nowYaw;
                if (AutoaimEKF_.yaw_sum != 0)
                {
                    nowYaw = world_yaw + AutoaimEKF_.Xe[7]  * predictTime;
                }
                Eigen::Vector3d WorldPredict1;
                Eigen::Vector3d WorldPredict2;
                Eigen::Vector3d WorldPredict3;
                Eigen::Vector3d Center;


                if (SwitchCount < 2 && SwitchCount > -2){
                    WorldPredict1 << nowWorldCoordinate.x() - 276 * sin(world_yaw) + 276 * sin(nowYaw) ,
                            nowWorldCoordinate.y() + 276 * cos(world_yaw) - 276 * cos(nowYaw) ,
                            nowWorldCoordinate.z();
                } else if (SwitchCount >= 2)
                {
                    WorldPredict1 << nowWorldCoordinate.x() - 276 * sin(world_yaw) + 276 * sin(nowYaw - 100.0/57.3) ,
                            nowWorldCoordinate.y() + 276 * cos(world_yaw) - 276 * cos(nowYaw - 100.0/57.3) ,
                            nowWorldCoordinate.z();
                } else if (SwitchCount <= -2)
                    WorldPredict1 << nowWorldCoordinate.x() - 276 * sin(world_yaw) + 276 * sin(nowYaw + 100.0/57.3) ,
                            nowWorldCoordinate.y() + 276 * cos(world_yaw) - 276 * cos(nowYaw + 100.0/57.3) ,
                            nowWorldCoordinate.z();





                predictWorldCoordinate->x() = WorldPredict1.x() ;
                predictWorldCoordinate->y() = WorldPredict1.y() ;
                predictWorldCoordinate->z() = WorldPredict1.z() ;

                Eigen::Vector3d predictW;

                predictW << predictWorldCoordinate->x(),predictWorldCoordinate->y(),predictWorldCoordinate->z();
                *predictCradleHeadCoordinate = nowQMatrix.inverse() * predictW;
                PredictorTool::GetPixelLocation(1,predictpixel,modelParam,*predictCradleHeadCoordinate);
//                *predictpixel = nowLocation;

            }
        }
            // 对滤波器进行初始化 将当前坐标传出
        else
        {
            Eigen::Matrix<double, 9, 1> Xr;
            Xr<<nowWorldCoordinate.x() - 260 * sin(yaw_armor), 0, nowWorldCoordinate.y() + 260 * cos(yaw_armor), 0, nowWorldCoordinate.z(), 0, yaw_sum, 0, 260;
            AutoaimEKF_.init(Xr,Model);
            *predictWorldCoordinate = nowWorldCoordinate;
            *predictCradleHeadCoordinate = nowCradleHeadCoordinate;
            *predictpixel = nowLocation;
        }
        Eigen::Vector3d pre;
        pre<<AutoaimEKF_.Xe[0],AutoaimEKF_.Xe[2],AutoaimEKF_.Xe[4];
        PredictorTool::GetAll(pre,yaw_armor,AutoaimEKF_.Xe[8],nowQMatrix,Get1,Get2,Get3,Get4,modelParam);
    }
    else
    {
        // 初始化坐标
        Eigen::Vector3d nowCradleHeadCoordinate;//云台坐标系
        Eigen::Vector3d nowWorldCoordinate;//世界坐标系(重力加速度竖直向下)
        unsigned long recordSize;
        recordSize = armorRecord->size();

        // 读取当前坐标缓存数据
        uint64_t nowTime = std::get<0>(armorRecord->at(0));
        float nowDistance = std::get<1>(armorRecord->at(0));
        cv::Point2f nowLocation = std::get<2>(armorRecord->at(0));
        std::array<float, 4> nowQuaterniond = std::get<3>(armorRecord->at(0));
        EClassicalArmorNumber nowNumber = std::get<4>(armorRecord->at(0));
        // 得到云台坐标系与世界坐标系的旋转矩阵
        Eigen::Quaterniond Q_raw(nowQuaterniond[0],nowQuaterniond[1],nowQuaterniond[3],nowQuaterniond[2]);
        // 四元数得到的旋转矩阵
        Eigen::Matrix3d nowQMatrix = Q_raw.toRotationMatrix();
        // 使用了相机外参，将坐标系从相机光心变换到转动中心
        // 图像到相机再到云台坐标系
        PredictorTool::GetRelativeCorrdinate(nowDistance,
                                             nowLocation,
                                             false,
                                             modelParam,
                                             &nowCradleHeadCoordinate);
        // 云台到惯性坐标系是imu的旋转矩阵
        nowWorldCoordinate = nowQMatrix * nowCradleHeadCoordinate;

        //1.0测试专用 先将结算的世界坐标输出
        *predictWorldCoordinate = nowWorldCoordinate;
        *predictWorldCoordinate1 = nowWorldCoordinate;
        *predictCradleHeadCoordinate = nowCradleHeadCoordinate;
        *predictCradleHeadCoordinate1 = nowCradleHeadCoordinate;
        *predictpixel = nowLocation;
    }
}


//得到重投影的像素点
void Predictor::Getpiexl(HuarayCameraModelParam &modelParam,std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>> &nowTarget,
                         Eigen::Vector3d predictRelativeCoordinate,cv::Point2f *pixelTarget){


}

// 预测器反陀螺部分算法
void Predictor::AntiTop(HuarayCameraModelParam modelParam,
                        std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64>> *armorDataSequence,
                        std::vector<float> Distances,
                        cv::Point2d *centerLocation,
                        std::vector<int> *numberIndex,
                        std::vector<double> *Radi) {

    //获取序列长度
    unsigned int armorDataSize = armorDataSequence->size();
    int last_armornum = std::get<0>(armorDataSequence ->at(armorDataSize - 2)).size();
    //获取序列最新数据
    std::vector<ClassicalArmor> nowArmors = std::get<0>(armorDataSequence->at(armorDataSize - 1));
    std::vector<float> nowYaws = std::get<1>(armorDataSequence->at(armorDataSize - 1));
    uint64 *disappearTime = &std::get<2>(armorDataSequence->at(armorDataSize - 1));
    //获取目标装甲板数目
    int armornum = AntiTop_.matchNum;
//    int armorsize = armors.size();

    std::vector<double> Radius;
    std::vector<int> numberindex;

    //计算车中心点
    auto getCenter = [&]()  -> void {
        ClassicalArmor armor1 = nowArmors[0];
        ClassicalArmor armor2 = nowArmors[1];

        Eigen::Vector3d Location1;
        Eigen::Vector3d Location2;
        PredictorTool::GetRelativeCorrdinate(Distances[0],armor1.Center,false,modelParam,&Location1);
        PredictorTool::GetRelativeCorrdinate(Distances[1],armor2.Center,false,modelParam,&Location2);

        cv::Point2d ctPts;
        AntiTop_.computeCenter(nowArmors,&Radius,Location1,Location2,&ctPts,nowYaws[0],nowYaws[1]);
//        std::cout << Radius[0] << " " << Radius[1] << std::endl;
        centerLocation -> x = ctPts.x;
        centerLocation -> y = ctPts.y;

        AntiTop_.giveIndexToArmor(nowArmors,Radius,&numberindex);
    };

    //获取上一帧数据
    std::vector<ClassicalArmor> lastArmors = std::get<0>(armorDataSequence->at(armorDataSize - 2));
    std::vector<float> lastYaws = std::get<1>(armorDataSequence->at(armorDataSize - 2));
//    std::cout << armornum << std::endl;
    //根据装甲板数目进行处理
    switch (armornum) {
        case 0:{
//            std::cout << *disappearTime <<std::endl;
            //目标丢失大于两秒，则解除锁定状态
            if (*disappearTime > 2)
            {
                *disappearTime = 0;
                AntiTop_.IsLocked = false;
                dualarmor_first = false;
            }
            if (last_armornum != 0)
            {
                armornumchange = true;
                std::cout << "Lost target !!!" <<std::endl;
            }


//            std::cout << "ERROR: There are no target armor!" << std::endl;
        };break;
        case 1:{
            *disappearTime = 0;
            if (ori_dict && !Radi->empty())
            {
                AntiTop_.giveIndexToArmor(nowArmors,Radius,&numberindex);
            }

            if (last_armornum == 1)
            {
                armornumchange = false;
                AntiTop_.distinctOrientation(lastArmors,nowArmors);
            }

            //连续两帧装甲板数目发生跳变，则认为识别发生跳变
            if (last_armornum != 1 && armornumchange)
            {
                armorHop = true;
            }

            //装甲板数目发生跳变
            if (last_armornum != 1)
            {
                armornumchange = true;
            }
        };break;
        case 2:{

            *disappearTime = 0;

            //首次出现两块装甲板，则计算中心点
            if (dualarmor_first){
//                dualarmorsequence.emplace_back(nowArmors);
                dualarmor_first = false;
                getCenter();
            }

//            if (!ori_dict || sequence_size < 10){
//                dualarmorsequence.emplace_back(armors);
//            }

            //未判定方向，则判断选转方向
            if (!ori_dict && last_armornum == 2){
                AntiTop_.IsrotateRight = AntiTop_.distinctOrientation(lastArmors,nowArmors);
                armornumchange = false;
                ori_dict = true;
            }

            //每次装甲板数目从1到2，则计算一次中心点
            if (ori_dict && last_armornum == 1){
                getCenter();
            }
            //装甲板数目未发生跳变，则判断旋转方向
            if (ori_dict && last_armornum == 2){
                AntiTop_.IsrotateRight = AntiTop_.distinctOrientation(lastArmors,nowArmors);
                armornumchange = false;
            }

            //连续两帧装甲板数目发生跳变，则认为识别发生跳变
            if (last_armornum != 2 && armornumchange)
            {
                armorHop = true;
            }

            //装甲板数目发生跳变
            if (last_armornum != 2)
            {
                armornumchange = true;
            }


//            AntiTop_.predictAll(,);
        };break;
    }


    *numberIndex = numberindex;
//    last_armornum = armornum;
    *Radi = Radius;

}

// 打击线性运动目标的滤波器
void Predictor::FightLinearTarget(double &gapTime,
                                  PredictorModel &PredictFunc,
                                  Eigen::Vector3d  &WorldCoordinate,
                                  double euler){

    // 将世界坐标转化为云台角度
    AutoaimEKF_.Q = AutoaimEKF_.SetQMatrix(gapTime);
    AutoaimEKF_.R = param_.R;
//    std::cout << AutoaimEKF_.Q <<std::endl;
    PredictFunc.delta_t = gapTime;
    AutoaimEKF_.StatePredict(WorldCoordinate,euler,PredictFunc); //预测

    // 更新滤波器，输入坐标Yr，得到目标位置与速度的最优估计Xe
    AutoaimEKF_.StateUpdate(WorldCoordinate,euler);

}

// 击打前哨站部分代码
void Predictor::FightOutPost(bool isPredict,
                             Predictor *predictor,
                             HuarayCameraModelParam modelParam,
                             std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>>> *armorRecord,
                             double predict_time,
                             cv::Point2f *pixelTarget) {

//    // 预测器的互斥锁
//    std::lock_guard<std::mutex> lockGuard(operateMutex_);
//
//    // 缓存不为空则进行下一步
//    if ((armorLocationSequence->size()>5) && (quaterniondSequence->size()>5))
//    {
//        std::tuple<Eigen::Vector3d , Eigen::Matrix3d, float> resultTuple;
//        // 若为同一装甲板则进行下一步
//        if (PredictorTool::IsSameArmor(modelParam, armorLocationSequence, quaterniondSequence, resultTuple))
//        {
//            Eigen::Vector3d nowWorldCoordinate = std::get<0>(resultTuple);
//
//            // 将世界坐标转化为云台角度
//            Eigen::Matrix<double, 3, 1> Xr;
//            Xr << nowWorldCoordinate(0, 0), nowWorldCoordinate(1, 0), nowWorldCoordinate(2, 0);
//            Eigen::Matrix<double, 3, 1> Yr;
//
//            // 初始化观测值与预测方程
//            Measure Measure;
//            PredictorModel Model;
//
//            // 得到观测值并更新预测器
//            Measure::measure(Xr, Yr);
//            EKF_.OutPostPredict(Model);
//
//            // 更新滤波器，输入坐标得到，目标位置与速度的最优估计
//            Eigen::Matrix<double, 5, 1> Xe = EKF_.OutPostUpdate(Measure, Yr);
//
//            // 定义预测时间
//            Model.delta_t = static_cast<double>(predict_time);
//            Eigen::Matrix<double, 5, 1> Xp;
//
//            // 通过匀速直线模型预测
//            Model.OutPost(Xe, Xp);
//
//            // 得到预测位置
//            Eigen::Matrix<double, 3, 1> predictLocation;
//            predictLocation = {Xp(0, 0), Xp(2, 0), Xp(4, 0)};
//
//            // 若预测点不会消失则预测,最好根据实测调整
//            doubl                    // 进行线性预测e alpha = (Xp(1,0) +Xp(3,0))/2;
//            if(alpha < 6.28318 && alpha > 3.14159)
//            {
//            // 将其转化为像素坐标
//            PredictorTool::GetPixelLocation(nowDistance,
//                                     pixelTarget,
//                                     nowCamera2World,
//                                     modelParam,
//                                     predictLocation);
//            }
//            else{
//                /// 切换
//                // 若目标会消失则放弃跟随
//                pixelTarget.x = 0;
//                pixelTarget.y = 0;
//            }
//        }
//        // 若为新装甲板则直接下发当前坐标并且初始化滤波器
//        else {
//            // 将当前二维坐标发给下位机
//            pixelTarget = armorLocationSequence->front().first;
//
//            Eigen::Vector3d nowWorldCoordinate =  std::get<0>(resultTuple);
//
//            // 初始化滤波器
//            Eigen::Matrix<double, 5, 1> Xr;
//            Xr << nowWorldCoordinate(0, 0), 0, nowWorldCoordinate(1, 0), 0, nowWorldCoordinate(2, 0);
//            EKF_.init(Xr);
//        }
//    }
//    else{
//        // 若四元数或装甲板序列为空，则将当前二维坐标发给下位机
//        pixelTarget = armorLocationSequence->front().first;
//    }
}

// 得到四元数与其对应的时间戳的缓存
bool Predictor::GetQuaterniondSequence(std::vector<std::tuple<std::array<float, 4>, uint64_t, float>> *QuaterniondSequence)
{
    // 预测器互斥锁
    std::lock_guard<std::mutex> lockGuard(bufferMutex_);

    // 定义四元数与其在系统下的时间
    std::tuple<std::array<float, 4>, uint64_t, float> sequence;
    for (unsigned int i = 0; i < predictorBuffer_.size(); i++)
    {
        // 保存四元数与时间
        std::get<0>(sequence) = predictorBuffer_[i].Q;
        std::get<1>(sequence) = predictorBuffer_[i].timeStamp;
        std::get<2>(sequence) = predictorBuffer_[i].distance;
        QuaterniondSequence->emplace_back(sequence);
    }
}

// 更新当前预测缓存
bool Predictor::UpdateLocationBuffer(const PredictorTool &predictorTool){

    // 预测器互斥锁
    std::lock_guard<std::mutex> lockGuard(bufferMutex_);

    // 刷新当前预测缓存
    RefreshPredictorBuffer(predictorTool,&predictorBuffer_);

    return true;
}

// 刷新预测器缓冲区
void Predictor::RefreshPredictorBuffer(const PredictorTool &package,
                                       std::deque<PredictorTool> *predictorBuffer) const{

    //
    if(!predictorBuffer->empty())
    {
        unsigned long bufferSize = predictorBuffer->size();
        float nowDistance = package.distance;
        float historyDistance = predictorBuffer->at(bufferSize - 1).distance;
        float gapDistance = nowDistance - historyDistance;

        if(gapDistance > 500)
        {
            predictorBuffer->clear();
        }

    }

    //
    predictorBuffer->emplace_back(package);

    while (predictorBuffer->size() > 25)
    {
        predictorBuffer->pop_front();
    }
}

// 风车预测
void Predictor::WindmillPredict() {
    ///风车预测
}

void Predictor::setPrivateMember(int index,
                                 bool dual_flag) {
    armorIndex = index;

    if (!dual_flag)
    {
        dualarmor_first = true;
    }
}

void Predictor::GetAntitop(class AntiTop Anti) {
    AntiTop_ = Anti;
}

void Predictor::UpdateAntitop(struct AntiTop *Anti) {
    *Anti = AntiTop_;
}
