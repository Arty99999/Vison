//
// Created by tony on 2022/4/23.
//

#include <algorithm>
#include "predictor_tool.h"
#include "solver.h"

//构造函数
PredictorTool::PredictorTool():Q({0,0,0,0}),
                               timeStamp(0),
                               index(0),
                               sysTimeStamp(),
                               operateMutex_(),
                               dataMutex_(),
                               distance()
{
}

// 拷贝构造函数
PredictorTool::PredictorTool(const PredictorTool &predictorTool) {
    Q = predictorTool.Q;
    timeStamp = predictorTool.timeStamp;
    index = predictorTool.index;
    sysTimeStamp = predictorTool.sysTimeStamp;
    distance = predictorTool.distance;
    IsBlue = predictorTool.IsBlue;
}




bool PredictorTool::PackageData(uint64_t timestamp, std::vector<PredictorCommand> *BodyPredictorCommand)
{
    // 预测器的互斥锁
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    // 初始化结果值
    bool result = false;

    // 若预测指令存在
    if(!BodyPredictorCommand->empty())
    {
        uint64_t imuTimeStamp[5000];
        uint64_t MatchTime[5000];
        std::vector<uint64_t> disparityTime;
        double minValue = 0;
        int packI = -1;

        for(int j =0; j<5000; j++) imuTimeStamp[j] = 0;

        // 得到对比时间的值
        for(int i = 0; i < BodyPredictorCommand->size()-1; i++)
        {
            // 将每个IMU数据帧的时间戳读出来
            imuTimeStamp[i] = BodyPredictorCommand->at(i).nowTime;
            MatchTime[i] = std::labs(imuTimeStamp[i] - timestamp);//相对时间
            disparityTime.emplace_back(MatchTime[i]);
        }

        // 得到与图像时间差最小的四元数及其下标
        minValue = *std::min_element(disparityTime.begin(), disparityTime.end());

        packI = static_cast<int>(std::min_element(disparityTime.begin(), disparityTime.end()) - disparityTime.begin());

        disparityTime.clear();

        // 如果时间差距符合要求，则对数据进行打包
        if (packI != -1)
        {
            Q = BodyPredictorCommand->at(packI).Q;
            // 相机得到图像的时间戳
            timeStamp = timestamp;
            // BodyPredictorCommand 的索引
            index = packI;
            // 串口读取数据的时间戳
            sysTimeStamp = BodyPredictorCommand->at(packI).nowTime;
            IsBlue = BodyPredictorCommand->at(packI).IsBlue;
            result = true;
        }
        else
        {
            // 如果下位机发来的数据不符合要求，则对相机数据进行打包
            Q = {0, 0, 0, 0};
            timeStamp = timeStamp;
            index = 0;
            sysTimeStamp = timestamp;
            result = false;
        }
    }
    else
    {
        // 如果下位机发来的数据不存在，则对相机数据进行打包
        Q = {0, 0, 0, 0};
        timeStamp = timestamp;
        index = 0;
        sysTimeStamp = timestamp;
        result = false;
    }

    return result;
}


// 对四元数，图像，时间戳进行打包
bool PredictorTool::GetPackage(uint64_t timestamp,
                               std::vector<PredictorCommand> *BodyPredictorCommand){

    // 预测器的互斥锁
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化结果值
    bool result = false;

    // 若预测指令存在
    if(!BodyPredictorCommand->empty())
    {
        // 定义打包匹配时间
        double systemTimeZero;
        double uartTimeInSys[1000];
        uint64_t uartTimeGap[1000];
        double minValue = 0;
        int packI = -1;
        double PictureTime;
        double MatchTime[1000];
        std::vector<double> disparityTime;

        // 设置此次命令在系统时间下的时间起点
        systemTimeZero = static_cast<double>(BodyPredictorCommand->at(0).nowTime)/1000000;

        // 去除10s以上的大的值,并且减去相机采图延迟与通信延迟
        if(systemTimeZero > 10000)
        {
            int SystemTimeZero = static_cast<int>(systemTimeZero);
            systemTimeZero = static_cast<double> (SystemTimeZero % 10000) - 1;
        }
        PictureTime = static_cast<double>(timestamp)/1000000;
        if(PictureTime > 10000)
        {
            int pictureTime = static_cast<int>(PictureTime);
            PictureTime = static_cast<double>(pictureTime % 10000) - 2.5;
        }

        // 得到对比时间的值
        for(int i = 0; i < BodyPredictorCommand->size()-1; i++)
        {
            // 应该是5ms每个
            uartTimeGap[i] = BodyPredictorCommand->at(i).TimeStamp - BodyPredictorCommand->at(0).TimeStamp;
            // 如果数据超出范围则赋最大值
            if(uartTimeGap[i] > 2000)
            {
                uartTimeGap[i] = 2000;
            }
            uartTimeInSys[i] = systemTimeZero + static_cast<double>(uartTimeGap[i]);
            MatchTime[i] = std::fabs(uartTimeInSys[i] - PictureTime);
            disparityTime.emplace_back(MatchTime[i]);
        }

        // 得到与图像时间差最小的四元数及其下标
        minValue = *std::min_element(disparityTime.begin(), disparityTime.end());
        packI = static_cast<int>(std::min_element(disparityTime.begin(), disparityTime.end()) - disparityTime.begin());

//        std::cout<<"minValue: "<<minValue<<"  packI: "<<packI<<std::endl;
        disparityTime.clear();

        // 如果时间差距符合要求，则对数据进行打包
        if (packI != -1)
        {
            Q = BodyPredictorCommand->at(packI).Q;
            // 相机得到图像的时间戳
            timeStamp = timestamp;
            // BodyPredictorCommand 的索引
            index = packI;
            // 串口读取数据的时间戳
            sysTimeStamp = BodyPredictorCommand->at(packI).nowTime;
            IsBlue = BodyPredictorCommand->at(packI).IsBlue;
            stand = BodyPredictorCommand->at(packI).stand;
            result = true;
        }
        else
        {
            // 如果下位机发来的数据不符合要求，则对相机数据进行打包
            Q = {0, 0, 0, 0};
            timeStamp = timestamp;
            index = 0;
            sysTimeStamp = timestamp;
            result = false;
        }
    }
    else
    {
        // 如果下位机发来的数据不存在，则对相机数据进行打包
        Q = {0, 0, 0, 0};
        timeStamp = timestamp;
        index = 0;
        sysTimeStamp = timestamp;
        result = false;
    }

    return result;
}

// 工具函数，得到相机与世界的变换矩阵
void PredictorTool::GetRotationMatrix(std::array<float, 4> quaterniond,
                                      Eigen::Isometry3d &Camera2IMU,
                                      Eigen::Matrix3d &IMU2World,
                                      Eigen::Matrix3d &Camera2World) {
    // 由于旋转矩阵是正交阵，转置等于求逆
    Eigen::Quaterniond Q_raw(quaterniond[0],quaterniond[1],quaterniond[2],quaterniond[3]);
    Eigen::Quaterniond Q(Q_raw.matrix().transpose());
    Eigen::Matrix3d QMatrix = Q.matrix().cast<double>();
    Camera2World = (Camera2IMU.rotation() * QMatrix * IMU2World).transpose();
}

// 判断是不是同一个装甲板
bool PredictorTool::IsSameArmor(HuarayCameraModelParam &modelParam,
                                std::vector<std::pair<cv::Point2f, uint64_t>> *armorLocationSequence,
                                std::vector<std::tuple<std::array<float, 4>, uint64_t, float>> *quaterniondSequence,
                                std::tuple<Eigen::Vector3d , Eigen::Matrix3d, float> resultTuple) {
//    // 初始化标志位
//    bool isSame = false;
//
//    // 得到最近两次坐标的时间差
//    unsigned long armorSize;
//    unsigned long QSize;
//    uint64_t nowTime;
//    uint64_t pastTime;
//    armorSize = armorLocationSequence -> size();
//    QSize = quaterniondSequence -> size();
//    nowTime = armorLocationSequence -> at(armorSize - 1).second;
//    pastTime = armorLocationSequence -> at(armorSize - 2).second;
//    uint64_t gapTime = (nowTime - pastTime)/1000000;
//
//    // 如果间隔时间大于6ms，则认为不是同一块装甲板
//    if(gapTime < 6)
//    {
//        // 得到当前目标二维坐标,四元数和距离
//        cv::Point2f nowLocation = armorLocationSequence->at(armorSize - 1).first;
//        std::array<float, 4> nowQuaterniond = std::get<0>(quaterniondSequence->at(QSize-1));
//        float nowDistance = std::get<2>(quaterniondSequence->at(QSize-1));
//        Eigen::Vector3d nowCameraCoordinate;
//        Eigen::Matrix3d nowCamera2World;
//
//        // 得到当前目标的世界坐标
//        PredictorTool::GetCameraCorrdinate(nowDistance,
//                                        nowLocation,
//                                        false,
//                                        modelParam,
//                                        nowCameraCoordinate);
//        PredictorTool::GetRotationMatrix(nowQuaterniond,modelParam.EigenCameraIMU, nowCamera2World);
//        std::get<0>(resultTuple) = nowCamera2World * nowCameraCoordinate;
//        std::get<1>(resultTuple) = nowCamera2World;
//        std::get<2>(resultTuple) = nowDistance;
//        std::cout<<std::get<0>(resultTuple).x()<<"  "<<std::get<0>(resultTuple).y()<<"  "<<std::get<0>(resultTuple).z()<<std::endl;
//
//        // 得到目标二维坐标,四元数和距离
//        cv::Point2f pastLocation = armorLocationSequence->at(armorSize - 2).first;
//        std::array<float, 4> pastQuaterniond = std::get<0>(quaterniondSequence->at(QSize-2));
//        float pastDistance = std::get<2>(quaterniondSequence->at(QSize-2));
//        Eigen::Vector3d pastCameraCoordinate;
//        Eigen::Matrix3d pastCamera2World;
//
//        // 得到当前目标的世界坐标
//        PredictorTool::GetCameraCorrdinate(pastDistance,
//                                        pastLocation,
//                                        false,
//                                        modelParam,
//                                        pastCameraCoordinate);
//        PredictorTool::GetRotationMatrix(pastQuaterniond, modelParam.EigenCameraIMU, pastCamera2World);
//        Eigen::Vector3d pastWorldCoordinate = pastCamera2World * pastCameraCoordinate;
//
//        // 计算两者距离之差
//        double gapXDsitance = (std::get<0>(resultTuple).x()-pastWorldCoordinate.x());
//        double gapYDsitance = (std::get<0>(resultTuple).y()-pastWorldCoordinate.y());
//        double gapZDsitance = (std::get<0>(resultTuple).z()-pastWorldCoordinate.z());
//        double gapDistance = std::sqrt(gapXDsitance * gapXDsitance+gapYDsitance * gapYDsitance+gapZDsitance * gapZDsitance);
//
//        // 如果距离差距小于十厘米则认为是同一块装甲板
//        if (gapDistance < 100)
//        {
//            isSame = true;
//            std::cout<<"They are same armor"<<std::endl;
//        }
//    }
//
//    // 最后返回判别结果
//    return isSame;
}

// 得到在世界坐标系位置上的相机坐标系坐标
void PredictorTool::GetRelativeCorrdinate(const float &distance,
                                          const cv::Point2f &target,
                                          const bool &isCorrectDistortion,
                                          const HuarayCameraModelParam &modelParam,
                                          Eigen::Vector3d *CradleHeadCoordinate){

    auto correctDistortion =[](const cv::Point2f &disortedTarget,
                               const HuarayCameraModelParam &modelParam,
                               cv::Point2f *correctedTarget) ->void{
        // 将目标点的坐标保存在Mat中
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = disortedTarget.x;
        mat.at<float>(0, 1) = disortedTarget.y;

        // 调整mat的通道为2，矩阵的行列形状不变
        mat=mat.reshape(2);
        cv::undistortPoints(mat,
                            mat,
                            modelParam.CvIntrinsics,
                            modelParam.CvDistortions,
                            cv::Mat(),
                            modelParam.CvIntrinsics);
        mat=mat.reshape(1);

        // 存储校正后的目标点
        correctedTarget->x = mat.at<float>(0, 0);
        correctedTarget->y = mat.at<float>(0, 1);
    };

    // 建立目标点像素坐标的齐次形式
    Eigen::Vector3d pixelCoordinate(target.x, target.y, 1);

    // 判断是否需要对目标点进行畸变校正
    if (isCorrectDistortion)
    {
        // 对目标点进行畸变校正
        cv::Point2f correctedTarget;

        correctDistortion(target, modelParam, &correctedTarget);

        // 更新目标点像素坐标的齐次形式
        pixelCoordinate(0) = correctedTarget.x;
        pixelCoordinate(1) = correctedTarget.y;

    }

    // 计算目标点方向与归一化平面的交点坐标
    Eigen::Vector3d normalizedCoordinate = modelParam.EigenIntrinsics.inverse() * pixelCoordinate;

    double n_x = normalizedCoordinate.x();
    double n_y = normalizedCoordinate.y();
    double n_z = normalizedCoordinate.z();

    // 计算目标点的相机坐标系坐标
    double ratio = distance / std::sqrt(n_x * n_x + n_y * n_y + n_z * n_z);
    double c_x = n_x*ratio;
    double c_y = n_y*ratio;
    double c_z = n_z*ratio;
    Eigen::Vector3d cameraCoordinate(c_x, c_y, c_z);

    // 得到相对坐标系坐标，使用了相机外参数矩阵
    Eigen::Vector3d translationVector = modelParam.EigenExtrinsics.translation();
    Eigen::Matrix3d rotationmat = modelParam.EigenExtrinsics.rotation();

    //TODO 此处调试时有改动，记得上车时调回来
    Eigen::Vector3d coordinate = rotationmat.inverse()*(cameraCoordinate - translationVector);
//    CradleHeadCoordinate->x() = cameraCoordinate.x();
//    CradleHeadCoordinate->y() = cameraCoordinate.y();
//    CradleHeadCoordinate->z() = cameraCoordinate.z();
    CradleHeadCoordinate->x() = coordinate.x();
    CradleHeadCoordinate->y() = coordinate.y();
    CradleHeadCoordinate->z() = coordinate.z();
}

// 从三维坐标得到二维坐标(陀螺仪坐标系->像素坐标系)
void PredictorTool::GetPixelLocation(const float &distance,
                                     cv::Point2f *target,
                                     const HuarayCameraModelParam &modelParam,
                                     Eigen::Vector3d &relativeCorrdinate){

    Eigen::Vector3d translationVector = modelParam.EigenExtrinsics.translation();
    Eigen::Matrix3d rotatin=modelParam.EigenExtrinsics.rotation();
    Eigen::Vector3d cameraCoordinate = rotatin*relativeCorrdinate + translationVector;
//    std::cout << cameraCoordinate <<std::endl;

    // 得到像素坐标系坐标
    cv::Mat ma;
    cv::eigen2cv(modelParam.EigenIntrinsics,ma);

    double ratio =  distance;
    double n_x =  cameraCoordinate.x()/cameraCoordinate.z();
    double n_y =  cameraCoordinate.y()/cameraCoordinate.z();
    double n_z =  cameraCoordinate.z()/cameraCoordinate.z();
    Eigen::Vector3d normalizedCoordinate(n_x,n_y,n_z);
//    std::cout<<"p'normalizedCoordinate: "<<normalizedCoordinate.transpose()<<std::endl;
    Eigen::Vector3d pixelCoordinate = modelParam.EigenIntrinsics * normalizedCoordinate;
//    std::cout << normalizedCoordinate << std::endl;
//    std::cout << ma << std::endl;
//    std::cout << modelParam.EigenIntrinsics <<std::endl;
//   std::cout << pixelCoordinate << std::endl;
    //将畸变加上
    /*double u=pixelCoordinate.x();
    double v=pixelCoordinate.y();
    double r=pixelCoordinate.x()*pixelCoordinate.x()+pixelCoordinate.y()*pixelCoordinate.y();

    pixelCoordinate.x()=u*(1+modelParam.EigenDistortions[0]*r+modelParam.EigenDistortions[1]*r*r+
            modelParam.EigenDistortions[4]*r*r*r)+2*modelParam.EigenDistortions[2]*u*v+modelParam.EigenDistortions[3]*(r+2*u*u);
    pixelCoordinate.y()=v*(1+modelParam.EigenDistortions[0]*r+modelParam.EigenDistortions[1]*r*r+
                           modelParam.EigenDistortions[4]*r*r*r)+2*modelParam.EigenDistortions[3]*v+modelParam.EigenDistortions[2]*(r+2*v*v);*/
    // 将像素坐标系坐标保存
    target->x = static_cast<float>(pixelCoordinate.x());
    target->y = static_cast<float>(pixelCoordinate.y());
}

// 从三维坐标得到二维坐标(相机坐标系->像素坐标系)
void PredictorTool::CameraGetPixelLocation(cv::Point2f *target,
                                           const HuarayCameraModelParam &modelParam,
                                           Eigen::Vector3d &relativeCorrdinate){

    Eigen::Vector3d cameraCoordinate = relativeCorrdinate;
    cv::Mat ma;
    cv::eigen2cv(modelParam.EigenIntrinsics,ma);

    double n_x =  cameraCoordinate.x()/cameraCoordinate.z();
    double n_y =  cameraCoordinate.y()/cameraCoordinate.z();
    double n_z =  cameraCoordinate.z()/cameraCoordinate.z();
    Eigen::Vector3d normalizedCoordinate(n_x,n_y,n_z);
//    std::cout<<"p'normalizedCoordinate: "<<normalizedCoordinate.transpose()<<std::endl;
    Eigen::Vector3d pixelCoordinate = modelParam.EigenIntrinsics * normalizedCoordinate;

    // 将像素坐标系坐标保存
    target->x = static_cast<float>(pixelCoordinate.x());
    target->y = static_cast<float>(pixelCoordinate.y());
}

//计算两条直线夹角的绝对值
float PredictorTool::getAbsAngle(Eigen::Vector2d Line1,
                                 Eigen::Vector2d Line2) {
    float angle1 = atan(Line1.y()/Line1.x());
    float angle2 = atan(Line2.y()/Line2.x());

//    std::cout << angle1 << std::endl;
    float angle = std::abs(angle1 - angle2);
    return angle;

}

//获取装甲板中心点的三维坐标
Eigen::Vector3d PredictorTool::getArmorCenterLocation(ClassicalArmor armor,
                                                      HuarayCameraModelParam &modelParam,
                                                      float *distance,
                                                      Eigen::Matrix3d *rotationMat) {

    auto computeDistance = [&](const std::vector<cv::Point3f> &objectPoints,
                               const std::vector<cv::Point2f> &pixelPoints,
                               const HuarayCameraModelParam &modelParam) ->std::pair<float , Eigen::Matrix<double, 3, 3>>
    {
        // 计算旋转向量和平移向量
        cv::Mat rotation(3, 1, CV_32F);
        cv::Mat translation(3, 1, CV_32F);
        cv::solvePnP(objectPoints,
                     pixelPoints,
                     modelParam.CvIntrinsics,
                     modelParam.CvDistortions,
                     rotation,
                     translation,
                     false,
                     cv::SOLVEPNP_IPPE);
        //std::cout<<rotation<<std::endl;
        cv::Mat rotationMat(3,3,CV_32F);
        cv::Rodrigues(rotation, rotationMat);
        Eigen::Matrix<double, 3, 3> rotationMatrix;
        cv::cv2eigen(rotationMat,rotationMatrix);
        // 计算距离；注意：使用平移向量的模作为距离比直接使用z轴的平移更准确一些
        // float distance = translation.at<float>(2, 0);
        float distance = std::sqrt(translation.at<float>(0, 0) * translation.at<float>(0, 0)
                                   + translation.at<float>(1, 0) * translation.at<float>(1, 0)
                                   + translation.at<float>(2, 0) * translation.at<float>(2, 0));
        //std::cout<<"Matrix: "<<rotationMatrix<<std::endl;
        Eigen::Matrix<double, 3, 3> R;
        R << 1,0,0,0,0,-1,0,1,0;
        rotationMatrix = R * rotationMatrix.inverse();
        // 返回计算结果
        return {distance,rotationMatrix};
    };
    std::vector<cv::Point2f> pixelPts;
    std::vector<cv::Point3f> objectPoints;

    // 此处的装甲板尺寸要随模型的变化而为之适应，yaml文件中的尺寸是在传统视觉下得到的尺寸，
    // 较为精准，但yolo标出来的角点随模型变化会有一定的变化，故尺寸也要随之变化
    if (armor.Number == EClassicalArmorNumber::BlueOne || armor.Number == EClassicalArmorNumber::RedOne ||
        armor.Number == EClassicalArmorNumber::BlueSentry || armor.Number  == EClassicalArmorNumber::RedSentry)
    {
        float laWidth =240 - 10;
        float laHeight = 48;

        objectPoints.emplace_back( -0.5*laWidth, 0.5*laHeight, 0);
        objectPoints.emplace_back( 0.5*laWidth, 0.5*laHeight, 0);
        objectPoints.emplace_back( 0.5*laWidth, -0.5*laHeight, 0);
        objectPoints.emplace_back( -0.5*laWidth, -0.5*laHeight, 0);
    }
    else if (armor.Number != EClassicalArmorNumber::Invalid)
    {
        float loWidth = 134 - 10;
        float loHeight = 48;
        objectPoints.emplace_back( -0.5*loWidth, 0.5*loHeight, 0);
        objectPoints.emplace_back( 0.5*loWidth, 0.5*loHeight, 0);
        objectPoints.emplace_back( 0.5*loWidth, -0.5*loHeight, 0);
        objectPoints.emplace_back( -0.5*loWidth, -0.5*loHeight, 0);
    }

    pixelPts.emplace_back(armor.LeftUpper);
    pixelPts.emplace_back(armor.RightUpper);
    pixelPts.emplace_back(armor.RightLower);
    pixelPts.emplace_back(armor.LeftLower);

    float Distance = computeDistance(objectPoints,pixelPts,modelParam).first;
    Eigen::Vector3d armorCenterLocation;

    GetRelativeCorrdinate(Distance,armor.Center, false,modelParam,&armorCenterLocation);
    if (distance != nullptr)
        *distance = Distance;
    if (rotationMat != nullptr)
        *rotationMat = computeDistance(objectPoints,pixelPts,modelParam).second;

    return armorCenterLocation;

}

void PredictorTool::eigenVec3d2cvPt3f(Eigen::Vector3d &eigenPoint,
                                      cv::Point3f *cvPoint) {

    cvPoint ->x = eigenPoint.x();
    cvPoint ->y = eigenPoint.y();
    cvPoint ->z = eigenPoint.z();
}

void PredictorTool::pnpArray(HuarayCameraModelParam &modelParam, ClassicalArmor &armor,
                             std::vector<cv::Point3f> *Coordinate) {
    std::vector<cv::Point2f> armorPts;

    armorPts.emplace_back(armor.LeftUpper);
    armorPts.emplace_back(armor.RightUpper);
    armorPts.emplace_back(armor.RightLower);
    armorPts.emplace_back(armor.LeftLower);

    std::vector<cv::Point3f> objectPts;
    float height = 50;
    float width = 124;

    //以左上角点为原点的世界坐标系坐标
    objectPts.emplace_back(0,0,0);
    objectPts.emplace_back(width,0,0);
    objectPts.emplace_back(width,-height,0);
    objectPts.emplace_back(0,-height,0);

    //世界坐标系位移量
    std::vector<cv::Mat> translations;
    translations.emplace_back((cv::Mat_<float>(1,3) << -width,0,0));
    translations.emplace_back((cv::Mat_<float>(1,3) << 0,height,0));
    translations.emplace_back((cv::Mat_<float>(1,3) << width,0,0));
    translations.emplace_back((cv::Mat_<float>(1,3) << 0,0,0));

    //将vector转变为cvMat
    cv::Mat object(objectPts.size(), 3, CV_32FC1);
    for (int i = 0; i < objectPts.size(); ++i) {
        object.at<float>(i, 0) = objectPts[i].x;
        object.at<float>(i, 1) = objectPts[i].y;
        object.at<float>(i, 2) = objectPts[i].z;
    }

    //返回装甲板距离距离
    cv::Mat rotationMat;
    cv::Mat translationMat;


    for (int i = 0; i < 4; ++i) {
        cv::solvePnP(objectPts,
                     armorPts,
                     modelParam.CvIntrinsics,
                     modelParam.CvDistortions,
                     rotationMat,
                     translationMat);

        Coordinate -> emplace_back(cv::Point3f(translationMat));
        cv::Mat TMat;
        //重复位移量，使得四个交点一起位移
        cv::repeat(translations[i],armorPts.size(),1,TMat);
//        std::cout << object.size<< "  " << TMat.size << std::endl;
        object = object + TMat;

        for (int j = 0; j < object.rows; ++j) {
            objectPts[j] = object.at<cv::Point3f>(j,0);
        }
    }

//    std::cout << objectPts << std::endl;

}

void PredictorTool::pictureAllArmor(std::vector<cv::Point3f> &Pts,
                                    cv::Mat *image,
                                    float yaw,
                                    HuarayCameraModelParam &modelParam,
                                    bool IsLargeArmor) {

    //根据大小装甲板给出矩形长宽
    float hei,wid;
    wid = IsLargeArmor? 230 :124;
    hei = 48;

    float sin15 = sin(15/57.3);
    float cos15_2 = cos(15/57.3);

    //绘制一块装甲板
    auto pictureAArmor = [&](cv::Point3f &Pt,
                             float &yaw,
                             bool front) ->void{

        //这个参数是用来绘制前后装甲板的参数
        int constant = front? 1 : -1;

        float sinyaw_2 = sin(yaw)/2;
        float abs_cosyaw_2 = abs(cos(yaw) / 2);

        std::vector<cv::Point2f> pixelPts;
        std::vector<Eigen::Vector3d> cornerPts;

        //根据中心点和yaw拓展出四个角点(和Remap类中的Compare函数一样)
        cornerPts.emplace_back(Pt.x - wid * abs_cosyaw_2 - hei * sin15 * sinyaw_2 ,
                               Pt.y - hei * cos15_2 ,
                               Pt.z - wid * sinyaw_2 + constant * hei * sin15 * abs_cosyaw_2);
        cornerPts.emplace_back(Pt.x + wid * abs_cosyaw_2 - hei * sin15 * sinyaw_2 ,
                               Pt.y - hei * cos15_2 ,
                               Pt.z + wid * sinyaw_2 + constant * hei * sin15 * abs_cosyaw_2);
        cornerPts.emplace_back(Pt.x + wid * abs_cosyaw_2 + hei * sin15 * sinyaw_2 ,
                               Pt.y - hei * cos15_2 ,
                               Pt.z + wid * sinyaw_2 - constant * hei * sin15 * abs_cosyaw_2);
        cornerPts.emplace_back(Pt.x - wid * abs_cosyaw_2 + hei * sin15 * sinyaw_2 ,
                               Pt.y - hei * cos15_2 ,
                               Pt.z - wid * sinyaw_2 - constant * hei * sin15 * abs_cosyaw_2);

        for (int i = 0; i < cornerPts.size(); ++i) {
            GetPixelLocation(0,&pixelPts[i],modelParam,cornerPts[i]);
        }
        for (int j = 0; j < 4; ++j) {
            int u = (j + 1) % 4;
            cv::line(*image,pixelPts[j],pixelPts[u],cv::Scalar (0,255,255),2);
        }
    };

    //车的前面装甲板和后面装甲板的集合(各两块)
    std::vector<cv::Point3f> frontArmor;
    std::vector<cv::Point3f> behindArmor;

    //
    if (Pts[0].x > Pts[2].x && Pts[0].y < Pts[2].y){
        frontArmor.emplace_back(Pts[0]);
        frontArmor.emplace_back(Pts[3]);
        behindArmor.emplace_back(Pts[1]);
        behindArmor.emplace_back(Pts[2]);
    }
    else{
        frontArmor.emplace_back(Pts[0]);
        frontArmor.emplace_back(Pts[1]);
        behindArmor.emplace_back(Pts[3]);
        behindArmor.emplace_back(Pts[2]);
    }
    //注意：这里需要frontarmors和behindarmors的0号元素同在左或同在右
    float yaw1 =  yaw? yaw-90/57.3:yaw+90/57.3;
    pictureAArmor(frontArmor[0],yaw,1);
    pictureAArmor(frontArmor[1], yaw1,1);
    pictureAArmor(behindArmor[0],yaw,0);
    pictureAArmor(behindArmor[1],yaw1,0);

}

// 判断当前目标是否在小陀螺
bool PredictorTool::IsInRotating(HuarayCameraModelParam &modelParam,
                                 std::vector<std::pair<cv::Point2f, uint64_t>> *armorLocationSequence,
                                 std::vector<std::tuple<std::array<float, 4>, uint64_t, float>> *quaterniondSequence) {

    bool isRotating = false;

}

//
void PredictorTool::GetAll(Eigen::Vector3d &world, double yaw, double r, Eigen::Matrix<double, 3, 3> rotation,
                           cv::Point2f &Get1, cv::Point2f &Get2,
                           cv::Point2f &Get3, cv::Point2f &Get4,
                           const HuarayCameraModelParam &modelParam) {
    Eigen::Vector3d armor1,armor2,armor3,armor4;
    Eigen::Vector3d relaarmor1,relaarmor2,relaarmor3,relaarmor4;
    armor1.x() = world.x() + r * sin(yaw);
    armor1.y() = world.y() - r * cos(yaw);
    armor1.z() = world.z();
    armor2.x() = world.x() - r * cos(yaw);
    armor2.y() = world.y() - r * sin(yaw);
    armor2.z() = world.z();
    armor3.x() = world.x() + r * cos(yaw);
    armor3.y() = world.y() + r * sin(yaw);
    armor3.z() = world.z();
    armor4.x() = world.x() - r * sin(yaw);
    armor4.y() = world.y() + r * cos(yaw);
    armor4.z() = world.z();
    relaarmor1 = rotation.inverse() * armor1;
    relaarmor2 = rotation.inverse() * armor2;
    relaarmor3 = rotation.inverse() * armor3;
    relaarmor4 = rotation.inverse() * armor4;
    double dis1 = std::sqrt(relaarmor1.x() * relaarmor1.x() + relaarmor1.y() * relaarmor1.y() + relaarmor1.z() * relaarmor1.z());
    double dis2 = std::sqrt(relaarmor2.x() * relaarmor2.x() + relaarmor2.y() * relaarmor2.y() + relaarmor2.z() * relaarmor2.z());
    double dis3 = std::sqrt(relaarmor3.x() * relaarmor3.x() + relaarmor3.y() * relaarmor3.y() + relaarmor3.z() * relaarmor3.z());
    double dis4 = std::sqrt(relaarmor4.x() * relaarmor4.x() + relaarmor4.y() * relaarmor4.y() + relaarmor4.z() * relaarmor4.z());
    PredictorTool::GetPixelLocation(dis1,&Get1,modelParam,relaarmor1);
    PredictorTool::GetPixelLocation(dis2,&Get2,modelParam,relaarmor2);
    PredictorTool::GetPixelLocation(dis3,&Get3,modelParam,relaarmor3);
    PredictorTool::GetPixelLocation(dis4,&Get4,modelParam,relaarmor4);
}

void PredictorTool::AVF(Eigen::Vector3d &worldC,unsigned long recordSize,Eigen::Matrix<double, 10, 3> &a,int &count)
{
    if(count == 10) count = 0;
    a(count,0) = worldC.x();
    a(count,1) = worldC.y();
    a(count,2) = worldC.z();
    count ++;
    worldC.x() = 0;
    worldC.y() = 0;
    worldC.z() = 0;
    for(int i = 0; i < 10; i++)
    {
        worldC.x() += a(i,0);
        worldC.y() += a(i,1);
        worldC.z() += a(i,2);
    }
    worldC.x() = worldC.x() / 10;
    worldC.y() = worldC.y() / 10;
    worldC.z() = worldC.z() / 10;
}

void PredictorTool::AVF1(double &rad,unsigned long recordSize,Eigen::Matrix<double, 10, 1> &b,int &count)
{
    if(count == 10) count = 0;
//    std::cout<<count<<std::endl;
    b(count,0) = rad;
    count ++;
    rad = 0;
    for(int i = 0; i < 10; i++)
    {
        rad += b(i,0);
    }

    rad = rad / 10;

}















