//
// Created by tony on 2022/4/23.
//

#ifndef CUBOT_BRAIN_PREDICTOR_TOOL_H
#define CUBOT_BRAIN_PREDICTOR_TOOL_H

#include "huaray_camera.h"
#include "easy_logger.h"
#include "robot_brain_core_param.h"
#include <cmath>
#include <algorithm>
#include "iostream"
#include "solver.h"
#include "classical_armor.h"
#include "predictor_command.h"
#include <shared_mutex>
#include <vector>

struct Tool{
    static const int Vec3d2Pt3f = 0;
    static const int Vec3d2Pt3d = 1;
    static const int Vec3f2Pt3f = 2;
    static const int Vec3i2Pt3i = 3;
    static const int Vec2d2Pt2f = 4;
    static const int Pt3f2Vec3d = 5;
    static const int Pt3d2Vec3d = 6;
    static const int Pt3i2Vec3i = 7;
    static const int Pt2f2Vec2d = 8;
};


/**
 * @brief 预测工具类
 * @note 为了适配系统，设计了部分工具
 */
class PredictorTool{


public:

    /**
     * @brief 构造函数
     */
    PredictorTool();

    /**
     * @brief 析构函数
     */
    ~PredictorTool()=default;

    /**
     * @brief 拷贝构造函数
     */
    PredictorTool(const PredictorTool &predictorTool);

    /**
     * @brief 数据打包工具
     */
    bool GetPackage(uint64_t timeStamp,
                    std::vector<PredictorCommand> *BodyPredictorCommand);

    /**
     * @brief 数据打包工具
     */
    bool PackageData(uint64_t timeStamp,
                     std::vector<PredictorCommand> *BodyPredictorCommand);

    /**
     * @brief 坐标系转换工具
     * @note 相机坐标系到世界坐标系
     */
    static void GetRotationMatrix(std::array<float, 4> quaterniond,
                                  Eigen::Isometry3d &Camera2IMU,
                                  Eigen::Matrix3d &IMU2World,
                                  Eigen::Matrix3d &Camera2World) ;



    /**
     * @brief 判断是否是同一装甲板
     */
    static bool IsSameArmor(HuarayCameraModelParam &modelParam,
                            std::vector<std::pair<cv::Point2f, uint64_t>> *armorLocationSequence,
                            std::vector<std::tuple<std::array<float, 4>, uint64_t, float>> *quaterniondSequence,
                            std::tuple<Eigen::Vector3d , Eigen::Matrix3d, float> resultTuple);

    /**
    * @brief 由图像的像素点通过相机内外参，得到云台坐标系的打击点
    * @param[in] distance                        pnp结算的距离
    * @param[in] target                          图像坐标系的像素点
    * @param[in] isCorrectDistortion             是否进行畸变矫正
    * @param[in] modelParam                      相机参数
    * @param[out] CradleHeadCoordinate           云台坐标系下的打击点
    * @return 是否转换到云台坐标系\n
    *         -<em>false</em> 尚未转换到云台坐标系\n
    *         -<em>true</em> 已经转换到云台坐标系\n
    */
    static void GetRelativeCorrdinate(const float &distance,
                                      const cv::Point2f &target,
                                      const bool &isCorrectDistortion,
                                      const HuarayCameraModelParam &modelParam,
                                      Eigen::Vector3d *CradleHeadCoordinate);
    /**
    * @brief 得到像素系坐标
    */
    static bool GetpixelCorrdinate(const float &distance,
                                   const cv::Point2f *target,
                                   const bool &isCorrectDistortion,
                                   const HuarayCameraModelParam &modelParam,
                                   Eigen::Vector3d &relativeCoordinate);
    /**
     * @brief 得到像素坐标系坐标(陀螺仪坐标系->像素坐标系)
     * @param target 返回的像素坐标值
     * @param modelParam 相机参数
     * @param relativeCorrdinate 陀螺仪坐标系下的三维坐标点
     *
     */
    static void GetPixelLocation(const float &distance,
                                 cv::Point2f *target,
                                 const HuarayCameraModelParam &modelParam,
                                 Eigen::Vector3d &relativeCorrdinate);

    /**
     * @brief 得到像素坐标系坐标(相机坐标系->像素坐标系)
     * @param target 返回的像素坐标值
     * @param modelParam 相机参数
     * @param relativeCorrdinate 相机坐标系下的三维坐标点
     */
    static void CameraGetPixelLocation(cv::Point2f *target,
                                       const HuarayCameraModelParam &modelParam,
                                       Eigen::Vector3d &relativeCorrdinate);

    /**
     * @brief 计算两条直线夹角的绝对值
     * @param Line1 第一条直线(两点的差)
     * @param Line2 第二条直线(两点的差)
     */
    static float getAbsAngle(Eigen::Vector2d Line1,
                             Eigen::Vector2d Line2);

/**
 * @brief 获取装甲板中心点的三维坐标
 * @param armor 装甲板
 * @param modelParam 相机参数
 * @return 装甲板三维坐标
 */
    static Eigen::Vector3d getArmorCenterLocation(ClassicalArmor armor,
                                                  HuarayCameraModelParam &modelParam,
                                                  float *distance = NULL,
                                                  Eigen::Matrix3d *rotationMat = NULL);


    /**
    * @brief Eigen::Vector3d转cv::Point3f
    * @param eigenPoint
    * @param cvPoint
    */
    static void eigenVec3d2cvPt3f(Eigen::Vector3d &eigenPoint,
                                  cv::Point3f *cvPoint);

    /**
     * @brief eigenVec类和cvPoint类相互转换
     * @tparam T 输入参数，eigenVec和cvPoint均可
     * @tparam U 输出参数
     * @param a
     * @param b
     * @param tool 转换的数据类型,为Tool结构体的成员
     */
    template<class T,class U>
    static void EigenCvSwap(T a, U *b, int tool);

    /**
 * @brief 获得装甲版角点的空间坐标
 * @param modelParam 相机参数
 * @param armor 装甲板
 * @param Coordinate 空间坐标
 */
    static void pnpArray(HuarayCameraModelParam &modelParam,
                         ClassicalArmor &armor,
                         std::vector<cv::Point3f> *Coordinate);

    /**
     * @brief 绘制车的所有预测装甲板
     * @param Pts 车装甲板的三维点集
     * @param image 显示图像
     * @param yaw 目标装甲板的yaw值
     * @param modelParam 相机参数
     * @param IsLargeArmor 是否为大装甲板
     *        /[true]   绘制大装甲板
     *        /[false]  绘制小装甲板
     * @note
     */
    static void pictureAllArmor(std::vector<cv::Point3f> &Pts,
                                cv::Mat *image,
                                float yaw,
                                HuarayCameraModelParam &modelParam,
                                bool IsLargeArmor);


    static void GetAll(Eigen::Vector3d &world,
                       double yaw,
                       double r,
                       Eigen::Matrix<double, 3, 3> rotation,
                       cv::Point2f &Get1,
                       cv::Point2f &Get2,
                       cv::Point2f &Get3,
                       cv::Point2f &Get4,
                       const HuarayCameraModelParam &modelParam);

    static void AVF(Eigen::Vector3d &world,
                    unsigned long recordSize,
                    Eigen::Matrix<double, 10, 3> &a,
                    int &count);

    static void AVF1(double &rad,
                     unsigned long recordSize,
                     Eigen::Matrix<double, 10, 1> &r,
                     int &count);

    /**
     * @brief 判断是否在小陀螺
     */
    static bool IsInRotating(HuarayCameraModelParam &modelParam,
                             std::vector<std::pair<cv::Point2f, uint64_t>> *armorLocationSequence,
                             std::vector<std::tuple<std::array<float, 4>, uint64_t, float>> *quaterniondSequence);

    float distance;                                     ///< 目标与相机的距离
    uint64_t frameTimeStamp;                            ///< 相机解算完数据后本地的时间戳
    std::array<float, 4> Q;                             ///< 串口发来的的四元数
    uint64_t timeStamp;                                 ///< 串口发来的的时间戳
    uint64_t sysTimeStamp;                              ///< 与串口时间对应的系统时间
    int index;                                          ///< 串口发来的本周期发送四元数的序号
    bool IsBlue;
    int stand;

private:
    std::mutex operateMutex_;                           ///< 抓包器的操作互斥锁
    mutable std::shared_mutex dataMutex_;               ///< 抓包器的操作读写锁
};

//Eigen::Vec和cv::Point相互转换
//存在问题
template<class T, class U>
void PredictorTool::EigenCvSwap(T a, U *b, int tool) {
    switch (tool) {
        case Tool::Pt3d2Vec3d:
        {
            b ->x() = a.x;
            b ->y() = a.y;
            b ->z() = a.z;
        };
        case Tool::Pt3f2Vec3d:
        {
            b ->x() = static_cast<double >(a.x);
            b ->y() = static_cast<double >(a.y);
            b ->z() = static_cast<double >(a.z);
        }
        case Tool::Pt3i2Vec3i:
        {
            b ->x() = a.x;
            b ->y() = a.y;
            b ->z() = a.z;
        }
        case Tool::Pt2f2Vec2d:
        {
            b ->x() = static_cast<double >(a.x);
            b ->y() = static_cast<double >(a.y);
        }
        case Tool::Vec3d2Pt3d:
        {
            b ->x = a.x();
            b ->y = a.y();
            b ->z = a.z();
        }
        case Tool::Vec3d2Pt3f:
        {
            b ->x = static_cast<float >(a.x());
            b ->y = static_cast<float >(a.y());
            b ->z = static_cast<float >(a.z());
        }
        case Tool::Vec3f2Pt3f:
        {
            b ->x = a.x();
            b ->y = a.y();
            b ->z = a.z();
        }
        case Tool::Vec3i2Pt3i:
        {
            b ->x = a.x();
            b ->y = a.y();
            b ->z = a.z();
        }
        case Tool::Vec2d2Pt2f:
        {
            b ->x = static_cast<float >(a.x());
            b ->y = static_cast<float >(a.y());
        }
        default:
        {
            std::cout << "Error: the input parameters does not match to the transformation!" << std::endl;
        }

    }
}

#endif //CUBOT_BRAIN_PREDICTOR_TOOL_H
