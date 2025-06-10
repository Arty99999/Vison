//
// Created by tony on 2022/4/21.
//

#ifndef CUBOT_BRAIN_ADAPTIVEEKF_H
#define CUBOT_BRAIN_ADAPTIVEEKF_H

#include <ceres/jet.h>
#include <Eigen/Dense>
#include "predictor_model.h"
#include "predictor_param.h"

/**
 * @brief 拓展卡尔曼滤波
 * @note 通过拓展卡尔曼滤波，得到当前的目标的位置与速度的最优估计，
 * @note 并通过运动模型进行预测。
 */
class AutoaimEKF {

public:

    /**
     * @brief 构造函数
     * @note 不能在隐式转换中使用
     */
    explicit AutoaimEKF();

    /**
     * @brief 初始化滤波器
     */
    void init(Eigen::Matrix<double, 9, 1> &X0,PredictorModel &PredictFunc);

    /**
     * @brief 滤波器释放函数
     */
    void Release();

    /**
     * @brief 判断滤波器是否为空的函数
     */
    static bool IsEmpty(AutoaimEKF &ekf);

    /**
     * @brief 线性运动预测步骤
     * @note 不是位置的预测，是卡尔曼滤波的预测
     */
    Eigen::Matrix<double, 9, 1> LinearPredict(PredictorModel &PredictFunc);

    Eigen::Matrix<double, 9, 1> StatePredict(Eigen::Matrix<double, 3, 1> nowWorldCoordinate,
                                               double euler,
                                                PredictorModel &PredictFunc);

    Eigen::Matrix<double, 9, 9> SetQMatrix(double t);

    Eigen::Matrix<double, 4, 4> SetVMatrix(Eigen::Matrix<double, 3, 1> Location);
    /**
     * @brief 线性运动更新步骤
     */
    Eigen::Matrix<double, 9, 1> LinearUpdate(Measure &Measure,
                                       const  Eigen::Matrix<double, 4, 1> &Y);

    Eigen::Matrix<double, 9, 1> StateUpdate(Eigen::Matrix<double, 3, 1> nowWorldCorrdinate,
                                              double euler);

    double handleYawJump(double distance, double &yaw);

    double HandleJump(double yaw);

    double HandleYawSum(double yaw);

    std::pair<double, double> GetYawNow(double sum);

    std::pair<bool,double> isTuoLuo(double yaw,double time);

    Eigen::Matrix<double, 9, 1> Xe;           ///< 估计状态变量
    Eigen::Matrix<double, 9, 1> Xp;           ///< 预测状态变量
    Eigen::Matrix<double, 9, 9> F;            ///< 预测雅克比
    Eigen::Matrix<double, 4, 9> H;            ///< 观测雅克比
    Eigen::Matrix<double, 9, 9> P;            ///< 状态协方差
    Eigen::Matrix<double, 9, 4> K;            ///< 卡尔曼增益
    Eigen::Matrix<double, 4, 1> Yp;           ///< 预测观测量
    Eigen::Matrix<double, 4, 1> Q_coe;        ///< 预测协方差系数
    Eigen::Matrix<double, 9, 9> Q;            ///< 预测过程协方差
    Eigen::Matrix<double, 4, 1> R_coe;        ///< 观测协方差系数
    Eigen::Matrix<double, 4, 4> R;            ///< 观测过程协方差
    Eigen::Matrix<double, 4, 4> V;            ///< 观测方程偏导矩阵
    Eigen::Matrix<double, 4, 4> Dk;           ///< 残差的方差
    Eigen::Matrix<double, 4, 1> ek;           ///< 残差
    double rk;                                ///< 检测函数
    double yaw_last;
    double yaw_sum;
    double yaw_d;
    bool jump_flag;
    bool angle_position;                          ///<
    std::pair<bool, int> yaw_change;          ///<
    std::vector<double> d_yaw;
    std::vector<double> d_time;
};

class Remap{

public:

    Remap();


    ~Remap() = default;

    /**
    * @brief 将装甲板从imu坐标系重投影到相机坐标系下
    * @param  distance   装甲板中心点到相机的距离
    * @param imucoordinate  装甲板在imu坐标系下的中心点坐标
    * @param RandTvec  imu坐标系转换到camera坐标系的T矩阵和R矩阵
    * @note 方便接下来和相机捕捉到的装甲板进行比较
    * @return 中心点像素
    */
    static cv::Point2f imu2camera(double distance,
                                  Eigen::Matrix<double,3,1> &imucoordinate,
                                  std::pair<Eigen::Matrix<double,3,3>,Eigen::Matrix<double,3,1>> RandTvec);

    /**
    * @brief  用于比较两个四边形
    * @param yaw  进行比较的yaw值
    * @param Coordinate  相机坐标系下的中心点坐标
    * @param pixelCoordinate   通过imu2camera函数获取的中心点的像素坐标
    * @param armor  用于比较的装甲板
    * @note  得到重投影和和原装甲板差值和重投影的四个像素角点
    */
    static std::pair<double,std::vector<cv::Point2f>> Compare(const float yaw,
                                                              HuarayCameraModelParam &modelParam,
                                                              cv::Point3f Coordinate,
                                                              ClassicalArmor armor,
                                                              bool stand);
    /**
     * @brief 将Compare函数得到的差值来剃度下降
     * @param Coordinate 相机坐标系下的中心点坐标
     * @param armor  用于比较的装甲板
     * @note
     * @return  accurateyaw
     */


    /**
     * @brief 将Compare函数得到的IOU来寻找合适的角度
     * @param Coordinate 相机坐标系下的中心点坐标
     * @param modelParam  相机参数
     * @param armor  用于比较的装甲板
     * @note
     * @return  accurateyaw
     */
    static float findAccurateYaw(cv::Point3f Coordinate,
                                 HuarayCameraModelParam &modelParam,
                                 ClassicalArmor armor,
                                 bool stand);

    /**
     * @brief 用phi优选法计算yaw值
     * @param Coordinate 相机坐标系下的中心点坐标
     * @param modelParam  相机参数
     * @param armor  用于比较的装甲板
     * @return  accurateyaw
     */
    static float PhiOptimization(cv::Point3f &Coordinate,
                                 HuarayCameraModelParam &modelParam,
                                 ClassicalArmor &armor,
                                 bool stand);


    /**
     * @brief 输出一系列点来判断是否适合梯度下降
     * @param Coordinate  相机坐标系下的中心点坐标
     * @param modelParam  相机参数
     * @param armor  用于比较的装甲板
     * @note -90～90度
     */
    static float test_gd(cv::Point3f Coordinate,
                         HuarayCameraModelParam &modelParam,
                         ClassicalArmor armor);

    /**
     * @brief 计算两个四边形的IOU
     * @param Points1  第一个四边形的四个角点坐标(cv::Point2i)
     * @param Points2  第二个四边形的四个角点坐标(cv::Point2i)
     * @note  通过在img1上画第一个四边形 ，在img2上画第二个四边形，在img3上同时画两个四边形，计算三个画面的面积来计算IOU
     * @return  IOU
     */
    static float drawAndComputeIOU(std::vector<cv::Point2i> Points1,
                                   std::vector<cv::Point2i> Points2);

    /**
     * @brief 比较两个矩形框的相似度
     * @param capPts 第一个四边形的四个角点坐标
     * @param remPts 第二个四边形的四个角点坐标
     * @return 相似度
     * @note 在使用iou比较的时候，会出现部分提高精度却使得准确度下降的情况
     */
    static float similarity(const std::vector<cv::Point2f>& capPts,
                            const std::vector<cv::Point2f>& remPts,
                            const float& yaw);

    static void yawFilter(std::vector<float> yawSequence);


    static float accurateyaw;                        ///< 精确的yaw值
    static bool IsSuitable;                          ///< 装甲板是否适合处理
    static cv::Mat Img1;                             ///< 画面1
    static cv::Mat Img2;                             ///< 画面2
    static cv::Mat Img3;                             ///< 画面3
    static cv::Mat toolImg;                          ///< 用于重置三个画面
};

/**
 * @brief 前哨站拓展卡尔曼滤波器
 * @note 通过拓展卡尔曼滤波，得到当前的目标的位置与速度的最优估计，
 * @note 并通过运动模型进行预测。
 */
class OutpostEKF {


};

/**
 * @brief 大小符拓展卡尔曼滤波器
 * @note 通过拓展卡尔曼滤波，得到当前的目标的位置与速度的最优估计，
 * @note 并通过运动模型进行预测。
 */
class WindmillEKF {

};

#endif //CUBOT_BRAIN_ADAPTIVEEKF_H
