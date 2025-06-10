//
// Created by plutoli on 2021/8/13.
//

#ifndef CUBOT_BRAIN_SOLVER_H
#define CUBOT_BRAIN_SOLVER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "easy_logger.h"
#include "solver_param.h"
#include "huaray_camera_model_param.h"

/**
 * @brief 多项式拟合残差
 * @remark 多项式函数表达式：y = a[0] * t^n + a[1] * t^(n-1) + ... + a[n - 1] * t
 * @note Ceres库的安装和使用参考网址：http://ceres-solver.org/
 */
struct PolynomialResidual
{
public:
    PolynomialResidual(double x, double y, EPolynomialOrder polynomialOrder) :
        x_(x),
        y_(y),
        polynomialOrder_(polynomialOrder)
    {
    }

    template <typename T>
    bool operator()(const T* const coeffs, T* residual) const
    {
        // 拟合1阶多项式
        if (polynomialOrder_ == EPolynomialOrder::One)
        {
            residual[0] = y_ - (coeffs[0] * x_);
        }

        // 拟合2阶多项式
        if (polynomialOrder_ == EPolynomialOrder::Two)
        {
            residual[0] = y_ - (coeffs[0] * std::pow(x_, 2) +
                                coeffs[1] * x_);
        }

        // 拟合3阶多项式
        if (polynomialOrder_ == EPolynomialOrder::Three)
        {
            residual[0] = y_ - (coeffs[0] * std::pow(x_, 3) +
                                coeffs[1] * std::pow(x_, 2) +
                                coeffs[2] * x_);
        }

        // 拟合4阶多项式
        if (polynomialOrder_ == EPolynomialOrder::Four)
        {
            residual[0] = y_ - (coeffs[0] * std::pow(x_, 4) +
                                coeffs[1] * std::pow(x_, 3) +
                                coeffs[2] * std::pow(x_, 2) +
                                coeffs[3] * x_);
        }

        // 拟合5阶多项式
        if (polynomialOrder_ == EPolynomialOrder::Five)
        {
            residual[0] = y_ - (coeffs[0] * std::pow(x_, 5) +
                                coeffs[1] * std::pow(x_, 4) +
                                coeffs[2] * std::pow(x_, 3) +
                                coeffs[3] * std::pow(x_, 2) +
                                coeffs[4] * x_);
        }

        // 拟合6阶多项式
        if (polynomialOrder_ == EPolynomialOrder::Six)
        {
            residual[0] = y_ - (coeffs[0] * std::pow(x_, 6) +
                                coeffs[1] * std::pow(x_, 5) +
                                coeffs[2] * std::pow(x_, 4) +
                                coeffs[3] * std::pow(x_, 3) +
                                coeffs[4] * std::pow(x_, 2) +
                                coeffs[5] * x_);
        }

        // 拟合7阶多项式
        if (polynomialOrder_ == EPolynomialOrder::Seven)
        {
            residual[0] = y_ - (coeffs[0] * std::pow(x_, 7) +
                                coeffs[1] * std::pow(x_, 6) +
                                coeffs[2] * std::pow(x_, 5) +
                                coeffs[3] * std::pow(x_, 4) +
                                coeffs[4] * std::pow(x_, 3) +
                                coeffs[5] * std::pow(x_, 2) +
                                coeffs[6] * x_);
        }

        // 拟合8阶多项式
        if (polynomialOrder_ == EPolynomialOrder::Eight)
        {
            residual[0] = y_ - (coeffs[0] * std::pow(x_, 8) +
                                coeffs[1] * std::pow(x_, 7) +
                                coeffs[2] * std::pow(x_, 6) +
                                coeffs[3] * std::pow(x_, 5) +
                                coeffs[4] * std::pow(x_, 4) +
                                coeffs[5] * std::pow(x_, 3) +
                                coeffs[6] * std::pow(x_, 2) +
                                coeffs[7] * x_);
        }

        // 拟合9阶多项式
        if (polynomialOrder_ == EPolynomialOrder::Nine)
        {
            residual[0] = y_ - (coeffs[0] * std::pow(x_, 9) +
                                coeffs[1] * std::pow(x_, 8) +
                                coeffs[2] * std::pow(x_, 7) +
                                coeffs[3] * std::pow(x_, 6) +
                                coeffs[4] * std::pow(x_, 5) +
                                coeffs[5] * std::pow(x_, 4) +
                                coeffs[6] * std::pow(x_, 3) +
                                coeffs[7] * std::pow(x_, 2) +
                                coeffs[8] * x_);
        }

        // 返回计算结果
        return true;
    }

private:
    const double x_;
    const double y_;
    const EPolynomialOrder polynomialOrder_;
};

/**
 * @brief 正弦函数拟合残差；正弦函数与RoboMaster机甲大师赛的大符旋转速度函数一致
 * @remark 正弦函数表达式：y = a * sin(w * t + fai) + 2.09 - a
 * @note Ceres库的安装和使用参考网址：http://ceres-solver.org/
 */
struct SineResidual
{
public:
    SineResidual(double x, double y) :
        x_(x),
        y_(y)
    {
    }

    template <typename T>
    bool operator()(const T* const coeffs, T* residual) const
    {
        // 计算残差
        residual[0] = y_ - (coeffs[0] * sin(coeffs[1] * x_ + coeffs[2]) + 2.09 - coeffs[0]);

        // 返回计算结果
        return true;
    }

private:
    const double x_;
    const double y_;
};

/**
 * @brief 正弦积分函数拟合残差；被积分的正弦函数与RoboMaster机甲大师赛的大符旋转速度函数一致
 * @remark 对：y = a * sin(w * t + fai) + 2.09 - a 在[0, t]上进行积分 \n
 *         得：Integral(y) = -(a/w) * cos(w * t + fai) + (2.09 - a) * t + (a/w) * cos(fai) \n
 * @note Ceres库的安装和使用参考网址：http://ceres-solver.org/
 */
struct SineIntegralResidual
{
public:
    SineIntegralResidual(double x, double y) :
        x_(x),
        y_(y)
    {
    }

    template <typename T>
    bool operator()(const T* const coeffs, T* residual) const
    {
        // 计算残差
        residual[0] = y_ - (-(coeffs[0] / coeffs[1]) * cos(coeffs[1] * x_ + coeffs[2]) + (2.09 - coeffs[0]) * x_
                      + (coeffs[0] / coeffs[1]) * cos(coeffs[2]));

        // 返回计算结果
        return true;
    }

private:
    const double x_;
    const double y_;
};

/**
 * @brief 目标解算器
 * @note 目标解算器采用Ceres库进行多项式拟合；Ceres库的安装和使用参考网址：http://ceres-solver.org/
 */
class Solver
{
public:
    /**
    * @brief 构造函数
    */
    Solver();

    /**
     * @brief 析构函数
     */
    ~Solver();

    /**
     * @brief 获取目标解算器参数
     * @return 目标解算器参数
     */
    SolverParam GetParam();

    /**
     * @brief 设置目标解算器参数
     * @param[in] param 目标解算器参数
     * @return 设置结果\n
     *         -<em>false</em> 设置失败\n
     *         -<em>true</em> 设置成功\n
     */
    bool SetParam(const SolverParam &param);

    /**
     * @brief 获取目标解算器的初始化状态
     * @return 目标解算器的初始化状态\n
     *         -<em>false</em> 尚未初始化\n
     *         -<em>true</em> 已经初始化\n
     */
    bool IsInitialized();

    /**
     * @brief 获取目标解算器的初始化时间戳
     * @return 目标解算器的初始化时间戳
     */
    uint64_t GetInitTimestamp();

    /**
     * @brief 初始化目标解算器
     * @return 初始化结果\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     */
    bool Init();

    /**
     * @brief 释放目标解算器资源
     * @return 资源释放结果\n
     *         -<em>false</em> 资源释放失败\n
     *         -<em>true</em> 资源释放成功\n
     */
    bool Release();

    /**
     * @brief 对装甲板进行距离补偿
     * @param[in] workMode          工作模式
     * @param[in] bulletVelocity    子弹速度
     * @param[in] distance          装甲板到相机的距离
     * @param[out] offsetPixel      装甲板在像素坐标系x轴和y轴的补偿值
     * @return 补偿结果\n
     *         -<em>false</em> 补偿失败\n
     *         -<em>true</em> 补偿成功\n
     */
    bool CompensateArmorDistance(const EWorkMode &workMode,
                                 const EBulletVelocity &bulletVelocity,
                                 const float &distance,
                                 std::pair<float, float> *offsetPixel) const;

    /**
     * @brief 对装甲板进行速度补偿
     * @param[in] workMode              工作模式
     * @param[in] armorLocationSequence 装甲板位置时序
     * @param[in] predictTime           预测时间长度；单位：毫秒
     * @param[out] offsetPixel          装甲板在像素坐标系x轴和y轴的补偿值
     * @return 补偿结果\n
     *         -<em>false</em> 补偿失败\n
     *         -<em>true</em> 补偿成功\n
     */
    bool CompensateArmorVelocity(const EWorkMode &workMode,
                                 const std::vector<std::pair<cv::Point2f, uint64_t>> &armorLocationSequence,
                                 const float &predictTime,
                                 std::pair<float, float> *offsetPixel) const;

    /**
     * @brief 对Buff进行距离补偿
     * @param[in] angle         扇叶角度
     * @param[out] offsetPixel  扇叶在像素坐标系x轴和y轴的补偿值
     * @return 补偿结果\n
     *         -<em>false</em> 补偿失败\n
     *         -<em>true</em> 补偿成功\n
     */
    bool CompensateBuffDistance(const float &angle, std::pair<float, float> *offsetPixel) const;

    /**
     * @brief 对Buff进行速度补偿
     * @param[in] workMode              工作模式
     * @param[in] logoLocation          Logo位置
     * @param[in] fanLocationSequence   扇叶位置时序
     * @param[out] offsetPixel          扇叶在像素坐标系x轴和y轴的补偿值
     * @return 补偿结果\n
     *         -<em>false</em> 补偿失败\n
     *         -<em>true</em> 补偿成功\n
     */
    bool CompensateBuffVelocity(const EWorkMode &workMode,
                                const cv::Point2f &logoLocation,
                                const std::vector<std::pair<cv::Point2f, uint64_t>> &fanLocationSequence,
                                std::pair<float, float> *offsetPixel) const;

    /**
     * @brief 计算坐标样本集合
     * @param[in] targetSequence 目标位置时序
     * @param[out] samples_x     x轴样本集合
     * @param[out] samples_y     y轴样本集合
     * @note samples_x[i].first：当前目标位置与最后一个目标位置的时间戳偏移；单位：毫秒 \n
     *       samples_x[i].second：当前目标位置与最后一个目标位置的x轴像素坐标值偏移；单位：像素 \n
     *       samples_y[i].first：当前目标位置与最后一个目标位置的时间戳偏移；单位：毫秒 \n
     *       samples_y[i].second：当前目标位置与最后一个目标位置的y轴像素坐标值偏移；单位：像素 \n
     */
    static void ComputeCoordinateSamples(const std::vector<std::pair<cv::Point2f, uint64_t>> &targetSequence,
                                         std::vector<std::pair<float, float>> *samples_x,
                                         std::vector<std::pair<float, float>> *samples_y);

    /**
     * @brief 计算弧度样本集合
     * @param[in] origin          坐标原点
     * @param[in] targetSequence  目标位置时序
     * @param[out] samples        样本集合
     * @note samples[i].first：当前目标位置与最后一个目标位置的时间戳偏移；单位：毫秒 \n
     *       samples[i].second：坐标原点到当前目标位置的方向矢量与坐标原点到最后一个目标位置的方向矢量夹角；单位：弧度 \n
     */
    static void ComputeRadianSamples(const cv::Point2f &origin,
                                     const std::vector<std::pair<cv::Point2f, uint64_t>> &targetSequence,
                                     std::vector<std::pair<float, float>> *samples);

    /**
     * @brief 通过多项式计算像素偏移
     * @param[in] coeffs        多项式系数
     * @param[in] offsetTime    偏移时间；单位：毫秒
     * @param[out] offsetPixel  像素偏移
     */
    static void ComputeOffsetPixelByPolynomial(const std::vector<float> &coeffs,
                                               const float &offsetTime,
                                               float *offsetPixel);

    /**
     * @brief 通过旋转计算像素偏移
     * @param[in] origin        坐标原点
     * @param[in] target        目标位置
     * @param[in] offsetRadian  相对坐标原点到目标位置方向矢量的旋转弧度
     * @param[out] offsetPixel  像素偏移
     */
    static void ComputeOffsetPixelByRotation(const cv::Point2f &origin,
                                             const cv::Point2f &target,
                                             const float &offsetRadian,
                                             std::pair<float, float> *offsetPixel);

    /**
     * @brief 拟合多项式函数\n
     *        得：y = a[0] * t^n + a[1] * t^(n-1) + ... + a[n - 1] * t \n
     * @param[in] samples               样本数据；t[i]=samples[i].first, y[i]=samples[i].second
     * @param[in] polynomialOrder       拟合的多项式阶数
     * @param[in] maxIterationNumber    最大迭代次数
     * @param[in] initialCoeffs         初始系数；元素的个数等于拟合多项式的阶数
     * @param[out] fittedCoeffs         拟合得到的系数；元素的个数等于拟合多项式的阶数
     * @return 拟合结果\n
     *         -<em>false</em> 拟合失败\n
     *         -<em>true</em> 拟合成功\n
     * @remark 采用Ceres库进行多项式拟合，参考网址：http://ceres-solver.org/
     */
    static bool FitPolynomial(const std::vector<std::pair<float, float>> &samples,
                              const EPolynomialOrder &polynomialOrder,
                              const int &maxIterationNumber,
                              const std::vector<float> &initialCoeffs,
                              std::vector<float> *fittedCoeffs);

    /**
     * @brief 拟合正弦函数\n
     *        由：y = a * sin(w * t + fai) + 2.09 - a \n
     *        令：coeffs[0]=a, coeffs[1]=w, coeffs[2]=fai
     *        得：y = coeffs[0] * sin(coeffs[1] * t + coeffs[2]) + 2.09 - coeffs[0]
     * @param[in] samples               样本数据；t[i]=samples[i].first, y[i]=samples[i].second
     * @param[in] maxIterationNumber    最大迭代次数
     * @param[in] initialCoeffs         初始系数；共有3个元素
     * @param[out] fittedCoeffs         拟合得到的系数；共有3个元素
     * @return 拟合结果\n
     *         -<em>false</em> 拟合失败\n
     *         -<em>true</em> 拟合成功\n
     * @remark 采用Ceres库进行多项式拟合，参考网址：http://ceres-solver.org/
     */
    static bool FitSine(const std::vector<std::pair<float, float>> &samples,
                        const int &maxIterationNumber,
                        const std::vector<float> &initialCoeffs,
                        std::vector<float> *fittedCoeffs);

    /**
     * @brief 拟合正弦积分函数\n
     *        对：y = a * sin(w * t) + 2.09 - a 在[0, t]上进行积分\n
     *        得：Integral(y) = -(a/w) * cos(w * t + fai) + (2.09 - a) * t + (a/w) * cos(fai) \n
     *        令：f=Integral(y), coeffs[0]=a, coeffs[1]=w, coeffs[2]=fai \n
     *        得：f = -(coeffs[0]/coeffs[1]) * cos(coeffs[1] * t + coeffs[2]) + (2.09 - coeffs[0]) * t
     *               + (coeffs[0]/coeffs[1]) * coeffs[2] \n
     * @param[in] samples               样本数据；t[i]=samples[i].first, y[i]=samples[i].second
     * @param[in] maxIterationNumber    最大迭代次数
     * @param[in] initialCoeffs         初始系数；共有3个元素
     * @param[out] fittedCoeffs         拟合得到的系数；共有3个元素
     * @return 拟合结果\n
     *         -<em>false</em> 拟合失败\n
     *         -<em>true</em> 拟合成功\n
     * @remark 采用Ceres库进行多项式拟合，参考网址：http://ceres-solver.org/
     */
    static bool FitSineIntegral(const std::vector<std::pair<float, float>> &samples,
                                const int &maxIterationNumber,
                                const std::vector<float> &initialCoeffs,
                                std::vector<float> *fittedCoeffs);

    /**
     * @brief 计算目标点坐标系的坐标原点到机器人坐标系坐标原点的欧式距离；单位：毫米
     * @param[in] objectPoints  目标点坐标系的点集
     * @param[in] pixelPoints   像素坐标系的点集
     * @param[in] modelParam    相机的模型参数
     * @return 目标点坐标系的坐标原点到机器人坐标系坐标原点的欧式距离；单位：毫米
     */
    static std::pair<float,Eigen::Matrix<double, 3, 3>> ComputeDistance(const std::vector<cv::Point3f> &objectPoints,
                                                                        const std::vector<cv::Point2f> &pixelPoints,
                                                                        const HuarayCameraModelParam &modelParam);

    /**
     * @brief 计算从origin到target的向量与像素坐标系x轴正方向的夹角；取值范围：[0, 360]
     * @param[in] origin 坐标原点
     * @param[in] target 目标点
     * @return 向量与像素坐标系x轴正方向的夹角
     */
    static float ComputeAngle(const cv::Point2f &origin, const cv::Point2f &target);

    /**
     * @brief 将目标击打位置在像素坐标系下的坐标解算为云台在水平方向(Yaw)和垂直方向(Pitch)的偏转角度
     * @param[in] distance              目标击打位置到相机光心的距离；单位：mm
     * @param[in] target                目标击打位置在像素坐标系下的坐标
     * @param[in] isCorrectDistortion   是否对目标击打位置进行畸变校正
     * @param[in] modelParam            相机模型参数
     * @param[out] offsetAngle          云台在水平方向(Yaw)和垂直方向(Pitch)的偏转角度
     * @note 坐标系设置：水平向右为x轴正方向，垂直向下为y轴正方向，水平向前为z轴正方向\n
     */
    static void Solve(const float &distance,
                      const cv::Point2f &target,
                      const bool &isCorrectDistortion,
                      const HuarayCameraModelParam &modelParam,
                      std::pair<float, float> *offsetAngle);

    /**
     * @brief 对击打目标点进行畸变校正
     * @param[in] disortedTarget    畸变的目标点坐标
     * @param[in] modelParam        相机模型参数
     * @param[out] correctedTarget  校正后的目标点坐标
     */
    static void CorrectDistortion(const cv::Point2f &disortedTarget,
                                  const HuarayCameraModelParam &modelParam,
                                  cv::Point2f *correctedTarget);

    /**
     *
     *  @brief 计算角度补偿
     * @param[in]  QMatrix imu旋转矩阵
     * @param[in] Eigen::Vector3d predictWorldCoordinate    世界下的目标点坐标
     * @param[out]  offsetAngle   云台转动角度
     * @return 需要的角度；
     */
    static bool Getpitchandrow(Eigen::Vector3d &predictcameraCoordinate, Eigen::Vector3d &predictWorldCoordinate,
                               std::pair<float, float> *offsetAngle,Solver &solver);
private:
    SolverParam param_;                      ///< 目标解算器参数
    std::atomic<bool> isInitialized_;        ///< 目标解算器的初始化状态
    std::atomic<uint64_t> initTimestamp_;    ///< 目标解算器的初始化时间戳
    std::mutex operateMutex_;                ///< 目标解算器的操作互斥锁

    /**
     * @brief 对小符进行速度补偿
     * @param[in] logoLocation          Logo位置
     * @param[in] fanLocationSequence   扇叶位置时序
     * @param[out] offsetPixel          扇叶在像素坐标系x轴和y轴的补偿值
     * @return 补偿结果\n
     *         -<em>false</em> 补偿失败\n
     *         -<em>true</em> 补偿成功\n
     */
    bool CompensateSmallBuffVelocity(const cv::Point2f &logoLocation,
                                     const std::vector<std::pair<cv::Point2f, uint64_t>> &fanLocationSequence,
                                     std::pair<float, float> *offsetPixel) const;

    /**
     * @brief 对大符进行速度补偿
     * @param[in] logoLocation          Logo位置
     * @param[in] fanLocationSequence   扇叶位置时序
     * @param[out] offsetPixel          扇叶在像素坐标系x轴和y轴的补偿值
     * @return 补偿结果\n
     *         -<em>false</em> 补偿失败\n
     *         -<em>true</em> 补偿成功\n
     */
    bool CompensateLargeBuffVelocity(const cv::Point2f &logoLocation,
                                     const std::vector<std::pair<cv::Point2f, uint64_t>> &fanLocationSequence,
                                     std::pair<float, float> *offsetPixel) const;
};

#endif //CUBOT_BRAIN_SOLVER_H