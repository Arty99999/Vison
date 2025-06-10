//
// Created by whales on 23-3-18.
//

#ifndef CUBOT_BRAIN_CLASSICAL_WINDMILL_PREDICTOR_H
#define CUBOT_BRAIN_CLASSICAL_WINDMILL_PREDICTOR_H

#include "classical_windmill_fan.h"
#include "iostream"

enum Mode{
    NORMAL = 1,     // 普通模式
    RUNE = 2,       // 大符模式
    NORMAL_RUNE = 3 // 小符模式
};


// 创建速度和时间戳的类型
struct SpeedTime
{
    double angleSpeed;
    uint64_t time;

    SpeedTime(double speed = 0.0, uint64_t t = 0)
    {
        angleSpeed = speed;
        time = t;
    }
};
enum class EClassicalStatus
{
    Invalid = 0,        ///< 未识别到装甲板
    First = 1,          ///< 第一次识别到装甲板
    Shoot = 2,          ///< 一直识别到装甲板
    Lost = 3            ///< 出现掉帧情况

};

// 预测相关数据及函数
class Fitting
{
public:
    std::vector<ClassicalWindmillFan> buffArmor_;           // 大符靶心缓存区
    std::vector<SpeedTime> fittingData;                     // 线性拟合的缓存区
    EClassicalStatus armorStatus;                           // 当前大符状态
    Mode runeMode;                                          // 判断大小符
    bool IsJudgement = true;                               // 判断是否进行XY方向判断
    bool IsPositive = false;                                // 判断角度正负
    bool IsXPositive = false;                               // 判断X方向
    bool IsYPositive = false;                               // 判断Y方向
    double pastSpeed = 0;                                   // 从前速度
    double LateTime = 0.01;                                 // 其余延迟时间
    double delayTime = 0.3 ;                                // 时间延迟

    /**
     * @brief 线性插值
     * @param newData 待插值数据
     */
    void LinearInterpolation(SpeedTime newData);

    /**
     * @brief 计算世界坐标系
     */
    void CalWorldCoordinate(Eigen::Vector3d CameraCoordinate, Eigen::Vector3d LogoCameraCoordinate, double* realDistanceTarget);

    /**
     * @brief 角速度计算
     * @param armor_1 现装甲板
     * @param armor_2 前一装甲板
     * @return 偏差角
     */
    double calPalstance(ClassicalWindmillFan &armor_1, ClassicalWindmillFan &armor_2);

    /**
     * @brief 计算角度
     * @param armor_1 装甲板
     * @param logoCenter Logo中心点坐标
     * @return 返回当前装甲板角度
     */
    double calAngle(Eigen::Vector3d &armorCenter, Eigen::Vector3d &logoCenter);

    /**
     * @brief 清空缓存
     */
    void clearData();

    /**
     * @brief 计算距离
     * @param Point_1
     * @param Point_2
     * @return 两点之间的距离
     */
    double calDistance(Eigen::Vector3d &Point_1, Eigen::Vector3d &Point_2);

    /**
     * @brief 数据状态
     * @param armor 装甲板
     */
    void DataStatus(ClassicalWindmillFan &armor);

    /**
     * @brief 进行预测
     * @param armor 装甲板
     * @param LogoCenter Logo中心点
     * @param nextPoint 预测的待击打点
     * @param runMode 运行模式
     */
    bool run(ClassicalWindmillFan &armor,
             Eigen::Vector3d &LogoCenter,
             Eigen::Vector3d *nextPoint);

    void call_param ();

    /**
     * @brief 判断坐标系转换
     */
    bool JudgeCoordinate(Eigen::Vector3d &RightUpperPoint, Eigen::Vector3d &WorldLogoCoordinate);

private:
    std::mutex predictMutex_;   // 大符预测互斥锁

    double _a = 0.9;            // 振幅 [0.780, 1.045]
    double _w = 1.884;          // 角频率 [1.884, 2.000]
    double _t0 = 0.0;           // 初相 [0, 2pai/1.884]

    double max_T0 = 3.34;       // 2*pai/1.884
    double N = 100;             // 角速度采样数目
    double timeInterval = 0.02; // 采样时间间隔 /s
    double t0_N = 30;           // 周期采样数目
    double DN = 2;              // 逐差法间隔
    double WN = 100;

    double startTime;           // 开始时间

    bool isClockwise = false;           // 顺时针
    bool isCurve = false;       // 是否拟合
    bool isDirection = false;   // 是否确定方向

    /**
     * @brief 转向判断
     */
    void SteeringJudgment();

    /**
     * @brief 大符预测
     * @param armor 装甲板
     * @param logoCenter Logo中心点
     * @return nextPoint 预测的待击打点坐标
     */
    void WindmillPredict(ClassicalWindmillFan &armor, Eigen::Vector3d &logoCenter, Eigen::Vector3d *nextPoint);

    /**
     * @brief 小符预测
     */
    void NormalWindmillPredict(ClassicalWindmillFan &armor, Eigen::Vector3d &logoCenter, Eigen::Vector3d *nextPoint);

    /**
     * @brief 根据延迟判断偏转角
     */
    void calOffsetAngle(uint64_t &time, double *deltaAngle);

    /**
     * @brief 角速度拟合
     */
    void FittingCurve();

    /**
     * @brief 角频率估计
     */
    void wFitting();

    /**
     * @brief 相位估计；
     */
    void tFitting();

    /**
     * @brief 离散傅里叶变换得到正弦值
     */
    double get_F_s(int n, double f_k, int k, int _N)
    {
        return f_k * sin(2.0 * M_PI * (double)n / (double)_N * (double)k * timeInterval);
    }

    /**
     * @brief 离散傅里叶变换得到余弦值
     */
    double get_F_c(int n, double f_k, int k, int _N)
    {
        return f_k * cos(2.0 * M_PI * (double)n / (double)_N * (double)k * timeInterval);
    }

    /**
    *  @brief 离散傅里叶变换，得到角频率为2*pi*n/N的幅值
    *  @return 模的平方
    */
    double get_F(int n)                 // n为频率，N为数目
    {
        double c = 0.0, s = 0.0;
        if (isClockwise)                               // 顺时针
            for (int i = 0; i < fittingData.size(); i++)
            {
                c += get_F_c(n, (fittingData[i].angleSpeed - (2.090 - _a)), i, N);    // b = 2.090 - a
                s += get_F_s(n, (fittingData[i].angleSpeed - (2.090 - _a)), i, N);
            }
        else
            for (int i = 0; i < fittingData.size(); i++)
            {
                c += get_F_c(n, (-fittingData[i].angleSpeed - (2.090 - _a)), i, N);
                s += get_F_s(n, (-fittingData[i].angleSpeed - (2.090 - _a)), i, N);
//                std::cout << "C: " << c << std::endl;
            }

        return sqrt(c * c + s * s);
    }

    /**
     *  @brief  求不同相位时的积分,规整化速度值
     */
    double get_integral(double t_)
    {
        double sum = 0;
        if (isClockwise)
            for (int i = 0; i < fittingData.size(); i++)
            {
                sum += sin((i * timeInterval + t_) * _w) * (fittingData[i].angleSpeed - (2.090 - _a)) / _a;
            }
        else
            for (int i = 0; i < fittingData.size(); i++)
            {
                sum += sin((i * timeInterval + t_) * _w) * (-fittingData[i].angleSpeed - (2.090 - _a)) / _a;
            }

        return sum;
    }

    /**
     *  @brief  根据旋转角度和半径计算下一点(装甲板四个角点)的像素位置
     *  @param  point   动点
     *  @param  org     原点
     *  @param  angle   旋转角度
     */
    Eigen::Vector3d calNextPosition(Eigen::Vector3d &armorCenter, Eigen::Vector3d &logoCenter, double &angle);
};

#endif //CUBOT_BRAIN_CLASSICAL_WINDMILL_PREDICTOR_H
