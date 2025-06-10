//
// Created by plutoli on 2021/8/13.
//

#include "solver.h"

// ******************************  Solver类的公有函数  ******************************

// 构造函数
Solver::Solver():
    param_(),
    isInitialized_(false),
    initTimestamp_(0),
    operateMutex_()
{
}

// 析构函数
Solver::~Solver()
{
    // 释放目标解算器
    if (IsInitialized())
    {
        Release();
    }
}

// 获取目标解算参数
SolverParam Solver::GetParam()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return param_;
}

// 设置目标解算参数
bool Solver::SetParam(const SolverParam &param)
{
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
        log = "[" + param_.Key + "] - SolverParam was set failure because Solver has been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 记录目标解算器参数
    param_ = param;

    // 记录日志信息
    log = "[" + param_.Key + "] - SolverParam was set successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回设置结果
    return true;
}

// 获取目标解算器的初始化状态
bool Solver::IsInitialized()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isInitialized_;
}

// 获取目标解算器的初始化时间戳
uint64_t Solver::GetInitTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return initTimestamp_;
}

// 初始化目标解算器
bool Solver::Init()
{
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
        log = "[" + param_.Key + "] - Solver can not be initialized repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 设置初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    initTimestamp_ = now.time_since_epoch().count();

    // 设置初始化状态
    isInitialized_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - Solver was initialized successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回初始化结果
    return true;
}

// 释放目标解算器资源
bool Solver::Release()
{
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

    // 记录日志信息
    log = "[" + param_.Key + "] - Solver was released successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回初始化结果
    return true;
}

// 对装甲板进行距离补偿
bool Solver::CompensateArmorDistance(const EWorkMode &workMode,
                                     const EBulletVelocity &bulletVelocity,
                                     const float &distance,
                                     std::pair<float, float> *offsetPixel) const
{
    // 判断目标解算器是否初始化
    if (!isInitialized_)
    {
        return false;
    }

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 根据工作模式和子弹速度搜索匹配的装甲板距离补偿参数
    int distanceParamIndex = -1;
    for (int i = 0; i < param_.ArmorDistanceParameters.size(); ++i)
    {
        // 判断工作模式是否匹配
        // 检查std::vector中是否包含特定元素，详见：https://blog.csdn.net/luoyayun361/article/details/108009585
        bool isWorkModeMatched = false;
        if ((std::find(param_.ArmorDistanceParameters[i].WorkModes.begin(),
                       param_.ArmorDistanceParameters[i].WorkModes.end(),
                       workMode) != param_.ArmorDistanceParameters[i].WorkModes.end()))
        {
            isWorkModeMatched = true;
        }

        // 判断子弹速度是否匹配
        bool isBulletVelocityMatched = false;
        if (param_.ArmorDistanceParameters[i].BulletVelocity == bulletVelocity)
        {
            isBulletVelocityMatched = true;
        }

        // 如果工作模式和子弹速度均匹配，则使用当前装甲板距离补偿参数进行补偿
        if (isWorkModeMatched && isBulletVelocityMatched)
        {
            distanceParamIndex = i;
            break;
        }
    }

    // 判断是否搜索到匹配的装甲板距离补偿参数
    if (distanceParamIndex < 0)
    {
        log = "[" + param_.Key + "] - Solver::CompensateArmorDistance() failure because there is no matched param. "\
              "workMode = " + std::to_string(static_cast<int>(workMode)) + "; "\
              "bulletVelocity = " + std::to_string(static_cast<int>(bulletVelocity));
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 根据距离搜索匹配的分段补偿值
    int offsetIndex = -1;
    ArmorDistanceParam distanceParam = param_.ArmorDistanceParameters[distanceParamIndex];
    for (int i = 0; i < distanceParam.Offsets.size(); ++i)
    {
        if ((distance >= distanceParam.Offsets[i].Lower) && (distance < distanceParam.Offsets[i].Upper))
        {
            offsetIndex = i;
            break;
        }
    }

    // 判断是否搜索到匹配的分段补偿值
    if (offsetIndex < 0)
    {
        log = "[" + param_.Key + "] - Solver::CompensateArmorDistance() failure because there is no matched offset. "\
              "workMode = " + std::to_string(static_cast<int>(workMode)) + "; "\
              "bulletVelocity = " + std::to_string(static_cast<int>(bulletVelocity)) + "; "\
              "distance = " + std::to_string(distance);
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 返回x轴和y轴的补偿值
    ArmorDistanceOffset offset = distanceParam.Offsets[offsetIndex];
    offsetPixel->first = offset.Offset_x;
    offsetPixel->second = offset.Offset_y;

    // 返回补偿结果
    return true;
}

// 对装甲板进行速度补偿
bool Solver::CompensateArmorVelocity(const EWorkMode &workMode,
                                     const std::vector<std::pair<cv::Point2f, uint64_t>> &armorLocationSequence,
                                     const float &predictTime,
                                     std::pair<float, float> *offsetPixel) const
{
    // 判断目标解算器是否初始化
    if (!isInitialized_)
    {
        return false;
    }

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 根据工作模式搜索匹配的装甲板速度补偿参数
    // 检查std::vector中是否包含特定元素，详见：https://blog.csdn.net/luoyayun361/article/details/108009585
    int velocityParamIndex = -1;
    for (int i = 0; i < param_.ArmorVelocityParameters.size(); ++i)
    {
        // 判断工作模式是否匹配
        if ((std::find(param_.ArmorVelocityParameters[i].WorkModes.begin(),
                       param_.ArmorVelocityParameters[i].WorkModes.end(),
                       workMode) != param_.ArmorVelocityParameters[i].WorkModes.end()))
        {
            velocityParamIndex = i;
            break;
        }
    }

    // 判断是否搜索到匹配的装甲板速度补偿参数
    if (velocityParamIndex < 0)
    {
        log = "[" + param_.Key + "] - Solver::CompensateArmorVelocity() failure because there is no matched param. "\
              "workMode = " + std::to_string(static_cast<int>(workMode));
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 初始化装甲板速度补偿参数
    ArmorVelocityParam velocityParam = param_.ArmorVelocityParameters[velocityParamIndex];

    // 判断装甲板位置时序的元素数量是否太少
    if (armorLocationSequence.size() < velocityParam.SampleNumber)
    {
        return false;
    }

    // 根据补偿参数中的样本数量截取装甲板位置时序
    std::vector<std::pair<cv::Point2f, uint64_t>> truncatedSequence(velocityParam.SampleNumber);
    unsigned int truncatedNumber = armorLocationSequence.size() - velocityParam.SampleNumber;
    std::copy(armorLocationSequence.begin() + truncatedNumber,
              armorLocationSequence.end(),
              truncatedSequence.begin());

    // 初始化拟合多项式系数的维数
    auto dimension = static_cast<unsigned int>(velocityParam.Order);

    // 计算多项式拟合样本
    std::vector<std::pair<float, float>> samples_x;
    std::vector<std::pair<float, float>> samples_y;
    ComputeCoordinateSamples(truncatedSequence, &samples_x, &samples_y);

    // 初始化装甲板沿x轴和y轴运动多项式的初始系数和拟合系数
    std::vector<float> initialCoeffs_x(dimension, 0.0);
    std::vector<float> initialCoeffs_y(dimension, 0.0);
    std::vector<float> fittedCoeffs_x;
    std::vector<float> fittedCoeffs_y;

    // 拟合装甲板沿x轴和y轴运动的多项式
    FitPolynomial(samples_x,
                  velocityParam.Order,
                  velocityParam.MaxIterationNumber,
                  initialCoeffs_x,
                  &fittedCoeffs_x);
    FitPolynomial(samples_y,
                  velocityParam.Order,
                  velocityParam.MaxIterationNumber,
                  initialCoeffs_y,
                  &fittedCoeffs_y);

    // 根据拟合出来的多项式参数，计算x轴的速度补偿和y轴的速度补偿
    float offsetPixel_x;
    float offsetPixel_y;
    ComputeOffsetPixelByPolynomial(fittedCoeffs_x, predictTime, &offsetPixel_x);
    ComputeOffsetPixelByPolynomial(fittedCoeffs_y, predictTime, &offsetPixel_y);

    // 返回像素补偿值
    offsetPixel->first = offsetPixel_x;
    offsetPixel->second = offsetPixel_y;

    // 返回补偿结果
    return true;
}

// 对Buff进行距离补偿
bool Solver::CompensateBuffDistance(const float &angle, std::pair<float, float> *offsetPixel) const
{
    // 判断目标解算器是否初始化
    if (!isInitialized_)
    {
        return false;
    }

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 根据角度搜索匹配的分段补偿值
    int offsetIndex = -1;
    for (int i = 0; i < param_.BuffDistanceParameter.Offsets.size(); ++i)
    {
        if ((angle >= param_.BuffDistanceParameter.Offsets[i].Lower) &&
            (angle < param_.BuffDistanceParameter.Offsets[i].Upper))
        {
            offsetIndex = i;
            break;
        }
    }

    // 判断是否搜索到匹配的分段补偿值
    if (offsetIndex < 0)
    {
        log = "[" + param_.Key + "] - Solver::CompensateBuffDistance() failure because there is no matched offset. "\
              "angle = " + std::to_string(angle);
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 返回像素补偿值
    BuffDistanceOffset offset = param_.BuffDistanceParameter.Offsets[offsetIndex];
    offsetPixel->first = offset.Offset_x;
    offsetPixel->second = offset.Offset_y;

    // 返回补偿结果
    return true;
}

// 对Buff进行速度补偿
bool Solver::CompensateBuffVelocity(const EWorkMode &workMode,
                                    const cv::Point2f &logoLocation,
                                    const std::vector<std::pair<cv::Point2f, uint64_t>> &fanLocationSequence,
                                    std::pair<float, float> *offsetPixel) const
{
    // 判断目标解算器是否初始化
    if (!isInitialized_)
    {
        return false;
    }

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 判断工作模式是否匹配
    if ((workMode != EWorkMode::ShootSmallBuff) && (workMode != EWorkMode::ShootLargeBuff))
    {
        log = "[" + param_.Key + "] - Solver::CompensateBuffVelocity() failure because workMode is invalid. "\
              "workMode = " + std::to_string(static_cast<int>(workMode));
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 初始化补偿结果
    bool result = false;

    // 对小符进行速度补偿
    if (workMode == EWorkMode::ShootSmallBuff)
    {
        result = CompensateSmallBuffVelocity(logoLocation, fanLocationSequence, offsetPixel);
    }

    // 对大符进行速度补偿
    if (workMode == EWorkMode::ShootLargeBuff)
    {
        result = CompensateLargeBuffVelocity(logoLocation, fanLocationSequence, offsetPixel);
    }

    // 返回补偿结果
    return result;
}

// 计算坐标样本集合
void Solver::ComputeCoordinateSamples(const std::vector<std::pair<cv::Point2f, uint64_t>> &targetSequence,
                                      std::vector<std::pair<float, float>> *samples_x,
                                      std::vector<std::pair<float, float>> *samples_y)
{
    // 初始化x轴多项式拟合数据和y轴多项式拟合数据的坐标原点
    float origin_x = targetSequence.back().first.x;
    float origin_y = targetSequence.back().first.y;
    float origin_timestamp = static_cast<float>(targetSequence.back().second) / 1000000.0f;

    // 构造x轴的坐标样本集合；注意：使用毫秒为时间轴的单位
    for (unsigned int i = 0; i < targetSequence.size(); ++i)
    {
        std::pair<float, float> sample_x;
        sample_x.first = static_cast<float>(targetSequence[i].second) / 1000000.0f - origin_timestamp;
        sample_x.second = targetSequence[i].first.x - origin_x;
        samples_x->emplace_back(sample_x);
    }

    // 构造y轴的坐标样本集合；注意：使用毫秒为时间轴的单位
    for (unsigned int i = 0; i < targetSequence.size(); ++i)
    {
        std::pair<float, float> sample_y;
        sample_y.first = static_cast<float>(targetSequence[i].second) / 1000000.0f - origin_timestamp;
        sample_y.second = targetSequence[i].first.y - origin_y;
        samples_y->emplace_back(sample_y);
    }
}

// 计算弧度样本集合
void Solver::ComputeRadianSamples(const cv::Point2f &origin,
                                  const std::vector<std::pair<cv::Point2f, uint64_t>> &targetSequence,
                                  std::vector<std::pair<float, float>> *samples)
{
    // 判断目标位置时序是否为空
    if (targetSequence.empty())
    {
        return;
    }

    // 计算从坐标原点到目标点的方向矢量
    // 注意：在此创建了一个坐标原点水平向右为x轴正方向，坐标原点垂直向上为y轴正方向的坐标系
    std::vector<cv::Point2f> directions;
    for (unsigned int i = 0; i < targetSequence.size(); ++i)
    {
        cv::Point2f direction;
        direction.x = targetSequence[i].first.x - origin.x;
        direction.y = targetSequence[i].first.y - origin.y;
        directions.emplace_back(direction);
    }

    // 计算从最后一个方向矢量旋转到所有方向矢量的弧度值。以坐标原点为中心，顺时针旋转为正，逆时针旋转为负
    std::vector<float> radians;
    cv::Point2f lastDirection = directions.back();
    for (unsigned int i = 0; i < directions.size(); ++i)
    {
        // 使用二维向量的叉乘和点乘计算弧度值。参考网址如下：
        // https://www.cnblogs.com/zhoug2020/p/7508722.html
        // https://blog.csdn.net/qq_39534332/article/details/100170970
        float dotProduct = lastDirection.x * directions[i].x + lastDirection.y * directions[i].y;
        float crossProduct = lastDirection.x * directions[i].y - lastDirection.y * directions[i].x;
        float radian = std::atan2(crossProduct, dotProduct);  // std::atan2()的取值范围(-pai, pai]
        radians.emplace_back(radian);
    }

    // 计算目标位置时序中所有元素相对于最后一个元素的时间戳的偏差；单位：毫秒
    std::vector<float> timestamps;
    float lastTimestamp = static_cast<float>(targetSequence.back().second) / 1000000.0f;
    for (unsigned int i = 0; i < targetSequence.size(); ++i)
    {
        float timestamp = static_cast<float>(targetSequence[i].second) / 1000000.0f - lastTimestamp;
        timestamps.emplace_back(timestamp);
    }

    // 填充样本集合
    for (unsigned int i = 0; i < targetSequence.size(); ++i)
    {
        std::pair<float, float> sample;
        sample.first = timestamps[i];
        sample.second = radians[i];
        samples->emplace_back(sample);
    }
}

// 通过多项式计算像素偏移
void Solver::ComputeOffsetPixelByPolynomial(const std::vector<float> &coeffs,
                                            const float &offsetTime,
                                            float *offsetPixel)
{
    // 根据拟合出来的多项式系数，计算像素偏移
    *offsetPixel = 0.0;
    unsigned int dimension = coeffs.size();
    for (unsigned int i = 0; i < dimension; ++i)
    {
        *offsetPixel += static_cast<float>(std::pow(offsetTime, dimension - i) * coeffs[i]);
    }
}

// 通过旋转计算像素偏移
void Solver::ComputeOffsetPixelByRotation(const cv::Point2f &origin,
                                          const cv::Point2f &target,
                                          const float &offsetRadian,
                                          std::pair<float, float> *offsetPixel)
{
    // 初始化坐标原点到目标位置的方向矢量
    cv::Point2f direction;
    direction.x = target.x - origin.x;
    direction.y = target.y - origin.y;

    // 使用Eigen库通过轴角式旋转计算旋转之后的目标位置向量。参考网址如下：
    // https://www.cnblogs.com/lovebay/p/11215028.html
    // https://blog.csdn.net/qq_35097289/article/details/94002101
    Eigen::Vector3f currentVector(direction.x, direction.y, 0.0);
    Eigen::AngleAxisf angleAxisf(offsetRadian, Eigen::Vector3f(0, 0, 1.0));
    Eigen::Vector3f rotatedVecotr = angleAxisf.matrix() * currentVector;

    // 计算旋转之后的目标位置与当前目标位置的像素偏差
    offsetPixel->first = rotatedVecotr.x() + origin.x - target.x;
    offsetPixel->second = rotatedVecotr.y() + origin.y - target.y;
}

// 拟合多项式函数
bool Solver::FitPolynomial(const std::vector<std::pair<float, float>> &samples,
                           const EPolynomialOrder &polynomialOrder,
                           const int &maxIterationNumber,
                           const std::vector<float> &initialCoeffs,
                           std::vector<float> *fittedCoeffs)
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 初始化多项式阶数
    auto order = static_cast<unsigned int>(polynomialOrder);

    // 判断样本数据的个数是否合法
    if (samples.size() < order)
    {
        log = "Solver::FitPolynomial() failure because samples's size is less than polynomial order. "\
              "samples size = " + std::to_string(samples.size()) + " "\
              "polynomialOrder = " + std::to_string(static_cast<int>(polynomialOrder));
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 判断初始系数的个数是否合法
    if (initialCoeffs.size() != order)
    {
        log = "Solver::FitPolynomial() failure because initialCoeffs's size is not equal to polynomialOrder. "\
              "initialCoeffs size = " + std::to_string(initialCoeffs.size()) + " "\
              "polynomialOrder = " + std::to_string(static_cast<int>(polynomialOrder));
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 初始化曲线拟合问题
    ceres::Problem problem;

    // 初始化优化参数集合
    double coeffs[order];
    for (unsigned int i = 0; i < order; ++i)
    {
        coeffs[i] = initialCoeffs[i];
    }

    // 添加残差块
    for (unsigned int i = 0; i < samples.size(); ++i)
    {
        // 创建多项式残差
        auto residual = new PolynomialResidual(samples[i].first, samples[i].second, polynomialOrder);

        // 拟合1阶多项式
        if (polynomialOrder == EPolynomialOrder::One)
        {
            auto costFunction = new ceres::AutoDiffCostFunction<PolynomialResidual, 1, 1>(residual);
            problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
            continue;
        }

        // 拟合2阶多项式
        if (polynomialOrder == EPolynomialOrder::Two)
        {
            auto costFunction = new ceres::AutoDiffCostFunction<PolynomialResidual, 1, 2>(residual);
            problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
            continue;
        }

        // 拟合3阶多项式
        if (polynomialOrder == EPolynomialOrder::Three)
        {
            auto costFunction = new ceres::AutoDiffCostFunction<PolynomialResidual, 1, 3>(residual);
            problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
            continue;
        }

        // 拟合4阶多项式
        if (polynomialOrder == EPolynomialOrder::Four)
        {
            auto costFunction = new ceres::AutoDiffCostFunction<PolynomialResidual, 1, 4>(residual);
            problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
            continue;
        }

        // 拟合5阶多项式
        if (polynomialOrder == EPolynomialOrder::Five)
        {
            auto costFunction = new ceres::AutoDiffCostFunction<PolynomialResidual, 1, 5>(residual);
            problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
            continue;
        }

        // 拟合6阶多项式
        if (polynomialOrder == EPolynomialOrder::Six)
        {
            auto costFunction = new ceres::AutoDiffCostFunction<PolynomialResidual, 1, 6>(residual);
            problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
            continue;
        }

        // 拟合7阶多项式
        if (polynomialOrder == EPolynomialOrder::Seven)
        {
            auto costFunction = new ceres::AutoDiffCostFunction<PolynomialResidual, 1, 7>(residual);
            problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
            continue;
        }

        // 拟合8阶多项式
        if (polynomialOrder == EPolynomialOrder::Eight)
        {
            auto costFunction = new ceres::AutoDiffCostFunction<PolynomialResidual, 1, 8>(residual);
            problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
            continue;
        }

        // 拟合9阶多项式
        if (polynomialOrder == EPolynomialOrder::Nine)
        {
            auto costFunction = new ceres::AutoDiffCostFunction<PolynomialResidual, 1, 9>(residual);
            problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
            continue;
        }
    }

    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = maxIterationNumber;

    // 拟合多项式
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    // 如果需要查看Ceres库的求解结果，可以取消该条语句的注释
    std::cout << summary.BriefReport() << std::endl;
    // std::cout << summary.FullReport() << std::endl;

    // 输出拟合得到的系数
    fittedCoeffs->clear();
    for (unsigned int i = 0; i < order; ++i)
    {
        fittedCoeffs->emplace_back(coeffs[i]);
    }

    // 返回拟合结果
    return true;
}

// 拟合正弦函数
bool Solver::FitSine(const std::vector<std::pair<float, float>> &samples,
                     const int &maxIterationNumber,
                     const std::vector<float> &initialCoeffs,
                     std::vector<float> *fittedCoeffs)
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 判断样本数据的个数是否合法
    if (samples.size() < 3)
    {
        log = "Solver::FitSine() failure because samples's size is less than 3. "\
              "samples size = " + std::to_string(samples.size());
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 判断初始系数的个数是否合法
    if (initialCoeffs.size() != 3)
    {
        log = "Solver::FitSine() failure because initialCoeffs's size is not equal to 3. "\
              "initialCoeffs size = " + std::to_string(initialCoeffs.size());
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 初始化曲线拟合问题
    ceres::Problem problem;

    // 初始化优化参数集合
    double coeffs[3];
    coeffs[0] = initialCoeffs[0];
    coeffs[1] = initialCoeffs[1];
    coeffs[2] = initialCoeffs[2];

    // 添加残差块
    for (unsigned int i = 0; i < samples.size(); ++i)
    {
        auto residual = new SineResidual(samples[i].first, samples[i].second);
        auto costFunction = new ceres::AutoDiffCostFunction<SineResidual, 1, 3>(residual);
        problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
    }

    // 设置优化参数范围
    problem.SetParameterLowerBound(coeffs, 0, 0.780);
    problem.SetParameterUpperBound(coeffs, 0, 1.045);
    problem.SetParameterLowerBound(coeffs, 1, 1.884);
    problem.SetParameterUpperBound(coeffs, 1, 2.0);
    problem.SetParameterLowerBound(coeffs, 2, -CV_PI);
    problem.SetParameterUpperBound(coeffs, 2, CV_PI);

    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = maxIterationNumber;

    // 拟合正弦函数
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    // 如果需要查看Ceres库的求解结果，可以取消该条语句的注释
    std::cout << summary.BriefReport() << std::endl;
    // std::cout << summary.FullReport() << std::endl;

    // 输出拟合得到的系数
    fittedCoeffs->clear();
    fittedCoeffs->emplace_back(coeffs[0]);
    fittedCoeffs->emplace_back(coeffs[1]);
    fittedCoeffs->emplace_back(coeffs[2]);

    // 返回拟合结果
    return true;
}

// 拟合正弦积分函数
bool Solver::FitSineIntegral(const std::vector<std::pair<float, float>> &samples,
                             const int &maxIterationNumber,
                             const std::vector<float> &initialCoeffs,
                             std::vector<float> *fittedCoeffs)
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 判断样本数据的个数是否合法
    if (samples.size() < 3)
    {
        log = "Solver::FitSineIntegral() failure because samples's size is less than 3. "\
              "samples size = " + std::to_string(samples.size());
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 判断初始系数的个数是否合法
    if (initialCoeffs.size() != 3)
    {
        log = "Solver::FitSineIntegral() failure because initialCoeffs's size is not equal to 3. "\
              "initialCoeffs size = " + std::to_string(initialCoeffs.size());
        logger.Save(ELogType::Error, log);
        return false;
    }

    // 初始化曲线拟合问题
    ceres::Problem problem;

    // 初始化优化参数集合
    double coeffs[3];
    coeffs[0] = initialCoeffs[0];
    coeffs[1] = initialCoeffs[1];
    coeffs[2] = initialCoeffs[2];

    // 添加残差块
    for (unsigned int i = 0; i < samples.size(); ++i)
    {
        auto residual = new SineIntegralResidual(samples[i].first, samples[i].second);
        auto costFunction = new ceres::AutoDiffCostFunction<SineIntegralResidual, 1, 3>(residual);
        problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(0.5), coeffs);
    }

    // 设置优化参数范围
    problem.SetParameterLowerBound(coeffs, 0, 0.780);
    problem.SetParameterUpperBound(coeffs, 0, 1.045);
    problem.SetParameterLowerBound(coeffs, 1, 1.884);
    problem.SetParameterUpperBound(coeffs, 1, 2.0);
    problem.SetParameterLowerBound(coeffs, 2, 0.0);
    problem.SetParameterUpperBound(coeffs, 2, 2.0 * CV_PI);

    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = maxIterationNumber;

    // 拟合正弦积分函数
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    // 如果需要查看Ceres库的求解结果，可以取消该条语句的注释
    std::cout << summary.BriefReport() << std::endl;
    // std::cout << summary.FullReport() << std::endl;

    // 输出拟合得到的系数
    fittedCoeffs->clear();
    fittedCoeffs->emplace_back(coeffs[0]);
    fittedCoeffs->emplace_back(coeffs[1]);
    fittedCoeffs->emplace_back(coeffs[2]);

    // 返回拟合结果
    return true;
}

// 计算目标点坐标系的坐标原点到机器人坐标系坐标原点的欧式距离；单位：毫米
std::pair<float , Eigen::Matrix<double, 3, 3>> Solver::ComputeDistance(const std::vector<cv::Point3f> &objectPoints,
                                                                        const std::vector<cv::Point2f> &pixelPoints,
                                                                        const HuarayCameraModelParam &modelParam)
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
}

// 计算从origin到target的向量与像素坐标系x轴正方向的夹角；取值范围：[0, 360]
float Solver::ComputeAngle(const cv::Point2f &origin, const cv::Point2f &target)
{
    // 计算从坐标原点到目标点的方向矢量
    cv::Point2f direction;
    direction.x = target.x - origin.x;
    direction.y = target.y - origin.y;

    // 计算方向矢量与x轴正方向的夹角弧度值
    float radian = std::atan2(direction.y, direction.x);  // std::atan2()的取值范围(-pai, pai]
    if (radian < 0.0)
    {
        radian += 2 * CV_PI;
    }

    // 计算方向矢量与x轴正方向夹角的角度值
    auto angle = static_cast<float>(radian * 180.0 / CV_PI);

    // 返回角度值
    return angle;
}

// 将目标击打位置在像素坐标系下的坐标解算为云台在水平方向(Yaw)和垂直方向(Pitch)的偏转角度\n
// 世界坐标系/相机坐标系/图像坐标系/像素坐标系关系：https://blog.csdn.net/xueluowutong/article/details/80950915 \n
// OpenCV中的相机标定(*)：https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html \n
// OpenCV中的畸变校正(*)：https://docs.opencv.org/3.4/d4/d94/tutorial_camera_calibration.html \n
// OpenCV中的畸变模拟(*)：https://www.cnblogs.com/mafuqiang/p/8134617.html \n
// OpenCV中的去畸变接口函数：https://docs.opencv.org/4.5.5/d9/d0c/group__calib3d.html#ga69f2545a8b62a6b0fc2ee060dc30559d \n
// 畸变校正详解(*)：https://blog.csdn.net/humanking7/article/details/45037239 \n
// 相机的径向畸变和切向畸变公式： https://blog.csdn.net/spw_1201/article/details/78417551 \n
void Solver::Solve(const float &distance,
                   const cv::Point2f &target,
                   const bool &isCorrectDistortion,
                   const HuarayCameraModelParam &modelParam,
                   std::pair<float, float> *offsetAngle)
{
    // 建立目标点像素坐标的齐次形式
    Eigen::Vector3d pixelCoordinate(target.x, target.y, 1);

    // 判断是否需要对目标点进行畸变校正
    if (isCorrectDistortion)
    {
        // 对目标点进行畸变校正
        cv::Point2f correctedTarget;
        CorrectDistortion(target, modelParam, &correctedTarget);

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
    double c_x = n_x * ratio;
    double c_y = n_y * ratio;
    double c_z = n_z * ratio;

    // 计算目标点的世界坐标系坐标
    Eigen::Vector3d cameraCoordinate(c_x, c_y, c_z);
    Eigen::Matrix3d rotationMatrix = modelParam.EigenExtrinsics.rotation();
    Eigen::Vector3d translationVector = modelParam.EigenExtrinsics.translation();
    Eigen::Vector3d wordCoordinate = rotationMatrix.inverse() * (cameraCoordinate - translationVector);

    // 根据目标点的世界坐标系坐标，计算水平方向(Yaw)和垂直方向(Pitch)的偏转角度；
    // 注意：经过偏转之后，相机中心正对着目标点
    double w_x = wordCoordinate.x();
    double w_y = wordCoordinate.y();
    double w_z = wordCoordinate.z();
    double yaw = std::atan2(w_x, w_z) * 180.0 / M_PI;
    double pitch = std::atan2(w_y, w_z) * 180.0 / M_PI;

    // 根据相机的平移向量，计算水平方向(Yaw)和垂直方向(Pitch)的偏转角度修正值；修正之后，枪口中心正对着目标点；
    // 注意：该部分为近似计算，尽可能使得枪口偏转之后正对着目标点
    double deltaYaw = translationVector.x() * 180.0 / (M_PI * distance);
    double deltaPitch = translationVector.y() * 180.0 / (M_PI * distance);

    // 计算水平方向(Yaw)和垂直方向(Pitch)的修正之后的偏转角度
    offsetAngle->first = static_cast<float>(yaw - deltaYaw);
    offsetAngle->second = static_cast<float>(pitch - deltaPitch);
}

// 对击打目标点进行畸变校正
// 使用OpenCV接口函数对某个点去畸变：https://zhuanlan.zhihu.com/p/137053640 \n
// cv::undistortPoints()函数：https://docs.opencv.org/4.5.5/d9/d0c/group__calib3d.html#ga69f2545a8b62a6b0fc2ee060dc30559d \n
void Solver::CorrectDistortion(const cv::Point2f &disortedTarget,
                               const HuarayCameraModelParam &modelParam,
                               cv::Point2f *correctedTarget)
{
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
}

bool Solver::Getpitchandrow(Eigen::Vector3d &predictcameraCoordinate, Eigen::Vector3d &predictWorldCoordinate,
                            std::pair<float, float> *offsetAngle,Solver &solver){

    float xx=0.0;
    float yy=0.0;
    float yaw_world = static_cast<float>(std::atan2(predictWorldCoordinate.x(), predictWorldCoordinate.y()) * 180.0 / M_PI);
    float  pitch_world = static_cast<float>(std::atan2(predictWorldCoordinate.z(), std::sqrt(predictWorldCoordinate.x()*predictWorldCoordinate.x()+predictWorldCoordinate.y()*predictWorldCoordinate.y())) * 180.0 / M_PI);
    //对pitch进行补偿
    float pitch_offset = 0.0;
    float pitch_new = pitch_world;
    //通过solver读取参数
    float bullet_speed = solver.param_.AngleDistanceOffsetParameter.bullet_speed;//弹丸初速度
//    std::cout<<"bullet_speed: "<<bullet_speed<<std::endl;
    //使用迭代法求解pitch补偿的最大迭代次数
    int max_iter = solver.param_.AngleDistanceOffsetParameter.max_iter;
    //停止迭代的最小误差(单位m)
    float stop_error = solver.param_.AngleDistanceOffsetParameter.stop_error;
    //龙格库塔法求解落点的迭代次数
    int R_K_iter = solver.param_.AngleDistanceOffsetParameter.R_K_iter;
    const float k = solver.param_.AngleDistanceOffsetParameter.k; //25°C,1atm,小弹丸  摩擦力？？？
    const float g = solver.param_.AngleDistanceOffsetParameter.g; //重力加速度
    //单位均化为米
    float dist_vertical = static_cast<float>(predictWorldCoordinate.z()/1000);
    float dist_horizonal = static_cast<float>(sqrt(predictWorldCoordinate.x()*predictWorldCoordinate.x()+predictWorldCoordinate.y()*predictWorldCoordinate.y())/1000);
    float vertical_tmp = dist_vertical;
    //开始使用龙格库塔法求解弹道补偿
    for (int i = 0; i < max_iter; i++)
    {
        //初始化名
        float x = 0.0;
        float y = 0.0;
        float p =static_cast<float>(tan(pitch_new / 180 * CV_PI));
        float v = bullet_speed;
        float u = v / sqrt(1 + pow(p,2));
        float delta_x = dist_horizonal / R_K_iter;
        for (int j = 0; j < R_K_iter; j++)
        {
            float k1_u = -k * u * sqrt(1 + pow(p, 2));
            float k1_p = -g / pow(u, 2);
            float k1_u_sum = u + k1_u * (delta_x / 2);
            float k1_p_sum = p + k1_p * (delta_x / 2);

            float k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
            float k2_p = -g / pow(k1_u_sum, 2);
            float k2_u_sum = u + k2_u * (delta_x / 2);
            float k2_p_sum = p + k2_p * (delta_x / 2);
            float k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
            float k3_p = -g / pow(k2_u_sum, 2);
            float k3_u_sum = u + k3_u * delta_x;
            float k3_p_sum = p + k3_p * delta_x;
            float k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
            float k4_p = -g / pow(k3_u_sum, 2);
            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);
            x += delta_x;
            y += p * delta_x;
        }
        //评估迭代结果,若小于迭代精度需求则停止迭代
        float error = dist_vertical - y;
        if (abs(error) <= stop_error)
        {
            xx = x;
            yy = y;
            break;
        }
        else
        {
            vertical_tmp += error;
            // xyz_tmp[1] -= error;
            //std::cout << "dist_vertical: [" << dist_vertical<< "]" << std::endl;
            pitch_new = atan(vertical_tmp/ dist_horizonal) * 180 / CV_PI;
        }
        // std::cout << "pitch_new: [" << pitch_new<< "]" << std::endl;
    }
    pitch_offset = pitch_new - pitch_world;
    float yaw = static_cast<float>(std::atan2(predictcameraCoordinate.x(), predictcameraCoordinate.y()) * 180.0 / M_PI);
    float pitch = static_cast<float>(std::atan2(predictcameraCoordinate.z(), std::sqrt(predictcameraCoordinate.y() * predictcameraCoordinate.y() + predictcameraCoordinate.x() * predictcameraCoordinate.x())) * 180.0 / M_PI);
    //std::cout<<"pitch: "<<pitch<<"  pitch_offset: "<<pitch_offset<<std::endl;

    float yaw_offset = static_cast<float>(std::atan2(2,370) * 180.0 / M_PI);

    // 返回计算结果
    //std::cout<<"角度计算的误差:  "<<dist_horizonal-xx<<"   "<<dist_vertical-yy<<std::endl;
    if(pitch<60&&pitch>-60&&yaw>-60&&yaw<60)//判断度度是否合理
    {
        offsetAngle->first = yaw-0.7 ;
        offsetAngle->second = pitch + pitch_offset - 3;
//        std::cout<<"pitch: "<<pitch + pitch_offset<<std::endl;
    }

    else
    {
        offsetAngle->first = 0;
        offsetAngle->second = 0;
    }
    return true;
}
// ******************************  Solver类的私有函数  ******************************

// 对小符进行速度补偿
bool Solver::CompensateSmallBuffVelocity(const cv::Point2f &logoLocation,
                                         const std::vector<std::pair<cv::Point2f, uint64_t>> &fanLocationSequence,
                                         std::pair<float, float> *offsetPixel) const
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 判断扇叶位置时序的元素数量是否太少
    if (fanLocationSequence.size() < param_.SmallBuffVelocityParameter.SampleNumber)
    {
        return false;
    }

    // 根据补偿参数中的样本数量截取位置时序
    std::vector<std::pair<cv::Point2f, uint64_t>> truncatedSequence(param_.SmallBuffVelocityParameter.SampleNumber);
    unsigned int truncatedNumber = fanLocationSequence.size() - param_.SmallBuffVelocityParameter.SampleNumber;
    std::copy(fanLocationSequence.begin() + truncatedNumber,
              fanLocationSequence.end(),
              truncatedSequence.begin());

    // 计算弧度样本集合
    std::vector<std::pair<float, float>> samples;
    ComputeRadianSamples(logoLocation, truncatedSequence, &samples);

    // 根据弧度样本集合中弧度变化趋势，计算弧度偏移
    float offsetRadian = param_.SmallBuffVelocityParameter.OffsetRadian;
    if (samples[0].second > 0)
    {
        offsetRadian = -offsetRadian;
    }

    // 根据弧度偏移，计算像素偏移
    ComputeOffsetPixelByRotation(logoLocation, fanLocationSequence.back().first, offsetRadian, offsetPixel);

    // 返回补偿结果
    return true;
}

// 对大符进行速度补偿
bool Solver::CompensateLargeBuffVelocity(const cv::Point2f &logoLocation,
                                         const std::vector<std::pair<cv::Point2f, uint64_t>> &fanLocationSequence,
                                         std::pair<float, float> *offsetPixel) const
{
    // 判断扇叶位置时序的元素数量是否太少
    if (fanLocationSequence.size() < param_.LargeBuffVelocityParameter.SampleNumber)
    {
        return false;
    }

    // 根据补偿参数中的样本数量截取位置时序
    std::vector<std::pair<cv::Point2f, uint64_t>> truncatedSequence(param_.LargeBuffVelocityParameter.SampleNumber);
    unsigned int truncatedNumber = fanLocationSequence.size() - param_.LargeBuffVelocityParameter.SampleNumber;
    std::copy(fanLocationSequence.begin() + truncatedNumber,
              fanLocationSequence.end(),
              truncatedSequence.begin());

    // 计算弧度样本集合
    std::vector<std::pair<float, float>> samples;
    ComputeRadianSamples(logoLocation, truncatedSequence, &samples);

    // 计算角速度样本集合
    std::vector<float> palstances;
    for (unsigned int i = 1; i < samples.size(); ++i)
    {
        float deltaTime = (samples[i].first - samples[i - 1].first) / 1000;
        float deltaRadian = samples[i].second - samples[i - 1].second;
        palstances.emplace_back(deltaRadian / deltaTime);
    }

    // 计算角速度之和
    float totalPalstance = 0.0;
    for (unsigned int i = 0; i < palstances.size(); ++i)
    {
        totalPalstance += palstances[i];
    }

    // 计算平均角速度，判断是否符合速度要求
    float averagePalstance = std::abs(totalPalstance / static_cast<float>(palstances.size()));
    if ((averagePalstance > param_.LargeBuffVelocityParameter.LowSpeedThreshold) &&
        (averagePalstance < param_.LargeBuffVelocityParameter.HighSpeedOffsetRadian))
    {
        return false;
    }

    // 初始化弧度偏移
    float offsetRadian = 0.0;

    // 计算扇叶处于低速状态下的弧度偏移
    if (averagePalstance <= param_.LargeBuffVelocityParameter.LowSpeedThreshold)
    {
        offsetRadian = param_.LargeBuffVelocityParameter.LowSpeedOffsetRadian;
        if (samples[0].second > 0)
        {
            offsetRadian = -offsetRadian;
        }
    }

    // 计算扇叶处于高速状态下的弧度偏移
    if (averagePalstance >= param_.LargeBuffVelocityParameter.HighSpeedThreshold)
    {
        // 根据弧度样本集合中弧度大小的变化趋势，计算弧度偏移
        offsetRadian = param_.LargeBuffVelocityParameter.HighSpeedOffsetRadian;
        if (samples[0].second > 0)
        {
            offsetRadian = -offsetRadian;
        }
    }

    // 根据弧度偏移，计算像素偏移
    ComputeOffsetPixelByRotation(logoLocation, fanLocationSequence.back().first, offsetRadian, offsetPixel);

    // 返回补偿结果
    return true;
}