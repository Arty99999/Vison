//
// Created by plutoli on 2021/8/2.
//

#include <fstream>
#include "classical_armor_recognizer.h"
#include "preprocess.h"
#include "postprocess.h"
#include "calibrator.h"
#include "utils.h"
#include "types.h"
// ******************************  ClassicalArmorRecognizer类的公有函数  ******************************

// 构造函数
ClassicalArmorRecognizer::ClassicalArmorRecognizer():
        param_(),
        isInitialized_(false),
        initTimestamp_(0),
        armorHogDescriptor_(),
        armorHogSvm_(),
        operateMutex_(),
        oneArmorBuffer_(),
        twoArmorBuffer_(),
        threeArmorBuffer_(),
        fourArmorBuffer_(),
        fiveArmorBuffer_(),
        sentryArmorBuffer_(),
        outpostArmorBuffer_(),
        baseArmorBuffer_(),
        armorBufferMutex_(),
        oneTargetBuffer_(),
        twoTargetBuffer_(),
        threeTargetBuffer_(),
        fourTargetBuffer_(),
        fiveTargetBuffer_(),
        sentryTargetBuffer_(),
        outpostTargetBuffer_(),
        baseTargetBuffer_(),
        targetBufferMutex_()
{
}

// 析构函数
ClassicalArmorRecognizer::~ClassicalArmorRecognizer()
{
    // 释放装甲板识别器系统资源
    if (IsInitialized())
    {
        Release();
    }
    cudaStreamDestroy(stream);

    for (int i = 0; i < 2; ++i)
    {
        CHECK(cudaFree(vBufferD[i]));
    }

    CHECK(cudaFree(transposeDevice));
    CHECK(cudaFree(decodeDevice));

    delete [] outputData;

    delete context;
    delete engine;
    delete runtime;
}

// 获取装甲板识别器参数
ClassicalArmorRecognizerParam ClassicalArmorRecognizer::GetParam()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return param_;
}
void ClassicalArmorRecognizer::SetParamColor(bool IsBlue) {
    if (IsBlue)  param_.LightBarColor = EClassicalLightBarColor::Blue;
    else param_.LightBarColor = EClassicalLightBarColor::Red;
}
// 设置装甲板识别器参数
bool ClassicalArmorRecognizer::SetParam(const ClassicalArmorRecognizerParam &param)
{
    // 锁定装甲板识别器操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断装甲板识别器是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - ClassicalArmorRecognizerParam was set failure because "\
              "ClassicalArmorRecognizer has been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 记录装甲板识别器参数
    param_ = param;

    // 记录日志信息
    log = "[" + param_.Key + "] - ClassicalArmorRecognizerParam was set successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回参数设置结果
    return true;
}

// 获取装甲板识别器的初始化状态
bool ClassicalArmorRecognizer::IsInitialized()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isInitialized_;
}

// 获取装甲板识别器的初始化时间戳
uint64_t ClassicalArmorRecognizer::GetInitTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return initTimestamp_;
}

// 初始化装甲板识别器
bool ClassicalArmorRecognizer::Init()
{
    // 锁定装甲板识别器操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断装甲板识别器是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - ClassicalArmorRecognizer can not be initialized repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 清空装甲板数据缓冲区
    oneArmorBuffer_.clear();
    twoArmorBuffer_.clear();
    threeArmorBuffer_.clear();
    fourArmorBuffer_.clear();
    fiveArmorBuffer_.clear();
    sentryArmorBuffer_.clear();
    outpostArmorBuffer_.clear();
    baseArmorBuffer_.clear();

    // 清空目标点数据缓冲区
    oneTargetBuffer_.clear();
    twoTargetBuffer_.clear();
    threeTargetBuffer_.clear();
    fourTargetBuffer_.clear();
    fiveTargetBuffer_.clear();
    sentryTargetBuffer_.clear();
    outpostTargetBuffer_.clear();
    baseTargetBuffer_.clear();

    // 设置初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    initTimestamp_ = now.time_since_epoch().count();

    // 设置初始化状态
    if(YoloInitTensorRT())
    {
        isInitialized_ = true;
    }

    // 记录日志信息
    log = "[" + param_.Key + "] - ClassicalArmorRecognizer was initialized successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回初始化结果
    return true;
}

// 释放装甲板识别器资源
bool ClassicalArmorRecognizer::Release()
{
    // 锁定装甲板识别器操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断装甲板识别器是否已经初始化
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - ClassicalArmorRecognizer can not be released repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 释放ArmorHogSvm模型
    if (!armorHogSvm_.empty())
    {
        armorHogSvm_.release();
    }

    // 重置初始化时间戳
    initTimestamp_ = 0;

    // 重置初始化状态
    isInitialized_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - ClassicalArmorRecognizer was released successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回初始化结果
    return true;
}

bool ClassicalArmorRecognizer::DetectArmorsByYolo(const cv::Mat &rawImage, std::vector<ClassicalArmor> &Armors)
{
    if(!isInitialized_)
    {
        return false;
    }
    DetectByTensorRT(rawImage, Armors);
    return true;
}


bool ClassicalArmorRecognizer::DetectArmorsByTensorrt(const cv::Mat &rawImage, std::vector<ClassicalArmor> &Armors)
{

    if(!isInitialized_)
    {
        return false;
    }

    DetectByTensorRT(rawImage, Armors);
    return true;
}
// 评估精装甲板
bool ClassicalArmorRecognizer::EvaluatePolishedArmors(const std::vector<ClassicalArmor> &polishedArmors,
                                                      std::vector<float> *scores) const
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 判断精装甲板集合是否为空
    if (polishedArmors.empty())
    {
        return false;
    }

    // 提取装甲板面积和图像中心点偏转距离
    std::vector<float> areas;
    std::vector<float> offsets;
    for (unsigned int i = 0; i < polishedArmors.size(); ++i)
    {
        areas.emplace_back(polishedArmors[i].Area);
        offsets.emplace_back(polishedArmors[i].Offset);
    }

    // 计算装甲板最大面积、最小面积、最大图像中心点偏转距离、最小图像中心点偏转距离
    float maxArea = *std::max_element(areas.begin(), areas.end());
    float minArea = *std::min_element(areas.begin(), areas.end());
    float maxOffset = *std::max_element(offsets.begin(), offsets.end());
    float minOffset = *std::min_element(offsets.begin(), offsets.end());

    // 归一化装甲板面积
    if ((maxArea - minArea) < 0.01)
    {
        for (unsigned int i = 0; i < areas.size(); ++i)
        {
            areas[i] = 1.0;
        }
    }
    else
    {
        for (unsigned int i = 0; i < areas.size(); ++i)
        {
            areas[i] = (areas[i] - minArea) / (maxArea - minArea);
        }
    }

    // 归一化图像中心点偏转距离
    if ((maxOffset - minOffset) < 0.01)
    {
        for (unsigned int i = 0; i < offsets.size(); ++i)
        {
            offsets[i] = 1.0;
        }
    }
    else
    {
        for (unsigned int i = 0; i < offsets.size(); ++i)
        {
            offsets[i] = (maxOffset - offsets[i]) / (maxOffset - minOffset);
        }
    }

    // 计算装甲板得分
    for (unsigned int i = 0; i < polishedArmors.size(); ++i)
    {
        // 初始化装甲板评估权值
        ClassicalArmorEvaluationWeight weight;
        weight.OffsetCoeff = 0.0;
        weight.AreaCoeff = 0.0;
        weight.ImportanceCoeff = 0.0;

        // 检索装甲板评估权值
        for (unsigned int j = 0; j < param_.EvaluationWeights.size(); ++j)
        {
            if (polishedArmors[i].Number == param_.EvaluationWeights[j].ArmorNumber)
            {
                weight.ArmorNumber = param_.EvaluationWeights[j].ArmorNumber;
                weight.OffsetCoeff = param_.EvaluationWeights[j].OffsetCoeff;
                weight.AreaCoeff = param_.EvaluationWeights[j].AreaCoeff;
                weight.ImportanceCoeff = param_.EvaluationWeights[j].ImportanceCoeff;
                break;
            }
        }

        // 计算并存储装甲板得分
        float score = areas[i] * weight.AreaCoeff + offsets[i] * weight.OffsetCoeff + weight.ImportanceCoeff;
        scores->emplace_back(score);
    }

    // 返回评估结果
    return true;
}

// 更新装甲板数据缓冲区
bool ClassicalArmorRecognizer::UpdateArmorBuffer(const ClassicalArmor &polishedArmor)
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 锁定装甲板数据缓冲区
    std::lock_guard<std::mutex> lockGuard(armorBufferMutex_);

    // 刷新装甲板数据缓冲区
    switch (polishedArmor.Number)
    {
        case EClassicalArmorNumber::BlueOne:
            RefreshArmorBuffer(polishedArmor, &oneArmorBuffer_);
            break;

        case EClassicalArmorNumber::BlueTwo:
            RefreshArmorBuffer(polishedArmor, &twoArmorBuffer_);
            break;

        case EClassicalArmorNumber::BlueThree:
            RefreshArmorBuffer(polishedArmor, &threeArmorBuffer_);
            break;

        case EClassicalArmorNumber::BlueFour:
            RefreshArmorBuffer(polishedArmor, &fourArmorBuffer_);
            break;

        case EClassicalArmorNumber::BlueFive:
            RefreshArmorBuffer(polishedArmor, &fiveArmorBuffer_);
            break;

        case EClassicalArmorNumber::BlueSentry:
            RefreshArmorBuffer(polishedArmor, &sentryArmorBuffer_);
            break;

        case EClassicalArmorNumber::BlueOutpost:
            RefreshArmorBuffer(polishedArmor, &outpostArmorBuffer_);
            break;

        case EClassicalArmorNumber::BlueBase:
            RefreshArmorBuffer(polishedArmor, &baseArmorBuffer_);
            break;

        default:
            break;
    }

    // 返回刷新结果
    return true;
}

// 清空装甲板数据缓冲区
void ClassicalArmorRecognizer::ClearArmorBuffer()
{
    std::lock_guard<std::mutex> lockGuard(armorBufferMutex_);
    oneArmorBuffer_.clear();
    twoArmorBuffer_.clear();
    threeArmorBuffer_.clear();
    fourArmorBuffer_.clear();
    fiveArmorBuffer_.clear();
    sentryArmorBuffer_.clear();
    outpostArmorBuffer_.clear();
    baseArmorBuffer_.clear();
}

// 获取某个装甲板位置时序
bool ClassicalArmorRecognizer::GetArmorLocationSequence(const EClassicalArmorNumber &armorNumber,
                                                        std::vector<std::pair<cv::Point2f, uint64_t>> *locationSequence)
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 锁定装甲板数据缓冲区
    std::lock_guard<std::mutex> lockGuard(armorBufferMutex_);

    // 填充装甲板位置时序
    switch (armorNumber)
    {
        case EClassicalArmorNumber::BlueOne:
        {
            for (unsigned int i = 0; i < oneArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(oneArmorBuffer_[i].Center, oneArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::BlueTwo:
        {
            for (unsigned int i = 0; i < twoArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(twoArmorBuffer_[i].Center, twoArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::BlueThree:
        {
            for (unsigned int i = 0; i < threeArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(threeArmorBuffer_[i].Center, threeArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::BlueFour:
        {
            for (unsigned int i = 0; i < fourArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(fourArmorBuffer_[i].Center, fourArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::BlueFive:
        {
            for (unsigned int i = 0; i < fiveArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(fiveArmorBuffer_[i].Center, fiveArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::BlueSentry:
        {
            for (unsigned int i = 0; i < sentryArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(sentryArmorBuffer_[i].Center, sentryArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::BlueOutpost:
        {
            for (unsigned int i = 0; i < outpostArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(outpostArmorBuffer_[i].Center, outpostArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::BlueBase:
        {
            for (unsigned int i = 0; i < baseArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(baseArmorBuffer_[i].Center, baseArmorBuffer_[i].Timestamp);
            }
            break;
        }

        default:
            break;
    }

    // 返回获取结果
    return true;
}

// 更新目标点数据缓冲区
bool ClassicalArmorRecognizer::UpdateTargetBuffer(const EClassicalArmorNumber &armorNumber,
                                                  const cv::Point2f &target,
                                                  const uint64_t &timestamp)
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 锁定装甲板数据缓冲区
    std::lock_guard<std::mutex> lockGuard(targetBufferMutex_);

    // 刷新目标点数据缓冲区
    switch (armorNumber)
    {
        case EClassicalArmorNumber::BlueOne:
            RefreshTargetBuffer(target, timestamp, &oneTargetBuffer_);
            break;

        case EClassicalArmorNumber::BlueTwo:
            RefreshTargetBuffer(target, timestamp, &twoTargetBuffer_);
            break;

        case EClassicalArmorNumber::BlueThree:
            RefreshTargetBuffer(target, timestamp, &threeTargetBuffer_);
            break;

        case EClassicalArmorNumber::BlueFour:
            RefreshTargetBuffer(target, timestamp, &fourTargetBuffer_);
            break;

        case EClassicalArmorNumber::BlueFive:
            RefreshTargetBuffer(target, timestamp, &fiveTargetBuffer_);
            break;

        case EClassicalArmorNumber::BlueSentry:
            RefreshTargetBuffer(target, timestamp, &sentryTargetBuffer_);
            break;

        case EClassicalArmorNumber::BlueOutpost:
            RefreshTargetBuffer(target, timestamp, &outpostTargetBuffer_);
            break;

        case EClassicalArmorNumber::BlueBase:
            RefreshTargetBuffer(target, timestamp, &baseTargetBuffer_);
            break;

        default:
            break;
    }

    // 返回刷新结果
    return true;
}

// 清空目标点数据缓冲区
void ClassicalArmorRecognizer::ClearTargetBuffer()
{
    std::lock_guard<std::mutex> lockGuard(targetBufferMutex_);
    oneTargetBuffer_.clear();
    twoTargetBuffer_.clear();
    threeTargetBuffer_.clear();
    fourTargetBuffer_.clear();
    fiveTargetBuffer_.clear();
    sentryTargetBuffer_.clear();
    outpostTargetBuffer_.clear();
    baseTargetBuffer_.clear();
}

// 获取符合时间戳要求的历史击打目标点集合
bool ClassicalArmorRecognizer::GetTarget(const EClassicalArmorNumber &armorNumber,
                                         const uint64_t &timestamp,
                                         const uint64_t &timestampOffset,
                                         std::vector<std::pair<cv::Point2f, uint64_t>> *targets)
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 锁定装甲板数据缓冲区
    std::lock_guard<std::mutex> lockGuard(targetBufferMutex_);

    // 检索符合时间戳要求的击打目标点
    switch (armorNumber)
    {
        case EClassicalArmorNumber::BlueOne:
            SearchForTarget(oneTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::BlueTwo:
            SearchForTarget(twoTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::BlueThree:
            SearchForTarget(threeTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::BlueFour:
            SearchForTarget(fourTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::BlueFive:
            SearchForTarget(fiveTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::BlueSentry:
            SearchForTarget(sentryTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::BlueOutpost:
            SearchForTarget(outpostTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::BlueBase:
            SearchForTarget(baseTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        default:
            break;
    }

    // 返回检索结果
    return true;
}

// 创建普通装甲板图像
void ClassicalArmorRecognizer::CreateCommonArmorsImage(const std::vector<ClassicalArmor> &armors,
                                                       const cv::Mat &rawImage,
                                                       cv::Mat *commonArmorsImage)
{
    // 使用原始图像初始化普通装甲板图像
    *commonArmorsImage = rawImage.clone();

    // 判断装甲板集合是否为空
    if (armors.empty())
    {
        // 在图像上方输出提示信息
        cv::putText(*commonArmorsImage,
                    "There is no armor in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);

        return;
    }

    // 绘制装甲板的多边形轮廓，在图像上方输出装甲板参数
    for (int i = 0; i < armors.size(); ++i)
    {
        // 绘制装甲板的多边形轮廓
        cv::line(*commonArmorsImage,
                 armors[i].LeftUpper,
                 armors[i].RightUpper,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*commonArmorsImage,
                 armors[i].RightUpper,
                 armors[i].RightLower,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*commonArmorsImage,
                 armors[i].RightLower,
                 armors[i].LeftLower,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*commonArmorsImage,
                 armors[i].LeftLower,
                 armors[i].LeftUpper,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);

        // 在装甲板轮廓的左上角输出装甲板索引
        cv::putText(*commonArmorsImage,
                    std::to_string(i + 1),
                    armors[i].LeftUpper,
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);

        // 在装甲板轮廓的中心点输出装甲板编号
        cv::putText(*commonArmorsImage,
                    std::to_string(static_cast<int>(armors[i].Number)),
                    armors[i].Center,
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(255, 255, 255),
                    2);

        // 在图像上方输出装甲板参数
        std::string text = std::to_string(i + 1) +\
                           ":  Area = " + std::to_string(armors[i].Area) +\
                           ";  Offset = " + std::to_string(armors[i].Offset) +\
                           ";  Number = " + std::to_string(static_cast<int>(armors[i].Number));
        cv::putText(*commonArmorsImage,
                    text,
                    cv::Point(10, 20 * i + 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    1);
    }
}

// 创建评估装甲板图像
void ClassicalArmorRecognizer::CreateEvaluatedArmorsImage(const std::vector<ClassicalArmor> &armors,
                                                          const std::vector<float> &scores,
                                                          const cv::Mat &rawImage,
                                                          cv::Mat *evaluatedArmorsImage)
{
    // 使用原始图像初始化评估装甲板图像
    *evaluatedArmorsImage = rawImage.clone();

    // 判断装甲板集合是否为空
    if (armors.empty())
    {
        // 在图像上方输出提示信息
        cv::putText(*evaluatedArmorsImage,
                    "There is no armor in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 判断装甲板集合和得分集合中的元素个数是否相等
    if (armors.size() != scores.size())
    {
        // 在图像上方输出提示信息
        cv::putText(*evaluatedArmorsImage,
                    "The size of armors doesn't equal to the size of scores",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 复制装甲板和得分
    std::vector<ClassicalArmor> unsortedArmors;
    std::vector<float> unsortedScores;
    std::copy(armors.begin(), armors.end(), std::back_inserter(unsortedArmors));
    std::copy(scores.begin(), scores.end(), std::back_inserter(unsortedScores));

    // 对装甲板和得分进行排序
    std::vector<ClassicalArmor> sortedArmors;
    std::vector<float> sortedScores;
    while (!unsortedScores.empty())
    {
        auto maxScoreIterator = std::max_element(unsortedScores.begin(), unsortedScores.end());
        long maxScoreIndex = std::distance(unsortedScores.begin(), maxScoreIterator);
        sortedArmors.emplace_back(unsortedArmors[maxScoreIndex]);
        sortedScores.emplace_back(unsortedScores[maxScoreIndex]);
        unsortedScores.erase(unsortedScores.begin() + maxScoreIndex);
        unsortedArmors.erase(unsortedArmors.begin() + maxScoreIndex);
    }

    // 绘制评估装甲板轮廓，在图像上方输出装甲板评估信息
    for (int i = 0; i < sortedScores.size(); ++i)
    {
        // 绘制评估装甲板轮廓
        cv::line(*evaluatedArmorsImage,
                 sortedArmors[i].LeftUpper,
                 sortedArmors[i].RightUpper,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*evaluatedArmorsImage,
                 sortedArmors[i].RightUpper,
                 sortedArmors[i].RightLower,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*evaluatedArmorsImage,
                 sortedArmors[i].RightLower,
                 sortedArmors[i].LeftLower,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*evaluatedArmorsImage,
                 sortedArmors[i].LeftLower,
                 sortedArmors[i].LeftUpper,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);

        // 在装甲板轮廓左上角输出装甲板索引
        cv::putText(*evaluatedArmorsImage,
                    std::to_string(i + 1),
                    sortedArmors[i].LeftUpper,
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);

        // 在装甲板轮廓的中心点输出装甲板编号
        cv::putText(*evaluatedArmorsImage,
                    std::to_string(static_cast<int>(sortedArmors[i].Number)),
                    sortedArmors[i].Center,
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(255, 255, 255),
                    2);

        // 在图像上方输出装甲板评估信息
        std::string text = std::to_string(i + 1) +\
                           ":  Area = " + std::to_string(sortedArmors[i].Area) +\
                           ";  Offset = " + std::to_string(sortedArmors[i].Offset) +\
                           ";  Number = " + std::to_string(static_cast<int>(sortedArmors[i].Number)) +\
                           ";  Score = " + std::to_string(sortedScores[i]);
        cv::putText(*evaluatedArmorsImage,
                    text,
                    cv::Point(10, 20 * i + 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    1);
    }
}

// 创建锁定装甲板图像
void ClassicalArmorRecognizer::CreateLockedArmorImage(const ClassicalArmor &lockedArmor,
                                                      const float &distance,
                                                      const std::vector<cv::Point2f> &armorLocations,
                                                      const std::pair<float, float> &distanceCompensation,
                                                      const std::pair<float, float> &velocityCompensation,
                                                      const cv::Mat &rawImage,
                                                      cv::Mat *lockedArmorImage)
{
    // 使用原始图像初始化锁定装甲板图像
    *lockedArmorImage = rawImage.clone();

    // 判断锁定装甲板是否合法
    if (lockedArmor.Number == EClassicalArmorNumber::Invalid)
    {
        // 在图像上方输出提示信息
        cv::putText(*lockedArmorImage,
                    "There is no locked armor in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 绘制锁定装甲板的历史位置
    for (unsigned int i = 0; i < armorLocations.size(); ++i)
    {
        cv::circle(*lockedArmorImage,
                   armorLocations[i],
                   2,
                   cv::Scalar(0, 255, 255),
                   2);
    }

    // 绘制锁定装甲板轮廓
    cv::line(*lockedArmorImage,
             lockedArmor.LeftUpper,
             lockedArmor.RightUpper,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);
    cv::line(*lockedArmorImage,
             lockedArmor.RightUpper,
             lockedArmor.RightLower,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);
    cv::line(*lockedArmorImage,
             lockedArmor.RightLower,
             lockedArmor.LeftLower,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);
    cv::line(*lockedArmorImage,
             lockedArmor.LeftLower,
             lockedArmor.LeftUpper,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);

    // 绘制锁定装甲板轮廓的中心点
    cv::circle(*lockedArmorImage,
               lockedArmor.Center,
               3,
               cv::Scalar(0, 255, 0),
               3);

    // 绘制补偿之后的击打目标点
    cv::Point2f compensatedTarget;
    compensatedTarget.x = lockedArmor.Center.x + distanceCompensation.first + velocityCompensation.first;
    compensatedTarget.y = lockedArmor.Center.y + distanceCompensation.second + velocityCompensation.second;
    cv::circle(*lockedArmorImage,
               compensatedTarget,
               10,
               cv::Scalar(0, 0, 255),
               2);

    // 在图像上方输出锁定装甲板的编号
    std::string text1 = "Armor number: " + std::to_string(static_cast<int>(lockedArmor.Number));
    cv::putText(*lockedArmorImage,
                text1,
                cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

    // 在图像上方输出击打目标点的距离信息
    std::string text2 = "Distance: " + std::to_string(distance);
    cv::putText(*lockedArmorImage,
                text2,
                cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

    // 在图像上方输出击打目标点的距离补偿信息
    std::string text3 = "Distance compensation: [" +
                        std::to_string(distanceCompensation.first) + ", " +
                        std::to_string(distanceCompensation.second) + "]";
    cv::putText(*lockedArmorImage,
                text3,
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

    // 在图像上方输出击打目标点的速度补偿信息
    std::string text4 = "Velocity compensation: [" +\
                        std::to_string(velocityCompensation.first) + ", " +\
                        std::to_string(velocityCompensation.second) + "]";
    cv::putText(*lockedArmorImage,
                text4,
                cv::Point(10, 80),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);
}

// 创建比较装甲板图像
void ClassicalArmorRecognizer::CreateComparedArmorImage(const ClassicalArmor &comparedArmor,
                                                        const std::vector<std::pair<cv::Point2f, uint64_t>> &oldTargets,
                                                        const cv::Mat &rawImage,
                                                        cv::Mat *comparedArmorImage)
{
    // 使用原始图像初始化比较装甲板图像
    *comparedArmorImage = rawImage.clone();

    // 判断比较装甲板是否合法
    if (comparedArmor.Number == EClassicalArmorNumber::Invalid)
    {
        // 在图像上方输出提示信息
        cv::putText(*comparedArmorImage,
                    "There is no compared armor in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 输出历史击打目标点和当前装甲板的标识颜色
    cv::putText(*comparedArmorImage,
                "Current comparedArmor's color: Green;  Old target's color: Yellow",
                cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

    // 输出装甲板中心点坐标
    std::string armorCenterText = "Armor center: [" +
                                  std::to_string(comparedArmor.Center.x) + ", " +
                                  std::to_string(comparedArmor.Center.y) + "]";
    cv::putText(*comparedArmorImage,
                armorCenterText,
                cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

    // 输出历史击打目标点的坐标和时间戳偏移
    if (oldTargets.empty())
    {
        cv::putText(*comparedArmorImage,
                    "There is no old target",
                    cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    1);
    }
    else
    {
        for (int i = 0; i < oldTargets.size(); ++i)
        {
            // 计算时间戳偏差；单位：毫秒
            uint64_t timestampOffset = (comparedArmor.Timestamp - oldTargets[i].second) / 1000000;

            // 输出历史击打目标点信息
            std::string oldTargetText = "Old target: [" +
                                        std::to_string(oldTargets[i].first.x) + ", " +
                                        std::to_string(oldTargets[i].first.y) + "];  Timestamp offset: " +
                                        std::to_string(timestampOffset) + " ms";
            cv::putText(*comparedArmorImage,
                        oldTargetText,
                        cv::Point(10, 60 + 20 * i),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(0, 255, 255),
                        1);

            // 绘制历史击打目标点
            cv::circle(*comparedArmorImage,
                       oldTargets[i].first,
                       3,
                       cv::Scalar(0, 255, 255),
                       3);
        }
    }

    // 绘制装甲板的轮廓
    cv::line(*comparedArmorImage,
             comparedArmor.LeftUpper,
             comparedArmor.RightUpper,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);
    cv::line(*comparedArmorImage,
             comparedArmor.RightUpper,
             comparedArmor.RightLower,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);
    cv::line(*comparedArmorImage,
             comparedArmor.RightLower,
             comparedArmor.LeftLower,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);
    cv::line(*comparedArmorImage,
             comparedArmor.LeftLower,
             comparedArmor.LeftUpper,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);

    // 绘制装甲板的中心点
    cv::circle(*comparedArmorImage,
               comparedArmor.Center,
               3,
               cv::Scalar(0, 255, 0),
               3);
}

// 创建测距装甲板图像
void ClassicalArmorRecognizer::CreateRangedArmorImage(const std::vector<ClassicalArmor> &polishedArmors,
                                                      const std::vector<float> &distances,
                                                      const cv::Mat &rawImage,
                                                      cv::Mat *rangedArmorImage)
{
    // 使用原始图像初始化测距装甲板图像
    *rangedArmorImage = rawImage.clone();

    // 判断精装甲板集合是否为空
    if (polishedArmors.empty())
    {
        // 在图像上方输出提示信息
        cv::putText(*rangedArmorImage,
                    "There is no polished armor in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 判断精装甲板集合和距离集合中的元素个数是否相等
    if (polishedArmors.size() != distances.size())
    {
        // 在图像上方输出提示信息
        cv::putText(*rangedArmorImage,
                    "The size of armors doesn't equal to the size of distances",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 绘制测距装甲板轮廓，在图像上方输出装甲板测距信息
    for (int i = 0; i < polishedArmors.size(); ++i)
    {
        // 绘制测距装甲板轮廓
        cv::line(*rangedArmorImage,
                 polishedArmors[i].LeftUpper,
                 polishedArmors[i].RightUpper,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*rangedArmorImage,
                 polishedArmors[i].RightUpper,
                 polishedArmors[i].RightLower,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*rangedArmorImage,
                 polishedArmors[i].RightLower,
                 polishedArmors[i].LeftLower,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*rangedArmorImage,
                 polishedArmors[i].LeftLower,
                 polishedArmors[i].LeftUpper,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);

        // 在装甲板轮廓左上角输出装甲板索引
        cv::putText(*rangedArmorImage,
                    std::to_string(i + 1),
                    polishedArmors[i].LeftUpper,
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);

        // 在装甲板轮廓的中心点输出装甲板编号
        cv::putText(*rangedArmorImage,
                    std::to_string(static_cast<int>(polishedArmors[i].Number)),
                    polishedArmors[i].Center,
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(255, 255, 255),
                    2);

        // 在图像上方输出装甲板距离信息
        std::string text = std::to_string(i + 1) +\
                           ":  Number = " + std::to_string(static_cast<int>(polishedArmors[i].Number)) +\
                           ";  Distance = " + std::to_string(distances[i]);
        cv::putText(*rangedArmorImage,
                    text,
                    cv::Point(10, 20 * i + 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    1);
    }
}

// 创建追踪装甲板图像
void ClassicalArmorRecognizer::CreateTrackedArmorImage(const ClassicalArmor &trackedArmor,
                                                       const std::vector<cv::Point2f> &armorLocations,
                                                       const cv::Mat &rawImage,
                                                       cv::Mat *trackedArmorImage)
{
    // 使用原始图像初始化追踪装甲板图像
    *trackedArmorImage = rawImage.clone();

    // 判断追踪装甲板是否合法
    if (trackedArmor.Number == EClassicalArmorNumber::Invalid)
    {
        // 在图像上方输出提示信息
        cv::putText(*trackedArmorImage,
                    "There is no valid tracked armor in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 绘制追踪装甲板的历史位置
    for (unsigned int i = 0; i < armorLocations.size(); ++i)
    {
        cv::circle(*trackedArmorImage,
                   armorLocations[i],
                   2,
                   cv::Scalar(0, 0, 255),
                   2);
    }

    // 绘制追踪装甲板轮廓
    cv::line(*trackedArmorImage,
             trackedArmor.LeftUpper,
             trackedArmor.RightUpper,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);
    cv::line(*trackedArmorImage,
             trackedArmor.RightUpper,
             trackedArmor.RightLower,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);
    cv::line(*trackedArmorImage,
             trackedArmor.RightLower,
             trackedArmor.LeftLower,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);
    cv::line(*trackedArmorImage,
             trackedArmor.LeftLower,
             trackedArmor.LeftUpper,
             cv::Scalar(0, 255, 0),
             1,
             8,
             0);

    // 绘制锁定装甲板轮廓的中心点
    cv::circle(*trackedArmorImage,
               trackedArmor.Center,
               3,
               cv::Scalar(0, 255, 0),
               3);

    // 计算相邻两块装甲板中心点之间的最大位置偏差
    float maxLocationOffset = 0.0;
    if (armorLocations.size() > 1)
    {
        for (unsigned int i = 0; i < armorLocations.size() - 1; ++i)
        {
            float offset_x = armorLocations[i].x - armorLocations[i + 1].x;
            float offset_y = armorLocations[i].y - armorLocations[i + 1].y;
            float offset = std::sqrt(offset_x * offset_y + offset_y * offset_y);
            if (offset > maxLocationOffset)
            {
                maxLocationOffset = offset;
            }
        }
    }

    // 在图像右下角输出追踪装甲板的编号
    cv::putText(*trackedArmorImage,
                std::to_string(static_cast<int>(trackedArmor.Number)),
                trackedArmor.RightLower,
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(255, 255, 255),
                2);

    // 在图像上方输出历史位置个数和相邻目标点最大距离
    std::string text = "Location number: " + std::to_string(armorLocations.size()) +
                       ";  Max location offset: " + std::to_string(maxLocationOffset);
    cv::putText(*trackedArmorImage,
                text,
                cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

}


// *********************************  ClassicalArmorRecognizer类的私有函数  *********************************

// 刷新装甲板数据缓冲区
void ClassicalArmorRecognizer::RefreshArmorBuffer(const ClassicalArmor &polishedArmor,
                                                  std::deque<ClassicalArmor> *armorBuffer) const
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 判断位置缓冲区是否为空
    if (!armorBuffer->empty())
    {
        // 计算缓冲区内存储的最后一块装甲板和当前装甲板之间的位置偏差
        float x1 = polishedArmor.Center.x;
        float y1 = polishedArmor.Center.y;
        float x2 = armorBuffer->back().Center.x;
        float y2 = armorBuffer->back().Center.y;
        auto offset = static_cast<float>(std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)));

        // 如果位置偏差过大，则清空装甲板位置缓冲区
        if (offset > param_.MaxLocationOffset)
        {
            // 清空风车位置缓冲区
            armorBuffer->clear();

            // 记录日志信息
            std::string armorNumberString = std::to_string(static_cast<int>(polishedArmor.Number));
            log = "[" + param_.Key + "] - ClassicalArmorRecognizer's armor[" + armorNumberString + "] buffer was cleared "\
                  "because armor's location changed too much";
            logger.Save(ELogType::Warn, log);
        }
    }

    // 存储装甲板
    armorBuffer->emplace_back(polishedArmor);

    // 删除超出记忆时间长度的装甲板信息；时间戳的单位是纳秒，记忆时间长度的单位是毫秒
    uint64_t maxMemoryLength = param_.MaxMemoryLength * 1000000;
    while ((polishedArmor.Timestamp - armorBuffer->front().Timestamp) > maxMemoryLength)
    {
        armorBuffer->pop_front();
    }
}

// 刷新目标点数据缓冲区
void ClassicalArmorRecognizer::RefreshTargetBuffer(const cv::Point2f &target,
                                                   const uint64_t &timestamp,
                                                   std::deque<std::pair<cv::Point2f, uint64_t>> *targetBuffer) const
{
    // 存储击打目标点
    targetBuffer->emplace_back(target, timestamp);

    // 删除超出记忆时间长度的元素；时间戳的单位是纳秒，记忆时间长度的单位是毫秒
    uint64_t maxMemoryLength = param_.MaxMemoryLength * 1000000;
    while ((timestamp - targetBuffer->front().second) > maxMemoryLength)
    {
        targetBuffer->pop_front();
    }
}

// 从目标点数据缓冲区中搜索符合时间戳约束的目标点
void ClassicalArmorRecognizer::SearchForTarget(const std::deque<std::pair<cv::Point2f, uint64_t>> &targetBuffer,
                                               const uint64_t &timestamp,
                                               const uint64_t &timestampOffset,
                                               std::vector<std::pair<cv::Point2f, uint64_t>> *targets)
{
    for (unsigned int i = 0; i < targetBuffer.size(); ++i)
    {
        if ((targetBuffer[i].second >= (timestamp - timestampOffset)) &&
            (targetBuffer[i].second <= (timestamp + timestampOffset)))
        {
            targets->emplace_back(targetBuffer[i]);
        }
    }
}

// 通过多边形填充的方式计算装甲板的像素面积
float ClassicalArmorRecognizer::ComputeArmorAreaByPolygon(const std::vector<cv::Point2f> &armorVertices)
{
    // 检索多边形顶点的x轴和y轴最大坐标
    float min_x = FLT_MAX;
    float min_y = FLT_MAX;
    float max_x = FLT_MIN;
    float max_y = FLT_MIN;
    for (unsigned int i = 0; i < armorVertices.size(); ++i)
    {
        min_x = (armorVertices[i].x < min_x)? armorVertices[i].x : min_x;
        min_y = (armorVertices[i].y < min_y)? armorVertices[i].y : min_y;
        max_x = (armorVertices[i].x > max_x)? armorVertices[i].x : max_x;
        max_y = (armorVertices[i].y > max_y)? armorVertices[i].y : max_y;
    }

    // 计算图像的宽度和高度
    int width = static_cast<int>(max_x - min_x + 1.0);
    int height = static_cast<int>(max_y - min_y + 1.0);

    // 确保图像宽度和高度的合法性
    width = (width > 0)? width : 1;
    height = (height > 0)? height : 1;

    // 初始化原始图像
    cv::Mat image(height, width, CV_8UC1);
    image.setTo(0);

    // 重新计算多边形顶点坐标
    std::vector<cv::Point> contour;
    for (unsigned int i = 0; i < armorVertices.size(); ++i)
    {
        int vertex_x = static_cast<int>(armorVertices[i].x - min_x);
        int vertex_y = static_cast<int>(armorVertices[i].y - min_y);
        contour.emplace_back(cv::Point(vertex_x, vertex_y));
    }

    // 绘制并填充多边形轮廓
    std::vector<std::vector<cv::Point>> contours;
    contours.emplace_back(contour);
    cv::polylines(image, contours, true, cv::Scalar(255), 2, cv::LINE_AA);
    cv::fillPoly(image, contours, cv::Scalar(255));

    // 计算多边形面积
    float area = 0.0;
    for (int i = 0; i < image.cols; i++)
    {
        for (int j = 0; j < image.rows; j++)
        {
            if (image.at<uchar>(i, j) == 255)
            {
                area += 1.0;
            }
        }
    }

    // 返回多边形面积
    return area;
}

// 通过轮廓的方式计算装甲板的像素面积
float ClassicalArmorRecognizer::ComputeArmorAreaByContour(const std::vector<cv::Point2f> &armorVertices)
{
    return static_cast<float>(cv::contourArea(armorVertices));
}

// 计算装甲板中心点到原始图像中心点的偏转距离
float ClassicalArmorRecognizer::ComputeArmorOffset(const cv::Point2f &armorCenter,
                                                   const cv::Point2f &rawImageCenter)
{
    float offset_x = (armorCenter.x - rawImageCenter.x) * (armorCenter.x - rawImageCenter.x);
    float offset_y = (armorCenter.y - rawImageCenter.y) * (armorCenter.y - rawImageCenter.y);
    return std::sqrt(offset_x + offset_y);
}



class TRT_Logger : public nvinfer1::ILogger
{
    nvinfer1::ILogger::Severity _verbosity;
    std::ostream* _ostream;

public:
    TRT_Logger(Severity verbosity = Severity::kWARNING, std::ostream& ostream = std::cout)
            : _verbosity(verbosity)
            , _ostream(&ostream)
    {
    }
    void log(Severity severity, const char* msg) noexcept override
    {
        if (severity <= _verbosity)
        {
            time_t rawtime = std::time(0);
            char buf[256];
            strftime(&buf[0], 256, "%Y-%m-%d %H:%M:%S", std::gmtime(&rawtime));
            const char* sevstr = (severity == Severity::kINTERNAL_ERROR ? "    BUG" : severity == Severity::kERROR
                                                                                      ? "  ERROR"
                                                                                      : severity == Severity::kWARNING ? "WARNING" : severity == Severity::kINFO ? "   INFO"
                                                                                                                                                                 : "UNKNOWN");
            (*_ostream) << "[" << buf << " " << sevstr << "] " << msg << std::endl;
        }
    }
};



void ClassicalArmorRecognizer::loadEngine(const std::string strModelName)
{
    trtFile_ = strModelName;
    std::ifstream engineFile(trtFile_, std::ios::binary);
    long int fsize = 0;
    engineFile.seekg(0, engineFile.end);
    fsize = engineFile.tellg();
    engineFile.seekg(0, engineFile.beg);
    std::vector<char> engineString(fsize);
    engineFile.read(engineString.data(), fsize);
    if (engineString.size() == 0) { std::cout << "Failed getting serialized engine!" << std::endl; return; }
    std::cout << "Succeeded getting serialized engine!" << std::endl;

    runtime = createInferRuntime(gLogger);
    engine = runtime->deserializeCudaEngine(engineString.data(), fsize);
    if (engine == nullptr) { std::cout << "Failed loading engine!" << std::endl; return; }
    std::cout << "Succeeded loading engine!" << std::endl;
}

// tensorrt进行推理初始化
bool ClassicalArmorRecognizer::YoloInitTensorRT()
{
    gLogger = Logger(ILogger::Severity::kERROR);
    cudaSetDevice(0);
    CHECK(cudaStreamCreate(&stream));
    std::string model_path = param_.YoloFaceOpenvinoPath;  // 模型权重路径
    loadEngine(model_path);
    context = engine->createExecutionContext();
    kInputH = param_.ImageSize;
    kInputW = param_.ImageSize;
    kConfThresh = param_.ConfThreshold;
    kNmsThresh = param_.NMSThreshold;
    context->setBindingDimensions(0, Dims32 {4, {1, 3, kInputH, kInputW}});

    // get engine output info
    Dims32 outDims = context->getBindingDimensions(1);  // [1, 56, 8400], 56 = 4 + 1 + 51 = bbox + class + keypoints
    OUTPUT_CANDIDATES = outDims.d[2];  // 8400
    int outputSize = 1;  // 56 * 8400
    for (int i = 0; i < outDims.nbDims; i++){
        outputSize *= outDims.d[i];
    }

    // prepare output data space on host
    outputData = new float[1 + kMaxNumOutputBbox * kNumBoxElement];
    // prepare input and output space on device
    vBufferD.resize(2, nullptr);
    CHECK(cudaMalloc(&vBufferD[0], 3 * kInputH * kInputW * sizeof(float)));
    CHECK(cudaMalloc(&vBufferD[1], outputSize * sizeof(float)));

    CHECK(cudaMalloc(&transposeDevice, outputSize * sizeof(float)));
    CHECK(cudaMalloc(&decodeDevice, (1 + kMaxNumOutputBbox * kNumBoxElement) * sizeof(float)));

    return true;
}
//检测框可视化
void ClassicalArmorRecognizer::draw_image(const cv::Mat &img, std::vector<Detection> &inferResult, bool drawBbox, bool kptLine) {
    for (size_t j = 0; j < inferResult.size(); j++)
    {
        // draw bboxes
        if (drawBbox){
            cv::Scalar bboxColor(0, 255, 0);
            cv::Rect r(
                    round(inferResult[j].bbox[0]),
                    round(inferResult[j].bbox[1]),
                    round(inferResult[j].bbox[2] - inferResult[j].bbox[0]),
                    round(inferResult[j].bbox[3] - inferResult[j].bbox[1])
            );
            cv::rectangle(img, r, bboxColor, 2);
            std::string className = vClassNames[(int)inferResult[j].classId];

            std::string labelStr = className + " " + std::to_string(inferResult[j].conf).substr(0, 4);

            cv::Size textSize = cv::getTextSize(labelStr, cv::FONT_HERSHEY_PLAIN, 1.2, 2, NULL);
            cv::Point topLeft(r.x, r.y - textSize.height - 3);
            cv::Point bottomRight(r.x + textSize.width, r.y);
            cv::rectangle(img, topLeft, bottomRight, bboxColor, -1);
            cv::putText(img, labelStr, cv::Point(r.x, r.y - 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        }

        // draw key points
        int x, y;
        float conf;
        int radius = std::min(img.rows, img.cols) / 100;
        cv::Scalar kptColor(0,0,255);
        std::vector<std::vector<float>> vScaledKpts = inferResult[j].vKpts;
//        std::cout<<vScaledKpts.size()<<std::endl;
        for (size_t k = 0; k < vScaledKpts.size(); k++){
            x = (int)vScaledKpts[k][0];
            y = (int)vScaledKpts[k][1];
            // conf = vScaledKpts[k][2];
            if (x < 0 || x > img.cols || y < 0 || y > img.rows) continue;
            // if (conf < 0.5) continue;
            cv::circle(img, cv::Point(x, y), radius, kptColor, -1);
        }

        // draw skeleton between key points
        if (kptLine){
            int kpt1_idx, kpt2_idx, kpt1_x, kpt1_y, kpt2_x, kpt2_y;
            float kpt1_conf, kpt2_conf;
            int skeleton_width = std::min(img.rows, img.cols) / 300;
            cv::Scalar skeletonColor(255,0,0);
            for (size_t m = 0; m < skeleton.size(); m++){
                kpt1_idx = skeleton[m][0] - 1;
                kpt2_idx = skeleton[m][1] - 1;
                kpt1_x = (int)vScaledKpts[kpt1_idx][0];
                kpt1_y = (int)vScaledKpts[kpt1_idx][1];
                // kpt1_conf = vScaledKpts[kpt1_idx][2];
                kpt2_x = (int)vScaledKpts[kpt2_idx][0];
                kpt2_y = (int)vScaledKpts[kpt2_idx][1];
                // kpt2_conf = vScaledKpts[kpt2_idx][2];
                // if (kpt1_conf < 0.1 || kpt2_conf < 0.1) continue;
                if (kpt1_x > img.cols || kpt1_y > img.rows || kpt1_x < 0 || kpt1_y < 0) continue;
                if (kpt2_x > img.cols || kpt2_y > img.rows || kpt2_x < 0 || kpt2_y < 0) continue;
                cv::line(img, cv::Point(kpt1_x, kpt1_y), cv::Point(kpt2_x, kpt2_y), skeletonColor, skeleton_width, cv::LINE_AA);
            }
        }
    }

}
// tensorrt推理
bool ClassicalArmorRecognizer::DetectByTensorRT(const cv::Mat &rawImage, std::vector<ClassicalArmor> &Armors)
{
// put input on device, then letterbox、bgr to rgb、hwc to chw、normalize.
    preprocess(rawImage, (float*)vBufferD[0], kInputH, kInputW, stream);

    // tensorrt inference
    context->enqueueV2(vBufferD.data(), stream, nullptr);
    // transpose [56 8400] convert to [8400 56]
    transpose((float*)vBufferD[1], transposeDevice, OUTPUT_CANDIDATES, 4 + kNumClass + kNumKpt * kKptDims, stream);
    // convert [8400 56] to [58001, ], 58001 = 1 + 1000 * (4bbox + cond + cls + keepflag + 51kpts)
    int nk = kNumKpt * kKptDims;  // number of keypoints total, default 51
    decode(transposeDevice, decodeDevice, OUTPUT_CANDIDATES, kNumClass, nk, kConfThresh, kMaxNumOutputBbox, kNumBoxElement, stream);
    // cuda nms
    nms(decodeDevice, kNmsThresh, kMaxNumOutputBbox, kNumBoxElement, stream);

    CHECK(cudaMemcpyAsync(outputData, decodeDevice, (1 + kMaxNumOutputBbox * kNumBoxElement) * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);

    std::vector<Detection> vDetections;
    int count = std::min((int)outputData[0], kMaxNumOutputBbox);
    for (int i = 0; i < count; i++){
        int pos = 1 + i * kNumBoxElement;
        int keepFlag = (int)outputData[pos + 6];
        if (keepFlag == 1){
            Detection det;
            memcpy(det.bbox, &outputData[pos], 4 * sizeof(float));
            det.conf = outputData[pos + 4];
            det.classId = (int)outputData[pos + 5];
            memcpy(det.kpts, &outputData[pos + 7], kNumKpt * kKptDims * sizeof(float));
            vDetections.push_back(det);
        }
    }

    EClassicalArmorNumber armorNumber;

    for (size_t j = 0; j < vDetections.size(); j++)
    {
        scale_bbox(rawImage, vDetections[j].bbox, kInputW, kInputH);
        vDetections[j].vKpts = scale_kpt_coords(rawImage, vDetections[j].kpts,kInputW,kInputH,kNumKpt,kKptDims);
        // 将类别索引转换为装甲板编号
        ClassicalArmor::ConvertToClassicalArmorNumber(vDetections[j].classId, &armorNumber);

        //计算检测框中心点、装甲板中心点和图像中心点
        cv::Point2f LeftUpperPoint, RightUpperPoint, RightLowerPoint, LeftLowerPoint,Center;
        LeftUpperPoint = cv::Point2f(vDetections[j].vKpts[0][0] , vDetections[j].vKpts[0][1]);
        LeftLowerPoint = cv::Point2f(vDetections[j].vKpts[1][0], vDetections[j].vKpts[1][1]);
        RightLowerPoint = cv::Point2f(vDetections[j].vKpts[2][0], vDetections[j].vKpts[2][1]);
        RightUpperPoint = cv::Point2f(vDetections[j].vKpts[3][0], vDetections[j].vKpts[3][1]);
        Center.x = (LeftLowerPoint.x + LeftUpperPoint.x + RightLowerPoint.x + RightUpperPoint.x) / 4;
        Center.y = (LeftLowerPoint.y + LeftUpperPoint.y + RightLowerPoint.y + RightUpperPoint.y) / 4;

        cv::Point2f center ((vDetections[j].bbox[2]-vDetections[j].bbox[0])/2,(vDetections[j].bbox[3]-vDetections[j].bbox[1])/2);

        cv::Point2f ImageCenter(static_cast<float>(rawImage.cols) / 2, static_cast<float>(rawImage.rows) / 2);

        std::vector<std::string> class_labels = {"BG", "B1", "B2","B3", "B4", "B5","BO", "BBS","BBB", "RG","R1",
                                                 "R2","R3", "R4", "R5","RO", "RBs", "RBb","NG", "N1", "N2",
                                                 "N3", "N4", "N5","NO", "NBS", "NBB","PG","P1", "P2", "P3","P4",
                                                 "P5", "PO","PBS", "PBB"};

        float Offset = ComputeArmorOffset(Center, ImageCenter);
        // 构造检测结果对象并添加到 Armors 中
        ClassicalArmor armor;
        armor.StrNumber = class_labels[vDetections[j].classId];
        armor.Number = armorNumber;
        armor.boxCenter.x = center.x;
        armor.boxCenter.y = center.y;
        armor.Center = Center;
        armor.boxHeight = vDetections[j].bbox[3]-vDetections[j].bbox[1];
        armor.boxWidth = vDetections[j].bbox[2]-vDetections[j].bbox[0];
        armor.Area = (vDetections[j].bbox[3]-vDetections[j].bbox[1])*(vDetections[j].bbox[2]-vDetections[j].bbox[0]);
        armor.boxLeftUpper = cv::Point2f(vDetections[j].bbox[0],vDetections[j].bbox[1]);
        armor.boxRightLower = cv::Point2f(vDetections[j].bbox[2],vDetections[j].bbox[3]);;
        armor.LeftUpper = LeftUpperPoint;
        armor.RightUpper = RightUpperPoint;
        armor.RightLower = RightLowerPoint;
        armor.LeftLower = LeftLowerPoint;
        armor.boxScore = vDetections[j].conf;
        armor.cofScore = vDetections[j].conf;
        armor.Offset = Offset;

        // 将检测结果加入到 Armors 中

        EClassicalLightBarColor targetColor = param_.LightBarColor;
        if (targetColor == EClassicalLightBarColor::Red)
        {
            if (armor.Number == EClassicalArmorNumber::RedOne ||
                armor.Number == EClassicalArmorNumber::RedTwo ||
                armor.Number == EClassicalArmorNumber::RedThree ||
                armor.Number == EClassicalArmorNumber::RedFour ||
                armor.Number == EClassicalArmorNumber::RedFive ||
                armor.Number == EClassicalArmorNumber::RedSentry ||
                armor.Number == EClassicalArmorNumber::RedBase ||
                armor.Number == EClassicalArmorNumber::RedBaseBig ||
                armor.Number == EClassicalArmorNumber::RedOutpost)
            {

                    Armors.push_back(armor);

            }
        }

        if (targetColor == EClassicalLightBarColor::Blue)
        {
                if (armor.Number == EClassicalArmorNumber::BlueOne ||
                        armor.Number == EClassicalArmorNumber::BlueTwo ||
                        armor.Number == EClassicalArmorNumber::BlueThree ||
                        armor.Number == EClassicalArmorNumber::BlueFour ||
                        armor.Number == EClassicalArmorNumber::BlueFive ||
                        armor.Number == EClassicalArmorNumber::BlueSentry ||
                        armor.Number == EClassicalArmorNumber::BlueBase ||
                        armor.Number == EClassicalArmorNumber::BlueOutpost ||
                        armor.Number == EClassicalArmorNumber::BlueBaseBig)
                {

                    Armors.push_back(armor);

                }
        }
    }
    draw_image(rawImage, vDetections,true ,true);
    return true;
}
// 归一化
bool ClassicalArmorRecognizer::Normalize(const cv::Mat &rawImage)
{
    //    img.convertTo(img, CV_32F);
    int row = rawImage.rows;
    int col = rawImage.cols;

    inputImage.resize(row * col * rawImage.channels());

    for (unsigned int c = 0; c < 3; c++)
    {
        for (unsigned int i = 0; i < row; i++)
        {
            for (unsigned int j = 0; j < col; j++)
            {
                // 获取矩阵的像素值，并且进行归一化
                float pix = rawImage.ptr<uchar>(i)[j * 3 + 2 - c];
                inputImage[c * row * col + i * col + j] = pix / 255.0;
            }
        }
    }
    return true;
}



// sigmoid 函数
float ClassicalArmorRecognizer::sigmoid(float x)
{
    return (1 / (1 + exp(-x)));
}




bool ClassicalArmorRecognizer::Nms(std::vector<ClassicalArmor> &inputArmor)
{
    // 从大到小进行排序
        std::sort(inputArmor.begin(),
              inputArmor.end(),
              [](ClassicalArmor a, ClassicalArmor b){return a.boxScore > b.boxScore;});

    std::vector<float> vArea(inputArmor.size());
    for (unsigned int i = 0; i < inputArmor.size(); ++i)
    {
        // 计算每个检测框的面积
        vArea[i] = (inputArmor.at(i).boxRightLower.x - inputArmor.at(i).boxLeftUpper.x + 1)
                   * (inputArmor.at(i).boxRightLower.y - inputArmor.at(i).boxLeftUpper.y + 1);
    }

    // 用于标识该检测框是否以被选中
    std::vector<bool> isSuppressed(inputArmor.size(), false);
    for (int i = 0; i < int(inputArmor.size()); ++i)
    {
        if (isSuppressed[i]) { continue; }
        for (int j = i + 1; j < int(inputArmor.size()); ++j)
        {
            if (isSuppressed[j]) { continue; }
            float xx1 = (std::max)(inputArmor[i].boxLeftUpper.x, inputArmor[j].boxLeftUpper.x);
            float yy1 = (std::max)(inputArmor[i].boxLeftUpper.y, inputArmor[j].boxLeftUpper.y);
            float xx2 = (std::min)(inputArmor[i].boxRightLower.x, inputArmor[j].boxRightLower.x);
            float yy2 = (std::min)(inputArmor[i].boxRightLower.y, inputArmor[j].boxRightLower.y);

            float w = (std::max)(float(0), xx2 - xx1 + 1);
            float h = (std::max)(float(0), yy2 - yy1 + 1);
            float inter = w * h;
            float ovr = inter / (vArea[i] + vArea[j] - inter);

            if (ovr >= param_.NMSThreshold)
            {
                isSuppressed[j] = true;
            }
        }
    }
    // return post_nms;
    int idx_t = 0;
    inputArmor.erase(std::remove_if(inputArmor.begin(),
                                        inputArmor.end(),
                                        [&idx_t, &isSuppressed](const ClassicalArmor& f){ return isSuppressed[idx_t++]; }),
                                        inputArmor.end());

    return true;
}