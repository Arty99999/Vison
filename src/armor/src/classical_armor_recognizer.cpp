//
// Created by plutoli on 2021/8/2.
//

#include "classical_armor_recognizer.h"

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
}

// 获取装甲板识别器参数
ClassicalArmorRecognizerParam ClassicalArmorRecognizer::GetParam()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return param_;
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

    // 创建装甲板的Hog特征描述子
    cv::Size windowSize(param_.ArmorHogWindowWidth, param_.ArmorHogWindowHeight);
    cv::Size blockSize(param_.ArmorHogBlockWidth, param_.ArmorHogBlockHeight);
    cv::Size strideSize(param_.ArmorHogStrideWidth, param_.ArmorHogStrideHeight);
    cv::Size cellSize(param_.ArmorHogCellWidth, param_.ArmorHogCellHeight);
    int bins = param_.ArmorHogBins;
    armorHogDescriptor_ = cv::HOGDescriptor(windowSize, blockSize, strideSize, cellSize, bins);

    // 判断ArmorHogSvm模型文件是否存在
    bool isModelExist = true;
    if (::access(param_.ArmorHogSvmFileName.c_str(), F_OK) == -1)
    {
        log = "[" + param_.Key + "] - ArmorHogSvm file does not exist";
        logger.Save(ELogType::Error, log);
        isModelExist = false;
    }

    // 判断ArmorHogSvm模型文件是否可读
    bool isModelReadable = true;
    if (::access(param_.ArmorHogSvmFileName.c_str(), R_OK) == -1)
    {
        log = "[" + param_.Key + "] - ArmorHogSvm file can not be read";
        logger.Save(ELogType::Error, log);
        isModelReadable = false;
    }

    // 加载ArmorHogSvm模型
    if (isModelExist && isModelReadable)
    {
        try
        {
            armorHogSvm_ = cv::ml::SVM::load(param_.ArmorHogSvmFileName);
            log = "[" + param_.Key + "] - ArmorHogSvm model was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        catch (cv::Exception &exception)
        {
            log = "[" + param_.Key + "] - ArmorHogSvm model was loaded failure because: " + std::string(exception.err);
            logger.Save(ELogType::Error, log);
        }

        // 判断ArmorHogSvm模型和Hog特征描述子的维数是否匹配
        if ((!armorHogSvm_.empty()) && (armorHogSvm_->getVarCount() == armorHogDescriptor_.getDescriptorSize()))
        {
            log = "[" + param_.Key + "] - ArmorHogSvm model matched ArmorHogDescriptor successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + param_.Key + "] - ArmorHogSvm model matched ArmorHogDescriptor failure";
            logger.Save(ELogType::Error, log);
        }
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
    isInitialized_ = true;

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

// 对原始图像进行预处理
bool ClassicalArmorRecognizer::Preprocess(const cv::Mat &rawImage, cv::Mat *binaryImage) const
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 对原始图像进行缩放
    cv::Mat resizedImage;
    cv::resize(rawImage,
               resizedImage,
               cv::Size(),
               param_.DownsampleFactor,
               param_.DownsampleFactor);

    // 使用缩放后的图像初始化二值图
    *binaryImage = cv::Mat(resizedImage.rows, resizedImage.cols, CV_8UC1);

    // 将缩放后的图像转换到HSV颜色空间
    cv::Mat hsvImage;
    cv::cvtColor(resizedImage, hsvImage, cv::COLOR_BGR2HSV);

    // 对HSV颜色空间的缩放图像进行阈值分割
    if (param_.LightBarColor == EClassicalLightBarColor::Blue)
    {
        DeltaCV::FastInRange(hsvImage, param_.HSVThresholdForBlue, binaryImage);
    }
    else
    {
        cv::Mat binaryImage_1(resizedImage.rows, resizedImage.cols, CV_8UC1);
        cv::Mat binaryImage_2(resizedImage.rows, resizedImage.cols, CV_8UC1);
        DeltaCV::FastInRange(hsvImage, param_.HSVThresholdForRed_1, &binaryImage_1);
        DeltaCV::FastInRange(hsvImage, param_.HSVThresholdForRed_2, &binaryImage_2);
        *binaryImage = binaryImage_1 + binaryImage_2;
    }

    // 对二值图进行膨胀操作
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_CROSS,
                                                      cv::Size(2, 2),
                                                      cv::Point(-1, -1));
    cv::dilate(*binaryImage,
               *binaryImage,
               dilateElement,
               cv::Point(-1, -1),
               1);

    // 返回预处理结果
    return true;
}

// 检测粗灯条
bool ClassicalArmorRecognizer::DetectRawLightBars(const cv::Mat &binaryImage,
                                                  std::vector<ClassicalLightBar> *rawLightBars) const
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 检测二值图像中的轮廓信息
    // 参考网址：https://blog.csdn.net/keith_bb/article/details/70185209
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryImage,
                     contours,
                     hierarchy,
                     cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_NONE,
                     cv::Point(0, 0));

    // 根据轮廓创建粗灯条
    for (unsigned int i = 0; i < contours.size(); ++i)
    {
        // 计算粗灯条在原始图像上的轮廓
        std::vector<cv::Point> contour;
        for (unsigned int j = 0; j < contours[i].size(); ++j)
        {
            contour.emplace_back(contours[i][j] / param_.DownsampleFactor);
        }

        // 计算粗灯条在原始图像上的最小旋转外接矩形和最小正外接矩形
        // 参考网址1：https://blog.csdn.net/mailzst1/article/details/83141632
        // 参考网址2：https://www.cnblogs.com/panxiaochun/p/5478555.html
        cv::RotatedRect minRotatedRect = cv::minAreaRect(contour);
        cv::Rect minEnclosedRect = cv::boundingRect(contour);

        // 计算粗灯条在原始图像上的最小旋转外接矩形的4个顶点
        cv::Point2f vertices[4];
        minRotatedRect.points(vertices);

        // 根据最小旋转外接矩形及顶点坐标信息，计算粗灯条在原始图像上的高度/宽度/倾角/左上顶点/左下顶点/右上顶点/右下顶点
        float height;
        float width;
        float angle;
        cv::Point2f leftUpper;
        cv::Point2f leftLower;
        cv::Point2f rightUpper;
        cv::Point2f rightLower;
        if (minRotatedRect.size.height > minRotatedRect.size.width)
        {
            width = minRotatedRect.size.width;
            height = minRotatedRect.size.height;
            leftUpper = vertices[1];
            leftLower = vertices[0];
            rightUpper = vertices[2];
            rightLower = vertices[3];
            angle = std::abs(minRotatedRect.angle) + 90;
        }
        else
        {
            width = minRotatedRect.size.height;
            height = minRotatedRect.size.width;
            leftUpper = vertices[2];
            leftLower = vertices[1];
            rightUpper = vertices[3];
            rightLower = vertices[0];
            angle = std::abs(minRotatedRect.angle);
        }

        // 计算粗灯条的像素面积
        float area = static_cast<float>(cv::contourArea(contour));

        // 创建粗灯条
        ClassicalLightBar rawLightBar;
        rawLightBar.Contour = contour;
        rawLightBar.MinRotatedRect = minRotatedRect;
        rawLightBar.MinEnclosedRect = minEnclosedRect;
        rawLightBar.LeftUpper = leftUpper;
        rawLightBar.LeftLower = leftLower;
        rawLightBar.RightUpper = rightUpper;
        rawLightBar.RightLower = rightLower;
        rawLightBar.Width = width;
        rawLightBar.Height = height;
        rawLightBar.Angle = angle;
        rawLightBar.Center = rawLightBar.MinRotatedRect.center;
        rawLightBar.Area = area;

        // 保存粗灯条
        rawLightBars->emplace_back(rawLightBar);
    }

    // 返回检测结果
    return true;
}

// 检测精灯条
bool ClassicalArmorRecognizer::DetectPolishedLightBars(const std::vector<ClassicalLightBar> &rawLightBars,
                                                       std::vector<ClassicalLightBar> *polishedLightBars) const
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 根据装甲板识别器参数，对粗灯条进行过滤，得到精灯条
    for (unsigned int i = 0; i < rawLightBars.size(); ++i)
    {
        // 判断轮廓面积是否符合要求
        if ((rawLightBars[i].Area < param_.MinLightBarArea) || (rawLightBars[i].Area > param_.MaxLightBarArea))
        {
            continue;
        }

        // 判断高度是否符合要求
        if ((rawLightBars[i].Height < param_.MinLightBarHeight) || (rawLightBars[i].Height > param_.MaxLightBarHeight))
        {
            continue;
        }

        // 判断纵横比是否符合要求
        float aspectRatio = rawLightBars[i].Height / rawLightBars[i].Width;
        if (aspectRatio > param_.MaxLightBarAspectRatio)
        {
            continue;
        }

        // 判断倾角是否符合要求
        if ((rawLightBars[i].Angle < param_.MinLightBarAngle) || (rawLightBars[i].Angle > param_.MaxLightBarAngle))
        {
            continue;
        }

        // 保存精灯条
        polishedLightBars->emplace_back(rawLightBars[i]);
    }

    // 返回检测结果
    return true;
}

// 检测灯条对
bool ClassicalArmorRecognizer::DetectLightBarPairs(const std::vector<ClassicalLightBar> &polishedLightBars,
                                                   std::vector<std::pair<ClassicalLightBar, ClassicalLightBar>> *lightBarPairs) const
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 判断精灯条集合中元素的数量是否合法
    if (polishedLightBars.size() < 2)
    {
        return false;
    }

    // 逐一匹配集合中的灯条
    for (unsigned int i = 0; i < polishedLightBars.size() - 1; ++i)
    {
        for (unsigned int j = i + 1; j < polishedLightBars.size(); ++j)
        {
            // 判断灯条中心距比例是否符合要求
            float height = (polishedLightBars[i].Height + polishedLightBars[j].Height) / 2;
            cv::Point2f p1 = polishedLightBars[i].Center;
            cv::Point2f p2 = polishedLightBars[j].Center;
            float distance = std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
            distance = static_cast<float>((distance > 1.0) ? distance : 1.0);
            float distanceRatio = height / distance;
            if ((distanceRatio < param_.MinLightBarsDistanceRatio) || (distanceRatio > param_.MaxLightBarsDistanceRatio))
            {
                continue;
            }

            // 判断灯条偏差角度是否符合要求
            float angleOffset = std::abs(polishedLightBars[i].Angle - polishedLightBars[j].Angle);
            angleOffset = static_cast<float>((angleOffset < 90.0) ? angleOffset : (180.0 - angleOffset));
            if (angleOffset > param_.MaxLightBarsAngleOffset)
            {
                continue;
            }

            // 判断灯条高度比是否符合要求
            float heightRatio = polishedLightBars[i].Height / polishedLightBars[j].Height;
            heightRatio = static_cast<float>((heightRatio > 1.0) ? heightRatio: 1.0 / heightRatio);
            if (heightRatio > param_.MaxLightBarsHeightRatio)
            {
                continue;
            }

            // 判断两个灯条之间是否有其它灯条
            bool isContainOtherLightBar = false;
            cv::Rect bigRect = polishedLightBars[i].MinEnclosedRect | polishedLightBars[j].MinEnclosedRect;
            for (unsigned int k = 0; k < polishedLightBars.size(); ++k)
            {
                if ((k != i) && (k != j) && (bigRect.contains(polishedLightBars[k].Center)))
                {
                    isContainOtherLightBar = true;
                    break;
                }
            }

            // 如果两个灯条之间有其它灯条，则当前匹配失败
            if (isContainOtherLightBar)
            {
                continue;
            }

            // 创建并保存灯条对
            std::pair<ClassicalLightBar, ClassicalLightBar> lightBarPair;
            lightBarPair.first = polishedLightBars[i];
            lightBarPair.second = polishedLightBars[j];
            lightBarPairs->emplace_back(lightBarPair);
        }
    }

    // 返回检测结果
    return true;
}

// 检测粗装甲板
bool ClassicalArmorRecognizer::DetectRawArmors(const std::vector<std::pair<ClassicalLightBar, ClassicalLightBar>> &lightBarPairs,
                                               const cv::Mat &rawImage,
                                               std::vector<ClassicalArmor> *rawArmors) const
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 获取时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    uint64_t timestamp = now.time_since_epoch().count();

    // 逐一处理灯条对
    for (unsigned int i = 0; i < lightBarPairs.size(); ++i)
    {
        // 初始化左侧灯条和右侧灯条
        ClassicalLightBar leftLightBar;
        ClassicalLightBar rightLightBar;
        if (lightBarPairs[i].first.Center.x < lightBarPairs[i].second.Center.x)
        {
            leftLightBar = lightBarPairs[i].first;
            rightLightBar = lightBarPairs[i].second;
        }
        else
        {
            leftLightBar = lightBarPairs[i].second;
            rightLightBar = lightBarPairs[i].first;
        }

        // 初始化装甲板轮廓的4个角点
        cv::Point2f leftUpperPoint;
        cv::Point2f leftLowerPoint;
        cv::Point2f rightUpperPoint;
        cv::Point2f rightLowerPoint;

        // 计算装甲板高度与灯条高度的比值
        // 注意：大装甲板和小装甲板的高度是相同的，大装甲板和小装甲板上的灯条高度是相同的
        float heightRatio = param_.LargeArmorPhysicalHeight / param_.LightBarPhysicalHeight;

        // 根据左侧灯条的右上顶点和右下顶点坐标，计算装甲板轮廓的左上顶点和左下顶点坐标
        auto leftRadian = static_cast<float>(leftLightBar.Angle * CV_PI / 180.0);
        float leftOffset_x = leftLightBar.Height * heightRatio * cos(leftRadian) / 2;
        float leftOffset_y = leftLightBar.Height * heightRatio * sin(leftRadian) / 2;
        cv::Point2f rightCenter = (leftLightBar.RightUpper + leftLightBar.RightLower) / 2;
        leftUpperPoint.x = rightCenter.x + leftOffset_x;
        leftUpperPoint.y = rightCenter.y - leftOffset_y;
        leftLowerPoint.x = rightCenter.x - leftOffset_x;
        leftLowerPoint.y = rightCenter.y + leftOffset_y;
        //std::cout<<"leftup: "<<leftUpperPoint.x<<" "<<leftUpperPoint.y<<" "<<std::endl;
        // 根据右侧灯条的左上顶点和左下顶点坐标，计算装甲板轮廓的右上顶点和右下顶点坐标
        auto rightRadian = static_cast<float>(rightLightBar.Angle * CV_PI / 180.0);
        float rightOffset_x = rightLightBar.Height * heightRatio * cos(rightRadian) / 2;
        float rightOffset_y = rightLightBar.Height * heightRatio * sin(rightRadian) / 2;
        cv::Point2f leftCenter = (rightLightBar.LeftUpper + rightLightBar.LeftLower) / 2;
        rightUpperPoint.x = leftCenter.x + rightOffset_x;
        rightUpperPoint.y = leftCenter.y - rightOffset_y;
        rightLowerPoint.x = leftCenter.x - rightOffset_x;
        rightLowerPoint.y = leftCenter.y + rightOffset_y;
        // 初始化装甲板轮廓区域图像
        cv::Mat armorRegionImage;

        // 对装甲板轮廓区域进行透视变换，得到装甲板轮廓区域的彩色图像
        std::vector<cv::Point2f> armorRegionCorners;
        armorRegionCorners.emplace_back(leftUpperPoint);
        armorRegionCorners.emplace_back(rightUpperPoint);
        armorRegionCorners.emplace_back(rightLowerPoint);
        armorRegionCorners.emplace_back(leftLowerPoint);
        PerspectiveTransform(armorRegionCorners, rawImage, &armorRegionImage);

        // 将装甲板轮廓区域的彩色图像转换为灰度图像
        cv::cvtColor(armorRegionImage, armorRegionImage, cv::COLOR_BGR2GRAY);
        cv::medianBlur(armorRegionImage, armorRegionImage, 3);
        armorRegionImage.convertTo(armorRegionImage, CV_8UC1, 3.0, 0);

        // 创建粗装甲板
        ClassicalArmor rawArmor;

        // 记录装甲板的左右灯条
        rawArmor.LeftLightBar = leftLightBar;
        rawArmor.RightLightBar = rightLightBar;

        // 记录装甲板的4个角点
        rawArmor.LeftUpper = leftUpperPoint;
        rawArmor.LeftLower = leftLowerPoint;
        rawArmor.RightUpper = rightUpperPoint;
        rawArmor.RightLower = rightLowerPoint;
        float gap = abs(((rightUpperPoint.x - leftUpperPoint.x) / (rightUpperPoint.y - rightLowerPoint.y)));
        if (gap > 1.5) rawArmor.type =1;
        else rawArmor.type = 0;

        // 记录装甲板的中心
        rawArmor.Center = (leftUpperPoint + leftLowerPoint + rightUpperPoint + rightLowerPoint) / 4;

        // 计算装甲板的像素面积
        // rawArmor.Area = ComputeArmorAreaByPolygon(armorRegionCorners);
        rawArmor.Area = ComputeArmorAreaByContour(armorRegionCorners);

        // 计算装甲板中心点的偏转距离
        cv::Point2f rawImageCenter(static_cast<float>(rawImage.cols) / 2, static_cast<float>(rawImage.rows) / 2);
        rawArmor.Offset = ComputeArmorOffset(rawArmor.Center, rawImageCenter);

        // 记录装甲板的轮廓区域图像
        rawArmor.Image = armorRegionImage;

        // 记录装甲板的时间戳
        rawArmor.Timestamp = timestamp;

        // 设置装甲板的编号
        rawArmor.Number = EClassicalArmorNumber::Invalid;

        // 保存粗装甲板
        rawArmors->emplace_back(rawArmor);
    }

    // 返回检测结果
    return true;
}

// 检测精装甲板
bool ClassicalArmorRecognizer::DetectPolishedArmors(const std::vector<ClassicalArmor> &rawArmors,
                                                    std::vector<ClassicalArmor> *polishedArmors) const {
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_) {
        return false;
    }

    // 模型读取
    std::string PATH = "/home/cubot/cubot4/src/armor/src/CNN_18x28_7.30_1.onnx";
    cv::dnn::Net net_ = cv::dnn::readNetFromONNX(PATH);

    // 模型加速
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    for (unsigned int i = 0; i < rawArmors.size(); ++i) {
        // 创建精装甲板
        ClassicalArmor polishedArmor = rawArmors[i];
        cv::Mat image = polishedArmor.Image;
        cv::Mat Image = image(cv::Range(0, 28), cv::Range(5, 23));
//      cv::resize(Image, Image, cv::Size(24,28));
//        cv::threshold(Image,Image,200,255,cv::THRESH_BINARY);
//        cv::resize(Image,Image,cv::Size(56, 56),2,2,cv::INTER_LINEAR);
//        cv::imwrite("/home/zhj/123/1.jpg", Image);
//         cv::imshow("1",Image);
//
//         cv::waitKey(75);
//
////        std::cout<<Image.size<<std::endl;
//
        cv::Mat dst;
//        double maxVal = 0;
//        double minVal = 0;
//        cv::minMaxLoc(Image, &minVal, &maxVal);
//        Image.convertTo(dst, Image.type(), 3.5, -135.0);
        Image.convertTo(dst, Image.type(), 16, -150);
//        cv::Mat pic;
//        cv::resize(dst, pic, cv::Size(280, 280), cv::INTER_LINEAR);
//        cv::imshow("kk", pic);
//        cv::waitKey(25);
        cv::Mat blob;
        blob = cv::dnn::blobFromImage(dst, 1.0, cv::Size(18, 28));


        // 模型运算
        net_.setInput(blob);
        cv::Mat outputs = net_.forward();
        int max_num = std::max_element(outputs.begin<float>(), outputs.end<float>()) - outputs.begin<float>();
        // 结果收集-
        EClassicalArmorNumber armorNumber = EClassicalArmorNumber::Invalid;
        ClassicalArmor::ConvertToClassicalArmorNumber(max_num, &armorNumber);

        polishedArmor.Number = armorNumber;
        polishedArmors->emplace_back(polishedArmor);
    }

    // 返回检测结果
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
        case EClassicalArmorNumber::One:
            RefreshArmorBuffer(polishedArmor, &oneArmorBuffer_);
            break;

        case EClassicalArmorNumber::Two:
            RefreshArmorBuffer(polishedArmor, &twoArmorBuffer_);
            break;

        case EClassicalArmorNumber::Three:
            RefreshArmorBuffer(polishedArmor, &threeArmorBuffer_);
            break;

        case EClassicalArmorNumber::Four:
            RefreshArmorBuffer(polishedArmor, &fourArmorBuffer_);
            break;

        case EClassicalArmorNumber::Five:
            RefreshArmorBuffer(polishedArmor, &fiveArmorBuffer_);
            break;

        case EClassicalArmorNumber::Sentry:
            RefreshArmorBuffer(polishedArmor, &sentryArmorBuffer_);
            break;

        case EClassicalArmorNumber::Outpost:
            RefreshArmorBuffer(polishedArmor, &outpostArmorBuffer_);
            break;

        case EClassicalArmorNumber::Base:
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
        case EClassicalArmorNumber::One:
        {
            for (unsigned int i = 0; i < oneArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(oneArmorBuffer_[i].Center, oneArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::Two:
        {
            for (unsigned int i = 0; i < twoArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(twoArmorBuffer_[i].Center, twoArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::Three:
        {
            for (unsigned int i = 0; i < threeArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(threeArmorBuffer_[i].Center, threeArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::Four:
        {
            for (unsigned int i = 0; i < fourArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(fourArmorBuffer_[i].Center, fourArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::Five:
        {
            for (unsigned int i = 0; i < fiveArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(fiveArmorBuffer_[i].Center, fiveArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::Sentry:
        {
            for (unsigned int i = 0; i < sentryArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(sentryArmorBuffer_[i].Center, sentryArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::Outpost:
        {
            for (unsigned int i = 0; i < outpostArmorBuffer_.size(); ++i)
            {
                locationSequence->emplace_back(outpostArmorBuffer_[i].Center, outpostArmorBuffer_[i].Timestamp);
            }
            break;
        }

        case EClassicalArmorNumber::Base:
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
        case EClassicalArmorNumber::One:
            RefreshTargetBuffer(target, timestamp, &oneTargetBuffer_);
            break;

        case EClassicalArmorNumber::Two:
            RefreshTargetBuffer(target, timestamp, &twoTargetBuffer_);
            break;

        case EClassicalArmorNumber::Three:
            RefreshTargetBuffer(target, timestamp, &threeTargetBuffer_);
            break;

        case EClassicalArmorNumber::Four:
            RefreshTargetBuffer(target, timestamp, &fourTargetBuffer_);
            break;

        case EClassicalArmorNumber::Five:
            RefreshTargetBuffer(target, timestamp, &fiveTargetBuffer_);
            break;

        case EClassicalArmorNumber::Sentry:
            RefreshTargetBuffer(target, timestamp, &sentryTargetBuffer_);
            break;

        case EClassicalArmorNumber::Outpost:
            RefreshTargetBuffer(target, timestamp, &outpostTargetBuffer_);
            break;

        case EClassicalArmorNumber::Base:
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
        case EClassicalArmorNumber::One:
            SearchForTarget(oneTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::Two:
            SearchForTarget(twoTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::Three:
            SearchForTarget(threeTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::Four:
            SearchForTarget(fourTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::Five:
            SearchForTarget(fiveTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::Sentry:
            SearchForTarget(sentryTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::Outpost:
            SearchForTarget(outpostTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        case EClassicalArmorNumber::Base:
            SearchForTarget(baseTargetBuffer_, timestamp, timestampOffset, targets);
            break;

        default:
            break;
    }

    // 返回检索结果
    return true;
}

// 训练并保存ArmorHogSvm分类模型
bool ClassicalArmorRecognizer::TrainArmorHogSvm(const std::vector<std::pair<std::string, EClassicalArmorNumber>> &trainingSet,
                                                const cv::TermCriteria &termCriteria,
                                                const std::string &modelFileName) const
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        log = "ArmorHogSvm model was trained failure because ClassicalArmorRecognizer has not been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 提取训练样本路径和和样本对应的装甲板编号
    std::vector<cv::String> imageVector;
    std::vector<EClassicalArmorNumber> armorNumberVector;
    for (int i = 0; i < trainingSet.size(); ++i)
    {
        // 遍历训练样本文件夹下的所有图像
        std::vector<cv::String> tempImageVector;
        cv::glob(trainingSet[i].first, tempImageVector, false);
        imageVector.insert(imageVector.end(), tempImageVector.begin(), tempImageVector.end());

        // 记录训练样本对应的装甲板编号
        for (unsigned int j = 0; j < tempImageVector.size(); ++j)
        {
            armorNumberVector.emplace_back(trainingSet[i].second);
        }
    }

    // 计算特征向量和标签
    std::vector<std::vector<float>> hogFeaturesVector;
    std::vector<int> labelVector;
    for (int i = 0; i < imageVector.size(); ++i)
    {
        // 读取训练样本图像
        cv::Mat image = cv::imread(imageVector[i]);
        if(image.empty() || (image.cols != param_.ArmorHogWindowWidth) || (image.rows != param_.ArmorHogWindowHeight))
        {
            log = "Traning sample is invalid. Path = " + (std::string)imageVector[i];
            logger.Save(ELogType::Error, log);
        }
        else
        {
            // 保存特征向量
            std::vector<float> hogFeatures;
            ComputeArmorHogFeatures(image, &hogFeatures);
            hogFeaturesVector.emplace_back(hogFeatures);

            // 保存标签
            labelVector.emplace_back(static_cast<int>(armorNumberVector[i]));
        }
    }

    // 计算Hog特征矩阵
    int hogFeatureDimension = static_cast<int>(armorHogDescriptor_.getDescriptorSize());
    cv::Mat hogFeatureMat(static_cast<int>(hogFeaturesVector.size()), hogFeatureDimension, CV_32FC1);
    for (int i = 0; i < hogFeaturesVector.size(); ++i)
    {
        for (int j = 0; j < hogFeatureDimension; ++j)
        {
            hogFeatureMat.at<float>(i, j) = hogFeaturesVector[i][j];
        }
    }

    // 计算标签矩阵
    cv::Mat labelMat(labelVector);

    // 训练ArmorHogSvm分类器模型
    cv::Ptr<cv::ml::SVM> armorHogSvm = cv::ml::SVM::create();
    armorHogSvm->setType(cv::ml::SVM::C_SVC);
    armorHogSvm->setKernel(cv::ml::SVM::LINEAR);
    armorHogSvm->setTermCriteria(termCriteria);
    bool result = armorHogSvm->train(hogFeatureMat, cv::ml::ROW_SAMPLE, labelMat);

    // 将训练好的ArmorHogSvm分类器模型保存为xml文件
    if (result)
    {
        try
        {
            armorHogSvm->SVM::save(modelFileName);
            log = "ArmorHogSvm model was trained and saved successful";
            logger.Save(ELogType::Info, log);
        }
        catch (cv::Exception &exception)
        {
            log = "[" + param_.Key + "] - ArmorHogSvm model was saved failure because: " + std::string(exception.err);
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "ArmorHogSvm model was trained failure";
        logger.Save(ELogType::Error, log);
    }

    // 记录日志信息
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回训练结果
    return result;
}

// 创建灯条图像
void ClassicalArmorRecognizer::CreateLightBarsImage(const std::vector<ClassicalLightBar> &lightBars,
                                                    const cv::Mat &rawImage,
                                                    cv::Mat *lightBarsImage)
{
    // 使用原始图像初始化灯条图像
    *lightBarsImage = rawImage.clone();

    // 判断灯条集合是否为空
    if (lightBars.empty())
    {
        // 在图像上方输出提示信息
        cv::putText(*lightBarsImage,
                    "There is no light bar in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);

        return;
    }

    // 绘制灯条的旋转外接矩形，在图像上方输出灯条的参数
    for (int i = 0; i < lightBars.size(); ++i)
    {
        // 绘制灯条的旋转外接矩形
        cv::line(*lightBarsImage,
                 lightBars[i].LeftLower,
                 lightBars[i].LeftUpper,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*lightBarsImage,
                 lightBars[i].LeftUpper,
                 lightBars[i].RightUpper,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*lightBarsImage,
                 lightBars[i].RightUpper,
                 lightBars[i].RightLower,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);
        cv::line(*lightBarsImage,
                 lightBars[i].RightLower,
                 lightBars[i].LeftLower,
                 cv::Scalar(0, 255, 0),
                 1,
                 8,
                 0);

        // 在灯条轮廓左上角标记灯条索引
        cv::putText(*lightBarsImage,
                    std::to_string(i + 1),
                    lightBars[i].LeftUpper,
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);

        // 在图像上方输出灯条参数
        std::string text = std::to_string(i + 1) +\
                           ":  Area = " + std::to_string(lightBars[i].Area) +\
                           ";  Width = " + std::to_string(lightBars[i].Width) +\
                           ";  Height = " + std::to_string(lightBars[i].Height) +\
                           ";  AspectRatio = " + std::to_string(lightBars[i].Height / lightBars[i].Width) +\
                           ";  Angle = " + std::to_string(lightBars[i].Angle);
        cv::putText(*lightBarsImage,
                    text,
                    cv::Point(10, 20 * i + 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    1);
    }
}

// 创建灯条对图像
void ClassicalArmorRecognizer::CreateLightBarPairsImage(const std::vector<std::pair<ClassicalLightBar, ClassicalLightBar>> &lightBarPairs,
                                                        const cv::Mat &rawImage,
                                                        cv::Mat *lightBarPairsImage)
{
    // 使用原始图像初始化灯条对图像
    *lightBarPairsImage = rawImage.clone();

    // 判断灯条对集合是否为空
    if (lightBarPairs.empty())
    {
        // 在图像上方输出提示信息
        cv::putText(*lightBarPairsImage,
                    "There is no light bar pair in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);

        return;
    }

    // 绘制灯条对的多边形轮廓
    for (unsigned int i = 0; i < lightBarPairs.size(); ++i)
    {
        cv::line(*lightBarPairsImage,
                 (lightBarPairs[i].first.LeftUpper + lightBarPairs[i].first.RightUpper) / 2,
                 (lightBarPairs[i].second.LeftUpper + lightBarPairs[i].second.RightUpper) / 2,
                 cv::Scalar(0,255,0),
                 1,
                 8,
                 0);
        cv::line(*lightBarPairsImage,
                 (lightBarPairs[i].second.LeftUpper + lightBarPairs[i].second.RightUpper) / 2,
                 (lightBarPairs[i].second.LeftLower + lightBarPairs[i].second.RightLower) / 2,
                 cv::Scalar(0,255,0),
                 1,
                 8,
                 0);
        cv::line(*lightBarPairsImage,
                 (lightBarPairs[i].second.LeftLower + lightBarPairs[i].second.RightLower) / 2,
                 (lightBarPairs[i].first.LeftLower + lightBarPairs[i].first.RightLower) / 2,
                 cv::Scalar(0,255,0),
                 1,
                 8,
                 0);
        cv::line(*lightBarPairsImage,
                 (lightBarPairs[i].first.LeftLower + lightBarPairs[i].first.RightLower) / 2,
                 (lightBarPairs[i].first.LeftUpper + lightBarPairs[i].first.RightUpper) / 2,
                 cv::Scalar(0,255,0),
                 1,
                 8,
                 0);
    }
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
void ClassicalArmorRecognizer::CreatePredictorArmorImage(const ClassicalArmor &trackedArmor,
                                                         const cv::Point2f &armorLocation,
                                                         const cv::Mat &rawImage,
                                                         cv::Mat *predictImage) {





}


// *********************************  ClassicalArmorRecognizer类的私有函数  *********************************

// 对装甲板轮廓区域进行透视变换
bool ClassicalArmorRecognizer::PerspectiveTransform(const std::vector<cv::Point2f> &armorRegionCorners,
                                                    const cv::Mat &rawImage,
                                                    cv::Mat *armorRegionImage) const
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 根据Hog特征描述子窗口的宽度和高度，初始化装甲板轮廓透视变换之后的角点坐标
    std::vector<cv::Point2f> destCorners;
    cv::Point2f leftUpperPoint;
    leftUpperPoint.x = 0.0;
    leftUpperPoint.y = 0.0;
    cv::Point2f leftLowerPoint;
    leftLowerPoint.x = 0.0;
    leftLowerPoint.y = static_cast<float>(param_.ArmorHogWindowHeight - 1);
    cv::Point2f rightUpperPoint;
    rightUpperPoint.x = static_cast<float>(param_.ArmorHogWindowWidth - 1);
    rightUpperPoint.y = 0.0;
    cv::Point2f rightLowerPoint;
    rightLowerPoint.x = static_cast<float>(param_.ArmorHogWindowWidth - 1);
    rightLowerPoint.y = static_cast<float>(param_.ArmorHogWindowHeight - 1);
    destCorners.emplace_back(leftUpperPoint);
    destCorners.emplace_back(rightUpperPoint);
    destCorners.emplace_back(rightLowerPoint);
    destCorners.emplace_back(leftLowerPoint);

    // 计算透视变换矩阵
    cv::Mat warpMatrix = cv::getPerspectiveTransform(armorRegionCorners, destCorners);

    // 进行透视变换
    cv::warpPerspective(rawImage,
                        *armorRegionImage,
                        warpMatrix,
                        cv::Size(param_.ArmorHogWindowWidth, param_.ArmorHogWindowHeight));

    // 返回变换结果
    return true;
}

// 计算装甲板轮廓区域的Hog特征
bool ClassicalArmorRecognizer::ComputeArmorHogFeatures(const cv::Mat &armorRegionImage,
                                                       std::vector<float> *armorHogFeatures) const
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 判断装甲板轮廓区域图像的尺寸是否合法
    if ((armorRegionImage.empty())
        || (armorRegionImage.cols != param_.ArmorHogWindowWidth)
        || (armorRegionImage.rows != param_.ArmorHogWindowHeight))
    {
        return false;
    }

    // 计算hog特征描述
    armorHogDescriptor_.compute(armorRegionImage, *armorHogFeatures);

    // 返回计算结果
    return true;
}

// 使用HogSvm模型检测装甲板编号
bool ClassicalArmorRecognizer::DetectArmorNumberByHogSvm(const std::vector<float> &armorHogFeatures,
                                                         EClassicalArmorNumber *armorNumber) const
{
    // 判断装甲板识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 判断ArmorHogSvm分类模型是否满足检测条件
    if ((armorHogSvm_.empty()) || (armorHogSvm_->getVarCount() != armorHogFeatures.size()))
    {
        return false;
    }

    // 检测装甲板编号
    int predictResult = static_cast<int>(armorHogSvm_->predict(armorHogFeatures));
    bool convertResult = ClassicalArmor::ConvertToClassicalArmorNumber(predictResult, armorNumber);

    // 返回检测结果
    return convertResult;
}

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