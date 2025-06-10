//
// Created by plutoli on 2022/1/23.
//

#include "classical_windmill_recognizer.h"
#define FOR(i,j,k) for (int i = j ; i < k ; i++)
// ******************************  ClassicalWindmillRecognizer类的公有函数  ******************************

// 构造函数
ClassicalWindmillRecognizer::ClassicalWindmillRecognizer():
        param_(),
        isInitialized_(false),
        initTimestamp_(0),
        logoHogDescriptor_(),
        logoHogSvm_(),
        operateMutex_(),
        logoBuffer_(),
        logoBufferMutex_(),
        fanBuffer_(),
        fanBufferMutex_(),
        targetBuffer_(),
        targetBufferMutex_()
{
}

// 析构函数
ClassicalWindmillRecognizer::~ClassicalWindmillRecognizer()
{
    // 释放系统资源
    if (IsInitialized())
    {
        Release();
    }
}

// 获取风车识别器参数
ClassicalWindmillRecognizerParam ClassicalWindmillRecognizer::GetParam()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return param_;
}

// 设置风车识别器参数
bool ClassicalWindmillRecognizer::SetParam(const ClassicalWindmillRecognizerParam &param)
{
    // 锁定风车识别器操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断风车识别器是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - ClassicalWindmillRecognizerParam was set failure "\
              "because ClassicalWindmillRecognizer has been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 记录风车识别器参数
    param_ = param;

    // 记录日志信息
    log = "[" + param_.Key + "] - ClassicalWindmillRecognizerParam was set successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回参数设置结果
    return true;
}

// 获取风车识别器的初始化状态
bool ClassicalWindmillRecognizer::IsInitialized()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return isInitialized_;
}

// 获取风车识别器的初始化时间戳
uint64_t ClassicalWindmillRecognizer::GetInitTimestamp()
{
    std::lock_guard<std::mutex> lockGuard(operateMutex_);
    return initTimestamp_;
}

// 初始化风车识别器
bool ClassicalWindmillRecognizer::Init()
{
    // 锁定风车识别器操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断风车识别器是否已经初始化
    if (isInitialized_)
    {
        log = "[" + param_.Key + "] - ClassicalWindmillRecognizer can not be initialized repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 创建风车Logo的Hog特征描述子
    cv::Size windowSize(param_.LogoHogWindowWidth, param_.LogoHogWindowHeight);
    cv::Size blockSize(param_.LogoHogBlockWidth, param_.LogoHogBlockHeight);
    cv::Size strideSize(param_.LogoHogStrideWidth, param_.LogoHogStrideHeight);
    cv::Size cellSize(param_.LogoHogCellWidth, param_.LogoHogCellHeight);
    int bins = param_.LogoHogBins;
    logoHogDescriptor_ = cv::HOGDescriptor(windowSize, blockSize, strideSize, cellSize, bins);

    // 判断LogoHogSvm模型文件是否存在
    bool isModelExist = true;
    if (::access(param_.LogoHogSvmFileName.c_str(), F_OK) == -1)
    {
        log = "[" + param_.Key + "] - LogoHogSvm file does not exist";
        logger.Save(ELogType::Error, log);
        isModelExist = false;
    }

    // 判断LogoHogSvm模型文件是否可读
    bool isModelReadable = true;
    if (::access(param_.LogoHogSvmFileName.c_str(), R_OK) == -1)
    {
        log = "[" + param_.Key + "] - LogoHogSvm file can not be read";
        logger.Save(ELogType::Error, log);
        isModelReadable = false;
    }

    // 加载LogoHogSvm模型
    if (isModelExist && isModelReadable)
    {
        try
        {
            logoHogSvm_ = cv::ml::SVM::load(param_.LogoHogSvmFileName);
            log = "[" + param_.Key + "] - LogoHogSvm model was loaded successful";
            logger.Save(ELogType::Info, log);
        }
        catch (cv::Exception &exception)
        {
            log = "[" + param_.Key + "] - LogoHogSvm model was loaded failure because: " + std::string(exception.err);
            logger.Save(ELogType::Error, log);
        }

        // 判断LogoHogSvm模型和Hog特征描述子的维数是否匹配
        if ((!logoHogSvm_.empty()) && (logoHogSvm_->getVarCount() == logoHogDescriptor_.getDescriptorSize()))
        {
            log = "[" + param_.Key + "] - LogoHogSvm model matched LogoHogDescriptor successful";
            logger.Save(ELogType::Info, log);
        }
        else
        {
            log = "[" + param_.Key + "] - LogoHogSvm model matched LogoHogDescriptor failure";
            logger.Save(ELogType::Error, log);
        }
    }

    // 清空数据缓冲区
    logoBuffer_.clear();
    fanBuffer_.clear();
    targetBuffer_.clear();

    // 设置初始化时间戳
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    initTimestamp_ = now.time_since_epoch().count();

    // 设置初始化状态
    isInitialized_ = true;

    // 记录日志信息
    log = "[" + param_.Key + "] - ClassicalWindmillRecognizer was initialized successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回初始化结果
    return true;
}

// 释放风车识别器资源
bool ClassicalWindmillRecognizer::Release()
{
    // 锁定风车识别器操作
    std::lock_guard<std::mutex> lockGuard(operateMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断风车识别器是否已经初始化
    if (!isInitialized_)
    {
        log = "[" + param_.Key + "] - ClassicalWindmillRecognizer can not be released repeatedly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 释放LogoHogSvm模型
    if (!logoHogSvm_.empty())
    {
        logoHogSvm_.release();
    }

    // 重置初始化时间戳
    initTimestamp_ = 0;

    // 重置初始化状态
    isInitialized_ = false;

    // 记录日志信息
    log = "[" + param_.Key + "] - ClassicalWindmillRecognizer was released successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回初始化结果
    return true;
}

// 对原始图像进行预处理
bool ClassicalWindmillRecognizer::Preprocess(const cv::Mat &rawImage, cv::Mat *binaryImageFan, cv::Mat *binaryImageTarget) const
{
    // 判断风车识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 对原始图像进行缩放
    cv::Mat resizedImage;

//    cv::Mat sharpImage;
//    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
//    filter2D(rawImage, sharpImage, -1, kernel);
//    cv::imshow("sharpImage", sharpImage);
//    cv::imshow("rawImage", rawImage);

    cv::resize(rawImage,
               resizedImage,
               cv::Size(),
               param_.DownsampleFactor,
               param_.DownsampleFactor);

    // 使用缩放后的图像初始化二值图
    *binaryImageFan = cv::Mat(resizedImage.rows, resizedImage.cols, CV_8UC1);
    *binaryImageTarget = cv::Mat(resizedImage.rows, resizedImage.cols, CV_8UC1);

    // 将缩放后的图像转换到HSV颜色空间
    cv::Mat hsvImage;
    cv::cvtColor(resizedImage, hsvImage, cv::COLOR_BGR2HSV);

    // 对HSV颜色空间的缩放图像进行阈值分割
    if (param_.WindmillColor == EClassicalWindmillColor::Blue)
    {
//        std::vector<cv::Mat> Channels;
//        cv::split(hsvImage,Channels);
//        FOR(i,0,hsvImage.cols)
//            FOR(j,0,hsvImage.rows){
//
//            uchar Hvalue =  Channels[0].at<uchar>(j,i);
//            uchar Svalue =  Channels[1].at<uchar>(j,i);
//            uchar Vvalue =  Channels[2].at<uchar>(j,i);
//
//            binaryImageFan->at<uchar>(j,i) = (
//                    Hvalue <= param_.FanHSVThresholdForBlue.HueUpper&&
//                    Hvalue >= param_.FanHSVThresholdForBlue.HueLower&&
//
//                    Svalue <= param_.FanHSVThresholdForBlue.SaturationUpper&&
//                    Svalue >= param_.FanHSVThresholdForBlue.SaturationLower&&
//
//                    Vvalue <= param_.FanHSVThresholdForBlue.ValueUpper&&
//                    Vvalue >= param_.FanHSVThresholdForBlue.ValueLower) ? 255 : 0;
//
//            binaryImageTarget->at<uchar>(j,i) = (
//                    Hvalue <= param_.TargetHSVThresholdForBlue.HueUpper&&
//                    Hvalue >= param_.TargetHSVThresholdForBlue.HueLower&&
//
//                    Svalue <= param_.TargetHSVThresholdForBlue.SaturationUpper&&
//                    Svalue >= param_.TargetHSVThresholdForBlue.SaturationLower&&
//
//                    Vvalue <= param_.TargetHSVThresholdForBlue.ValueUpper&&
//                    Vvalue >= param_.TargetHSVThresholdForBlue.ValueLower) ? 255 : 0;
//
//        }

        DeltaCV::InRange(hsvImage, param_.FanHSVThresholdForBlue, binaryImageFan);
        DeltaCV::InRange(hsvImage, param_.TargetHSVThresholdForBlue, binaryImageTarget);
//        DeltaCV::FastInRange(hsvImage, param_.FanHSVThresholdForBlue, binaryImageFan);
//        DeltaCV::FastInRange(hsvImage, param_.TargetHSVThresholdForBlue, binaryImageTarget);

    }
    else
    {
        cv::Mat binaryImage_1(resizedImage.rows, resizedImage.cols, CV_8UC1);
        cv::Mat binaryImage_2(resizedImage.rows, resizedImage.cols, CV_8UC1);
        cv::Mat binaryImage_3(resizedImage.rows, resizedImage.cols, CV_8UC1);
        cv::Mat binaryImage_4(resizedImage.rows, resizedImage.cols, CV_8UC1);

        DeltaCV::InRange(hsvImage, param_.FanHSVThresholdForRed_1, &binaryImage_1);
        DeltaCV::InRange(hsvImage, param_.FanHSVThresholdForRed_2, &binaryImage_2);
        DeltaCV::InRange(hsvImage, param_.TargetHSVThresholdForRed_1, &binaryImage_3);
        DeltaCV::InRange(hsvImage, param_.TargetHSVThresholdForRed_2, &binaryImage_4);

//        DeltaCV::FastInRange(hsvImage, param_.FanHSVThresholdForRed_1, &binaryImage_1);
//        DeltaCV::FastInRange(hsvImage, param_.FanHSVThresholdForRed_2, &binaryImage_2);
//        DeltaCV::FastInRange(hsvImage, param_.TargetHSVThresholdForRed_1, &binaryImage_3);
//        DeltaCV::FastInRange(hsvImage, param_.TargetHSVThresholdForRed_2, &binaryImage_4);
        *binaryImageFan = binaryImage_1 + binaryImage_2;
        *binaryImageTarget = binaryImage_3 + binaryImage_4;

    }

//    cv::imshow( " bef_binaryImageFan " ,*binaryImageFan);
//    cv::imshow( " bef_binaryImageTarget " ,*binaryImageTarget);

    // 对二值化图像进行膨胀操作。膨胀函数的ksize参数可以适当调整，确保膨胀之后未激活扇叶扇柄上流动的箭头和扇叶头部融为一体，\n
    // 同时确保扇叶头部位置内部还有孔洞，便于进行中心点检测。\n
    // OpenCV中的图像膨胀和腐蚀函数介绍详见：https://zhuanlan.zhihu.com/p/110330329

    cv::Size closeElementSize(param_.CloseElementWidth , param_.CloseElementHeight );

    cv::Mat closeElement = cv::getStructuringElement(cv::MORPH_RECT,
                                                     closeElementSize,
                                                     cv::Point(-1, -1));

    cv::Mat erodeElementFan = cv::getStructuringElement(cv::MORPH_RECT,
                                                      cv::Size(2, 2),
                                                      cv::Point(-1, -1));

    cv::Mat erodeElementTarget = cv::getStructuringElement(cv::MORPH_RECT,
                                                        cv::Size(2, 2),
                                                        cv::Point(-1, -1));

    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT,
                                                     cv::Size(3, 3),
                                                     cv::Point(-1, -1));

//    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT,
//                                                     cv::Size(3,3),
//                                                     cv::Point(-1, -1));


//    cv::imshow("src",*binaryImageFan);

    cv::erode(*binaryImageFan,
              *binaryImageFan,
              erodeElementFan);

    cv::morphologyEx(*binaryImageFan,
                     *binaryImageFan,
                     cv::MORPH_CLOSE,
                     closeElement,
                     cv::Point(-1, -1),
                     1);

//    cv::dilate(*binaryImageFan,
//               *binaryImageFan,
//               dilateElement);


//    cv::erode(*binaryImageTarget,
//              *binaryImageTarget,
//              erodeElementTarget);

    cv::morphologyEx(*binaryImageTarget,
                     *binaryImageTarget,
                     cv::MORPH_CLOSE,
                     closeElement,
                     cv::Point(-1, -1),
                     1);


//
//    cv::erode(*binaryImageTarget,
//              *binaryImageTarget,
//              erodeElementTarget);
//
//    cv::imshow("binaryImageFan", *binaryImageFan);
//    cv::imshow("binaryImageTarget", *binaryImageTarget);


//    cv::erode(*binaryImageFan,
//              *binaryImageTarget,
//              erodeElement);


    // 返回预处理结果
    return true;
}

// 检测风车Logo集合
bool ClassicalWindmillRecognizer::DetectWindmillLogos(const cv::Mat &binaryImage,
                                                      const cv::Mat &rawImage,
                                                      std::vector<ClassicalWindmillLogo> *windmillLogos) const
{
    // 判断风车识别器是否初始化完毕
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

//    cv::Mat test_image(rawImage.rows,rawImage.cols,CV_8UC1,cv::Scalar(0));
//
//    for ( auto i : contours )
//        for ( auto j : i )
//            test_image.at<uchar>(j.y,j.x) = 255;
//
//    cv::imshow(" test_image ",test_image);

    // 根据轮廓创建风车Logo
    for (unsigned int i = 0; i < contours.size(); ++i)
    {
        // 计算风车Logo在原始图像上的轮廓
        std::vector<cv::Point> contour;
        for (unsigned int j = 0; j < contours[i].size(); ++j)
        {
            contour.emplace_back(contours[i][j] / param_.DownsampleFactor);
        }

        // 计算风车Logo在原始图像上的轮廓面积，判断轮廓面积是否合法
        float contourArea = static_cast<float>(cv::contourArea(contour));
//        std::cout << " contourArea " << contourArea << std::endl;
        if ((contourArea < param_.MinLogoContourArea) || (contourArea > param_.MaxLogoContourArea))
        {
            continue;
        }

        // 计算风车Logo在原始图像上的最小正外接矩形，判断纵横比是否合法
        cv::Rect minEnclosedRect = cv::boundingRect(contour);

//        cv::Mat tempimag(rawImage);
//        DrawRect(tempimag, minEnclosedRect, cv::Scalar(255,255,255), 3);
//        cv::imshow(" test_rect_logo ", tempimag);

        float aspectRatio = static_cast<float>(minEnclosedRect.height) / static_cast<float>(minEnclosedRect.width);
        if ((aspectRatio < param_.MinLogoAspectRatio) || (aspectRatio > param_.MaxLogoAspectRatio))
        {
            continue;
        }

//        puts("yes");
//        cv::Mat tests_image(rawImage.rows,rawImage.cols,CV_8UC1,cv::Scalar(0));
//            for ( auto j : contour )
//                tests_image.at<uchar>(j.y,j.x) = 255;
//
//        cv::imshow(" tesst_image ",tests_image);

        // 计算风车Logo的中心点
        cv::Point2f center;
        center.x = static_cast<float>(minEnclosedRect.x + minEnclosedRect.width / 2.0);
        center.y = static_cast<float>(minEnclosedRect.y + minEnclosedRect.height / 2.0);

//        std::cout << "minEnclosedRectWidth: " << minEnclosedRect.width << std::endl;
//        std::cout << "minEnclosedRectHeight: " << minEnclosedRect.height << std::endl;
//        std::cout << "centerX: " << center.x << std::endl;
//        std::cout << "centerY: " << center.y << std::endl;

        std::vector<cv::Point2f> Point;
        cv::Point2f LeftUpper(center.x - minEnclosedRect.width / 2.0, center.y - minEnclosedRect.height / 2.0);
        cv::Point2f LeftLower(center.x - minEnclosedRect.width / 2.0, center.y + minEnclosedRect.height / 2.0);
        cv::Point2f RightUpper(center.x + minEnclosedRect.width / 2.0, center.y - minEnclosedRect.height / 2.0);
        cv::Point2f RightLower(center.x + minEnclosedRect.width / 2.0, center.y + minEnclosedRect.height / 2.0);
        Point.emplace_back(LeftUpper);
        Point.emplace_back(RightUpper);
        Point.emplace_back(RightLower);
        Point.emplace_back(LeftLower);

        // 计算风车Logo图像
        cv::Mat image = rawImage(minEnclosedRect);
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        cv::medianBlur(image, image, 3);
        cv::resize(image,
                   image,
                   cv::Size(param_.LogoHogWindowWidth, param_.LogoHogWindowHeight));

//        cv::imshow(" hog_svm " ,image);
//        cv::waitKey(0);

        // 创建并保存风车Logo
        ClassicalWindmillLogo windmillLogo;
        windmillLogo.IsValid = false;
        windmillLogo.Contour = contour;
        windmillLogo.ContourArea = contourArea;
        windmillLogo.MinEnclosedRect = minEnclosedRect;
        windmillLogo.ContourCenter = center;
        windmillLogo.Image = image;
        windmillLogo.AspectRatio = aspectRatio;
        windmillLogo.LogoBoundingPoints = Point;
        windmillLogo.Timestamp = 0;
        windmillLogos->emplace_back(windmillLogo);
    }

    // 使用LogoHogSvm模型检测风车Logo的合法性
    CheckWindmillLogosByHogSvm(windmillLogos);

    // 返回检测结果
    return true;
}

// 检测风车扇叶集合
bool ClassicalWindmillRecognizer::DetectWindmillFans(const cv::Mat &binaryImageFan,
                                                     const cv::Mat &binaryImageTarget,
                                                     const cv::Mat &rawImage,
                                                     ClassicalWindmillFan *windmillFan,
                                                     std::vector<ClassicalWindmillFan> *targetContours) const
{
    // 判断风车识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    cv::Mat src(rawImage);
    // 检索二值图像中的所有轮廓，并填充轮廓的索引结构
    // cv::findContours()函数使用说明详见：https://www.cnblogs.com/wojianxin/p/12602490.html
    //  hierarchyFan[i][4]  ==  [Next, Previous, First Child, Parent]
    //     Next             同一层级的下一条轮廓
    //     Previous         同一层级的上一条轮廓
    //     First Child      第一条子轮廓
    //     Parent           轮廓的父轮廓
    std::vector<cv::Vec4i> hierarchyFan;
    std::vector<std::vector<cv::Point>> contoursFan;
    cv::findContours(binaryImageFan,
                     contoursFan,
                     hierarchyFan,
                     cv::RETR_TREE,
                     cv::CHAIN_APPROX_NONE,
                     cv::Point(0, 0));
//
    cv::Mat tempsimag(src.rows,src.cols,CV_8UC1,cv::Scalar(0));

//    for (auto ni : contoursFan)
//        for ( auto n : ni)
//        tempsimag.at<uchar>(n.y,n.x) = 255;
//    cv::imshow(" tests_findContours ",tempsimag);

    std::vector<ClassicalWindmillFan> WindmillFans;
    for(unsigned int i = 0; i < contoursFan.size(); i++)
    {


//        cv::Mat tempsimag(src.rows,src.cols,CV_8UC1,cv::Scalar(0));

        std::vector<cv::Point> contour;
        for(unsigned int j = 0; j < contoursFan[i].size(); j++)
        {
            contour.emplace_back(contoursFan[i][j] / param_.DownsampleFactor);
        }

//        cv::Mat tempssimag(src.rows,src.cols,CV_8UC1,cv::Scalar(0));

        // 找到外轮廓
        if(hierarchyFan[i][3] != -1)
        {
            continue;
        }

        // 要保证每一个内嵌轮廓的上一轮廓面积不要太大，若太大，则不合法
        if(hierarchyFan[i][2] != -1)
        {
            bool isok = true;
            for ( unsigned int k = hierarchyFan[i][2];  (hierarchyFan[k][0]) != -1 ; k = hierarchyFan[k][0] ) {

                float contourArea = static_cast<float>(cv::contourArea(contoursFan[k]));

//                std::cout << " area =  " << contourArea << std::endl;
                if( contourArea > 1500 )  isok = false;
//                std::cout << " isok =  " << isok << std::endl;
            }

            if (!isok) continue;

        }


        // 计算扇叶在原始图像上的面积，判断轮廓是否合法
        float contourArea = static_cast<float>(cv::contourArea(contour));
//        std::cout << "contourArea:" << contourArea << std::endl;
        if(contourArea < param_.MinFanContourArea || contourArea > param_.MaxFanContourArea)
        {
            continue;
        }

//        for (auto n : contoursFan[i])
//            tempssimag.at<uchar>(n.y,n.x) = 255;
//        cv::imshow(" testss_findContours ",tempssimag);


//        for (auto n : contoursFan[i])
//            tempssimag.at<uchar>(n.y,n.x) = 255;
//        cv::imshow(" testss_findContours ",tempssimag);

        // 计算扇叶在原始图像上的外部轮廓最小旋转外接矩形，判断外部轮廓纵横比是否合法
        cv::RotatedRect minRect = cv::minAreaRect(contour);
//        drawRotatedRect(src, minRect, cv::Scalar(255,255,255),3);
//        std::cout << "contourArea: " << contourArea << std::endl;
        float rectWidth = minRect.size.width;
        float rectHeight = minRect.size.height;
        float rectAspectRatio = rectHeight / rectWidth;
        if(rectAspectRatio < param_.MinFanAspectRatio || rectAspectRatio > param_.MaxFanAspectRatio)
        {
            continue;
        }

//        for (auto n : contoursFan[i])
//            tempsimag.at<uchar>(n.y,n.x) = 255;
//        cv::imshow(" testss_findContours ",tempsimag);
//        cv::waitKey(0);

        // 角点数目
        std::vector<cv::Point> contourPoly;
        cv::approxPolyDP(contour, contourPoly, 0.5, true);


        // 进行保存
        ClassicalWindmillFan WindmillFan;
        WindmillFan.IsValid = false;
        WindmillFan.Contour = contour;
        WindmillFan.ContourArea = contourArea;
        WindmillFan.AspectRatio = rectAspectRatio;
        WindmillFan.RotatedRect = minRect;
        WindmillFan.ContourCenter = minRect.center;
        WindmillFan.ContourPoly = contourPoly;
        WindmillFan.PolyNum = contourPoly.size();
        WindmillFan.Timestamp = 0;
        WindmillFans.emplace_back(WindmillFan);
        // 记录扇叶轮廓的索引
    }

//    std::cout << "Size: " << WindmillFans.size() << std::endl;


    // 进行角点筛选
    float num = FLT_MIN;
    int PolyIndex = -1;
    bool IsAccomplish = false;
    for(unsigned int i = 0; i < WindmillFans.size(); i++)
    {
        if(WindmillFans[i].PolyNum > num)
        {
            IsAccomplish = true;
            num = WindmillFans[i].PolyNum;
            PolyIndex = i;
        }
    }
    if(!IsAccomplish)
    {
        return false;
    }

    *windmillFan = WindmillFans[PolyIndex];

//    std::cout << "<-----PolyNUum----> " << num << std::endl;
//    std::cout << "ValidArea: " << (*windmillFan).ContourArea << std::endl;
//    drawRotatedRect(src, (*windmillFan).RotatedRect, cv::Scalar(255,255,255), 3); //------------------------------------------------------------------------------------------

    std::vector<cv::Vec4i> hierarchyTarget;
    std::vector<std::vector<cv::Point>> contoursTarget;
    cv::findContours(binaryImageTarget,
                     contoursTarget,
                     hierarchyTarget,
                     cv::RETR_TREE,
                     cv::CHAIN_APPROX_NONE,
                     cv::Point(0, 0));

    cv::Mat tempimag(src.rows,src.cols,CV_8UC1,cv::Scalar(0));

//    for (auto mi : contoursTarget)
//        for (auto n : mi)
//        tempimag.at<uchar>(n.y,n.x) = 255;
//    cv::imshow(" test_findContours_target ",tempimag);

    float minArea = 0x3f3f3f;
    // 寻找头部并筛选保存
    for(unsigned int i = 0; i < contoursTarget.size(); i++)
    {
//        cv::Mat testimge(src.rows,src.cols,CV_8UC1,cv::Scalar(0));
//        cv::Mat tes2timge(src.rows,src.cols,CV_8UC1,cv::Scalar(0));
//        cv::Mat tes4timge(src.rows,src.cols,CV_8UC1,cv::Scalar(0));
//        cv::Mat tes3timge(src.rows,src.cols,CV_8UC1,cv::Scalar(0));

//
//            for (auto n : contoursTarget[i])
//            {
//                testimge.at<uchar>(n.y,n.x) = 255;
//            }
//            cv::imshow(" tes1t_target ", testimge);

//        if(hierarchyTarget[i][2] != -1 && contoursTarget[static_cast<uint>(hierarchyTarget[i][2])].size() < 200 && hierarchyTarget[i][3] == -1)  //第一版
        if ( hierarchyTarget[i][2] != -1 && hierarchyTarget[hierarchyTarget[i][2]][2] == -1  && hierarchyTarget[i][3] != -1 )       //第二版
//          if(hierarchyTarget[i][2] != -1 && contoursTarget[static_cast<uint>(hierarchyTarget[i][2])].size() < 200 && hierarchyTarget[i][3] == -1)  //第三版
        {

//            puts("111");
//            for (auto n : contoursTarget[i])
//            {
//                tes2timge.at<uchar>(n.y,n.x) = 255;
//            }
//            cv::imshow(" test2_target ", tes2timge);

            std::vector<cv::Point> contour;
            for (unsigned int j = 0; j < contoursTarget[i].size(); j++)
            {
                contour.emplace_back(contoursTarget[i][j] / param_.DownsampleFactor);
            }

//            cv::RotatedRect tempRect = cv::minAreaRect(contoursTarget[static_cast<uint>(hierarchyTarget[i][2])]);
//            cv::RotatedRect tempRect = cv::minAreaRect(contour);
//            drawRotatedRect(src, tempRect, cv::Scalar(255,255,255),3);

            // 计算扇头在原始图像上的面积，判断扇头是否合法
            float ContourArea = static_cast<float>(cv::contourArea(contour));
//            std::cout << " area = " << ContourArea << std::endl;
            if (ContourArea < param_.MinTargetContourArea || ContourArea > param_.MaxTargetContourArea)
            {
                continue;
            }

//            std::cout << ContourArea << std::endl;

            // 计算扇头在原始图像上的轮廓最小旋转外接矩形，判断轮廓纵横比是否合法
            cv::Rect minEnclosedRect = cv::boundingRect(contour);
            cv::RotatedRect minRect = cv::minAreaRect(contour);
//            drawRotatedRect(src, minRect, cv::Scalar(255,255,255),3);
            float aspectRatio = static_cast<float>(minEnclosedRect.height) / static_cast<float>(minEnclosedRect.width);
//            std::cout << "rectAspectRatio:" << aspectRatio << std::endl;
            if (aspectRatio < param_.MinTargetAspectRatio || aspectRatio > param_.MaxTargetAspectRatio)
            {
                continue;
            }

//            for (auto n : contoursTarget[i])
//            {
//                tes3timge.at<uchar>(n.y,n.x) = 255;
//            }
//            cv::imshow(" tes3t_target ", tes3timge);

            cv::Point2f center;
            center.x = static_cast<float>(minEnclosedRect.x + minEnclosedRect.width / 2.0);
            center.y = static_cast<float>(minEnclosedRect.y + minEnclosedRect.height / 2.0);

//            cv::circle(src, center, 3, cv::Scalar(255,255,255),3);
//            cv::Point2f center;
//            float radius = 0;
//            cv::minEnclosingCircle(contour, center, radius);

            // 创建并保存符合条件的扇头
            ClassicalWindmillFan targetContour;
            targetContour.IsValid = false;
            targetContour.targetContour = contour;
            targetContour.targetContourArea = ContourArea;
            targetContour.AspectRatio = aspectRatio;
            targetContour.targetRect = minEnclosedRect;
            targetContour.RotatedRect = minRect;
            targetContour.targetCenter = center;
            targetContour.Timestamp = 0;
            targetContours->emplace_back(targetContour);
        }
    }

    return true;
}

// 更新风车识别器Logo位置缓冲区
bool ClassicalWindmillRecognizer::UpdateLogoBuffer(const ClassicalWindmillLogo &validLogo)
{
    // 判断风车识别器的初始化状态和Logo的合法性
    if ((!isInitialized_) || (!validLogo.IsValid))
    {
        return false;
    }

    // 锁定Logo位置缓冲区
    std::lock_guard<std::mutex> lockGuard(logoBufferMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 更新Logo位置缓冲区
    if (!logoBuffer_.empty())
    {
        // 计算Logo位置偏差
        float offset_x = validLogo.ContourCenter.x - logoBuffer_.back().ContourCenter.x;
        float offset_y = validLogo.ContourCenter.y - logoBuffer_.back().ContourCenter.y;
        auto offset = static_cast<float>(std::sqrt(std::pow(offset_x, 2) + std::pow(offset_y, 2)));

        // 如果位置偏差过大，则清空Logo位置缓冲区
        if (offset > param_.MaxLogoLocationOffset)
        {
            // 清空Logo位置缓冲区
            logoBuffer_.clear();

            // 记录日志信息
            log = "[" + param_.Key + "] - ClassicalWindmillRecognizer's logo buffer was cleared "\
                  "because logo's location changed too much";
            logger.Save(ELogType::Warn, log);
        }
    }

    // 存储Logo
    logoBuffer_.emplace_back(validLogo);

    // 删除超出记忆时间长度的元素；时间戳的单位是纳秒，记忆时间长度的单位是毫秒
    uint64_t maxMemoryLength = param_.MaxMemoryLength * 1000000;
    while ((validLogo.Timestamp - logoBuffer_.front().Timestamp) > maxMemoryLength)
    {
        logoBuffer_.pop_front();
    }

    // 返回更新结果
    return true;
}

// 清空风车识别器Logo位置缓冲区
void ClassicalWindmillRecognizer::ClearLogoBuffer()
{
    std::lock_guard<std::mutex> lockGuard(logoBufferMutex_);
    logoBuffer_.clear();
}

// 获取风车Logo位置时序
bool ClassicalWindmillRecognizer::GetLogoLocationSequence(std::vector<std::pair<cv::Point2f, uint64_t>> *logoLocationSequence)
{
    // 判断风车识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 锁定Logo位置缓冲区
    std::lock_guard<std::mutex> lockGuard(logoBufferMutex_);

    // 填充风车Logo位置时序
    for (unsigned int i = 0; i < logoBuffer_.size(); ++i)
    {
        logoLocationSequence->emplace_back(logoBuffer_[i].ContourCenter, logoBuffer_[i].Timestamp);
    }

    // 返回获取结果
    return true;
}

// 更新风车识别器扇叶位置缓冲区
bool ClassicalWindmillRecognizer::UpdateFanBuffer(const ClassicalWindmillFan &validFan)
{
    // 判断风车识别器的初始化状态和风车扇叶的合法性
    if ((!isInitialized_) || (!validFan.IsValid))
    {
        return false;
    }

    // 锁定扇叶位置缓冲区
    std::lock_guard<std::mutex> lockGuard(fanBufferMutex_);

    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 更新扇叶位置缓冲区
    if (!fanBuffer_.empty())
    {
        // 计算扇叶位置偏差
        float offset_x = validFan.targetCenter.x - fanBuffer_.back().targetCenter.x;
        float offset_y = validFan.targetCenter.y - fanBuffer_.back().targetCenter.y;
        auto offset = static_cast<float>(std::sqrt(std::pow(offset_x, 2) + std::pow(offset_y, 2)));

        // 如果位置偏差过大，则清空扇叶位置缓冲区
        if (offset > param_.MaxFanLocationOffset)
        {
            // 清空扇叶位置缓冲区
            fanBuffer_.clear();

            // 记录日志信息
            log = "[" + param_.Key + "] - ClassicalWindmillRecognizer's fan buffer was cleared "\
                  "because fan's location changed too much";
            logger.Save(ELogType::Warn, log);
        }
    }

    // 存储扇叶
    fanBuffer_.emplace_back(validFan);

    // 删除超出记忆时间长度的元素；时间戳的单位是纳秒，记忆时间长度的单位是毫秒
    uint64_t maxMemoryLength = param_.MaxMemoryLength * 1000000;
    while ((validFan.Timestamp - fanBuffer_.front().Timestamp) > maxMemoryLength)
    {
        fanBuffer_.pop_front();
    }

    // 返回更新结果
    return true;
}

// 清空风车识别器扇叶位置缓冲区
void ClassicalWindmillRecognizer::ClearFanBuffer()
{
    std::lock_guard<std::mutex> lockGuard(fanBufferMutex_);
    fanBuffer_.clear();
}

// 获取风车扇叶位置时序
bool ClassicalWindmillRecognizer::GetFanLocationSequence(std::vector<std::pair<cv::Point2f, uint64_t>> *fanLocationSequence)
{
    // 判断风车识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 锁定扇叶位置缓冲区
    std::lock_guard<std::mutex> lockGuard(fanBufferMutex_);

    // 填充风车扇叶位置时序
    for (unsigned int i = 0; i < fanBuffer_.size(); ++i)
    {
        fanLocationSequence->emplace_back(fanBuffer_[i].targetCenter, fanBuffer_[i].Timestamp);
    }

    // 返回获取结果
    return true;
}

// 更新风车识别器目标点数据缓冲区
bool ClassicalWindmillRecognizer::UpdateTargetBuffer(const cv::Point2f &compensatedTarget, const uint64_t &timestamp)
{
    // 判断风车识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 锁定风车识别器目标点数据缓冲区
    std::lock_guard<std::mutex> lockGuard(targetBufferMutex_);

    // 存储补偿之后的击打目标点
    targetBuffer_.emplace_back(compensatedTarget, timestamp);

    // 删除超出记忆时间长度的元素；时间戳的单位是纳秒，记忆时间长度的单位是毫秒
    uint64_t maxMemoryLength = param_.MaxMemoryLength * 1000000;
    while ((timestamp - targetBuffer_.front().second) > maxMemoryLength)
    {
        targetBuffer_.pop_front();
    }

    // 返回更新结果
    return true;
}

// 清空风车识别器目标点数据缓冲区
void ClassicalWindmillRecognizer::ClearTargetBuffer()
{
    std::lock_guard<std::mutex> lockGuard(targetBufferMutex_);
    targetBuffer_.clear();
}

// 获取符合时间戳要求的历史击打目标点集合
bool ClassicalWindmillRecognizer::GetTarget(const uint64_t &timestamp,
                                            const uint64_t &timestampOffset,
                                            std::vector<std::pair<cv::Point2f, uint64_t>> *targets)
{
    // 判断风车识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 锁定风车识别器目标点数据缓冲区
    std::lock_guard<std::mutex> lockGuard(targetBufferMutex_);

    // 搜索符合时间戳要求的历史目标点
    for (unsigned int i = 0; i < targetBuffer_.size(); ++i)
    {
        if ((targetBuffer_[i].second >= (timestamp - timestampOffset)) &&
            (targetBuffer_[i].second <= (timestamp + timestampOffset)))
        {
            targets->emplace_back(targetBuffer_[i]);
        }
    }

    // 返回获取结果
    return true;
}

// 训练并保存LogoHogSvm模型
bool ClassicalWindmillRecognizer::TrainLogoHogSvm(const std::string &positivePath,
                                                  const std::string &negativePath,
                                                  const cv::TermCriteria &termCriteria,
                                                  const std::string &modelFileName) const
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断风车识别器是否初始化完毕
    if (!isInitialized_)
    {
        log = "LogoHogSvm model was trained failure because ClassicalWindmillRecognizer has not been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 初始化训练样本路径集合和样本隶属的类别集合
    std::vector<cv::String> imageVector;
    std::vector<bool> categoryVector;

    // 提取正样本
    std::vector<cv::String> positiveImageVector;
    cv::glob(positivePath, positiveImageVector, false);
    imageVector.insert(imageVector.end(), positiveImageVector.begin(), positiveImageVector.end());
    for (unsigned int i = 0; i < positiveImageVector.size(); ++i)
    {
        categoryVector.emplace_back(true);
    }

    // 提取负样本
    std::vector<cv::String> negativeImageVector;
    cv::glob(negativePath, negativeImageVector, false);
    imageVector.insert(imageVector.end(), negativeImageVector.begin(), negativeImageVector.end());
    for (unsigned int i = 0; i < negativeImageVector.size(); ++i)
    {
        categoryVector.emplace_back(false);
    }

    // 计算特征向量和标签向量
    std::vector<std::vector<float>> hogFeaturesVector;
    std::vector<int> labelVector;
    for (unsigned int i = 0; i < imageVector.size(); ++i)
    {
        // 读取训练样本图像
        cv::Mat image = cv::imread(imageVector[i]);
        if(image.empty() || (image.cols != param_.LogoHogWindowWidth) || (image.rows != param_.LogoHogWindowHeight))
        {
            log = "Traning sample is invalid. Path = " + (std::string)imageVector[i];
            logger.Save(ELogType::Error, log);
        }
        else
        {
            // 保存特征向量
            std::vector<float> hogFeatures;
            ComputeLogoHogFeatures(image, &hogFeatures);
            hogFeaturesVector.emplace_back(hogFeatures);

            // 保存标签
            labelVector.emplace_back(static_cast<int>(categoryVector[i]));
        }
    }

    // 计算Hog特征矩阵
    int hogFeatureDimension = static_cast<int>(logoHogDescriptor_.getDescriptorSize());
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

    // 训练LogoHogSvm分类器
    cv::Ptr<cv::ml::SVM> logoHogSvm = cv::ml::SVM::create();
    logoHogSvm->setType(cv::ml::SVM::C_SVC);
    logoHogSvm->setKernel(cv::ml::SVM::LINEAR);
    logoHogSvm->setTermCriteria(termCriteria);
    bool result = logoHogSvm->train(hogFeatureMat, cv::ml::ROW_SAMPLE, labelMat);

    // 将训练好的LogoHogSvm分类器保存为xml文件
    if (result)
    {
        try
        {
            logoHogSvm->save(modelFileName);
            log = "LogoHogSvm model was trained and saved successful";
            logger.Save(ELogType::Info, log);
        }
        catch (cv::Exception &exception)
        {
            log = "[" + param_.Key + "] - LogoHogSvm model was saved failure because: " + std::string(exception.err);
            logger.Save(ELogType::Error, log);
        }
    }
    else
    {
        log = "LogoHogSvm model was trained failure";
        logger.Save(ELogType::Error, log);
    }

    // 记录日志信息
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回训练结果
    return result;
}

// 计算从风车Logo中心点到风车扇叶中心点的方向矢量角度；角度范围为[0°, 360°)
void ClassicalWindmillRecognizer::ComputeWindmillFanAngle(const cv::Point2f &logoCenter,
                                                          const cv::Point2f &fanCenter,
                                                          float *angle)
{
    // 计算从风车Logo中心点到风车扇叶中心点的方向矢量
    cv::Point2f direction;
    direction.x = fanCenter.x - logoCenter.x;
    direction.y = logoCenter.y - fanCenter.y;

    // 计算方向矢量与x轴正方向的夹角弧度值
    float radian = std::atan2(direction.y, direction.x);  // std::atan2()的取值范围(-pai, pai]
    if (radian < 0.0)
    {
        radian += 2 * CV_PI;
    }

    // 计算方向矢量与x轴正方向的夹角角度值
    *angle = static_cast<float>(radian * 180.0 / CV_PI);
}

// 创建风车轮廓图像
void ClassicalWindmillRecognizer::CreateWindmillContoursImage(const cv::Mat &binaryImage,
                                                              const ClassicalWindmillRecognizerParam &param,
                                                              const cv::Mat &rawImage,
                                                              cv::Mat *windmillContoursImage)
{
    // 使用原始图像初始化风车轮廓图像
    *windmillContoursImage = rawImage.clone();

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

    // 计算原始图像上的轮廓、轮廓面积、最小旋转外接矩形、最小正外接矩形
    std::vector<float> zoomedAreas;
    std::vector<std::vector<cv::Point>> zoomedContours;
    std::vector<cv::Rect> zoomedEnclosedRects;
    std::vector<cv::RotatedRect> zoomedRotatedRects;
    for (unsigned int i = 0; i < contours.size(); ++i)
    {
        // 计算原始图像上的轮廓
        std::vector<cv::Point> zoomedContour;
        for (unsigned int j = 0; j < contours[i].size(); ++j)
        {
            zoomedContour.emplace_back(contours[i][j] / param.DownsampleFactor);
        }
        zoomedContours.emplace_back(zoomedContour);

        // 计算原始图像上的轮廓面积
        float zoomedArea = static_cast<float>(cv::contourArea(zoomedContour));
        zoomedAreas.emplace_back(zoomedArea);

        // 计算原始图像上的最小外接矩形
        cv::Rect zoomedEnclosedRect = cv::boundingRect(zoomedContour);
        zoomedEnclosedRects.emplace_back(zoomedEnclosedRect);

        // 计算原始图像上的最小旋转外接矩形
        cv::RotatedRect zoomedRotatedRect = cv::minAreaRect(zoomedContour);
        zoomedRotatedRects.emplace_back(zoomedRotatedRect);
    }

    // 对轮廓信息按照面积大小进行排序
    std::vector<float> sortedAreas;
    std::vector<std::vector<cv::Point>> sortedContours;
    std::vector<cv::Rect> sortedEnclosedRects;
    std::vector<cv::RotatedRect> sortedRotatedRects;
    for (unsigned int i = 0; i < zoomedAreas.size(); ++i)
    {
        std::vector<float>::iterator maxPosition = max_element(zoomedAreas.begin(), zoomedAreas.end());
        unsigned int maxIndex = maxPosition - zoomedAreas.begin();
        sortedAreas.emplace_back(zoomedAreas[maxIndex]);
        sortedContours.emplace_back(zoomedContours[maxIndex]);
        sortedEnclosedRects.emplace_back(zoomedEnclosedRects[maxIndex]);
        sortedRotatedRects.emplace_back(zoomedRotatedRects[maxIndex]);
        zoomedAreas[maxIndex] = -1.0;
    }

    // 判断轮廓集合是否为空
    if (sortedContours.empty())
    {
        // 在图像上方输出提示信息
        cv::putText(*windmillContoursImage,
                    "There is no contour in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
    }
    else
    {
        // 绘制所有轮廓
        cv::drawContours(*windmillContoursImage,
                         sortedContours,
                         -1,
                         cv::Scalar(0, 255, 255),
                         1);

        // 在各个轮廓中心点标注轮廓索引，在图像上方输出轮廓面积
        for (int i = 0; i < sortedContours.size(); ++i)
        {
            // 计算轮廓的中心点
            cv::Point2f center;
            cv::Point2f vertices[4];
            sortedRotatedRects[i].points(vertices);
            center.x = (vertices[0].x + vertices[1].x + vertices[2].x + vertices[3].x) / 4;
            center.y = (vertices[0].y + vertices[1].y + vertices[2].y + vertices[3].y) / 4;

            // 在轮廓中心点位置标注轮廓索引
            cv::putText(*windmillContoursImage,
                        std::to_string(i + 1),
                        center,
                        cv::FONT_HERSHEY_SIMPLEX,
                        1.0,
                        cv::Scalar(255, 255, 255),
                        2);

            // 在图像上方输出轮廓面积、最小正外接矩形的宽度和高度、最小旋转外接矩形的宽度和高度
            int area = static_cast<int>(sortedAreas[i]);
            int e_Width = sortedEnclosedRects[i].width;
            int e_Height = sortedEnclosedRects[i].height;
            int r_Width = static_cast<int>(sortedRotatedRects[i].size.width);
            int r_Height = static_cast<int>(sortedRotatedRects[i].size.height);
            std::string text = std::to_string(i + 1) +\
                               ":  Area = " + std::to_string(area) +\
                               ";  E_Width = " + std::to_string(e_Width) +\
                               ";  E_Height = " + std::to_string(e_Height) +\
                               ";  R_Width = " + std::to_string(r_Width) +\
                               ";  R_Height = " + std::to_string(r_Height);
            cv::putText(*windmillContoursImage,
                        text,
                        cv::Point(10, 20 * i + 20),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(0, 255, 0),
                        1);
        }
    }
}

// 创建风车Logo图像
void ClassicalWindmillRecognizer::CreateWindmillLogosImage(const std::vector<ClassicalWindmillLogo> &windmillLogos,
                                                           const cv::Mat &rawImage,
                                                           cv::Mat *windmillLogosImage)
{
    // 使用原始图像初始化风车Logo图像
    *windmillLogosImage = rawImage.clone();

    // 判断风车Logo集合是否为空
    if (windmillLogos.empty())
    {
        // 在图像上方输出提示信息
        cv::putText(*windmillLogosImage,
                    "There is no logo in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
    }
    else
    {
        for (int i = 0; i < windmillLogos.size(); ++i)
        {
            if (windmillLogos[i].IsValid)
            {
                // 绘制合法Logo的外部轮廓
                std::vector<std::vector<cv::Point>> contours;
                contours.emplace_back(windmillLogos[i].Contour);
                cv::drawContours(*windmillLogosImage,
                                 contours,
                                 -1,
                                 cv::Scalar(0, 255, 0),
                                 1);

                // 绘制合法Logo的中心点
                cv::circle(*windmillLogosImage,
                           windmillLogos[i].ContourCenter,
                           3,
                           cv::Scalar(0, 255, 0),
                           3);
            }
            else
            {
                // 绘制非法Logo的外部轮廓
                std::vector<std::vector<cv::Point>> contours;
                contours.emplace_back(windmillLogos[i].Contour);
                cv::drawContours(*windmillLogosImage,
                                 contours,
                                 -1,
                                 cv::Scalar(255, 255, 255),
                                 1);
            }

            // 在Logo中心点标注Logo索引
            cv::putText(*windmillLogosImage,
                        std::to_string(i + 1),
                        windmillLogos[i].ContourCenter,
                        cv::FONT_HERSHEY_SIMPLEX,
                        1.0,
                        cv::Scalar(255, 255, 255),
                        2);

            // 在图像上方输出Logo的属性
            int area = static_cast<int>(windmillLogos[i].ContourArea);
            int width = static_cast<int>(windmillLogos[i].MinEnclosedRect.width);
            int height = static_cast<int>(windmillLogos[i].MinEnclosedRect.height);
            std::string text = std::to_string(i + 1) +\
                               ":  IsValid = " + std::to_string(windmillLogos[i].IsValid) +\
                               ";  Area = " + std::to_string(area) +\
                               ";  Width = " + std::to_string(width) +\
                               ";  Height = " + std::to_string(height) +\
                               ";  AspectRatio = " + std::to_string(windmillLogos[i].AspectRatio);
            cv::putText(*windmillLogosImage,
                        text,
                        cv::Point(10, 20 * i + 20),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(0, 255, 0),
                        1);
        }
    }
}

// 创建风车扇叶图像
void ClassicalWindmillRecognizer::CreateWindmillFansImage(const std::vector<ClassicalWindmillFan> &windmillFans,
                                                          const cv::Mat &rawImage,
                                                          cv::Mat *windmillFansImage)
{
    // 使用原始图像初始化风车扇叶图像
    *windmillFansImage = rawImage.clone();

    // 判断风车扇叶集合是否为空
    if (windmillFans.empty())
    {
        // 在图像上方输出提示信息
        cv::putText(*windmillFansImage,
                    "There is no windmill fan in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
    }
    else
    {
        // 绘制风车扇叶的轮廓，在图像上方输出风车扇叶的参数
        for (int i = 0; i < windmillFans.size(); ++i)
        {
            // 绘制风车扇叶轮廓
            if (windmillFans[i].IsValid)
            {
                // 绘制合法扇叶的外部轮廓
                std::vector<std::vector<cv::Point>> externalContours;
                externalContours.emplace_back(windmillFans[i].Contour);
                cv::drawContours(*windmillFansImage,
                                 externalContours,
                                 -1,
                                 cv::Scalar(0, 255, 0),
                                 1);

                // 绘制合法扇叶的头部轮廓
                std::vector<std::vector<cv::Point>> headContours;
                headContours.emplace_back(windmillFans[i].targetContour);
                cv::drawContours(*windmillFansImage,
                                 headContours,
                                 -1,
                                 cv::Scalar(0, 255, 0),
                                 1);

                // 绘制合法扇叶头部轮廓的中心点
                cv::circle(*windmillFansImage,
                           windmillFans[i].targetCenter,
                           3,
                           cv::Scalar(0, 255, 0),
                           3);
            }
            else
            {
                // 绘制非法扇叶的外部轮廓
                std::vector<std::vector<cv::Point>> externalContours;
                externalContours.emplace_back(windmillFans[i].Contour);
                cv::drawContours(*windmillFansImage,
                                 externalContours,
                                 -1,
                                 cv::Scalar(255, 255, 255),
                                 1);

                std::vector<std::vector<cv::Point>> headContours;
                headContours.emplace_back(windmillFans[i].targetContour);
                cv::drawContours(*windmillFansImage,
                                 headContours,
                                 -1,
                                 cv::Scalar(255, 255, 255),
                                 1);
            }

            // 在风车扇叶轮廓的中心点位置标注扇叶索引
            int fanIndex_x = static_cast<int>(windmillFans[i].ContourCenter.x);
            int fanIndex_y = static_cast<int>(windmillFans[i].ContourCenter.y);
            cv::putText(*windmillFansImage,
                        std::to_string(i + 1),
                        cv::Point(fanIndex_x, fanIndex_y),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1.0,
                        cv::Scalar(255, 255, 255),
                        2);

            // 在图像上方输出风车扇叶参数
            int externalArea = static_cast<int>(windmillFans[i].ContourArea);
            int externalWidth = static_cast<int>(windmillFans[i].RotatedRect.size.width);
            int externalHeight = static_cast<int>(windmillFans[i].RotatedRect.size.height);
            std::string text = std::to_string(i + 1) +\
                               ":  IsValid = " + std::to_string(windmillFans[i].IsValid) +\
                               ";  ExternalArea = " + std::to_string(externalArea) +\
                               ";  ExternalWidth = " + std::to_string(externalWidth) +\
                               ";  ExternalHeight = " + std::to_string(externalHeight) +\
                               ";  ExternalAspectRatio = " + std::to_string(windmillFans[i].AspectRatio) +\
                               ";  OffsetAngle = " + std::to_string(windmillFans[i].OffsetAngle);
            cv::putText(*windmillFansImage,
                        text,
                        cv::Point(10, 20 * i + 20),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(0, 255, 0),
                        1);
        }
    }
}

// 创建风车目标图像
void ClassicalWindmillRecognizer::CreateWindmillObjectsImage(const std::vector<ClassicalWindmillLogo> &windmillLogos,
                                                             const std::vector<ClassicalWindmillFan> &windmillFans,
                                                             const cv::Mat &rawImage,
                                                             cv::Mat *windmillObjectsImage)
{
    // 使用原始图像初始化风车目标图像
    *windmillObjectsImage = rawImage.clone();

    // 检测合法的风车Logo
    bool isWindmillLogoReady = false;
    ClassicalWindmillLogo windmillLogo;
    for (unsigned int i = 0; i < windmillLogos.size(); ++i)
    {
        if (windmillLogos[i].IsValid)
        {
            windmillLogo = windmillLogos[i];
            isWindmillLogoReady = true;
            break;
        }
    }

    // 输出合法风车Logo参数
    if (!isWindmillLogoReady)
    {
        // 在图像上方输出提示信息
        cv::putText(*windmillObjectsImage,
                    "There is no valid windmill logo in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
    }
    else
    {
        // 绘制合法Logo的外部轮廓
        std::vector<std::vector<cv::Point>> contours;
        contours.emplace_back(windmillLogo.Contour);
        cv::drawContours(*windmillObjectsImage,
                         contours,
                         -1,
                         cv::Scalar(0, 255, 0),
                         1);

        // 绘制合法Logo的中心点
        cv::circle(*windmillObjectsImage,
                   windmillLogo.ContourCenter,
                   3,
                   cv::Scalar(0, 255, 0),
                   3);

        // 在图像上方输出合法Logo的参数
        int logo_x = static_cast<int>(windmillLogo.ContourCenter.x);
        int logo_y = static_cast<int>(windmillLogo.ContourCenter.y);
        std::string text = "Logo.x = " + std::to_string(logo_x) + ";  Logo.y = " + std::to_string(logo_y);
        cv::putText(*windmillObjectsImage,
                    text,
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    1);
    }

    // 检测合法的风车扇叶
    bool isWindmillFanReady = false;
    ClassicalWindmillFan windmillFan;
    for (unsigned int i = 0; i < windmillFans.size(); ++i)
    {
        if (windmillFans[i].IsValid)
        {
            windmillFan = windmillFans[i];
            isWindmillFanReady = true;
            break;
        }
    }

    // 输出合法风车扇叶参数
    if (!isWindmillFanReady)
    {
        // 在图像上方输出提示信息
        cv::putText(*windmillObjectsImage,
                    "There is no valid windmill fan in current image",
                    cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
    }
    else
    {
        // 绘制合法风车扇叶的外部轮廓
        std::vector<std::vector<cv::Point>> externalContours;
        externalContours.emplace_back(windmillFan.Contour);
        cv::drawContours(*windmillObjectsImage,
                         externalContours,
                         -1,
                         cv::Scalar(0, 255, 0),
                         1);

        // 绘制合法风车扇叶的头部轮廓
        std::vector<std::vector<cv::Point>> headContours;
        headContours.emplace_back(windmillFan.targetContour);
        cv::drawContours(*windmillObjectsImage,
                         headContours,
                         -1,
                         cv::Scalar(0, 255, 0),
                         1);

        // 绘制合法风车扇叶头部轮廓的中心点
        cv::circle(*windmillObjectsImage,
                   windmillFan.targetCenter,
                   3,
                   cv::Scalar(0, 255, 0),
                   3);

        // 在图像上方输出合法风车扇叶参数
        int fan_x = static_cast<int>(windmillFan.targetCenter.x);
        int fan_y = static_cast<int>(windmillFan.targetCenter.y);
        std::string text = "Fan.x = " + std::to_string(fan_x) + ";  Fan.y = " + std::to_string(fan_y);
        cv::putText(*windmillObjectsImage,
                    text,
                    cv::Point(10, 40),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    1);
    }

    // 如果同时检测到合法的风车扇叶和风车Logo，则绘制夹角信息
    if (isWindmillLogoReady && isWindmillFanReady)
    {
        // 绘制x轴
        cv::Point2f beginPoint_x(0.0, windmillLogo.ContourCenter.y);
        cv::Point2f endPoint_x(static_cast<float>(rawImage.cols), windmillLogo.ContourCenter.y);
        cv::line(*windmillObjectsImage,
                 beginPoint_x,
                 endPoint_x,
                 cv::Scalar(0, 0, 255),
                 1,
                 8,
                 0);

        // 绘制y轴
        cv::Point2f beginPoint_y(windmillLogo.ContourCenter.x, 0.0);
        cv::Point2f endPoint_y(windmillLogo.ContourCenter.x, static_cast<float>(rawImage.rows));
        cv::line(*windmillObjectsImage,
                 beginPoint_y,
                 endPoint_y,
                 cv::Scalar(0, 0, 255),
                 1,
                 8,
                 0);

        // 绘制方向矢量
        cv::line(*windmillObjectsImage,
                 windmillLogo.ContourCenter,
                 windmillFan.targetCenter,
                 cv::Scalar(0, 0, 255),
                 1,
                 8,
                 0);

        // 计算并输出角度信息
        float angle;
        ComputeWindmillFanAngle(windmillLogo.ContourCenter,
                                windmillFan.targetCenter,
                                &angle);
        cv::putText(*windmillObjectsImage,
                    std::to_string(angle),
                    windmillLogo.ContourCenter,
                    cv::FONT_HERSHEY_SIMPLEX,
                    1,
                    cv::Scalar(0, 255, 0),
                    2);
    }
}

// 创建风车角度图像
void ClassicalWindmillRecognizer::CreateWindmillAnglesImage(const cv::Point2f &logoCenter,
                                                            const std::vector<cv::Point2f> &fanCenters,
                                                            const cv::Mat &rawImage,
                                                            cv::Mat *windmillAnglesImage)
{
    // 使用原始图像初始化风车角度图像
    *windmillAnglesImage = rawImage.clone();

    // 判断风车扇叶中心点集合是否为空
    if (fanCenters.empty())
    {
        // 在图像上方输出提示信息
        cv::putText(*windmillAnglesImage,
                    "fanCenters is empty",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 计算从风车Logo中心点到每个风车扇叶中心点的方向矢量与x轴正方向的夹角；取值范围为[0°, 360°)
    std::vector<float> angles;
    for (unsigned int i = 0; i < fanCenters.size(); ++i)
    {
        float angle;
        ComputeWindmillFanAngle(logoCenter, fanCenters[i], &angle);
        angles.emplace_back(angle);
    }

    // 绘制x轴
    cv::Point2f beginPoint_x(0.0, logoCenter.y);
    cv::Point2f endPoint_x(static_cast<float>(rawImage.cols), logoCenter.y);
    cv::line(*windmillAnglesImage,
             beginPoint_x,
             endPoint_x,
             cv::Scalar(0, 0, 255),
             1,
             8,
             0);

    // 绘制y轴
    cv::Point2f beginPoint_y(logoCenter.x, 0.0);
    cv::Point2f endPoint_y(logoCenter.x, static_cast<float>(rawImage.rows));
    cv::line(*windmillAnglesImage,
             beginPoint_y,
             endPoint_y,
             cv::Scalar(0, 0, 255),
             1,
             8,
             0);

    // 绘制坐标原点
    cv::circle(*windmillAnglesImage,
               logoCenter,
               2,
               cv::Scalar(0, 0, 255),
               2);

    // 绘制风车扇叶中心点和方向矢量，输出方向矢量的相关参数
    for (int i = 0; i < fanCenters.size(); ++i)
    {
        // 绘制风车扇叶中心点
        cv::circle(*windmillAnglesImage,
                   fanCenters[i],
                   2,
                   cv::Scalar(0, 0, 255),
                   2);

        // 在风车扇叶中心点右侧标注风车扇叶索引
        cv::putText(*windmillAnglesImage,
                    std::to_string(i + 1),
                    fanCenters[i],
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    1);

        // 绘制方向矢量
        cv::line(*windmillAnglesImage,
                 logoCenter,
                 fanCenters[i],
                 cv::Scalar(0, 0, 255),
                 1,
                 8,
                 0);

        // 在图像上方输出方向矢量的参数
        std::string text = std::to_string(i + 1) +\
                               ":  Logo[" + std::to_string(logoCenter.x) + ", " + std::to_string(logoCenter.y) + "]"\
                               ";  Fan[" + std::to_string(fanCenters[i].x) + ", " + std::to_string(fanCenters[i].y) + "]"\
                               ";  Angle= " + std::to_string(angles[i]);
        cv::putText(*windmillAnglesImage,
                    text,
                    cv::Point(10, 20 * i + 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    1);
    }
}

// 创建锁定扇叶图像
void ClassicalWindmillRecognizer::CreateLockedFanImage(const ClassicalWindmillFan &validFan,
                                                       const std::vector<cv::Point2f> &fanCenters,
                                                       const std::pair<float, float> &distanceCompensation,
                                                       const std::pair<float, float> &velocityCompensation,
                                                       const cv::Mat &rawImage,
                                                       cv::Mat *lockedFanImage)
{
    // 使用原始图像初始化风车锁定扇叶图像
    *lockedFanImage = rawImage.clone();

    // 判断风车扇叶是否合法
    if (!validFan.IsValid)
    {
        cv::putText(*lockedFanImage,
                    "There is no valid windmill fan",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 输出风车扇叶的头部位置中心点坐标
    std::string text1 = "Fan's center: [" +
                        std::to_string(validFan.targetCenter.x) + ", " +
                        std::to_string(validFan.targetCenter.y) + "]";
    cv::putText(*lockedFanImage,
                text1,
                cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

    // 输出击打目标点的距离补偿信息
    std::string text2 = "Distance compensation: [" +
                        std::to_string(distanceCompensation.first) + ", " +
                        std::to_string(distanceCompensation.second) + "]";
    cv::putText(*lockedFanImage,
                text2,
                cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

    // 输出击打目标点的速度补偿信息
    std::string text3 = "Velocity compensation: [" +
                        std::to_string(velocityCompensation.first) + ", " +
                        std::to_string(velocityCompensation.second) + "]";
    cv::putText(*lockedFanImage,
                text3,
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

    // 绘制风车扇叶历史头部中心点
    for (unsigned int i = 0; i < fanCenters.size(); ++i)
    {
        cv::circle(*lockedFanImage,
                   fanCenters[i],
                   2,
                   cv::Scalar(0, 255, 255),
                   2);
    }

    // 绘制风车扇叶头部轮廓
    if (!validFan.targetContour.empty())
    {
        std::vector<std::vector<cv::Point>> contours;
        contours.emplace_back(validFan.targetContour);
        cv::drawContours(*lockedFanImage,
                         contours,
                         -1,
                         cv::Scalar(0, 255, 0),
                         1);
    }

    // 绘制风车扇叶头部轮廓中心点
    cv::circle(*lockedFanImage,
               validFan.targetCenter,
               3,
               cv::Scalar(0, 255, 0),
               3);

    // 绘制风车扇叶补偿之后的击打目标点
    cv::Point2f compensatedTarget;
    compensatedTarget.x = validFan.targetCenter.x + distanceCompensation.first + velocityCompensation.first;
    compensatedTarget.y = validFan.targetCenter.y + distanceCompensation.second + velocityCompensation.second;
    cv::circle(*lockedFanImage,
               compensatedTarget,
               3,
               cv::Scalar(0, 0, 255),
               3);
}

// 创建比较扇叶图像
void ClassicalWindmillRecognizer::CreateComparedFanImage(const ClassicalWindmillFan &validFan,
                                                         const std::vector<std::pair<cv::Point2f, uint64_t>> &oldTargets,
                                                         const cv::Mat &rawImage,
                                                         cv::Mat *comparedFanImage)
{
    // 使用原始图像初始化比较扇叶图像
    *comparedFanImage = rawImage.clone();

    // 判断风车扇叶是否合法
    if (!validFan.IsValid)
    {
        cv::putText(*comparedFanImage,
                    "There is no valid windmill fan",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 输出历史击打目标点和当前扇叶的标识颜色
    cv::putText(*comparedFanImage,
                "Current fan's color: Green;  Old target's color: Yellow",
                cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

    // 输出风车扇叶的头部位置坐标
    std::string fanCenterText = "Fan center: [" +
                                std::to_string(validFan.targetCenter.x) + ", " +
                                std::to_string(validFan.targetCenter.y) + "]";
    cv::putText(*comparedFanImage,
                fanCenterText,
                cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);

    // 输出历史击打目标点的坐标和时间戳偏移
    if (oldTargets.empty())
    {
        cv::putText(*comparedFanImage,
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
            uint64_t timestampOffset = (validFan.Timestamp - oldTargets[i].second) / 1000000;

            // 输出历史击打目标点信息
            std::string oldTargetText = "Old target: [" +
                                        std::to_string(oldTargets[i].first.x) + ", " +
                                        std::to_string(oldTargets[i].first.y) + "];  Timestamp offset: " +
                                        std::to_string(timestampOffset) + " ms";
            cv::putText(*comparedFanImage,
                        oldTargetText,
                        cv::Point(10, 60 + 20 * i),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(0, 255, 255),
                        1);

            // 绘制历史击打目标点
            cv::circle(*comparedFanImage,
                       oldTargets[i].first,
                       3,
                       cv::Scalar(0, 255, 255),
                       3);
        }
    }

    // 绘制扇叶的头部轮廓
    if (!validFan.targetContour.empty())
    {
        std::vector<std::vector<cv::Point>> headContours;
        headContours.emplace_back(validFan.targetContour);
        cv::drawContours(*comparedFanImage,
                         headContours,
                         -1,
                         cv::Scalar(0, 255, 0),
                         1);
    }

    // 绘制扇叶头部轮廓的中心点
    cv::circle(*comparedFanImage,
               validFan.targetCenter,
               3,
               cv::Scalar(0, 255, 0),
               3);
}

// 创建追踪Logo图像
void ClassicalWindmillRecognizer::CreateTrackedLogoImage(const ClassicalWindmillLogo &trackedLogo,
                                                         const std::vector<cv::Point2f> &logoLocations,
                                                         const cv::Mat &rawImage,
                                                         cv::Mat *trackedLogoImage)
{
    // 使用原始图像初始化追踪Logo图像
    *trackedLogoImage = rawImage.clone();

    // 判断追踪Logo是否合法
    if (!trackedLogo.IsValid)
    {
        // 在图像上方输出提示信息
        cv::putText(*trackedLogoImage,
                    "There is no valid tracked logo in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 绘制追踪Logo的历史位置
    for (unsigned int i = 0; i < logoLocations.size(); ++i)
    {
        cv::circle(*trackedLogoImage,
                   logoLocations[i],
                   2,
                   cv::Scalar(0, 255, 255),
                   2);
    }

    // 绘制追踪Logo的轮廓
    std::vector<std::vector<cv::Point>> contours;
    contours.emplace_back(trackedLogo.Contour);
    cv::drawContours(*trackedLogoImage,
                     contours,
                     -1,
                     cv::Scalar(0, 255, 0),
                     1);

    // 绘制追踪Logo的中心点
    cv::circle(*trackedLogoImage,
               trackedLogo.ContourCenter,
               3,
               cv::Scalar(0, 255, 0),
               3);

    // 计算相邻两块Logo中心点之间的最大位置偏差
    float maxLocationOffset = 0.0;
    if (logoLocations.size() > 1)
    {
        for (unsigned int i = 0; i < logoLocations.size() - 1; ++i)
        {
            float offset_x = logoLocations[i].x - logoLocations[i + 1].x;
            float offset_y = logoLocations[i].y - logoLocations[i + 1].y;
            float offset = std::sqrt(offset_x * offset_y + offset_y * offset_y);
            if (offset > maxLocationOffset)
            {
                maxLocationOffset = offset;
            }
        }
    }

    // 在图像上方输出历史位置个数和相邻Logo最大距离
    std::string text = "Location number: " + std::to_string(logoLocations.size()) +
                       ";  Max location offset: " + std::to_string(maxLocationOffset);
    cv::putText(*trackedLogoImage,
                text,
                cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);
}

// 创建追踪扇叶图像
void ClassicalWindmillRecognizer::CreateTrackedFanImage(const ClassicalWindmillFan &trackedFan,
                                                        const std::vector<cv::Point2f> &fanLocations,
                                                        const cv::Mat &rawImage,
                                                        cv::Mat *trackedFanImage)
{
    // 使用原始图像初始化追踪扇叶图像
    *trackedFanImage = rawImage.clone();

    // 判断追踪扇叶是否合法
    if (!trackedFan.IsValid)
    {
        // 在图像上方输出提示信息
        cv::putText(*trackedFanImage,
                    "There is no valid tracked fan in current image",
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
        return;
    }

    // 绘制追踪扇叶的历史位置
    for (unsigned int i = 0; i < fanLocations.size(); ++i)
    {
        cv::circle(*trackedFanImage,
                   fanLocations[i],
                   2,
                   cv::Scalar(0, 255, 255),
                   2);
    }

    // 绘制追踪扇叶的轮廓
    std::vector<std::vector<cv::Point>> contours;
    contours.emplace_back(trackedFan.targetContour);
    cv::drawContours(*trackedFanImage,
                     contours,
                     -1,
                     cv::Scalar(0, 255, 0),
                     1);

    // 绘制追踪扇叶的中心点
    cv::circle(*trackedFanImage,
               trackedFan.targetCenter,
               3,
               cv::Scalar(0, 255, 0),
               3);

    // 计算相邻两块扇叶中心点之间的最大位置偏差
    float maxLocationOffset = 0.0;
    if (fanLocations.size() > 1)
    {
        for (unsigned int i = 0; i < fanLocations.size() - 1; ++i)
        {
            float offset_x = fanLocations[i].x - fanLocations[i + 1].x;
            float offset_y = fanLocations[i].y - fanLocations[i + 1].y;
            float offset = std::sqrt(offset_x * offset_y + offset_y * offset_y);
            if (offset > maxLocationOffset)
            {
                maxLocationOffset = offset;
            }
        }
    }

    // 在图像上方输出历史位置个数和相邻扇叶最大距离
    std::string text = "Location number: " + std::to_string(fanLocations.size()) +
                       ";  Max location offset: " + std::to_string(maxLocationOffset);
    cv::putText(*trackedFanImage,
                text,
                cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);
}

// ******************************  ClassicalWindmillRecognizer类的私有函数  ******************************

// 计算风车Logo图像的Hog特征
bool ClassicalWindmillRecognizer::ComputeLogoHogFeatures(const cv::Mat &logoImage,
                                                         std::vector<float> *logoHogFeatures) const
{
    // 判断风车识别器是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    // 判断风车Logo图像的尺寸是否合法
    if ((logoImage.empty()) ||
        (logoImage.cols != param_.LogoHogWindowWidth) ||
        (logoImage.rows != param_.LogoHogWindowHeight))
    {
        return false;
    }

    // 计算Hog特征描述子
    logoHogDescriptor_.compute(logoImage, *logoHogFeatures);

    // 返回计算结果
    return true;
}

// 使用LogoHogSvm模型检测风车Logo的合法性
void ClassicalWindmillRecognizer::CheckWindmillLogosByHogSvm(std::vector<ClassicalWindmillLogo> *windmillLogos) const
{
    // 判断风车识别器是否初始化，判断LogoHogSvm模型是否为空
    if ((!isInitialized_) || logoHogSvm_.empty())
    {
        return;
    }

    // 使用LogoHogSvm模型判断风车Logo是否合法
    for (unsigned int i = 0; i < windmillLogos->size(); ++i)
    {
        // 计算风车Logo图像的Hog特征
        std::vector<float> logoHogFeatures;
        if (!ComputeLogoHogFeatures((*windmillLogos)[i].Image, &logoHogFeatures))
        {
            continue;
        }

        // 判断风车Logo图像的Hog特征是否满足检测条件
        if (logoHogSvm_->getVarCount() != logoHogFeatures.size())
        {
            continue;
        }

        // 更新风车Logo的合法性
        int predictResult = static_cast<int>(logoHogSvm_->predict(logoHogFeatures));
        (*windmillLogos)[i].IsValid = (predictResult > 0);
    }
}


void ClassicalWindmillRecognizer::DrawRect(cv::Mat &img, const cv::Rect &rect, const cv::Scalar &color, int thickness){

    cv::Point2f Vertex[4] = {

            {float (rect.x),float (rect.y)},
            {float (rect.x + rect.width),float (rect.y)},
            {float (rect.x + rect.width),float (rect.y + rect.height)},
            {float (rect.x),float (rect.y + rect.height)}
    };

    for(int i = 0 ; i < 4 ; i++)
    {
        cv::line(img , Vertex[i] , Vertex[(i + 1) % 4] , color , thickness);
    }
}

void ClassicalWindmillRecognizer::drawRect(cv::Mat &img, const cv::Rect &rect, cv::Point2f a, cv::Point2f b,const cv::Scalar &color, int thickness){

    int Distancex = sqrt( pow( a.x - b.x,2) );
    int Distancey = sqrt( pow( a.y - b.y,2) );

    Distancex *= a.x > b.x? -1 :1;
    Distancey *= a.y > b.y? -1 :1;

    cv::Point2i temprect = {rect.x,rect.y} ;
    temprect.x += Distancex;
    temprect.y += Distancey;

    cv::Point2f Vertex[4] = {

            {float (temprect.x),float (temprect.y)},
            {float (temprect.x + rect.width),float (temprect.y)},
            {float (temprect.x + rect.width),float (temprect.y + rect.height)},
            {float (temprect.x),float (temprect.y + rect.height)}
    };

    for(int i = 0 ; i < 4 ; i++)
    {
        cv::line(img , Vertex[i] , Vertex[(i + 1) % 4] , color , thickness);
    }
}


void ClassicalWindmillRecognizer::drawRotatedRect(cv::Mat &img,
                                                  const cv::RotatedRect &rect,
                                                  const cv::Scalar &color,
                                                  int thickness)
{
    cv::Point2f Vertex[4];
    rect.points(Vertex);
    for(int i = 0 ; i < 4 ; i++)
    {
        cv::line(img , Vertex[i] , Vertex[(i + 1) % 4] , color , thickness);
    }
}

double ClassicalWindmillRecognizer::calDistance(cv::Point2f &Point_1, cv::Point2f &Point_2)
{
    return sqrt((Point_1.x - Point_2.x) * (Point_1.x - Point_2.x) + (Point_1.y - Point_2.y) * (Point_1.y - Point_2.y));
}

//bool ClassicalWindmillRecognizer::FixedCorner(cv::Point2f &Point_1,
//                                              cv::Point2f &Point_2,
//                                              cv::Point2f &LogoCenter)
//{
//    double Angle_1 = atan2(Point_1.y - LogoCenter.y, Point_1.x - LogoCenter.x);
//    double Angle_2 = atan2(Point_2.y - LogoCenter.y, Point_2.x - LogoCenter.x);
//
//    double deltaAngle = Angle_1 - Angle_2;
//    if(Angle_1 > CV_PI / 2 && Angle_2 < -CV_PI / 2)
//    {
//        deltaAngle -= 2 * CV_PI;
//    }
//    if(Angle_1 < - CV_PI / 2 && Angle_2 > CV_PI / 2)
//    {
//        deltaAngle += 2 * CV_PI;
//    }
//
//    if(deltaAngle > 0)
//    {
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}

//bool ClassicalWindmillRecognizer::calFanHeadCenter(const cv::Mat &srcImage,
//                                                   cv::RotatedRect &minRectFan,
//                                                   cv::RotatedRect &minRectHead,
//                                                   cv::Point2f &LogoCenter,
//                                                   cv::Point2f *fanHeadCenter,
//                                                   std::array<cv::Point2f, 4> *PoundingPoints)
//{
//    cv::Point2f RectVertices[8];
//    minRectFan.points(RectVertices);
//    minRectHead.points(RectVertices + 4);
//    int point[8];
//    float dis[8];
//
//    // 索引初始化
//    for (unsigned int i = 0; i < 8; i++)
//    {
//        point[i] = i;
//        dis[i] = calDistance(RectVertices[i], LogoCenter);
//    }
//
//    // 对各个角点进行索引排序;从大到小
//    for (unsigned int i = 0; i < 7; i++)
//    {
//        for (unsigned int j = 0; j < 7 - i; j++)
//        {
//            if (dis[point[j]] < dis[point[j + 1]])
//            {
//                int temp;
//                temp = point[j];
//                point[j] = point[j + 1];
//                point[j + 1] = temp;
//            }
//        }
//    }
//
//    if(FixedCorner(RectVertices[point[4]], RectVertices[point[5]], LogoCenter))
//    {
//        PoundingPoints->at(1) = RectVertices[point[5]];
//        PoundingPoints->at(0) = RectVertices[point[4]];
//    }
//    else
//    {
//        PoundingPoints->at(0) = RectVertices[point[5]];
//        PoundingPoints->at(1) = RectVertices[point[4]];
//    }
//
//    if(FixedCorner(RectVertices[point[3]], RectVertices[point[2]], LogoCenter))
//    {
//        PoundingPoints->at(2) = RectVertices[point[2]];
//        PoundingPoints->at(3) = RectVertices[point[3]];
//    }
//    else
//    {
//        PoundingPoints->at(2) = RectVertices[point[3]];
//        PoundingPoints->at(3) = RectVertices[point[2]];
//    }
//
//    cv::putText(srcImage,
//                "0",
//                PoundingPoints->at(0),
//                cv::FONT_HERSHEY_SIMPLEX,
//                1.0,
//                cv::Scalar(255, 255, 255),
//                2);
//    cv::putText(srcImage,
//                "1",
//                PoundingPoints->at(1),
//                cv::FONT_HERSHEY_SIMPLEX,
//                1.0,
//                cv::Scalar(255, 255, 255),
//                2);
//    cv::putText(srcImage,
//                "2",
//                PoundingPoints->at(2),
//                cv::FONT_HERSHEY_SIMPLEX,
//                1.0,
//                cv::Scalar(255, 255, 255),
//                2);
//    cv::putText(srcImage,
//                "3",
//                PoundingPoints->at(3),
//                cv::FONT_HERSHEY_SIMPLEX,
//                1.0,
//                cv::Scalar(255, 255, 255),
//                2);
//
//    cv::line(srcImage, RectVertices[point[2]], LogoCenter, cv::Scalar(255,255,255));
//    cv::line(srcImage, RectVertices[point[3]], LogoCenter, cv::Scalar(255,255,255));
//    cv::line(srcImage, RectVertices[point[4]], LogoCenter, cv::Scalar(255,255,255));
//    cv::line(srcImage, RectVertices[point[5]], LogoCenter, cv::Scalar(255,255,255));
//    fanHeadCenter->x = (RectVertices[point[2]].x + RectVertices[point[3]].x + RectVertices[point[4]].x + RectVertices[point[5]].x) / 4;
//    fanHeadCenter->y = (RectVertices[point[2]].y + RectVertices[point[3]].y + RectVertices[point[4]].y + RectVertices[point[5]].y) / 4;
//}

void ClassicalWindmillRecognizer::CheckWindmillTargetsByAngle(cv::Point2f &LogoCenter, ClassicalWindmillFan *windmillFans, std::vector<ClassicalWindmillFan> *targetContours) const
{
    float AngleFan;
    float AngleTarget;
    float AngleOffset;
    ComputeWindmillFanAngle(LogoCenter, windmillFans->ContourCenter, &AngleFan);

    for(unsigned int i = 0; i < (*targetContours).size(); i++)
    {
        ComputeWindmillFanAngle(LogoCenter, (*targetContours)[i].targetCenter, &AngleTarget);
        AngleOffset = std::abs(AngleTarget - AngleFan);



//        std::cout << " AngleOffset " << AngleOffset <<std::endl;
        if(AngleOffset > param_.MaxFanTargetAngle || AngleOffset < param_.MinFanTargetAngle)
        {
            continue;
        }

        if (calDistance(LogoCenter,(*targetContours)[i].targetCenter) < 150 ) continue;

        (*targetContours)[i].IsValid = true;

        std::vector<cv::Point2f> Point;
        cv::Point2f LeftUpper((*targetContours)[i].targetCenter.x - (*targetContours)[i].targetRect.width / 2.0, (*targetContours)[i].targetCenter.y - (*targetContours)[i].targetRect.height / 2.0);
        cv::Point2f LeftLower((*targetContours)[i].targetCenter.x - (*targetContours)[i].targetRect.width / 2.0, (*targetContours)[i].targetCenter.y + (*targetContours)[i].targetRect.height / 2.0);
        cv::Point2f RightUpper((*targetContours)[i].targetCenter.x + (*targetContours)[i].targetRect.width / 2.0, (*targetContours)[i].targetCenter.y - (*targetContours)[i].targetRect.height / 2.0);
        cv::Point2f RightLower((*targetContours)[i].targetCenter.x + (*targetContours)[i].targetRect.width / 2.0, (*targetContours)[i].targetCenter.y + (*targetContours)[i].targetRect.height / 2.0);
        Point.emplace_back(LeftUpper);
        Point.emplace_back(RightUpper);
        Point.emplace_back(RightLower);
        Point.emplace_back(LeftLower);
        (*targetContours)[i].TargetBoundingPoints = Point;
    }

}


