//
// Created by plutoli on 2022/1/23.
//

#ifndef CUBOT_BRAIN_CLASSICAL_WINDMILL_RECOGNIZER_H
#define CUBOT_BRAIN_CLASSICAL_WINDMILL_RECOGNIZER_H

#include <cmath>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "delta_cv.h"
#include "classical_windmill_logo.h"
#include "classical_windmill_fan.h"
#include "classical_windmill_recognizer_param.h"

/**
 * @brief 风车识别器
 * @note 风车识别器使用步骤：\n
 *       Step1：创建风车识别器\n
 *       Step2：自行构造ClassicalWindmillRecognizerParam或从yaml文件中读取ClassicalWindmillRecognizerParam\n
 *       Step3：调用SetParam()函数设置识别器参数\n
 *       Step4：调用Init()函数初始化识别器\n
 *       Step5：使用识别器对原始图像进行预处理得到二值图\n
 *       Step6：使用识别器在二值图中检测风车Logo集合\n
 *       Step7：使用识别器在二值图中检测风车扇叶集合\n
 *       Step8：调用Release()函数释放识别器的系统资源\n
 */
class ClassicalWindmillRecognizer
{
public:
    /**
     * @brief 构造函数
     */
    ClassicalWindmillRecognizer();

    /**
     * @brief 析构函数
     */
    ~ClassicalWindmillRecognizer();

    /**
     * @brief 获取风车识别器参数
     * @return 风车识别器参数
     */
    ClassicalWindmillRecognizerParam GetParam();

    /**
     * @brief 设置风车识别器参数
     * @param[in] param 风车识别器参数
     * @return 风车识别器参数设置结果\n
     *         -<em>false</em> 设置失败\n
     *         -<em>true</em> 设置成功\n
     * @note 如果风车识别器已经初始化完毕，参数设置将会失败
     */
    bool SetParam(const ClassicalWindmillRecognizerParam &param);

    /**
     * @brief 获取风车识别器的初始化状态
     * @return 风车识别器的初始化状态\n
     *         -<em>false</em> 尚未初始化\n
     *         -<em>true</em> 已经初始化\n
     */
    bool IsInitialized();

    /**
     * @brief 获取风车识别器的初始化时间戳
     * @return 风车识别器的初始化时间戳
     */
    uint64_t GetInitTimestamp();

    /**
     * @brief 初始化风车识别器
     * @return 初始化结果\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     */
    bool Init();

    /**
     * @brief 释放风车识别器资源
     * @return 资源释放结果\n
     *         -<em>false</em> 资源释放失败\n
     *         -<em>true</em> 资源释放成功\n
     */
    bool Release();

    /**
     * @brief 对原始图像进行预处理
     * @param[in] rawImage      原始图像
     * @param[out] binaryImage  二值图像
     * @return 预处理结果\n
     *         -<em>false</em> 预处理失败\n
     *         -<em>true</em> 预处理成功\n
     * @note 如果风车识别器没有初始化，将返回false
     */
        bool Preprocess(const cv::Mat &rawImage, cv::Mat *binaryImageFan, cv::Mat *binaryImageTarget) const;

    /**
     * @brief 检测风车Logo集合
     * @param[in] binaryImage       二值图像
     * @param[in] rawImage          原始图像
     * @param[out] windmillLogos    风车Logo集合
     * @return 检测结果\n
     *         -<em>false</em> 检测失败\n
     *         -<em>true</em> 检测成功\n
     * @note 如果风车识别器没有初始化，将返回false
     */
    bool DetectWindmillLogos(const cv::Mat &binaryImage,
                             const cv::Mat &rawImage,
                             std::vector<ClassicalWindmillLogo> *windmillLogos) const;

    /**
     * @brief 检测风车扇叶集合；思路如下：\n
     *        Step1：调用cv::findContours()函数，使用cv::RETR_TREE模式搜索二值图像中的所有轮廓信息\n
     *        Step2：根据轮廓面积的上下限筛选一级轮廓信息，得到的轮廓即为扇叶轮廓\n
     *        Step3：调用cv::matchshapes()函数计算扇叶轮廓和未激活扇叶轮廓模板的差异率，差异率最低的即为未激活的风车扇叶\n
     *        Step4：搜索未激活风车扇叶轮廓的所有子轮廓，面积最大的子轮廓即为未激活风车扇叶的头部位置\n
     *        Step5：计算未激活风车扇叶头部位置轮廓的中心点，即为击打目标点\n
     *        cv::findContours()函数使用说明详见：https://www.cnblogs.com/wojianxin/p/12602490.html \n
     *        cv::matchshapes()函数使用说明详见：https://blog.csdn.net/the_future_way/article/details/117000423 \n
     * @param[in] binaryImage      二值图像
     * @param[out] windmillFans    风车扇叶集合
     * @return 检测结果\n
     *         -<em>false</em> 检测失败\n
     *         -<em>true</em> 检测成功\n
     * @note 如果风车识别器没有初始化，将返回false
     */
    bool DetectWindmillFans(const cv::Mat &binaryImageFan,
                            const cv::Mat &binaryImageTarget,
                            const cv::Mat &rawImage,
                            ClassicalWindmillFan *windmillFan,
                            std::vector<ClassicalWindmillFan> *targetContours) const;

    /**
     * @brief 计算两点之间的距离
     * @param Point_1 第一个点
     * @param Point_2 第二个点
     * @return 计算结果
     */
    static double calDistance(cv::Point2f &Point_1, cv::Point2f &Point_2);

    /**
     * @brief 得到待击打点坐标
     */
//    static bool calFanHeadCenter(const cv::Mat &srcImage,
//                                 cv::RotatedRect &minRectFan,
//                                 cv::RotatedRect &minRectHead,
//                                 cv::Point2f &LogoCenter,
//                                 cv::Point2f *fanHeadCenter,
//                                 std::array<cv::Point2f, 4> *PoundingPoints);


    /**
     * @brief 固定角点
     */
//    static bool FixedCorner(cv::Point2f &Point_1, cv::Point2f &Point_2, cv::Point2f &LogoCenter);

    /**
     * @brief 更新风车识别器Logo缓冲区
     * @param[in] validLogo  合法Logo
     * @return 更新结果\n
     *         -<em>false</em> 更新失败\n
     *         -<em>true</em> 更新成功\n
     * @note 如果风车识别器没有初始化或Logo不合法，将返回false
     */
    bool UpdateLogoBuffer(const ClassicalWindmillLogo &validLogo);

    /**
     * @brief 清空风车识别器Logo缓冲区
     */
    void ClearLogoBuffer();

    /**
     * @brief 获取风车Logo位置时序
     * @param[out] logoLocationSequence 风车Logo位置时序
     * @return 获取结果\n
     *         -<em>false</em> 获取失败\n
     *         -<em>true</em> 获取成功\n
     * @note 如果风车识别器没有初始化，将返回false
     */
    bool GetLogoLocationSequence(std::vector<std::pair<cv::Point2f, uint64_t>> *logoLocationSequence);

    /**
     * @brief 更新风车识别器扇叶缓冲区
     * @param[in] validFan  合法扇叶
     * @return 更新结果\n
     *         -<em>false</em> 更新失败\n
     *         -<em>true</em> 更新成功\n
     * @note 如果风车识别器没有初始化或扇叶不合法，将返回false
     */
    bool UpdateFanBuffer(const ClassicalWindmillFan &validFan);

    /**
     * @brief 清空风车识别器扇叶缓冲区
     */
    void ClearFanBuffer();

    /**
     * @brief 获取风车扇叶位置时序
     * @param[out] fanLocationSequence 风车扇叶位置时序
     * @return 获取结果\n
     *         -<em>false</em> 获取失败\n
     *         -<em>true</em> 获取成功\n
     * @note 如果风车识别器没有初始化，将返回false
     */
    bool GetFanLocationSequence(std::vector<std::pair<cv::Point2f, uint64_t>> *fanLocationSequence);

    /**
     * @brief 更新风车识别器目标点数据缓冲区
     * @param[in] compensatedTarget 补偿之后的击打目标点
     * @param[in] timestamp         时间戳
     * @return 更新结果\n
     *         -<em>false</em> 更新失败\n
     *         -<em>true</em> 更新成功\n
     * @note 如果风车识别器没有初始化，将返回false
     */
    bool UpdateTargetBuffer(const cv::Point2f &compensatedTarget, const uint64_t &timestamp);

    /**
     * @brief 清空风车识别器目标点数据缓冲区
     */
    void ClearTargetBuffer();

    /**
     * @brief 获取符合时间戳要求的历史击打目标点集合
     * @param[in] timestamp         历史击打目标点时间戳；单位：纳秒
     * @param[out] timestampOffset  历史击打目标点时间戳偏移；单位：纳秒
     * @param[out] targets          历史击打目标点集合
     * @return 获取结果\n
     *         -<em>false</em> 获取失败\n
     *         -<em>true</em> 获取成功\n
     */
    bool GetTarget(const uint64_t &timestamp,
                   const uint64_t &timestampOffset,
                   std::vector<std::pair<cv::Point2f, uint64_t>> *targets);

    /**
     * @brief 训练并保存LogoHogSvm模型
     * @param[in] positivePath      LogoHogSvm模型的正样本存储路径
     * @param[in] negativePath      LogoHogSvm模型的负样本存储路径
     * @param[in] termCriteria      LogoHogSvm模型训练的终止条件
     * @param[in] modelFileName     LogoHogSvm模型文件名称(包括存储路径和文件名)
     * @return 模型训练结果\n
     *         -<em>false</em> 模型训练失败\n
     *         -<em>true</em> 模型训练成功\n
     */
    [[nodiscard]] bool TrainLogoHogSvm(const std::string &positivePath,
                                       const std::string &negativePath,
                                       const cv::TermCriteria &termCriteria,
                                       const std::string &modelFileName) const;

    /**
     * @brief 计算从风车Logo中心点到风车扇叶中心点的方向矢量角度；角度范围为[0°, 360°)
     * @param[in] logoCenter    风车Logo中心点的坐标
     * @param[in] fanCenter     风车扇叶中心点的坐标
     * @param[out] angle        方向矢量的角度
     * @note Step1：以风车Logo中心点为坐标原点，水平向右为x轴正方向，垂直向上为y轴正方向，创建新的坐标系 \n
     *       Step2：在新的坐标系中，计算风车Logo中心点到风车扇叶中心点的方向矢量 \n
     *       Step3：方向矢量与x轴的夹角沿着x轴正方向逆时针旋从0°转逐渐增大，取值范围为[0°, 360°)
     */
    static void ComputeWindmillFanAngle(const cv::Point2f &logoCenter,
                                        const cv::Point2f &fanCenter,
                                        float *angle);

    /**
     * @brief 创建风车轮廓图像
     * @param[in] binaryImage               二值图像
     * @param[in] param                     风车识别器参数
     * @param[in] rawImage                  原始图像
     * @param[out] windmillContoursImage    风车轮廓图像
     */
    static void CreateWindmillContoursImage(const cv::Mat &binaryImage,
                                            const ClassicalWindmillRecognizerParam &param,
                                            const cv::Mat &rawImage,
                                            cv::Mat *windmillContoursImage);

    /**
     * @brief 创建风车Logo图像
     * @param[in] windmillLogos         风车Logo集合
     * @param[in] rawImage              原始图像
     * @param[out] windmillLogosImage   风车Logo图像
     */
    static void CreateWindmillLogosImage(const std::vector<ClassicalWindmillLogo> &windmillLogos,
                                         const cv::Mat &rawImage,
                                         cv::Mat *windmillLogosImage);

    /**
     * @brief 创建风车扇叶图像
     * @param[in] windmillFans          风车扇叶集合
     * @param[in] rawImage              原始图像
     * @param[out] windmillFansImage    风车扇叶图像
     */
    static void CreateWindmillFansImage(const std::vector<ClassicalWindmillFan> &windmillFans,
                                        const cv::Mat &rawImage,
                                        cv::Mat *windmillFansImage);

    /**
     * @brief 创建风车目标图像
     * @param[in] windmillLogos         风车Logo集合
     * @param[in] windmillFans          风车扇叶集合
     * @param[in] rawImage              原始图像
     * @param[out] windmillObjectsImage 风车目标图像
     */
    static void CreateWindmillObjectsImage(const std::vector<ClassicalWindmillLogo> &windmillLogos,
                                           const std::vector<ClassicalWindmillFan> &windmillFans,
                                           const cv::Mat &rawImage,
                                           cv::Mat *windmillObjectsImage);

    /**
     * @brief 创建风车角度图像
     * @param[in] logoCenter            Logo中心点坐标
     * @param[in] fanCenters            扇叶中心点坐标集合
     * @param[in] rawImage              原始图像
     * @param[out] windmillAnglesImage  角度图像
     */
    static void CreateWindmillAnglesImage(const cv::Point2f &logoCenter,
                                          const std::vector<cv::Point2f> &fanCenters,
                                          const cv::Mat &rawImage,
                                          cv::Mat *windmillAnglesImage);

    /**
     * @brief 创建锁定扇叶图像
     * @param[in] validFan              合法风车扇叶
     * @param[in] fanCenters            风车扇叶历史头部中心点集合
     * @param[in] distanceCompensation  风车扇叶当前头部位置的距离补偿
     * @param[in] velocityCompensation  风车扇叶当前头部位置的速度补偿
     * @param[in] rawImage              原始图像
     * @param[out] lockedFanImage       锁定扇叶图像
     */
    static void CreateLockedFanImage(const ClassicalWindmillFan &validFan,
                                     const std::vector<cv::Point2f> &fanCenters,
                                     const std::pair<float, float> &distanceCompensation,
                                     const std::pair<float, float> &velocityCompensation,
                                     const cv::Mat &rawImage,
                                     cv::Mat *lockedFanImage);

    /**
     * @brief 创建比较扇叶图像
     * @param[in] validFan              当前合法风车扇叶
     * @param[in] oldTargets            历史击打目标点集合
     * @param[in] rawImage              原始图像
     * @param[out] comparedFanImage     比较扇叶图像
     */
    static void CreateComparedFanImage(const ClassicalWindmillFan &validFan,
                                       const std::vector<std::pair<cv::Point2f, uint64_t>> &oldTargets,
                                       const cv::Mat &rawImage,
                                       cv::Mat *comparedFanImage);

    /**
     * @brief 创建追踪Logo图像
     * @param[in] trackedLogo       追踪Logo
     * @param[in] logoLocations     Logo的历史位置信息
     * @param[in] rawImage          原始图像
     * @param[out] trackedLogoImage 追踪Logo图像
     */
    static void CreateTrackedLogoImage(const ClassicalWindmillLogo &trackedLogo,
                                      const std::vector<cv::Point2f> &logoLocations,
                                      const cv::Mat &rawImage,
                                      cv::Mat *trackedLogoImage);

    /**
     * @brief 创建追踪扇叶图像
     * @param[in] trackedFan       追踪扇叶
     * @param[in] fanLocations     扇叶的历史位置信息
     * @param[in] rawImage         原始图像
     * @param[out] trackedFanImage 追踪扇叶图像
     */
    static void CreateTrackedFanImage(const ClassicalWindmillFan &trackedFan,
                                      const std::vector<cv::Point2f> &fanLocations,
                                      const cv::Mat &rawImage,
                                      cv::Mat *trackedFanImage);

    static void drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness);

    static void drawRect(cv::Mat &img, const cv::Rect &rect,cv::Point2f a, cv::Point2f b, const cv::Scalar &color, int thickness);


    static void DrawRect(cv::Mat &img, const cv::Rect &rect,const cv::Scalar &color, int thickness);

    /**
     * @brief 通过扇叶与目标的角度差判断目标是否合法
     * @param LogoCenter
     * @param windmillFans
     * @param targetContours
     */
    void CheckWindmillTargetsByAngle(cv::Point2f &LogoCenter, ClassicalWindmillFan *windmillFans, std::vector<ClassicalWindmillFan> *targetContours) const;

private:
    ClassicalWindmillRecognizerParam param_;                          ///< 风车识别器参数
    std::atomic<bool> isInitialized_;                                 ///< 风车识别器的初始化状态
    std::atomic<uint64_t> initTimestamp_;                             ///< 风车识别器的初始化时间戳
    cv::HOGDescriptor logoHogDescriptor_;                             ///< 风车识别器的Logo的Hog特征描述子
    cv::Ptr<cv::ml::SVM> logoHogSvm_;                                 ///< 风车识别器的LogoHogSvm分类模型
    std::mutex operateMutex_;                                         ///< 风车识别器的操作互斥锁
    std::deque<ClassicalWindmillLogo> logoBuffer_;                    ///< 风车识别器的Logo位置缓冲区
    std::mutex logoBufferMutex_;                                      ///< 风车识别器的Logo位置缓冲区互斥锁
    std::deque<ClassicalWindmillFan> fanBuffer_;                      ///< 风车识别器的扇叶位置缓冲区
    std::mutex fanBufferMutex_;                                       ///< 风车识别器的扇叶位置缓冲区互斥锁
    std::deque<std::pair<cv::Point2f, uint64_t>> targetBuffer_;       ///< 风车识别器的目标点数据缓冲区
    std::mutex targetBufferMutex_;                                    ///< 风车识别器的目标点数据缓冲区互斥锁

    /**
      * @brief 计算风车Logo图像的Hog特征
      * @param[in] logoImage        风车Logo图像(灰度图像)
      * @param[out] logoHogFeatures 风车Logo的Hog特征
      * @return 计算结果\n
      *         -<em>false</em> 计算失败\n
      *         -<em>true</em> 计算成功\n
      * @note 如果风车识别器尚未初始化或者风车图像的尺寸不合法，Hog特征将计算失败
      */
    bool ComputeLogoHogFeatures(const cv::Mat &logoImage, std::vector<float> *logoHogFeatures) const;

    /**
     * @brief 使用LogoHogSvm模型检测风车Logo的合法性
     * @param[in, out] windmillLogos  风车Logo集合
     */
    void CheckWindmillLogosByHogSvm(std::vector<ClassicalWindmillLogo> *windmillLogos) const;

};

#endif //CUBOT_BRAIN_CLASSICAL_WINDMILL_RECOGNIZER_H