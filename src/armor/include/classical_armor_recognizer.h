//
// Created by plutoli on 2021/8/2.
//

#ifndef CUBOT_BRAIN_CLASSICAL_ARMOR_RECOGNIZER_H
#define CUBOT_BRAIN_CLASSICAL_ARMOR_RECOGNIZER_H

#include <cmath>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "delta_cv.h"
#include "classical_armor.h"
#include "classical_armor_evaluation_weight.h"
#include "classical_armor_recognizer_param.h"

/**
 * @brief 装甲板识别器
 * @note 装甲板识别器使用步骤：\n
 *       Step1：创建装甲板识别器\n
 *       Step2：自行构造ClassicalArmorRecognizerParam或从yaml文件中读取ClassicalArmorRecognizerParam\n
 *       Step3：调用SetParam()函数设置识别器参数\n
 *       Step4：调用Init()函数初始化识别器\n
 *       Step5：使用识别器对原始图像进行预处理得到二值图\n
 *       Step6：使用识别器在二值图中检测粗灯条\n
 *       Step7：使用识别器在粗灯条中检测精灯条\n
 *       Step8：使用识别器根据几何条件检测灯条对\n
 *       Step9：使用识别器根据灯条对检测粗装甲板\n
 *       Step10：使用识别器根据粗装甲板轮廓区域图像识别装甲板编号，得到精装甲板\n
 *       Step11：使用识别器对精装甲板进行评估，得到装甲板分数\n
 *       Step12：调用Release()函数释放识别器的系统资源\n
 *  @remark HOG特征+SVM的使用原理如下：\n
 *          https://learnopencv.com/histogram-of-oriented-gradients/ \n
 */
class ClassicalArmorRecognizer
{
public:
    /**
     * @brief 构造函数
     */
    ClassicalArmorRecognizer();

    /**
     * @brief 析构函数
     */
    ~ClassicalArmorRecognizer();

    /**
     * @brief 获取装甲板识别器参数
     * @return 装甲板识别器参数
     */
    ClassicalArmorRecognizerParam GetParam();

    /**
     * @brief 设置装甲板识别器参数
     * @param[in] param 装甲板识别器参数
     * @return 装甲板识别器参数设置结果\n
     *         -<em>false</em> 设置失败\n
     *         -<em>true</em> 设置成功\n
     * @note 如果装甲板识别器已经初始化完毕，参数设置将会失败
     */
    bool SetParam(const ClassicalArmorRecognizerParam &param);

    /**
     * @brief 获取装甲板识别器的初始化状态
     * @return 装甲板识别器的初始化状态\n
     *         -<em>false</em> 尚未初始化\n
     *         -<em>true</em> 已经初始化\n
     */
    bool IsInitialized();

    /**
     * @brief 获取装甲板识别器的初始化时间戳
     * @return 装甲板识别器的初始化时间戳
     */
    uint64_t GetInitTimestamp();

    /**
     * @brief 初始化装甲板识别器
     * @return 初始化结果\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     */
    bool Init();

    /**
     * @brief 释放装甲板识别器资源
     * @return 资源释放结果\n
     *         -<em>false</em> 资源释放失败\n
     *         -<em>true</em> 资源释放成功\n
     */
    bool Release();

    /**
     * @brief 对原始图像进行预处理
     * @param[in] rawImage      原始图像
     * @param[out] binaryImage  二值图像
     * @return 原始图像预处理结果\n
     *         -<em>false</em> 预处理失败\n
     *         -<em>true</em> 预处理成功\n
     * @note 如果装甲板识别器没有初始化，将返回false
     */
    bool Preprocess(const cv::Mat &rawImage, cv::Mat *binaryImage) const;

    /**
     * @brief 检测粗灯条
     * @param[in] binaryImage       二值图像
     * @param[out] rawLightBars     粗灯条集合
     * @return 检测结果\n
     *         -<em>false</em> 检测失败\n
     *         -<em>true</em> 检测成功\n
     * @note 如果装甲板识别器没有初始化，将返回false
     */
    bool DetectRawLightBars(const cv::Mat &binaryImage,
                            std::vector<ClassicalLightBar> *rawLightBars) const;

    /**
     * @brief 检测精灯条
     * @param[in] rawLightBars          粗灯条集合
     * @param[out] polishedLightBars    精灯条集合
     * @return 检测结果\n
     *         -<em>false</em> 检测失败\n
     *         -<em>true</em> 检测成功\n
     * @note 如果装甲板识别器没有初始化，将返回false
     */
    bool DetectPolishedLightBars(const std::vector<ClassicalLightBar> &rawLightBars,
                                 std::vector<ClassicalLightBar> *polishedLightBars) const;

    /**
     * @brief 检测灯条对
     * @param[in] polishedLightBars 精灯条集合
     * @param[out] lightBarPairs    灯条对集合
     * @return 检测结果\n
     *         -<em>false</em> 检测失败\n
     *         -<em>true</em> 检测成功\n
     * @note 如果装甲板识别器没有初始化，将返回false
     */
    bool DetectLightBarPairs(const std::vector<ClassicalLightBar> &polishedLightBars,
                             std::vector<std::pair<ClassicalLightBar, ClassicalLightBar>> *lightBarPairs) const;

    /**
     * @brief 检测粗装甲板
     * @param[in] lightBarPairs 灯条对集合
     * @param[in] rawImage      原始图像
     * @param[in] rawArmors     粗装甲板集合
     * @return 检测结果\n
     *         -<em>false</em> 检测失败\n
     *         -<em>true</em> 检测成功\n
     * @note 如果装甲板识别器没有初始化，将返回false
     */
    bool DetectRawArmors(const std::vector<std::pair<ClassicalLightBar, ClassicalLightBar>> &lightBarPairs,
                         const cv::Mat &rawImage,
                         std::vector<ClassicalArmor> *rawArmors) const;

    /**
     * @brief 检测精装甲板
     * @param[in] rawArmors         粗装甲板集合
     * @param[in] polishedArmors    精装甲板集合
     * @return 检测结果\n
     *         -<em>false</em> 检测失败\n
     *         -<em>true</em> 检测成功\n
     * @note 如果装甲板识别器没有初始化，将返回false
     */
    bool DetectPolishedArmors(const std::vector<ClassicalArmor> &rawArmors,
                              std::vector<ClassicalArmor> *polishedArmors) const;

    /**
     * @brief 评估精装甲板
     * @param[in] polishedArmors   精装甲板集合
     * @param[out] scores          精装甲板得分集合，和输入的精装甲板集合中的元素一一对应
     * @return 评估结果\n
     *         -<em>false</em> 评估失败\n
     *         -<em>true</em> 评估成功\n
     * @note 如果装甲板识别器没有初始化，将返回false
     */
    bool EvaluatePolishedArmors(const std::vector<ClassicalArmor> &polishedArmors,
                                std::vector<float> *scores) const;

    /**
     * @brief 更新装甲板数据缓冲区
     * @param[in] polishedArmor 精装甲板
     * @return 更新结果\n
     *         -<em>false</em> 更新失败\n
     *         -<em>true</em> 更新成功\n
     * @note 如果装甲板识别器没有初始化，将返回false
     */
    bool UpdateArmorBuffer(const ClassicalArmor &polishedArmor);

    /**
     * @brief 清空装甲板数据缓冲区
     */
    void ClearArmorBuffer();

    /**
     * @brief 获取某个装甲板位置时序
     * @param[in] armorNumber       装甲板编号
     * @param[out] locationSequence 装甲板位置时序
     * @return 获取结果\n
     *         -<em>false</em> 获取失败\n
     *         -<em>true</em> 获取成功\n
     * @note 如果装甲板识别器没有初始化，将返回false
     */
    bool GetArmorLocationSequence(const EClassicalArmorNumber &armorNumber,
                                  std::vector<std::pair<cv::Point2f, uint64_t>> *locationSequence);

    /**
     * @brief 更新目标点数据缓冲区
     * @param[in] armorNumber   装甲板编号
     * @param[in] target        击打目标点
     * @param[in] timestamp     时间戳
     * @return 更新结果\n
     *         -<em>false</em> 更新失败\n
     *         -<em>true</em> 更新成功\n
     * @note 如果装甲板识别器没有初始化，将返回false
     */
    bool UpdateTargetBuffer(const EClassicalArmorNumber &armorNumber,
                            const cv::Point2f &target,
                            const uint64_t &timestamp);

    /**
     * @brief 清空目标点数据缓冲区
     */
    void ClearTargetBuffer();

    /**
     * @brief 获取符合时间戳要求的历史击打目标点集合
     * @param[in] armorNumber       装甲板编号
     * @param[in] timestamp         历史击打目标点时间戳；单位：纳秒
     * @param[out] timestampOffset  历史击打目标点时间戳偏移；单位：纳秒
     * @param[out] targets          历史击打目标点集合
     * @return 获取结果\n
     *         -<em>false</em> 获取失败\n
     *         -<em>true</em> 获取成功\n
     */
    bool GetTarget(const EClassicalArmorNumber &armorNumber,
                   const uint64_t &timestamp,
                   const uint64_t &timestampOffset,
                   std::vector<std::pair<cv::Point2f, uint64_t>> *targets);

    /**
     * @brief 训练并保存ArmorHogSvm分类模型
     * @param[in] trainingSet       ArmorHogSvm模型的训练集
     * @param[in] termCriteria      ArmorHogSvm模型训练的终止条件
     * @param[in] modelFileName     ArmorHogSvm模型文件名称(包括存储路径和文件名)
     * @return 训练结果\n
     *         -<em>false</em> 训练失败\n
     *         -<em>true</em> 训练成功\n
     */
    [[nodiscard]] bool TrainArmorHogSvm(const std::vector<std::pair<std::string, EClassicalArmorNumber>> &trainingSet,
                                        const cv::TermCriteria &termCriteria,
                                        const std::string &modelFileName) const;

    /**
     * @brief 创建灯条图像
     * @param[in] lightBars         灯条集合
     * @param[in] rawImage          原始图像
     * @param[out] lightBarsImage   灯条图像
     */
    static void CreateLightBarsImage(const std::vector<ClassicalLightBar> &lightBars,
                                     const cv::Mat &rawImage,
                                     cv::Mat *lightBarsImage);

    /**
     * @brief 创建灯条对图像
     * @param[in] lightBarPairs         灯条对集合
     * @param[in] rawImage              原始图像
     * @param[out] lightBarPairsImage   灯条对图像
     */
    static void CreateLightBarPairsImage(const std::vector<std::pair<ClassicalLightBar, ClassicalLightBar>> &lightBarPairs,
                                         const cv::Mat &rawImage,
                                         cv::Mat *lightBarPairsImage);

    /**
     * @brief 创建普通甲板图像
     * @param[in] armors             装甲板集合
     * @param[in] rawImage           原始图像
     * @param[out] commonArmorsImage 普通装甲板图像
     */
    static void CreateCommonArmorsImage(const std::vector<ClassicalArmor> &armors,
                                        const cv::Mat &rawImage,
                                        cv::Mat *commonArmorsImage);

    /**
     * @brief 创建评估装甲板图像
     * @param[in] armors                装甲板集合
     * @param[in] scores                装甲板得分集合
     * @param[in] rawImage              原始图像
     * @param[out] evaluatedArmorsImage 评估装甲板图像
     */
    static void CreateEvaluatedArmorsImage(const std::vector<ClassicalArmor> &armors,
                                           const std::vector<float> &scores,
                                           const cv::Mat &rawImage,
                                           cv::Mat *evaluatedArmorsImage);

    /**
     * @brief 创建锁定装甲板图像
     * @param[in] lockedArmor           锁定的目标装甲板
     * @param[in] distance              锁定的目标装甲板到相机的距离
     * @param[in] armorLocations        锁定的目标装甲板历史位置信息
     * @param[in] distanceCompensation  锁定的目标装甲板的距离补偿
     * @param[in] velocityCompensation  锁定的目标装甲板的速度补偿
     * @param[in] rawImage              原始图像
     * @param[out] lockedArmorImage     锁定的目标装甲板图像
     */
    static void CreateLockedArmorImage(const ClassicalArmor &lockedArmor,
                                       const float &distance,
                                       const std::vector<cv::Point2f> &armorLocations,
                                       const std::pair<float, float> &distanceCompensation,
                                       const std::pair<float, float> &velocityCompensation,
                                       const cv::Mat &rawImage,
                                       cv::Mat *lockedArmorImage);

    /**
     * @brief 创建比较装甲板图像
     * @param[in] comparedArmor         比较的目标装甲板
     * @param[in] oldTargets            装甲板的历史击打目标点集合
     * @param[in] rawImage              原始图像
     * @param[out] comparedArmorImage   装甲板比较图像
     */
    static void CreateComparedArmorImage(const ClassicalArmor &comparedArmor,
                                         const std::vector<std::pair<cv::Point2f, uint64_t>> &oldTargets,
                                         const cv::Mat &rawImage,
                                         cv::Mat *comparedArmorImage);

    /**
     * @brief 创建测距装甲板图像
     * @param[in] polishedArmors    精装甲板集合
     * @param[in] distances         精装甲板距离集合
     * @param[in] rawImage          原始图像
     * @param[out] rangedArmorImage 测距装甲板图像
     */
    static void CreateRangedArmorImage(const std::vector<ClassicalArmor> &polishedArmors,
                                       const std::vector<float> &distances,
                                       const cv::Mat &rawImage,
                                       cv::Mat *rangedArmorImage);

    /**
     * @brief 创建追踪装甲板图像
     * @param[in] trackedArmor       追踪装甲板
     * @param[in] armorLocations     装甲板的历史位置信息
     * @param[in] rawImage           原始图像
     * @param[out] trackedArmorImage 追踪装甲板图像
     */
    static void CreateTrackedArmorImage(const ClassicalArmor &trackedArmor,
                                        const std::vector<cv::Point2f> &armorLocations,
                                        const cv::Mat &rawImage,
                                        cv::Mat *trackedArmorImage);
    /**
     * @brief 创建预测装甲板图像
    * @param[in] trackedArmor       追踪装甲板
    * @param[in] armorLocations    预测装甲板的位置信息
    * @param[in] rawImage           原始图像
    * @param[out] trackedArmorImage 追踪装甲板图像
    */
    static void CreatePredictorArmorImage(const ClassicalArmor &trackedArmor,
            const cv::Point2f &armorLocation,
                                          const cv::Mat &rawImage,
                                          cv::Mat *predictImage);

private:
    ClassicalArmorRecognizerParam param_;                                ///< 装甲板识别器参数
    std::atomic<bool> isInitialized_;                                    ///< 装甲板识别器的初始化状态
    std::atomic<uint64_t> initTimestamp_;                                ///< 装甲板识别器的初始化时间戳
    cv::HOGDescriptor armorHogDescriptor_;                               ///< 装甲板识别器的装甲板Hog特征描述子
    cv::Ptr<cv::ml::SVM> armorHogSvm_;                                   ///< 装甲板识别器的装甲板HogSvm分类模型
    std::mutex operateMutex_;                                            ///< 装甲板识别器的操作互斥锁
    std::deque<ClassicalArmor> oneArmorBuffer_;                          ///< 装甲板识别器的1号装甲板位置数据缓冲区
    std::deque<ClassicalArmor> twoArmorBuffer_;                          ///< 装甲板识别器的2号装甲板位置数据缓冲区
    std::deque<ClassicalArmor> threeArmorBuffer_;                        ///< 装甲板识别器的3号装甲板位置数据缓冲区
    std::deque<ClassicalArmor> fourArmorBuffer_;                         ///< 装甲板识别器的4号装甲板位置数据缓冲区
    std::deque<ClassicalArmor> fiveArmorBuffer_;                         ///< 装甲板识别器的5号装甲板位置数据缓冲区
    std::deque<ClassicalArmor> sentryArmorBuffer_;                       ///< 装甲板识别器的哨兵装甲板位置数据缓冲区
    std::deque<ClassicalArmor> outpostArmorBuffer_;                      ///< 装甲板识别器的前哨站装甲板位置数据缓冲区
    std::deque<ClassicalArmor> baseArmorBuffer_;                         ///< 装甲板识别器的基地装甲板位置数据缓冲区
    std::mutex armorBufferMutex_;                                        ///< 装甲板识别器的位置数据缓冲区互斥锁
    std::deque<std::pair<cv::Point2f, uint64_t>> oneTargetBuffer_;       ///< 装甲板识别器的1号装甲板目标点数据缓冲区
    std::deque<std::pair<cv::Point2f, uint64_t>> twoTargetBuffer_;       ///< 装甲板识别器的2号装甲板目标点数据缓冲区
    std::deque<std::pair<cv::Point2f, uint64_t>> threeTargetBuffer_;     ///< 装甲板识别器的3号装甲板目标点数据缓冲区
    std::deque<std::pair<cv::Point2f, uint64_t>> fourTargetBuffer_;      ///< 装甲板识别器的4号装甲板目标点数据缓冲区
    std::deque<std::pair<cv::Point2f, uint64_t>> fiveTargetBuffer_;      ///< 装甲板识别器的5号装甲板目标点数据缓冲区
    std::deque<std::pair<cv::Point2f, uint64_t>> sentryTargetBuffer_;    ///< 装甲板识别器的哨兵装甲板目标点数据缓冲区
    std::deque<std::pair<cv::Point2f, uint64_t>> outpostTargetBuffer_;   ///< 装甲板识别器的前哨站装甲板目标点数据缓冲区
    std::deque<std::pair<cv::Point2f, uint64_t>> baseTargetBuffer_;      ///< 装甲板识别器的基地装甲板目标点数据缓冲区
    std::mutex targetBufferMutex_;                                       ///< 装甲板识别器的目标点数据缓冲区互斥锁

    /**
     * @brief 对装甲板轮廓区域进行透视变换
     * @param[in] armorRegionCorners    装甲板轮廓区域角点
     * @param[in] rawImage              原始图像
     * @param[out] armorRegionImage     装甲板轮廓区域图像(彩色图像)
     * @return 透视变换结果\n
     *         -<em>false</em> 变换失败\n
     *         -<em>true</em> 变换成功\n
     * @note 如果装甲板识别器尚未初始化，装甲板轮廓区域将变换失败
     */
    bool PerspectiveTransform(const std::vector<cv::Point2f> &armorRegionCorners,
                              const cv::Mat &rawImage,
                              cv::Mat *armorRegionImage) const;

    /**
     * @brief 计算装甲板轮廓区域的Hog特征
     * @param[in] armorRegionImage  装甲板轮廓区域图像(灰度图像)
     * @param[out] armorHogFeatures 装甲板轮廓区域的Hog特征
     * @return 计算结果\n
     *         -<em>false</em> 计算失败\n
     *         -<em>true</em> 计算成功\n
     * @note 如果装甲板识别器尚未初始化或者装甲板轮廓区域图像的尺寸不合法时，HOG特征将计算失败
     */
    bool ComputeArmorHogFeatures(const cv::Mat &armorRegionImage, std::vector<float> *armorHogFeatures) const;

    /**
     * @brief 使用HogSvm模型检测装甲板编号
     * @param[in] armorHogFeatures  装甲板轮廓区域的Hog特征
     * @param[out] armorNumber      装甲板编号
     * @return 检测结果\n
     *         -<em>false</em> 检测失败\n
     *         -<em>true</em> 检测成功\n
     * @note 如果装甲板识别器尚未初始化或者或者Hog特征维数与ArmorHogSvm模型不匹配时，装甲板编号将检测失败
     */
    bool DetectArmorNumberByHogSvm(const std::vector<float> &armorHogFeatures,
                                   EClassicalArmorNumber *armorNumber) const;

    /**
     * @brief 刷新装甲板数据缓冲区
     * @param[in] polishedArmor  精装甲板
     * @param[in] armorBuffer    要刷新的装甲板数据缓冲区
     */
    void RefreshArmorBuffer(const ClassicalArmor &polishedArmor,
                                   std::deque<ClassicalArmor> *armorBuffer) const;

    /**
     * @brief 刷新目标点数据缓冲区
     * @param[in] target        击打目标点
     * @param[in] timestamp     时间戳
     * @param[in] targetBuffer  要刷新的目标点缓冲区
     */
    void RefreshTargetBuffer(const cv::Point2f &target,
                                    const uint64_t &timestamp,
                                    std::deque<std::pair<cv::Point2f, uint64_t>> *targetBuffer) const;

    /**
     * @brief 从目标点数据缓冲区中搜索符合时间戳约束的目标点
     * @param[in] targetBuffer      目标点数据缓冲区
     * @param[in] timestamp         历史击打目标点时间戳；单位：纳秒
     * @param[out] timestampOffset  历史击打目标点时间戳偏移；单位：纳秒
     * @param[out] targets          历史击打目标点集合
     */
    static void SearchForTarget(const std::deque<std::pair<cv::Point2f, uint64_t>> &targetBuffer,
                                const uint64_t &timestamp,
                                const uint64_t &timestampOffset,
                                std::vector<std::pair<cv::Point2f, uint64_t>> *targets);

    /**
     * @brief 通过多边形填充的方式计算装甲板的像素面积
     * @param[in] armorVertices 装甲板的顶点
     * @return 装甲板的像素面积
     */
    static float ComputeArmorAreaByPolygon(const std::vector<cv::Point2f> &armorVertices);

    /**
     * @brief 通过轮廓的方式计算装甲板的像素面积
     * @param[in] armorVertices 装甲板的顶点
     * @return 装甲板的像素面积
     */
    static float ComputeArmorAreaByContour(const std::vector<cv::Point2f> &armorVertices);

    /**
     * @brief 计算装甲板中心点到原始图像中心点的偏转距离
     * @param[in] armorCenter       装甲板中心点
     * @param[in] rawImageCenter    原始图像中心点
     * @return 中心点偏转距离
     * @note offset = (center1.x - center2.x)^2 + (center1.y - center2.y)^2
     */
    static float ComputeArmorOffset(const cv::Point2f &armorCenter, const cv::Point2f &rawImageCenter);
};

#endif //CUBOT_BRAIN_CLASSICAL_ARMOR_RECOGNIZER_H