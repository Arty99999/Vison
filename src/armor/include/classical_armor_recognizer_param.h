//
// Created by plutoli on 2021/8/2.
//

#ifndef CUBOT_BRAIN_CLASSICAL_ARMOR_RECOGNIZER_PARAM_H
#define CUBOT_BRAIN_CLASSICAL_ARMOR_RECOGNIZER_PARAM_H

#include <string>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "bgr_weight.h"
#include "hsv_threshold.h"
#include "classical_light_bar.h"
#include "classical_armor_evaluation_weight.h"

/**
 * @brief 装甲板识别器参数
 */
class ClassicalArmorRecognizerParam
{
public:
    std::string Key;                           ///< 经典装甲板识别器的标识符

    // 预处理和阈值分割参数
    EClassicalLightBarColor LightBarColor;     ///< 将要识别的灯条颜色
    float DownsampleFactor;                    ///< 原始图像的降采样因子
    BGRWeight BGRWeightForBlue;                ///< 蓝色目标的加权灰度化权值
    BGRWeight BGRWeightForRed;                 ///< 红色目标的加权灰度化权值
    unsigned char GrayThresholdForBlue;        ///< 蓝色目标的灰度分割阈值；取值范围：[0x00, 0xFF]
    unsigned char GrayThresholdForRed;         ///< 红色目标的灰度分割阈值；取值范围：[0x00, 0xFF]
    HSVThreshold HSVThresholdForBlue;          ///< 蓝色目标的HSV分割阈值
    HSVThreshold HSVThresholdForRed_1;         ///< 红色目标的HSV分割阈值1
    HSVThreshold HSVThresholdForRed_2;         ///< 红色目标的HSV分割阈值2

    // 灯条检测参数
    float MaxLightBarArea;                      ///< 灯条的最大面积
    float MinLightBarArea;                      ///< 灯条的最小面积
    float MaxLightBarHeight;                    ///< 灯条的最大高度
    float MinLightBarHeight;                    ///< 灯条的最小高度
    float MaxLightBarAspectRatio;               ///< 灯条的最大纵横比；纵横比=灯条高度/灯条宽度；取值范围：≥1.0
    float MaxLightBarAngle;                     ///< 灯条的长边与x轴正方向夹角的最大值；取值范围：[0, 180]
    float MinLightBarAngle;                     ///< 灯条的长边与x轴正方向夹角的最小值；取值范围：[0, 180]

    // 灯条匹配参数
    float MaxLightBarsDistanceRatio;            ///< 装甲板上两个灯条的最大中心距比例；中心距比例=灯条平均高度/灯条中心距
    float MinLightBarsDistanceRatio;            ///< 装甲板上两个灯条的最小中心距比例；中心距比例=灯条平均高度/灯条中心距
    float MaxLightBarsAngleOffset;              ///< 装甲板上两个灯条的长边与x轴正方向夹角的角度偏差最大值；取值范围：[0, 90)
    float MaxLightBarsHeightRatio;              ///< 装甲板上两个灯条的最大高度比；高度比=长灯条高度/短灯条高度；取值范围：≥1.0

    // 装甲板检测参数
    int ArmorHogWindowWidth;                    ///< 装甲板Hog特征描述子窗口宽度
    int ArmorHogWindowHeight;                   ///< 装甲板Hog特征描述子窗口高度
    int ArmorHogBlockWidth;                     ///< 装甲板Hog特征描述子块宽度
    int ArmorHogBlockHeight;                    ///< 装甲板Hog特征描述子块高度
    int ArmorHogCellWidth;                      ///< 装甲板Hog特征描述子细胞宽度
    int ArmorHogCellHeight;                     ///< 装甲板Hog特征描述子细胞高度
    int ArmorHogStrideWidth;                    ///< 装甲板Hog特征描述子步长宽度
    int ArmorHogStrideHeight;                   ///< 装甲板Hog特征描述子步长高度
    int ArmorHogBins;                           ///< 装甲板Hog特征描述子梯度方向数
    std::string ArmorHogSvmFileName;            ///< 装甲板HogSvm模型文件名

    // 装甲板的物理参数
    float LightBarPhysicalHeight;               ///< 灯条的物理高度；单位：毫米
    float LightBarPhysicalWidth;                ///< 灯条的物理宽度；单位：毫米
    float LargeArmorPhysicalHeight;             ///< 大装甲板的物理高度(平行于灯条的边长)；单位：毫米
    float LargeArmorPhysicalWidth;              ///< 大装甲板的物理宽度(垂直于灯条的边长)；单位：毫米
    float SmallArmorPhysicalHeight;             ///< 小装甲板的物理高度(平行于灯条的边长)；单位：毫米
    float SmallArmorPhysicalWidth;              ///< 小装甲板的物理宽度(垂直于灯条的边长)；单位：毫米

    // 装甲板评估和预测参数
    float MaxLocationOffset;                                         ///< 装甲板缓冲区中存储的同一装甲板相邻时刻的最大位置偏差
    uint64_t MaxMemoryLength;                                        ///< 装甲板识别器最长记忆时间；单位：毫秒
    std::vector<ClassicalArmorEvaluationWeight> EvaluationWeights;   ///< 装甲板评估权值集合

    /**
    * @brief 构造函数
    */
    ClassicalArmorRecognizerParam();

    /**
     * @brief 析构函数
     */
    ~ClassicalArmorRecognizerParam() = default;

    /**
     * @brief 从yaml配置文件中加载装甲板识别器参数
     * @param[in]  yamlFileName     装甲板识别器参数配置文件名称
     * @param[out] recognizerParam  装甲板识别器参数
     * @return 装甲板识别器参数加载结果\n
     *         -<em>false</em> 加载失败\n
     *         -<em>true</em> 加载成功\n
     */
    static bool LoadFromYamlFile(const std::string &yamlFileName, ClassicalArmorRecognizerParam *recognizerParam);
};

#endif //CUBOT_BRAIN_CLASSICAL_ARMOR_RECOGNIZER_PARAM_H