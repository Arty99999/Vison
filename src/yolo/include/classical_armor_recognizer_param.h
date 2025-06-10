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
#include "classical_armor_evaluation_weight.h"
#include "classical_armor_solver.h"

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

    // YOLO
    std::string YoloFaceOpenvinoPath;           ///< 模型路径（xml）

    float BoxThreshold;                         ///< 检测框阈值
    float ConfThreshold;                        ///< 置信度阈值
    float NMSThreshold;                         ///< NMS阈值
    int ClassNumber;                            ///< 类别数量

    // Openvino
    int ImageSize;                              ///< 输入图片的大小

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