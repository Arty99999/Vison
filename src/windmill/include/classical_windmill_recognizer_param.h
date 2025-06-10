//
// Created by plutoli on 2022/1/23.
//

#ifndef CUBOT_BRAIN_CLASSICAL_WINDMILL_RECOGNIZER_PARAM_H
#define CUBOT_BRAIN_CLASSICAL_WINDMILL_RECOGNIZER_PARAM_H

#include <string>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "bgr_weight.h"
#include "hsv_threshold.h"

/**
 * @brief 经典风车颜色
 */
enum class EClassicalWindmillColor
{
    Red = 1,        ///< 红色风车
    Blue = 2        ///< 蓝色风车
};

/**
 * @brief 经典风车识别器参数
 * @remark HSV颜色空间中各个通道的含义详见：https://blog.csdn.net/qq_34440409/article/details/120563986 \n
 *         HSV颜色空间分割示例详见：https://blog.csdn.net/qq_34440409/article/details/120866761 \n
 *         HSV颜色空间各个通道的取值范围详见：https://www.cnblogs.com/wangyblzu/p/5710715.html \n
 */
class ClassicalWindmillRecognizerParam
{
public:
    std::string Key;                                       ///< 风车识别器的标识符

    // 预处理和阈值分割参数
    EClassicalWindmillColor WindmillColor;                 ///< 风车识别器要识别的风车颜色
    float DownsampleFactor;                                ///< 原始图像的降采样因子
    BGRWeight BGRWeightForBlue;                            ///< 蓝色目标的加权灰度化权值
    BGRWeight BGRWeightForRed;                             ///< 红色目标的加权灰度化权值
    unsigned char GrayThresholdForBlue;                    ///< 蓝色目标的灰度分割阈值；取值范围：[0x00, 0xFF]
    unsigned char GrayThresholdForRed;                     ///< 红色目标的灰度分割阈值；取值范围：[0x00, 0xFF]
    HSVThreshold FanHSVThresholdForBlue;                      ///< 蓝色目标的HSV分割阈值
    HSVThreshold FanHSVThresholdForRed_1;                     ///< 红色目标的HSV分割阈值1
    HSVThreshold FanHSVThresholdForRed_2;                     ///< 红色目标的HSV分割阈值2
    HSVThreshold TargetHSVThresholdForBlue;                      ///< 蓝色目标的HSV分割阈值
    HSVThreshold TargetHSVThresholdForRed_1;                     ///< 红色目标的HSV分割阈值1
    HSVThreshold TargetHSVThresholdForRed_2;                     ///< 红色目标的HSV分割阈值2
    int CloseElementWidth;                                ///< 扇叶二值图像的闭运算元素宽度
    int CloseElementHeight;                               ///< 扇叶二值图像的闭运算元素高度

    // 风车Logo检测参数
    float MaxLogoContourArea;                              ///< 风车Logo的最大轮廓面积
    float MinLogoContourArea;                              ///< 风车Logo的最小轮廓面积
    float MaxLogoAspectRatio;                              ///< 风车Logo的最大纵横比；纵横比=Logo高度/Logo宽度
    float MinLogoAspectRatio;                              ///< 风车Logo的最小纵横比；纵横比=Logo高度/Logo宽度
    int LogoHogWindowWidth;                                ///< 风车Logo的Hog特征描述子窗口宽度
    int LogoHogWindowHeight;                               ///< 风车Logo的Hog特征描述子窗口高度
    int LogoHogBlockWidth;                                 ///< 风车Logo的Hog特征描述子块宽度
    int LogoHogBlockHeight;                                ///< 风车Logo的Hog特征描述子块高度
    int LogoHogCellWidth;                                  ///< 风车Logo的Hog特征描述子细胞宽度
    int LogoHogCellHeight;                                 ///< 风车Logo的Hog特征描述子细胞高度
    int LogoHogStrideWidth;                                ///< 风车Logo的Hog特征描述子步长宽度
    int LogoHogStrideHeight;                               ///< 风车Logo的Hog特征描述子步长高度
    int LogoHogBins;                                       ///< 风车Logo的Hog特征描述子梯度方向数
    std::string LogoHogSvmFileName;                        ///< 风车Logo的HogSvm模型文件名

    // 风车扇叶检测参数
    float MaxFanContourArea;                               ///< 风车扇叶的最大外部轮廓面积
    float MinFanContourArea;                               ///< 风车扇叶的最小外部轮廓面积
    float MaxFanAspectRatio;                               ///< 风车扇叶的最大外部轮廓纵横比；纵横比=扇叶高度/扇叶宽度
    float MinFanAspectRatio;                               ///< 风车扇叶的最小外部轮廓纵横比；纵横比=扇叶高度/扇叶宽度
    float MaxTargetContourArea;                              ///< 风车靶心的最大外部轮廓面积
    float MinTargetContourArea;                              ///< 风车靶心的最小外部轮廓面积
    float MaxTargetAspectRatio;                              ///< 风车靶心的最大外部轮廓纵横比；纵横比=扇叶高度/扇叶宽度
    float MinTargetAspectRatio;                              ///< 风车靶心的最小外部轮廓纵横比；纵横比=扇叶高度/扇叶宽度
    float MaxFanTargetAngle;                               ///< 风车扇叶与靶心的最大夹角
    float MinFanTargetAngle;                               ///< 风车扇叶与靶心的最小夹角

    // 风车中间装甲板的物理参数
    float TargetPhysicalHeight;                            ///< 风车中间装甲板的物理高度
    float TargetPhysicalWidth;                             ///< 风车中间装甲板的物理宽度

    // 风车Logo的物理参数
    float LogoPhysicalHeight;                              ///< Logo的物理高度
    float LogoPhysicalWidth;                               ///< Logo的物理宽度

    // 风车识别器预测参数
    float MaxLogoLocationOffset;                           ///< 风车识别器数据缓冲区相邻两个元素Logo位置的最大偏差
    float MaxFanLocationOffset;                            ///< 风车识别器数据缓冲区相邻两个元素扇叶位置的最大偏差
    uint64_t MaxMemoryLength;                              ///< 风车识别器最长记忆时间；单位：毫秒

    /**
    * @brief 构造函数
    */
    ClassicalWindmillRecognizerParam();

    /**
     * @brief 析构函数
     */
    ~ClassicalWindmillRecognizerParam() = default;

    /**
     * @brief 转换风车颜色
     * @param[in]   input   输入的风车颜色数值
     * @param[out]  output  转换得到的风车颜色
     * @return 风车颜色转换结果\n
     *         -<em>false</em> 转换失败\n
     *         -<em>true</em> 转换成功\n
     * @note 风车颜色的取值为1/2，输入数据不在此范围内，则转换失败\n
     *         -<em>1</em> 红色风车\n
     *         -<em>2</em> 蓝色风车\n
     */
    static bool ConvertToWindmillColor(const int &input, EClassicalWindmillColor *output);

    /**
     * @brief 从yaml配置文件中加载风车识别器参数
     * @param[in]  yamlFileName     风车识别器参数配置文件名
     * @param[out] recognizerParam  风车识别器参数
     * @return 风车识别器参数加载结果\n
     *         -<em>false</em> 加载失败\n
     *         -<em>true</em> 加载成功\n
     */
    static bool LoadFromYamlFile(const std::string &yamlFileName, ClassicalWindmillRecognizerParam *recognizerParam);
};

#endif //CUBOT_BRAIN_CLASSICAL_WINDMILL_RECOGNIZER_PARAM_H