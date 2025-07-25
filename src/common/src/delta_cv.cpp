//
// Created by plutoli on 2021/8/9.
//

#include "delta_cv.h"

// ******************************  DeltaCV类的公有函数  ******************************

// 图像加权二值化
void DeltaCV::WeightedBinarize(const cv::Mat &rawImage,
                               const BGRWeight &weight,
                               const unsigned char &threshold,
                               cv::Mat *grayScaleImage)
{
    // 判断参数的维数是否合法
    assert(rawImage.channels() == 3);
    assert(grayScaleImage->channels() == 1);

    // 对输入的彩色图像进行三通道分割
    cv::Mat singleColorImage[3];
    cv::split(rawImage,singleColorImage);

    // 计算加权灰度图像
    cv::Mat weightedGrayImage;
    weightedGrayImage = singleColorImage[0] * weight.Blue + singleColorImage[1] * weight.Green + singleColorImage[2] * weight.Red;

    // 对加权灰度图像进行阈值分割
    cv::threshold(weightedGrayImage, *grayScaleImage, threshold, 255, cv::THRESH_BINARY);
}

// 快速图像加权二值化
void DeltaCV::FastWeidhtedBinarize(const cv::Mat &rawImage,
                                   const BGRWeight &weight,
                                   const unsigned char &threshold,
                                   cv::Mat *grayScaleImage){}
//{
//    // 判断参数的维数是否合法
//    assert(rawImage.channels() == 3);
//    assert(grayScaleImage->channels() == 1);
//
//    // 对图像数据中的BGR字节流进行分块，每一块包括96个字节(即32个像素)
//    int blockSize = 96;
//    int blockNumber = (rawImage.cols * rawImage.rows * 3) / blockSize;
//
//    // 加载阈值
//    __m256i thresholds = _mm256_setr_epi8(threshold, threshold, threshold, threshold,
//                                          threshold, threshold, threshold, threshold,
//                                          threshold, threshold, threshold, threshold,
//                                          threshold, threshold, threshold, threshold,
//                                          threshold, threshold, threshold, threshold,
//                                          threshold, threshold, threshold, threshold,
//                                          threshold, threshold, threshold, threshold,
//                                          threshold, threshold, threshold, threshold);
//
//    // 计算BGR各通道权值
//    unsigned short shift = 15;
//    auto multiplier = static_cast<float>(1 << shift);
//    auto bWeight = static_cast<unsigned short>(weight.Blue * multiplier);
//    auto gWeight = static_cast<unsigned short>(weight.Green * multiplier);
//    auto rWeight = static_cast<unsigned short>(weight.Red * multiplier);
//
//    // 构造每个数据块的蓝绿通道权值
//    __m256i bgWeights = _mm256_setr_epi16(bWeight, gWeight, bWeight, gWeight,
//                                          bWeight, gWeight, bWeight, gWeight,
//                                          bWeight, gWeight, bWeight, gWeight,
//                                          bWeight, gWeight, bWeight, gWeight);
//
//    // 构造每个数据块的红色通道权值
//    __m256i rWeights = _mm256_setr_epi16(rWeight, 0, rWeight, 0,
//                                         rWeight, 0, rWeight, 0,
//                                         rWeight, 0, rWeight, 0,
//                                         rWeight, 0, rWeight, 0);
//
//    // 循环构造并处理数据块
//    for (int i = 0; i < blockNumber; ++i)
//    {
//        // 一次读取96个字节
//        unsigned char *src = rawImage.data + i * 96;
//        __m128i data0 = _mm_loadu_si128((__m128i *)(src + 0));
//        __m128i data1 = _mm_loadu_si128((__m128i *)(src + 16));
//        __m128i data2 = _mm_loadu_si128((__m128i *)(src + 32));
//        __m128i data3 = _mm_loadu_si128((__m128i *)(src + 48));
//        __m128i data4 = _mm_loadu_si128((__m128i *)(src + 64));
//        __m128i data5 = _mm_loadu_si128((__m128i *)(src + 80));
//
//        // 构造bg00
//        __m128i bgMask00 = _mm_setr_epi8(0, -1, 1, -1, 3, -1, 4, -1,
//                                         6, -1, 7, -1, 9, -1, 10, -1);
//        __m128i bg00 = _mm_shuffle_epi8(data0, bgMask00);
//
//        // 构造bg10
//        __m128i bgMask10_0 = _mm_setr_epi8(12, -1, 13, -1, 15, -1, -1, -1,
//                                           -1, -1, -1, -1, -1, -1, -1, -1);
//        __m128i bg10_0 = _mm_shuffle_epi8(data0, bgMask10_0);
//        __m128i bgMask10_1 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, 0, -1,
//                                           2, -1, 3, -1, 5, -1, 6, -1);
//        __m128i bg10_1 = _mm_shuffle_epi8(data1, bgMask10_1);
//        __m128i bg10 = _mm_or_si128(bg10_0, bg10_1);
//
//        // 构造bg20
//        __m128i bgMask20_0 = _mm_setr_epi8(8, -1, 9, -1, 11, -1, 12, -1,
//                                           14, -1, 15, -1, -1, -1, -1, -1);
//        __m128i bg20_0 = _mm_shuffle_epi8(data1, bgMask20_0);
//        __m128i bgMask20_1 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1,
//                                           -1, -1, -1, -1, 1, -1, 2, -1);
//        __m128i bg20_1 = _mm_shuffle_epi8(data2, bgMask20_1);
//        __m128i bg20 = _mm_or_si128(bg20_0, bg20_1);
//
//        // 构造bg30
//        __m128i bgMask30 = _mm_setr_epi8(4, -1, 5, -1, 7, -1, 8, -1,
//                                         10, -1, 11, -1, 13, -1, 14, -1);
//        __m128i bg30 = _mm_shuffle_epi8(data2, bgMask30);
//
//        // 构造bg01
//        __m128i bgMask01 = _mm_setr_epi8(0, -1, 1, -1, 3, -1, 4, -1,
//                                         6, -1, 7, -1, 9, -1, 10, -1);
//        __m128i bg01 = _mm_shuffle_epi8(data3, bgMask01);
//
//        // 构造bg11
//        __m128i bgMask11_0 = _mm_setr_epi8(12, -1, 13, -1, 15, -1, -1, -1,
//                                           -1, -1, -1, -1, -1, -1, -1, -1);
//        __m128i bg11_0 = _mm_shuffle_epi8(data3, bgMask11_0);
//        __m128i bgMask11_1 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, 0, -1,
//                                           2, -1, 3, -1, 5, -1, 6, -1);
//        __m128i bg11_1 = _mm_shuffle_epi8(data4, bgMask11_1);
//        __m128i bg11 = _mm_or_si128(bg11_0, bg11_1);
//
//        // 构造bg21
//        __m128i bgMask21_0 = _mm_setr_epi8(8, -1, 9, -1, 11, -1, 12, -1,
//                                           14, -1, 15, -1, -1, -1, -1, -1);
//        __m128i bg21_0 = _mm_shuffle_epi8(data4, bgMask21_0);
//        __m128i bgMask21_1 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1,
//                                           -1, -1, -1, -1, 1, -1, 2, -1);
//        __m128i bg21_1 = _mm_shuffle_epi8(data5, bgMask21_1);
//        __m128i bg21 = _mm_or_si128(bg21_0, bg21_1);
//
//        // 构造bg31
//        __m128i bgMask31 = _mm_setr_epi8(4, -1, 5, -1, 7, -1, 8, -1,
//                                         10, -1, 11, -1, 13, -1, 14, -1);
//        __m128i bg31 = _mm_shuffle_epi8(data5, bgMask31);
//
//        // 合并生成bg0/bg1/bg2/bg3
//        __m256i bg0 = _mm256_combine_si128(bg00, bg01);
//        __m256i bg1 = _mm256_combine_si128(bg10, bg11);
//        __m256i bg2 = _mm256_combine_si128(bg20, bg21);
//        __m256i bg3 = _mm256_combine_si128(bg30, bg31);
//
//        // 构造r00
//        __m128i rMask00 = _mm_setr_epi8(2, -1, -1, -1, 5, -1, -1, -1,
//                                         8, -1, -1, -1, 11, -1, -1, -1);
//        __m128i r00 = _mm_shuffle_epi8(data0, rMask00);
//
//        // 构造r10
//        __m128i rMask10_0 = _mm_setr_epi8(14, -1, -1, -1, -1, -1, -1, -1,
//                                          -1, -1, -1, -1, -1, -1, -1, -1);
//        __m128i r10_0 = _mm_shuffle_epi8(data0, rMask10_0);
//        __m128i rMask10_1 = _mm_setr_epi8(-1, -1, -1, -1, 1, -1, -1, -1,
//                                          4, -1, -1, -1, 7, -1, -1, -1);
//        __m128i r10_1 = _mm_shuffle_epi8(data1, rMask10_1);
//        __m128i r10 = _mm_or_si128(r10_0, r10_1);
//
//        // 构造r20
//        __m128i rMask20_0 = _mm_setr_epi8(10, -1, -1, -1, 13, -1, -1, -1,
//                                          -1, -1, -1, -1, -1, -1, -1, -1);
//        __m128i r20_0 = _mm_shuffle_epi8(data1, rMask20_0);
//        __m128i rMask20_1 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1,
//                                          0, -1, -1, -1, 3, -1, -1, -1);
//        __m128i r20_1 = _mm_shuffle_epi8(data2, rMask20_1);
//        __m128i r20 = _mm_or_si128(r20_0, r20_1);
//
//        // 构造r30
//        __m128i rMask30 = _mm_setr_epi8(6, -1, -1, -1, 9, -1, -1, -1,
//                                        12, -1, -1, -1, 15, -1, -1, -1);
//        __m128i r30 = _mm_shuffle_epi8(data2, rMask30);
//
//        // 构造r01
//        __m128i rMask01 = _mm_setr_epi8(2, -1, -1, -1, 5, -1, -1, -1,
//                                        8, -1, -1, -1, 11, -1, -1, -1);
//        __m128i r01 = _mm_shuffle_epi8(data3, rMask01);
//
//        // 构造r11
//        __m128i rMask11_0 = _mm_setr_epi8(14, -1, -1, -1, -1, -1, -1, -1,
//                                          -1, -1, -1, -1, -1, -1, -1, -1);
//        __m128i r11_0 = _mm_shuffle_epi8(data3, rMask11_0);
//        __m128i rMask11_1 = _mm_setr_epi8(-1, -1, -1, -1, 1, -1, -1, -1,
//                                          4, -1, -1, -1, 7, -1, -1, -1);
//        __m128i r11_1 = _mm_shuffle_epi8(data4, rMask11_1);
//        __m128i r11 = _mm_or_si128(r11_0, r11_1);
//
//        // 构造r21
//        __m128i rMask21_0 = _mm_setr_epi8(10, -1, -1, -1, 13, -1, -1, -1,
//                                          -1, -1, -1, -1, -1, -1, -1, -1);
//        __m128i r21_0 = _mm_shuffle_epi8(data4, rMask21_0);
//        __m128i rMask21_1 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1,
//                                          0, -1, -1, -1, 3, -1, -1, -1);
//        __m128i r21_1 = _mm_shuffle_epi8(data5, rMask21_1);
//        __m128i r21 = _mm_or_si128(r21_0, r21_1);
//
//        // 构造r31
//        __m128i rMask31 = _mm_setr_epi8(6, -1, -1, -1, 9, -1, -1, -1,
//                                        12, -1, -1, -1, 15, -1, -1, -1);
//        __m128i r31 = _mm_shuffle_epi8(data5, rMask31);
//
//        // 合并生成r0/r1/r2/r3
//        __m256i r0 = _mm256_combine_si128(r00, r01);
//        __m256i r1 = _mm256_combine_si128(r10, r11);
//        __m256i r2 = _mm256_combine_si128(r20, r21);
//        __m256i r3 = _mm256_combine_si128(r30, r31);
//
//        // 进行乘法运算
//        __m256i gray32_0 = _mm256_srai_epi32(_mm256_add_epi32(_mm256_madd_epi16(bg0, bgWeights), _mm256_madd_epi16(r0, rWeights)), shift);
//        __m256i gray32_1 = _mm256_srai_epi32(_mm256_add_epi32(_mm256_madd_epi16(bg1, bgWeights), _mm256_madd_epi16(r1, rWeights)), shift);
//        __m256i gray32_2 = _mm256_srai_epi32(_mm256_add_epi32(_mm256_madd_epi16(bg2, bgWeights), _mm256_madd_epi16(r2, rWeights)), shift);
//        __m256i gray32_3 = _mm256_srai_epi32(_mm256_add_epi32(_mm256_madd_epi16(bg3, bgWeights), _mm256_madd_epi16(r3, rWeights)), shift);
//
//        // 进行阈值分割
//        __m256i gray8 = _mm256_packus_epi16(_mm256_packus_epi32(gray32_0, gray32_1), _mm256_packus_epi32(gray32_2, gray32_3));
//        __m256i result = _mm256_cmpge_epu8(gray8, thresholds);
//
//        // 存储阈值分割之后的结果
//        auto *dst = (__m256i *)(grayScaleImage->data + i * 32);
//        _mm256_storeu_si256((__m256i *)dst, result);
//    }
//
//    // 剩余不足一个块的像素进行单独处理
//    int remainPixelNumber = rawImage.cols * rawImage.rows - blockNumber * 32;
//    unsigned char *src = rawImage.data + blockNumber * 96;
//    unsigned char *dst = grayScaleImage->data + blockNumber * 32;
//    for (int i = 0; i < remainPixelNumber; ++i)
//    {
//        // 获取像素的三通道分量
//        unsigned char bChannel = *(src + i * 3 + 0);
//        unsigned char gChannel = *(src + i * 3 + 1);
//        unsigned char rChannel = *(src + i * 3 + 2);
//
//        // 计算加权灰度值，并进行阈值分割
//        unsigned char gray = (bWeight * bChannel + gWeight * gChannel + rWeight * rChannel) >> shift;
//        if (gray >= threshold)
//        {
//            *(dst + i) = 0xFF;
//        }
//        else
//        {
//            *(dst + i) = 0x00;
//        }
//    }
//}

// HSV三通道阈值分割
void DeltaCV::InRange(const cv::Mat &rawImage,
                      const HSVThreshold &threshold,
                      cv::Mat *grayScaleImage)
{
    cv::inRange(rawImage,
                cv::Scalar(threshold.HueLower, threshold.SaturationLower, threshold.ValueLower),
                cv::Scalar(threshold.HueUpper, threshold.SaturationUpper, threshold.ValueUpper),
                *grayScaleImage);
}

// 快速HSV三通道阈值分割
void DeltaCV::FastInRange(const cv::Mat &rawImage,
                          const HSVThreshold &threshold,
                          cv::Mat *grayScaleImage){}
//{
//    // 判断参数的维数是否合法
//    assert(rawImage.channels() == 3);
//    assert(grayScaleImage->channels() == 1);
//
//    // 对图像数据中的HSV字节流进行分块，每一块包括48个字节(即16个像素)
//    int blockSize = 48;
//    int blockNumber = (rawImage.cols * rawImage.rows * 3) / blockSize;
//
//    // 加载HueLower
//    __m128i hueLower = _mm_setr_epi8(threshold.HueLower, threshold.HueLower, threshold.HueLower, threshold.HueLower,
//                                     threshold.HueLower, threshold.HueLower, threshold.HueLower, threshold.HueLower,
//                                     threshold.HueLower, threshold.HueLower, threshold.HueLower, threshold.HueLower,
//                                     threshold.HueLower, threshold.HueLower, threshold.HueLower, threshold.HueLower);
//    // 加载HueUpper
//    __m128i hueUpper = _mm_setr_epi8(threshold.HueUpper, threshold.HueUpper, threshold.HueUpper, threshold.HueUpper,
//                                     threshold.HueUpper, threshold.HueUpper, threshold.HueUpper, threshold.HueUpper,
//                                     threshold.HueUpper, threshold.HueUpper, threshold.HueUpper, threshold.HueUpper,
//                                     threshold.HueUpper, threshold.HueUpper, threshold.HueUpper, threshold.HueUpper);
//    // 加载SaturationUpper
//    __m128i saturationLower = _mm_setr_epi8(threshold.SaturationLower, threshold.SaturationLower, threshold.SaturationLower, threshold.SaturationLower,
//                                            threshold.SaturationLower, threshold.SaturationLower, threshold.SaturationLower, threshold.SaturationLower,
//                                            threshold.SaturationLower, threshold.SaturationLower, threshold.SaturationLower, threshold.SaturationLower,
//                                            threshold.SaturationLower, threshold.SaturationLower, threshold.SaturationLower, threshold.SaturationLower);
//    // 加载SaturationUpper
//    __m128i saturationUpper = _mm_setr_epi8(threshold.SaturationUpper, threshold.SaturationUpper, threshold.SaturationUpper, threshold.SaturationUpper,
//                                            threshold.SaturationUpper, threshold.SaturationUpper, threshold.SaturationUpper, threshold.SaturationUpper,
//                                            threshold.SaturationUpper, threshold.SaturationUpper, threshold.SaturationUpper, threshold.SaturationUpper,
//                                            threshold.SaturationUpper, threshold.SaturationUpper, threshold.SaturationUpper, threshold.SaturationUpper);
//    // 加载ValueUpper
//    __m128i valueLower = _mm_setr_epi8(threshold.ValueLower, threshold.ValueLower, threshold.ValueLower, threshold.ValueLower,
//                                       threshold.ValueLower, threshold.ValueLower, threshold.ValueLower, threshold.ValueLower,
//                                       threshold.ValueLower, threshold.ValueLower, threshold.ValueLower, threshold.ValueLower,
//                                       threshold.ValueLower, threshold.ValueLower, threshold.ValueLower, threshold.ValueLower);
//    // 加载ValueUpper
//    __m128i valueUpper = _mm_setr_epi8(threshold.ValueUpper, threshold.ValueUpper, threshold.ValueUpper, threshold.ValueUpper,
//                                       threshold.ValueUpper, threshold.ValueUpper, threshold.ValueUpper, threshold.ValueUpper,
//                                       threshold.ValueUpper, threshold.ValueUpper, threshold.ValueUpper, threshold.ValueUpper,
//                                       threshold.ValueUpper, threshold.ValueUpper, threshold.ValueUpper, threshold.ValueUpper);
//
//    // 循环构造并处理数据块
//    for (int i = 0; i < blockNumber; ++i)
//    {
//        // 一次读取48个字节
//        unsigned char *src = rawImage.data + i * blockSize;
//
//        __m128i data0 = _mm_loadu_si128((__m128i *) (src + 0));
//        __m128i data1 = _mm_loadu_si128((__m128i *) (src + 16));
//        __m128i data2 = _mm_loadu_si128((__m128i *) (src + 32));
//
//        // 构造H0
//        __m128i HMask0 = _mm_setr_epi8(0, 3, 6, 9, 12, 15, -1, -1,
//                                       -1, -1, -1, -1, -1, -1, -1, -1);
//        __m128i H0 = _mm_shuffle_epi8(data0, HMask0);
//
//        // 构造H1
//        __m128i HMask1 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, 2, 5,
//                                       8, 11, 14, -1, -1, -1, -1, -1);
//        __m128i H1 = _mm_shuffle_epi8(data1, HMask1);
//
//        // 构造H2
//        __m128i HMask2 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1,
//                                       -1, -1, -1, 1, 4, 7, 10, 13);
//        __m128i H2 = _mm_shuffle_epi8(data2, HMask2);
//
//        // 构造H
//        __m128i H = _mm_or_si128(_mm_or_si128(H0, H1), H2);
//
//        // 构造S0
//        __m128i SMask0 = _mm_setr_epi8(1, 4, 7, 10, 13, -1, -1, -1,
//                                       -1, -1, -1, -1, -1, -1, -1, -1);
//        __m128i S0 = _mm_shuffle_epi8(data0, SMask0);
//
//        // 构造S1
//        __m128i SMask1 = _mm_setr_epi8(-1, -1, -1, -1, -1, 0, 3, 6,
//                                       9, 12, 15, -1, -1, -1, -1, -1);
//        __m128i S1 = _mm_shuffle_epi8(data1, SMask1);
//
//        // 构造S2
//        __m128i SMask2 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1,
//                                       -1, -1, -1, 2, 5, 8, 11, 14);
//        __m128i S2 = _mm_shuffle_epi8(data2, SMask2);
//
//        // 构造S
//        __m128i S = _mm_or_si128(_mm_or_si128(S0, S1), S2);
//
//        // 构造V0
//        __m128i VMask0 = _mm_setr_epi8(2, 5, 8, 11, 14, -1, -1, -1,
//                                       -1, -1, -1, -1, -1, -1, -1, -1);
//        __m128i V0 = _mm_shuffle_epi8(data0, VMask0);
//
//        // 构造V1
//        __m128i VMask1 = _mm_setr_epi8(-1, -1, -1, -1, -1, 1, 4, 7,
//                                       10, 13, -1, -1, -1, -1, -1, -1);
//        __m128i V1 = _mm_shuffle_epi8(data1, VMask1);
//
//        // 构造V2
//        __m128i VMask2 = _mm_setr_epi8(-1, -1, -1, -1, -1, -1, -1, -1,
//                                       -1, -1, 0, 3, 6, 9, 12, 15);
//        __m128i V2 = _mm_shuffle_epi8(data2, VMask2);
//
//        // 构造V
//        __m128i V = _mm_or_si128(_mm_or_si128(V0, V1), V2);
//
//        // Hue阈值判断
//        __m128i result00 = _mm_cmpge_epu8(H, hueLower);
//        __m128i result01 = _mm_cmple_epu8(H, hueUpper);
//        __m128i result0 = _mm_and_si128(result00, result01);
//
//        // Saturation阈值判断
//        __m128i result10 = _mm_cmpge_epu8(S, saturationLower);
//        __m128i result11 = _mm_and_si128(result0, result10);
//        __m128i result12 = _mm_cmple_epu8(S, saturationUpper);
//        __m128i result1 = _mm_and_si128(result11, result12);
//
//        // Value阈值判断
//        __m128i result20 = _mm_cmpge_epu8(V, valueLower);
//        __m128i result21 = _mm_and_si128(result1, result20);
//        __m128i result22 = _mm_cmple_epu8(V, valueUpper);
//        __m128i result = _mm_and_si128(result21, result22);
//
//        // 三通道变单通道则存储16个字节
//        auto *dst = (__m128i *)(grayScaleImage->data + i * 16);
//
//        // 存储阈值分割之后的结果
//        _mm_storeu_si128(dst, result);
//    }
//
//    // 剩余不足一个块的像素进行单独处理
//    int remainPixelNumber = rawImage.cols * rawImage.rows - blockNumber * 16;
//    unsigned char *src = rawImage.data + blockNumber * 48;
//    unsigned char *dst = grayScaleImage->data + blockNumber * 16;
//    for (int i = 0; i < remainPixelNumber; ++i)
//    {
//        // 获取像素的三通道分量
//        unsigned char hChannel = *(src + i * 3 + 0);
//        unsigned char sChannel = *(src + i * 3 + 1);
//        unsigned char vChannel = *(src + i * 3 + 2);
//
//        // HSV阈值分割
//        if ((hChannel >= threshold.HueLower) &&
//            (hChannel <= threshold.HueUpper) &&
//            (sChannel >= threshold.SaturationLower) &&
//            (sChannel <= threshold.SaturationUpper) &&
//            (vChannel >= threshold.ValueLower) &&
//            (vChannel <= threshold.ValueUpper))
//        {
//            *(dst + i) = 0xFF;
//        }
//        else
//        {
//            *(dst + i) = 0x00;
//        }
//    }
//}