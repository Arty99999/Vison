//
// Created by plutoli on 2021/8/9.
//

#include <mmintrin.h>       // MMX
#include <xmmintrin.h>      // SSE
#include <emmintrin.h>      // SSE2
#include <immintrin.h>      // AVX
#include <pmmintrin.h>      // SSE3
#include <smmintrin.h>

#define _mm_cmpge_up_epu8(a, b) _mm_cmpeq_epi8(_mm_max_epu8(a, b), a) //大于等于的留下
#define _mm_cmpge_down_epu8(a, b) _mm_cmpeq_epi8(_mm_min_epu8(a, b), a) //小于等于的留下

#define _mm_cmpgt_up_epu8(a,b) _mm_andnot_si128(_mm_cmpeq_epi8(a,b),_mm_cmpeq_epi8(_mm_max_epu8(a,b),a)) //大于的留下

#define _mm256_cmpge_up_epu8(a, b) _mm256_cmpeq_epi8(_mm256_max_epu8(a, b), a) //大于等于的留下 avx2
#define _mm256_cmpge_down_epu8(a, b) _mm256_cmpeq_epi8(_mm256_min_epu8(a, b), a) //小于等于的留下

#define _mm256_combine_si128(a,b) _mm256_insertf128_si256(_mm256_castsi128_si256(a),b,1) //将两个__mm128i 拼接成一个__mm256i a为low b为high


int main()
{
    unsigned char value = 0x10;
    unsigned char threshold = 0x09;
    __m256i values = _mm256_setr_epi8(value, value, value, value,
                                      value, value, value, value,
                                      value, value, value, value,
                                      value, value, value, value,
                                      value, value, value, value,
                                      value, value, value, value,
                                      value, value, value, value,
                                      value, value, value, value);

    __m256i thresholds = _mm256_setr_epi8(threshold, threshold, threshold, threshold,
                                          threshold, threshold, threshold, threshold,
                                          threshold, threshold, threshold, threshold,
                                          threshold, threshold, threshold, threshold,
                                          threshold, threshold, threshold, threshold,
                                          threshold, threshold, threshold, threshold,
                                          threshold, threshold, threshold, threshold,
                                          threshold, threshold, threshold, threshold);



    __m256i result = _mm256_max_epu8(values,thresholds);

    unsigned short shift = 14;
    unsigned short shift1 = 1 << shift;
    unsigned short W_R = 0.9 * shift1;



    __m256i WRC = _mm256_setr_epi8(10, 0, 0, 0, 10, 0, 0, 0,
                                    10, 0, 0, 0, 10, 0, 0, 0,
                                    10, 0, 0, 0, 10, 0, 0, 0,
                                    10, 0, 0, 0, 10, 0, 0, 0);
    __m256i SSE_WRC = _mm256_setr_epi16(W_R, 0, W_R, 0, W_R, 0, W_R, 0,
                                        W_R, 0, W_R, 0, W_R, 0, W_R, 0);

    __m256i ss = _mm256_madd_epi16(WRC, SSE_WRC);
}