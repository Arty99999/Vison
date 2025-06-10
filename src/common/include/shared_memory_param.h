//
// Created by plutoli on 2021/12/31.
//

#ifndef CUBOT_BRAIN_SHARED_MEMORY_PARAM_H
#define CUBOT_BRAIN_SHARED_MEMORY_PARAM_H

#include <sys/types.h>

/**
 * @brief 共享内存参数
 */
class SharedMemoryParam
{
public:
    /**
     * @brief 共享内存和数据同步信号量的键
     * @note 默认值为1
     */
    key_t Key;

    /**
     * @brief 共享内存的字节长度
     * @note 默认值为1024
     */
    size_t Size;

    /**
     * @brief 构造函数
     */
    SharedMemoryParam();

    /**
     * @brief 析构函数
     */
    ~SharedMemoryParam() = default;
};

#endif //CUBOT_BRAIN_SHARED_MEMORY_PARAM_H