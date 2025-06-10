#include <iostream>
#include <unistd.h>
#include "easy_logger.h"
#include "shared_memory.h"
#include "shared_memory_param.h"

int main()
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 创建共享内存参数
    SharedMemoryParam param;
    param.Size = 1000;
    param.Key = 4;

    // 创建并初始化共享内存
    SharedMemory sharedMemory;
    sharedMemory.SetParam(param);
    bool result = sharedMemory.Init();

    // 创建写入缓冲区
    unsigned char writerBuffer[4];
    unsigned char index = 1;

    while (true)
    {
        int writeNumber = 1111 * index;
        std::cout << "The write number = " << writeNumber << std::endl;
        writerBuffer[0] = index;
        writerBuffer[1] = index;
        writerBuffer[2] = index;
        writerBuffer[3] = index;
        sharedMemory.Write(4, writerBuffer);
        index++;
        if (index > 9)
        {
            index = 1;
        }
    }

    return 0;
}