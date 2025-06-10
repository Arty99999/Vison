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

    // 创建读取缓冲区
    unsigned char readBuffer[4];

    while (true)
    {
        sharedMemory.Read(4, readBuffer);
        int readNumber = 1000 * readBuffer[3] + 100 * readBuffer[2] + 10 * readBuffer[1] + readBuffer[0];
        std::cout << "The read number= " << readNumber << std::endl;
    }

    return 0;
}