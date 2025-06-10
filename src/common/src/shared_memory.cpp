//
// Created by plutoli on 2021/6/13.
//

#include <iostream>
#include <memory.h>
#include "shared_memory.h"

// ******************************  SharedMemory类的公有函数  ******************************

// 构造函数
SharedMemory::SharedMemory():
    param_(),
    isInitialized_(false),
    sharedMemoryID_(-1),
    semaphoreID_(-1),
    sharedMemoryAddress_(nullptr)
{
}

// 析构函数
SharedMemory::~SharedMemory()
{
    if (IsInitialized())
    {
        Release();
    }
}

// 获取共享内存参数
SharedMemoryParam SharedMemory::GetParam() const
{
    return param_;
}

// 设置共享内存参数
bool SharedMemory::SetParam(const SharedMemoryParam &param)
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断共享内存是否已经初始化
    if (isInitialized_)
    {
        log = "SharedMemoryParam was set failure because SharedMemory has been initialized";
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 记录共享内存参数
    param_ = param;

    // 记录日志信息
    log = "[" + std::to_string(param_.Key) + "] - SharedMemoryParam was set successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回设置结果
    return true;
}

// 获取共享内存的初始化状态
bool SharedMemory::IsInitialized() const
{
    return isInitialized_;
}

// 初始化共享内存
bool SharedMemory::Init()
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断共享内存是否已经初始化
    if (isInitialized_)
    {
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory can not be initialized repeatly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 排他性创建共享内存
    sharedMemoryID_ = ::shmget(param_.Key, param_.Size, ACCESS_BIT | IPC_CREAT | IPC_EXCL);
    if ((sharedMemoryID_ < 0) && (errno == EEXIST))
    {
        // 如果共享内存已经存在，则挂载共享内存；在挂载时，必须设置size=0
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory has been created";
        logger.Save(ELogType::Info, log);
        sharedMemoryID_ = ::shmget(param_.Key, 0, ACCESS_BIT | IPC_CREAT);
    }

    // 判断共享内存是否获取成功
    if (sharedMemoryID_ < 0)
    {
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory was got failure. "\
              "errno = " + std::to_string(errno);
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }
    else
    {
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory was got successful";
        logger.Save(ELogType::Info, log);
    }

    // 把共享内存连接到当前进程的地址空间
    if (sharedMemoryAddress_ == nullptr)
    {
        sharedMemoryAddress_ = static_cast<unsigned char *>(::shmat(sharedMemoryID_, nullptr, 0));
    }

    // 判断共享内存是否连接成功
    if (sharedMemoryAddress_ == (unsigned char *)-1)
    {
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory was attached failure. "\
              "errno = " + std::to_string(errno);
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }
    else
    {
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory was attached successful";
        logger.Save(ELogType::Info, log);
    }

    // 排他性创建信号量
    semaphoreID_ = ::semget(param_.Key,1,ACCESS_BIT | IPC_CREAT | IPC_EXCL);
    if (semaphoreID_ >= 0)
    {
        // TODO 该部分代码如果出现异常，可能会导致程序死锁。
        //      1、问题描述：如果在排他性创建信号量之后、新创建的信号量初始化成功之前出现异常，因为新创建的信号量
        //         没有赋予初值，将会导致后面的Semaphore_P()操作一直挂起，程序没有办法获取共享内存的读写权限。
        //      2、解决方案：可以将信号量的排他性创建和初始化做成原子操作。

        // 如果创建成功，初始化信号量
        union semun sem_union;
        sem_union.val = 1;
        if(::semctl(semaphoreID_, 0, SETVAL, sem_union) < 0)
        {
            log = "[" + std::to_string(param_.Key) + "] - Semaphore was set failure, please reboot computer. "\
                  "errno = " + std::to_string(errno);
            logger.Save(ELogType::Error, log);
            log = LOG_END;
            logger.Save(ELogType::Info, log);
            return false;
        }
        else
        {
            log = "[" + std::to_string(param_.Key) + "] - Semaphore was set successful";
            logger.Save(ELogType::Info, log);
        }
    }
    else
    {
        // 如果信号量已经存在，则挂载信号量
        if (errno == EEXIST)
        {
            semaphoreID_ = ::semget(param_.Key, 1, ACCESS_BIT | IPC_CREAT);
        }
    }

    // 判断信号量是否获取成功
    if (semaphoreID_ < 0)
    {
        log = "[" + std::to_string(param_.Key) + "] - Semaphore was got failure. "\
              "errno = " + std::to_string(errno);
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }
    else
    {
        log = "[" + std::to_string(param_.Key) + "] - Semaphore was got successful";
        logger.Save(ELogType::Info, log);
    }

    // 设置初始化标志
    isInitialized_ = true;

    // 记录日志信息
    log = "[" + std::to_string(param_.Key) + "] - SharedMemory was initialized successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回初始化结果
    return true;
}

// 释放共享内存资源
bool SharedMemory::Release()
{
    // 初始化日志信息，获取日志记录器的单例引用
    std::string log;
    EasyLogger &logger = EasyLogger::GetSingleInstance();

    // 记录日志信息
    log = LOG_BEGIN;
    logger.Save(ELogType::Info, log);

    // 判断共享内存是否已经初始化
    if (!isInitialized_)
    {
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory can not be released repeatly";
        logger.Save(ELogType::Warn, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }

    // 将共享内存从当前进程中分离
    if(::shmdt(sharedMemoryAddress_) < 0)
    {
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory was datached failure. "\
              "errno = " + std::to_string(errno);
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }
    else
    {
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory was datached successful";
        logger.Save(ELogType::Info, log);
    }

    // 删除共享内存
    if(::shmctl(sharedMemoryID_, IPC_RMID, nullptr) < 0)
    {
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory was deleted failure. "\
              "errno = " + std::to_string(errno);
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }
    else
    {
        log = "[" + std::to_string(param_.Key) + "] - SharedMemory was deleted successful";
        logger.Save(ELogType::Info, log);
    }

    // 删除信号量
    if (::semctl(semaphoreID_, 0, IPC_RMID) < 0)
    {
        log = "[" + std::to_string(param_.Key) + "] - Semaphore was deleted failure. "\
              "errno = " + std::to_string(errno);
        logger.Save(ELogType::Error, log);
        log = LOG_END;
        logger.Save(ELogType::Info, log);
        return false;
    }
    else
    {
        log = "[" + std::to_string(param_.Key) + "] - Semaphore was deleted successful";
        logger.Save(ELogType::Info, log);
    }

    // 重置私有数据
    isInitialized_ = false;
    sharedMemoryID_ = -1;
    semaphoreID_ = -1;
    sharedMemoryAddress_ = nullptr;

    // 记录日志信息
    log = "[" + std::to_string(param_.Key) + "] - SharedMemory was released successful";
    logger.Save(ELogType::Info, log);
    log = LOG_END;
    logger.Save(ELogType::Info, log);

    // 返回清理结果
    return true;
}

// 读取共享内存
size_t SharedMemory::Read(const size_t &readSize, unsigned char *readBuffer)
{
    // 判断是否满足读取条件
    if ((!isInitialized_) || (!Semaphore_P()))
    {
        return 0;
    }

    // 从共享内存中复制数据到读取缓冲区
    if (readSize < param_.Size)
    {
        ::memcpy(readBuffer, sharedMemoryAddress_, readSize);
    }
    else
    {
        ::memcpy(readBuffer, sharedMemoryAddress_, param_.Size);
    }

    // 释放共享内存权限
    Semaphore_V();

    // 返回实际读取的字节数
    return (readSize < param_.Size) ? readSize : param_.Size;
}

// 写入共享内存
size_t SharedMemory::Write(const size_t &writeSize, unsigned char *writeBuffer)
{
    // 判断是否满足写入条件
    if ((!isInitialized_) || (!Semaphore_P()))
    {
        return 0;
    }

    // 从写入缓冲区中复制数据到共享内存
    if (writeSize < param_.Size)
    {
        ::memcpy(sharedMemoryAddress_, writeBuffer, writeSize);
    }
    else
    {
        ::memcpy(sharedMemoryAddress_, writeBuffer, param_.Size);
    }


    // 释放共享内存权限
    Semaphore_V();

    // 返回写入结果
    return true;
}

// ******************************  SharedMemory类的私有函数  ******************************

// 信号量的P操作
bool SharedMemory::Semaphore_P() const
{
    // 判断共享内存是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    struct sembuf sem_buf;
    sem_buf.sem_num = 0;
    sem_buf.sem_op = -1;
    sem_buf.sem_flg = SEM_UNDO;

    return (::semop(semaphoreID_, &sem_buf, 1) >= 0);
}

// 信号量的V操作
bool SharedMemory::Semaphore_V() const
{
    // 判断共享内存是否初始化完毕
    if (!isInitialized_)
    {
        return false;
    }

    struct sembuf sem_buf;
    sem_buf.sem_num = 0;
    sem_buf.sem_op = 1;
    sem_buf.sem_flg = SEM_UNDO;

    return (::semop(semaphoreID_, &sem_buf, 1) >= 0);
}