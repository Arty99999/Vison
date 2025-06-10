//
// Created by plutoli on 2021/8/12.
//

#ifndef CUBOT_BRAIN_ROBOT_BRAIN_H
#define CUBOT_BRAIN_ROBOT_BRAIN_H

#include <atomic>
#include <mutex>
#include <vector>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "serial_port.h"
#include "robot_brain_param.h"
#include "robot_brain_core.h"
#include "robot_body_request.h"


/**
 * @brief 机器人大脑
 */
class RobotBrain
{
public:
    /**
     * @brief 析构函数
     */
    ~RobotBrain();

    /**
     * @brief 拷贝构造函数
     * @param[in] robotBrain 拷贝对象
     * @note 禁用拷贝构造函数
     */
    RobotBrain(const RobotBrain &robotBrain) = delete;

    /**
     * @brief 拷贝赋值运算符
     * @param[in] robotBrain 拷贝对象
     * @return 拷贝赋值结果
     * @note 禁用拷贝赋值运算符
     */
    RobotBrain& operator=(const RobotBrain &robotBrain) = delete;

    /**
     * @brief 移动构造函数
     * @param[in] robotBrain 移动对象
     * @note 禁用移动构造函数
     */
    RobotBrain(RobotBrain &&robotBrain) = delete;

    /**
     * @brief 移动赋值运算符
     * @param[in] robotBrain 移动对象
     * @return 移动赋值结果
     * @note 禁用移动赋值运算符
     */
    RobotBrain& operator=(RobotBrain &&robotBrain) = delete;

    /**
     * @brief 获取机器人大脑的单例引用
     * @return 机器人大脑的单例引用
     * @note 在此不返回机器人大脑的指针，避免用户使用delete指令导致对象被提前销毁
     */
    static RobotBrain& GetSingleInstance();

    /**
     * @brief 获取机器人大脑的通信串口引用
     * @return 机器人大脑的通信串口引用
     */
    SerialPort& GetSerialPort();

    /**
     * @brief 获取机器人大脑参数
     * @return 机器人大脑参数
     */
    RobotBrainParam GetParam();

    /**
     * @brief 设置机器人大脑参数
     * @param[in] param 机器人大脑参数
     * @return 设置结果\n
     *         -<em>false</em> 设置失败\n
     *         -<em>true</em> 设置成功\n
     */
    bool SetParam(const RobotBrainParam &param);

    /**
     * @brief 获取机器人大脑的存活状态
     * @return 机器人大脑的存活状态\n
     *         -<em>false</em> 已经销毁\n
     *         -<em>true</em> 尚且存活\n
     */
    bool IsAlive();

    /**
     * @brief 获取机器人大脑的扫描状态
     * @return 机器人大脑的扫描状态\n
     *         -<em>false</em> 停止扫描\n
     *         -<em>true</em> 正在扫描\n
     */
    bool IsScanning();

    /**
     * @brief 获取机器人大脑的初始化状态
     * @return 机器人大脑的初始化状态\n
     *         -<em>false</em> 尚未初始化\n
     *         -<em>true</em> 已经初始化\n
     */
    bool IsInitialized();

    /**
     * @brief 获取机器人大脑的打开状态
     * @return 机器人大脑的打开状态\n
     *         -<em>false</em> 尚未打开\n
     *         -<em>true</em> 已经打开\n
     */
    bool IsOpened();

    /**
     * @brief 获取机器人大脑的初始化时间戳
     * @return 机器人大脑的初始化时间戳
     */
    uint64_t GetInitTimestamp();

    /**
     * @brief 获取机器人大脑的打开时间戳
     * @return 机器人大脑的打开时间戳
     */
    uint64_t GetOpenTimestamp();

    /**
     * @brief 初始化机器人大脑
     * @return 初始化结果\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     */
    bool Init();

    /**
     * @brief 释放机器人大脑占用的系统资源
     * @return 资源释放结果\n
     *         -<em>false</em> 资源释放失败\n
     *         -<em>true</em> 资源释放成功\n
     */
    bool Release();

    /**
     * @brief 打开机器人大脑
     * @return 打开结果\n
     *         -<em>false</em> 打开失败\n
     *         -<em>true</em> 打开成功\n
     */
    bool Open();

    /**
     * @brief 关闭机器人大脑
     * @return 关闭结果\n
     *         -<em>false</em> 关闭失败\n
     *         -<em>true</em> 关闭成功\n
     */
    bool Close();

    /**
     * @brief 启动机器人大脑自检
     */
    void StartScan();

    /**
     * @brief 停止机器人大脑自检
     */
    void StopScan();

    /**
     * @brief 获取机器人大脑内核集合
     * @return 机器人大脑内核集合
     */
    std::vector<std::shared_ptr<RobotBrainCore>> GetRobotBrainCores();

    /**
     * @brief 将数据帧发送给机器人本体
     * @param[in] size    数据帧的字节长度
     * @param[in] frame   数据帧
     */
    void TransmitFrame(const unsigned int &size, const unsigned char *frame);

    /**
     * @brief 处理机器人本体请求
     * @param[in] request 机器人本体请求
     */
    void HandleRequest(const RobotBodyRequest &request);

private:
    RobotBrainParam param_;                                           ///< 机器人大脑参数
    std::atomic<bool> isAlive_;                                       ///< 机器人大脑的存活状态
    std::atomic<bool> isInitialized_;                                 ///< 机器人大脑的初始化状态
    std::atomic<bool> isOpened_;                                      ///< 机器人大脑的打开状态
    std::atomic<uint64_t> initTimestamp_;                             ///< 机器人大脑的初始化时间戳
    std::atomic<uint64_t> openTimestamp_;                             ///< 机器人大脑的打开时间戳
    std::mutex operateMutex_;                                         ///< 机器人大脑的操作互斥锁
    SerialPort serialPort_;                                           ///< 机器人大脑的通信串口
    std::vector<std::shared_ptr<RobotBrainCore>> robotBrainCores_;    ///< 机器人大脑内核集合
    std::atomic<bool> scanSwitch_;                                    ///< 机器人大脑的自检开关
    std::thread scanThread_;                                          ///< 机器人大脑的自检线程

    /**
    * @brief 构造函数
    * @note 设置构造函数为private，禁止用户自己声明并定义实例
    */
    RobotBrain();

    /**
     * @brief 机器人大脑自检，扫描通信串口和机器人大脑内核是否正常工作
     */
    void Scan();
};

// ******************************  RobotBrain类的回调函数  ******************************

/**
 * @brief 串口数据接收回调函数
 * @param[out] bytesToRead 串口接收到的字节数
 * @param[out] userData 回调函数关联的用户数据
 */
void HandleSerialPortDataReceived(const unsigned int &bytesToRead, void* userData);

#endif //CUBOT_BRAIN_ROBOT_BRAIN_H