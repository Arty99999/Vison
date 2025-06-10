//
// Created by plutoli on 2021/12/28.
//

#ifndef CUBOT_BRAIN_ROBOT_BRAIN_CORE_H
#define CUBOT_BRAIN_ROBOT_BRAIN_CORE_H

#include <atomic>
#include <mutex>
#include <vector>
#include <opencv2/opencv.hpp>
#include "easy_logger.h"
#include "huaray_camera.h"
#include "classical_armor_recognizer.h"
#include "classical_windmill_recognizer.h"
#include "solver.h"
#include "robot_brain_core_param.h"
#include "robot_buff_data.h"
#include "robot_fight_data.h"
#include "robot_brain_control_command.h"
#include "robot_brain_notify_command.h"
#include "robot_brain_core_statistics.h"
#include "predictor.h"
#include "classical_windmill_predictor.h"
#include "classcal_windmill_cerese_predictor.h"


/**
 * @brief 机器人大脑内核
 */
class RobotBrainCore
{
public:
    /**
    * @brief 构造函数
    */
    RobotBrainCore();

    /**
     * @brief 析构函数
     */
    ~RobotBrainCore();

    /**
     * @brief 拷贝构造函数
     * @param[in] robotBrainCore 拷贝对象
     * @note 禁用拷贝构造函数
     */
    RobotBrainCore(const RobotBrainCore &robotBrainCore) = delete;

    /**
     * @brief 拷贝赋值运算符
     * @param[in] robotBrainCore 拷贝对象
     * @return 拷贝赋值结果
     * @note 禁用拷贝赋值运算符
     */
    RobotBrainCore& operator=(const RobotBrainCore &robotBrainCore) = delete;

    /**
     * @brief 移动构造函数
     * @param[in] robotBrainCore 移动对象
     * @note 禁用移动构造函数
     */
    RobotBrainCore(RobotBrainCore &&robotBrainCore) = delete;

    /**
     * @brief 移动赋值运算符
     * @param[in] robotBrainCore 移动对象
     * @return 移动赋值结果
     * @note 禁用移动赋值运算符
     */
    RobotBrainCore& operator=(RobotBrainCore &&robotBrainCore) = delete;

    /**
     * @brief 获取机器人大脑内核的华睿相机引用
     * @return 华睿相机的引用
     */
    HuarayCamera& GetCamera();

    /**
     * @brief 获取机器人大脑内核的经典装甲板识别器引用
     * @return 经典装甲板识别器的引用
     */
    ClassicalArmorRecognizer& GetClassicalArmorRecognizer();

    /**
     * @brief 获取机器人大脑内核的经典风车识别器引用
     * @return 经典风车识别器的引用
     */
    ClassicalWindmillRecognizer& GetClassicalWindmillRecognizer();

    /**
     * @brief 获取机器人大脑内核的目标解算器引用
     * @return 目标解算器的引用
     */
    Solver& GetSolver();

    /**
     * @brief 获取机器人大脑内核参数
     * @return 机器人大脑内核参数
     */
    RobotBrainCoreParam GetParam();

    /**
     * @brief 设置机器人大脑内核参数
     * @param[in] param 机器人大脑内核参数
     * @return 设置结果\n
     *         -<em>false</em> 设置失败\n
     *         -<em>true</em> 设置成功\n
     * @note 如果机器人大脑内核已经打开，参数将会设置失败
     */
    bool SetParam(const RobotBrainCoreParam &param);

    /**
     * @brief 获取机器人大脑内核的统计信息
     * @return 机器人大脑内核的统计信息
     * @note 机器人大脑内核的统计信息的更新频率为1Hz
     */
    RobotBrainCoreStatistics GetStatistics();

    /**
     * @brief 获取机器人大脑内核的战斗数据
     * @param[out] fightData 机器人大脑内核的战斗数据
     */
    void GetFightData(RobotFightData *fightData);

    /**
     * @brief 获取机器人大脑内核的初始化状态
     * @return 机器人大脑内核的初始化状态\n
     *         -<em>false</em> 尚未初始化\n
     *         -<em>true</em> 已经初始化\n
     */
    bool IsInitialized();

    /**
     * @brief 获取机器人大脑内核的打开状态
     * @return 机器人大脑内核的打开状态\n
     *         -<em>false</em> 尚未打开\n
     *         -<em>true</em> 已经打开\n
     */
    bool IsOpened();

    /**
     * @brief 获取机器人大脑内核的暂停状态
     * @return 机器人大脑内核的暂停状态\n
     *         -<em>false</em> 尚未暂停\n
     *         -<em>true</em> 已经暂停\n
     */
    bool IsPaused();

    /**
     * @brief 获取机器人大脑内核的初始化时间戳
     * @return 机器人大脑内核的初始化时间戳
     */
    uint64_t GetInitTimestamp();

    /**
     * @brief 获取机器人大脑内核的打开时间戳
     * @return 机器人大脑内核的打开时间戳
     */
    uint64_t GetOpenTimestamp();

    /**
     * @brief 初始化机器人大脑内核
     * @return 初始化结果\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     * @note Init()和Release()配套使用
     */
    bool Init();

    /**
     * @brief 释放机器人大脑内核占用的系统资源
     * @return 资源释放结果\n
     *         -<em>false</em> 资源释放失败\n
     *         -<em>true</em> 资源释放成功\n
     * @note Init()和Release()配套使用
     */
    bool Release();

    /**
     * @brief 打开机器人大脑内核
     * @return 打开结果\n
     *         -<em>false</em> 打开失败\n
     *         -<em>true</em> 打开成功\n
     * @note Open()和Close()配套使用
     */
    bool Open();

    /**
     * @brief 关闭机器人大脑内核
     * @return 关闭结果\n
     *         -<em>false</em> 关闭失败\n
     *         -<em>true</em> 关闭成功\n
     * @note Open()和Close()配套使用
     */
    bool Close();

    /**
     * @brief 机器人大脑内核暂停工作
     * @return 暂停结果\n
     *         -<em>false</em> 暂停失败\n
     *         -<em>true</em> 暂停成功\n
     * @note Pause()和Resume()配套使用
     */
    bool Pause();

    /**
     * @brief 机器人大脑内核恢复工作
     * @return 恢复结果\n
     *         -<em>false</em> 恢复失败\n
     *         -<em>true</em> 恢复成功\n
     * @note Pause()和Resume()配套使用
     */
    bool Resume();

    /**
     * @brief 切换机器人大脑内核的工作模式
     * @param[in] workMode  机器人大脑内核的工作模式
     * @return 切换结果\n
     *         -<em>false</em> 切换失败\n
     *         -<em>true</em> 切换成功\n
     */
    bool SwitchWorkMode(const EWorkMode &workMode);

    /**
     * @brief 切换机器人大脑内核目标解算器补偿参数适配的子弹速度
     * @param[in] bulletVelocity    机器人大脑内核目标解算器补偿参数适配的子弹速度
     * @return 切换结果\n
     *         -<em>false</em> 切换失败\n
     *         -<em>true</em> 切换成功\n
     */
    bool SwitchBulletVelocity(const EBulletVelocity &bulletVelocity);

private:
    RobotBrainCoreParam param_;                                 ///< 机器人大脑内核参数
    HuarayCamera huarayCamera_;                                 ///< 机器人大脑内核使用的华睿相机
    ClassicalArmorRecognizer classicalArmorRecognizer_;         ///< 机器人大脑内核使用的经典装甲板识别器
    ClassicalWindmillRecognizer classicalWindmillRecognizer_;   ///< 机器人大脑内核使用的经典风车识别器
    Solver solver_;                                             ///< 机器人大脑内核使用的目标解算器
    Predictor predictor_;                                       ///<<机器人大脑内核使用的目标预测器
    Fitting fitting_;                                           ///< 机器人大脑内核使用的能量机关预测器
    Fitcoordinate Fit_;                                         ///< 机器人大脑内核使用的能量机关预测器2
    std::atomic<bool> isInitialized_;                           ///< 机器人大脑内核的初始化状态
    std::atomic<bool> isOpened_;                                ///< 机器人大脑内核的打开状态
    std::atomic<bool> isPaused_;                                ///< 机器人大脑内核的暂停状态
    std::atomic<uint64_t> initTimestamp_;                       ///< 机器人大脑内核的初始化时间戳
    std::atomic<uint64_t> openTimestamp_;                       ///< 机器人大脑内核的打开时间戳
    std::mutex operateMutex_;                                   ///< 机器人大脑内核的操作互斥锁
    std::mutex detectMutex_;                                    ///< 机器人大脑内核的识别互斥锁
    RobotBrainCoreStatistics statistics_;                       ///< 机器人大脑内核的统计信息
    mutable std::shared_mutex statisticsMutex_;                 ///< 机器人大脑内核的统计信息读写锁
    RobotFightData fightData_;                                  ///< 机器人大脑内核的战斗数据
    mutable std::shared_mutex fightDataMutex_;                  ///< 机器人大脑内核的战斗数据读写锁
    RobotBuffData buffData_;                                    ///< 机器人大脑内核的Buff数据
    mutable std::shared_mutex buffDataMutex_;                   ///< 机器人大脑内核的Buff数据读写锁
    std::atomic<bool> detectionSwitch_;                         ///< 机器人大脑内核的识别检测指令开关
    std::thread detectionThread_;                               ///< 机器人大脑内核的识别检测线程
    std::atomic<bool> controlBodySwitch_;                       ///< 机器人大脑内核的控制指令发布开关
    std::thread controlBodyThread_;                             ///< 机器人大脑内核的控制指令发布线程
    std::atomic<bool> notifyBodySwitch_;                        ///< 机器人大脑内核的提示指令发布开关
    std::thread notifyBodyThread_;                              ///< 机器人大脑内核的提示指令发布线程
    std::atomic<bool> fittingSwitch_;                           ///< 机器人能量机关预测开关
    std::thread fittingThread_;                                 ///< 机器人能量机关预测线程
    std::queue<std::vector<ClassicalArmor>> detectArmorQueue_;  ///< 机器人大脑内核装甲板识别队列
    std::condition_variable detectArmorCV_;                     ///< 机器人大脑内核装甲板识别条件变量
    std::vector<cv::Point2f> lu;
    std::vector<cv::Point2f> ld;
    std::vector<cv::Point2f> ru;
    std::vector<cv::Point2f> rd;

    /**
     * @brief 机器人大脑内核向机器人本体发布控制指令
     */
    void ControlBody();

    /**
     * @brief 机器人大脑内核向机器人本体发布提示指令
     */
    void NotifyBody();

    /**
     * @brief 机器人大脑内核识别装甲板
     */
    void DetectArmor();


    /**
    * @brief 机器人大脑内核通过经典的OpenCV加上卡尔曼滤波方法预测在战斗模式下的击打目标点参数
    * @param[in] huarayCameraData          华睿相机数据
    * @param[in] robotBrainCoreParam       机器人大脑内核参数
    * @param[out] predictWorldCoordinate    预测打击点在世界坐标系下的坐标
    * @param[out] predictCradleHeadCoordinate    预测打击点在云台坐标系下的坐标
    * @param[out] armorRecord              装甲板队列
    * @param[out] package
    * @return 是否找到击打目标点\n
    *         -<em>false</em> 尚未找到击打目标点\n
    *         -<em>true</em> 已经找到击打目标点\n
    */
    bool ComputeTargetInFightByYolo_EKF(const HuarayCameraData &huarayCameraData,
                                            const RobotBrainCoreParam &robotBrainCoreParam,
                                            Eigen::Vector3d  *predictCradleHeadCoordinate,
                                            Eigen::Vector3d  *predictWorldCoordinate,
                                            Eigen::Vector3d  *predictCradleHeadCoordinate1,
                                            Eigen::Vector3d  *predictWorldCoordinate1,
                                            bool &packageResult,
                                            bool &isFire,
                                            std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber>> *armorRecord,
                                            std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64>> *armorSequence,
                                            PredictorTool *package,
                                            double &r);

    bool ComputeTargetInFightByYolo_outpost(const HuarayCameraData &huarayCameraData,
                                            const RobotBrainCoreParam &robotBrainCoreParam,
                                            Eigen::Vector3d  *predictCradleHeadCoordinate,
                                            Eigen::Vector3d  *predictWorldCoordinate,
                                            Eigen::Vector3d  *predictCradleHeadCoordinate1,
                                            Eigen::Vector3d  *predictWorldCoordinate1,
                                            bool &packageResult,
                                            bool &isFire,
                                            std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber>> *armorRecord,
                                            std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64>> *armorSequence,
                                            PredictorTool *package,
                                            double &r);





    cv::Point2f MeanFiltering_LU(cv::Point2f LU);

    cv::Point2f MeanFiltering_LD(cv::Point2f LD);

    cv::Point2f MeanFiltering_RU(cv::Point2f RU);

    cv::Point2f MeanFiltering_RD(cv::Point2f RD);
};

#endif //CUBOT_BRAIN_ROBOT_BRAIN_CORE_H