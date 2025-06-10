//
// Created by tony on 2022/4/23.
//

#ifndef CUBOT_BRAIN_PREDICTOR_H
#define CUBOT_BRAIN_PREDICTOR_H


#include "predictor_model.h"
#include "predictor_EKF.h"
#include "predictor_tool.h"
#include "predictor_param.h"
#include "predictor_KF.h"
#include "predictor_AntiTop.h"
#include "solver.h"
#include "easy_logger.h"
#include <fstream>
///<  在头文件里 同一个同文件不能引用两次

/**
 * @brief 预测器
 * @note 从ekf中得到位置与速度的最优估计，
 * @note 之后位置加上速度乘以预测时间得到预测位置
 */
class Predictor {

public:

    /**
     * @brief 构造函数
     */
    Predictor();

    /**
     * @brief 析构函数
     */
    ~Predictor() = default;

    /**
     * @brief 预测器初始化函数
     */
    bool Init();

    /**
     * @brief 预测器释放函数
     */
    bool Release();

    /**
     * @brief 预测器判断初始化
     */
    bool IsInitialized_();

    /**
     * @brief 预测器参数
     * @param[in] param 预测器参数
     * @return 预测器参数设置结果\n
     *         -<em>false</em> 设置失败\n
     *         -<em>true</em> 设置成功\n
     */
    bool SetParam(PredictorParam &predictorParam);

    /**
     * @brief 得到预测器当前参数
     */
    PredictorParam GetParam();

     /**
     * @brief 自瞄移动预测实现函数
     *@param[in]  isPredict   是否预测
     *@param[in]  predictor   预测器
     *@param[in]  modelParam  相机参数
     *@param[in]  armorRecord  记忆的装甲板队列
     *@param[in]  predict_time  预测时间
     *@param[out] predictrelativeCoordinate  imu坐标系下的打击坐标
     *@param[out]  predictWorldCoordinate   世界坐标系下打击点坐标
     *@param[out]  predictpixel             预测打击点的像素坐标
     */
    void AutoaimPredict(bool &isPredict,
                        Predictor &predictor,
                        HuarayCameraModelParam &modelParam,
                        std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber>> *armorRecord,
                        std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64_t>> *ArmorsAndYaws,
                        double &predict_time,
                        Eigen::Vector3d *predictCradleHeadCoordinate,
                        Eigen::Vector3d *predictWorldCoordinate,
                        Eigen::Vector3d *predictCradleHeadCoordinate1,
                        Eigen::Vector3d *predictWorldCoordinate1,
                        cv::Point2f *predictpixel,
                        const Eigen::Matrix<double, 3, 3>& rotationMatrix,
                        cv::Point2f &Get1, cv::Point2f &Get2,
                        cv::Point2f &Get3, cv::Point2f &Get4,
                        int stand,
                        bool &isFire,
                        double &r);

    void AutoaimPredict_outpost(bool &isPredict,
                                Predictor &predictor,
                                HuarayCameraModelParam &modelParam,
                                std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>,EClassicalArmorNumber>> *armorRecord,
                                std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64_t>> *ArmorsAndYaws,
                                double &predict_time,
                                Eigen::Vector3d *predictCradleHeadCoordinate,
                                Eigen::Vector3d *predictWorldCoordinate,
                                Eigen::Vector3d *predictCradleHeadCoordinate1,
                                Eigen::Vector3d *predictWorldCoordinate1,
                                cv::Point2f *predictpixel,
                                cv::Point2f *predictpixelCenter,
                                const Eigen::Matrix<double, 3, 3>& rotationMatrix,
                                cv::Point2f &Get1, cv::Point2f &Get2,
                                cv::Point2f &Get3, cv::Point2f &Get4,
                                bool is_okFire,
                                bool &isFire,
                                double &r);
    /**
     * @brief 反陀螺预测函数
     */
    void AntiTop(HuarayCameraModelParam modelParam,
                 std::deque<std::tuple<std::vector<ClassicalArmor>,std::vector<float>,uint64>> *armorDataSequence,
                 std::vector<float> Distances,
                 cv::Point2d *centerLocation,
                 std::vector<int> *numberIndex,
                 std::vector<double> *Radi);

    /**
     * @brief 线性运动滤波器
     */
    void FightLinearTarget(double &gapTime,
                           PredictorModel &PredictFunc,
                           Eigen::Vector3d &Worldcoordinate,
                           double euler);

   /**
    * @brief 运动预测函数
    */
   void FightTarget(double &gapTime,
                          HuarayCameraModelParam &modelParam,
                          Eigen::Vector3d &nowTarget,
                          double &predict_time,
                          Eigen::Vector3d *predictTarget);

   /**
    * @brief 给私有元素赋值
    * @param index
    */
   void setPrivateMember(int index,
                         bool dual_flag = true);

    /**
     * @brief 击打前哨站预测函数
     */
    void FightOutPost(bool isPredict,
                       Predictor *predictor,
                       HuarayCameraModelParam modelParam,
                       std::deque<std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>>> *armorRecord,
                       double predict_time,
                       cv::Point2f *pixelTarget);

    /**
     * @brief 得到缓存的四元数与其在系统下的时间戳以及此时刻目标距离
     */
    bool GetQuaterniondSequence(std::vector<std::tuple<std::array<float, 4>, uint64_t, float>> *QuaterniondSequence);

    /**
     * @brief 风车预测实现函数
     */
    void WindmillPredict();

    /**
     * @brief 预测缓冲区更新函数
     */
    bool UpdateLocationBuffer(const PredictorTool &predictorTool);

    /**
     * @brief 得到预测点在图像上的坐标
     */
     void Getpiexl(HuarayCameraModelParam &modelParam,std::tuple<uint64_t, float, cv::Point2f, std::array<float, 4>> &nowTarget,
                   Eigen::Vector3d predictRelativeCoordinate,cv::Point2f *pixelTarget);

     void GetAntitop(class AntiTop Anti);

     void UpdateAntitop(class AntiTop *Anti);
private:

    /**
     * @brief 刷新预测器缓冲区
     */
    void RefreshPredictorBuffer(const PredictorTool &package,
                                       std::deque<PredictorTool> *predictorBuffer) const;

    std::mutex operateMutex_;                           ///< 预测器的操作互斥锁
    std::mutex bufferMutex_;
    PredictorParam param_;                              ///< 预测器参数
    std::atomic<bool> isInitialized_;                   ///< 预测器的初始化状态
    std::atomic<uint64_t> initTimestamp_;               ///< 预测器的初始化时间戳
    std::deque<PredictorTool> predictorBuffer_;         ///< 预测器的缓存区
    AutoaimEKF AutoaimEKF_;                             ///< 预测器的卡尔曼滤波器
    class AntiTop AntiTop_;                             ///< 预测器的反陀螺工具
    double stateCount;
    double last_z;                                      ///<
    bool dualarmor_first;                               ///< 是否首次出现两块装甲板
    bool ori_dict;                                      ///< 是否已经判定方向
    bool armorHop;                                      ///< 装甲板是否跳变
    bool armornumchange;                                ///< 装甲板数目是否变化
    int armorIndex;                                     ///< 被选择装甲板在容器中的索引
    int count = 0;
    int count1 = 0;
};

#endif //CUBOT_BRAIN_PREDICTOR_H
