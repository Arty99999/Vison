//
// Created by ly on 23-12-3.
//

#ifndef CUBOT_BRAIN_PREDICTOR_ANTITOP_H
#define CUBOT_BRAIN_PREDICTOR_ANTITOP_H

#include "predictor_EKF.h"

/**
 * @brief   反陀螺预测类
 * @note    配合EKF代码一起使用
 */
class AntiTop{


    //TODO 如果是平衡步兵如何预测？
public:
    /**
     *  构造函数
     */
    AntiTop();

    /**
     *  析构函数
     */
    ~AntiTop();

    /**
     *  成员初始化
     */
    void init();

    /**
     *  释放缓存
     */
    void release();

    /**
     * @brief   判断是否初始化
     * @return
     */
    bool IsInit();

    /**
     * @brief 获取目标数字标号的装甲板
     * @param armors 捕捉到的所有装甲板
     * @return 目标装甲板集合
     */
    std::vector<ClassicalArmor> getTargetArmor(std::vector<ClassicalArmor> &armors);

    /**
     * @brief 判断陀螺旋向
     * @note 只判断两帧的方向，建议多使用几次来判断整体方向
     * @param last_armors 上一帧的装甲板集合
     * @param now_armors 这一帧的装甲版集合
     */
    bool distinctOrientation(std::vector<ClassicalArmor> &last_armors,
                             std::vector<ClassicalArmor> &now_armros);

    /**
     * @brief 给装甲板编号(长轴0，短轴1)
     * @param armors 装甲版集合
     * @param R 半径集合(和armors一一对应)
     * @param numberIndexs 编号集合(和armors一一对应)
     */
    void giveIndexToArmor(std::vector<ClassicalArmor> &armors,
                          std::vector<double> R,
                          std::vector<int>* numberIndexs);

    /**
     * @brief 从目标装甲板集合中选取待击打装甲板
     * @param armors  目标装甲板集合
     * @param yaw1  第一块装甲板的yaw值
     * @param yaw2  第二块装甲板的yaw值(default = 0)
     * @return 待击打装甲板
     */
    ClassicalArmor chooseShootArmor(std::vector<ClassicalArmor> armors,
                                    float yaw1,
                                    float yaw2 = 0,
                                    int *index = NULL);

    /**
     * @brief 判断上一帧装甲板继承到这一帧为哪块(是否为同一块装甲板)
     * @param last_armors 识别到的两块目标装甲板组成的向量
     * @param armor 被继承装甲板
     * @return 继承的装甲板
     */
    ClassicalArmor armorInherit(std::vector<ClassicalArmor> last_armors,
                                ClassicalArmor armor);




    /**
     * @brief 通过这一帧计算目标车辆的中心坐标
     * @param[in] armors 识别到的两块目标装甲板组成的向量
     * @param[out] R 存放输出装甲板的半径(和armors一一对应)
     * @param[in] center1 第一块装甲板中心的陀螺仪坐标值
     * @param[in] center2 第二块装甲板中心的陀螺仪坐标值
     * @param[in] yaw1 第一块装甲板的yaw值
     * @param[in] yaw2 第二块装甲板的yaw值
     * @param[out] centerPoint 目标车辆中心点坐标
     * @note 装甲板中心点和yaw的顺序必须和armors中的顺序一致
     */
    void computeCenter(std::vector<ClassicalArmor> armors,
                       std::vector<double>* R,
                       Eigen::Vector3d center1,
                       Eigen::Vector3d center2 ,
                       cv::Point2d* centerPoint,
                       float yaw1, float yaw2 = 0);

    /**
     * TODO 直接写到predict里会不会更好？
     * @brief 预测一定时间后目标车辆所有装甲板的位姿
     * @param Attitude 目标车辆的位姿(x,vx,y,vy,ω) TODO 换成EKF的输出？
     * @param armors 识别到的两块目标装甲板组成的向量
     * @param predict_time 需要的预测时间
     * @param centers 预测四块装甲板组成的向量
     */
    void predictAll(Eigen::Matrix<double,9,1> &Attitude,
                    std::vector<ClassicalArmor> &armors,
                    double predict_time,
                    std::vector<cv::Point2d> *centers,
                    std::vector<int> armorIndexs,
                    HuarayCameraModelParam &modelParam,
                    float yaw1 ,    float yaw2 = 0);


    bool IsLocked;                                ///< 是否锁定目标中心
    bool Ischoosearmor;                           ///< 是否已经锁定装甲板
    bool IsrotateRight;                           ///< 陀螺是否右旋(否则为左旋)
    int matchNum;                                 ///< 目标装甲板捕捉数量
    int last_state;
    float anglethreshold;                         ///< 转换装甲板时的角度阈值
    double longRadius;                            ///< 陀螺长轴
    double shortRadius;                           ///< 陀螺短轴
    ClassicalArmor lockedShootArmor;              ///< 锁定的装甲板
    EClassicalArmorNumber targetNumber;           ///< 目标装甲板数字标号


};

#endif //CUBOT_BRAIN_PREDICTOR_ANTITOP_H
