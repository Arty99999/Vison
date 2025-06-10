//
// Created by gusha on 23-12-3.
//

#ifndef CUBOT_BRAIN_CLASSCAL_WINDMILL_CERESE_PREDICTOR_H
#define CUBOT_BRAIN_CLASSCAL_WINDMILL_CERESE_PREDICTOR_H

#include "classical_windmill_logo.h"
#include "classical_windmill_fan.h"
#include "classical_windmill_recognizer.h"
#include "solver.h"
#include "predictor_tool.h"

    enum RunMode{

        BIG_RUN = 1,           // 大符模式
        SMALL_RUN = 2          // 小符模式

    };

    // 创建速度和时间戳的类型
    struct W_Time_data
    {
        float omiga;
        float time;

        W_Time_data (double w = 0.0, float t = 0)
        {
            omiga = w;
            time = t;
        }
    };

    //采样拟合函数
    //           sqd = A * sin ( w * t + t0) + b
    //           param[0] is A  ||  param[1] is w  ||param[2] is t0

    class sincostfunction{
    public:

        sincostfunction(double _spd,double _t): spd(_spd),t(_t) {};

        template <class T>
        bool operator()(const T* const param, T* residual) const {

            residual[0] = spd - (

                    param[0] * sin ( param[1] * t + param[2]) + 2.090 - param[0]

            );
            return true;
        }

    private:
        const double spd;
        const double t;
    };

    class Fitcoordinate{

    public:

        bool                                            IsTargetReady ;                 //目标点是否识别
        bool                                            IsLogoReady ;                   //Logo点是否识别
        bool                                            IsClockwise ;                   //是否顺时针旋转
        bool                                            IsParam_ok ;                    //Param是否准备好
        uint64_t                                        delaytime ;                     //延迟击打时间  单位 ms
        uint64_t                                        Startime ;                      //初始时间
        RunMode                                         Mode ;                          //大小符选择
        ClassicalWindmillLogo                           ValidLogo;                      //合法Logo
        ClassicalWindmillFan                            ValidTarget;                    //合法目标中心点
        bool                                            isclockwiseOk;
        float                                           time_gap;                       //延迟开火标志位时间


        static std::vector<int> test;

        void ttest (){
            std::cout << " test " << test[0] <<std::endl;
        };

        /**
         * 构造函数
         * @param mode
         * @param delayT
         * @param isClockwise
         */
        Fitcoordinate ( RunMode mode = BIG_RUN , uint64_t delayT = 300 , bool isClockwise = true );

        ~ Fitcoordinate () = default;

        /**
         * @brief 对Logo中心点进行判断是否合法
         * @param[in] Logo      Logo的point坐标
         * @param[in] Logos     Logo坐标的存储容器
         * @param[out] ValidLogo
         */
        static bool _2DComputeLogoHOP ( ClassicalWindmillLogo Logo , std::vector <ClassicalWindmillLogo> &ValidLogos);


        /**
         * @brief 对Logo中心点进行判断是否合法
         * @param Logo
         * @param ValidLogos
         * @param QMatrix
         * @param modelParam
         * @return
         */
        static bool _3DComputeLogoHOP ( ClassicalWindmillLogo Logo , std::vector <ClassicalWindmillLogo> &ValidLogos , Eigen::Matrix3d QMatrix , const HuarayCameraModelParam &modelParam);

        /**
         * @brief 返回二维点的距离
         * @param a
         * @param b
         * @return
         */
        static float ComputeDistance ( cv::Point2f a ,cv::Point2f b );

        /**
         * @brief 返回小能量机关击打点（像素坐标系）
         * @param ValidLogo     合法Logo的point坐标
         * @param ValidTarget   合法target的point坐标
         * @param calpoint      像素坐标系击打点
         */
         bool Fit_SmallWindmill ( cv::Point2f &calpoint );

        /**
         * @brief 更新角速度时序元素
         * @param angles        存储检测时Target与Logo的角度
         * @param w_Timestamps  存储检测时的时间
         * @param sin_datas     存储Target位置时序元素
         * @return
         */
         bool GetSpeed_Data ( std::vector<float> &angles , std::vector<uint64_t> &w_Timestamps ,std::vector<W_Time_data> &sin_datas );

        /**
         * @brief 对位置时序元素进行加权平均
         * @param datas                 要平均的位置时序元素
         * @param[out] callbake_datas   平均后的位置时序元素
         * @param B                     B=0.5时相当于对两个点进行平均 即1/（1 - B）个点 平均太多后有延迟
         */
        static void Avg_weighted ( std::vector<W_Time_data>  datas, std::vector<W_Time_data> &callbake_datas , float B = 0.5);

        /**
         * @brief 拟合大符的旋转参数
         * @param sin_datas     存储Target位置时序元素
         * @param param         存储大符旋转参数
         * @param Mode
         * @return
         */
        static bool Fit_Data2Param (  std::vector<W_Time_data> sin_datas, double param[3], bool IsShow = false, RunMode Mode = BIG_RUN );

        /**
         * @brief 给出大符的预测点坐标 （像素坐标系）
         * @param[out] calpoint      像素坐标系击打点
         * @param w_Timestamps       存储检测时的时间
         * @param param              存储大符旋转参数
         * @return
         */
        bool Fit_BigWindmill ( cv::Point2f &calpoint , std::vector<uint64_t> w_Timestamps , double param[3]);

        /**
         * @brief 计算延时delaytime后的角度
         * @param sin_param         存储大符旋转参数
         * @param Delaytime         延迟击打时间
         * @return
         */
        float call_param2roll_angle (double sin_param[3] , float Delaytime ){

            return  -(sin_param[0] / sin_param[1]) * cos(sin_param[1] * Delaytime + sin_param[2])
                    + (2.09 - sin_param[0]) * Delaytime
                    + (sin_param[0] / sin_param[1]) * cos(sin_param[2]);
        }
        /**
         * @brief 运行目标点预测器
         * @param angles                存储检测时Target与Logo的角度
         * @param w_Timestamps          存储检测时的时间
         * @param sin_datas             存储Target位置时序元素
         * @param param                 存储大符旋转参数
         * @param modelParam            相机参数
         * @param[out] Pre_point        像素坐标预测点
         * @param[out] TargetCoordinate 云台坐标预测点
         */
        void Run ( std::vector<float> &angles , std::vector<uint64_t> &w_Timestamps ,std::vector<W_Time_data> &sin_datas , double *param , HuarayCameraModelParam modelParam,cv::Point2f &Pre_point ,Eigen::Vector3d &Pre_Coordinate);

        /**
         * @brief 给出大符的转向
         * @param[in] angleChange      两帧角度变化值
         */
        bool GetIsClockwise ( float angleChange );


    };


#endif //CUBOT_BRAIN_CLASSCAL_WINDMILL_CERESE_PREDICTOR_H
