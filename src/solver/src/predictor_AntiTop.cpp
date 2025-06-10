//
// Created by ly on 23-12-3.
//

#include "predictor_AntiTop.h"
#include "solver.h"


AntiTop::AntiTop() :
        IsLocked(false),
        Ischoosearmor(false),
        IsrotateRight(false),
        matchNum(0),
        anglethreshold(20/57.3),
        longRadius(0),
        shortRadius(0),
        lockedShootArmor(),
        targetNumber(EClassicalArmorNumber::Invalid)
{}

AntiTop::~AntiTop() = default;

void AntiTop::init(){

}

//获取目标装甲板
std::vector<ClassicalArmor> AntiTop::getTargetArmor(std::vector<ClassicalArmor> &armors) {

    std::vector<ClassicalArmor> targetArmors;
    for (int i = 0; i < armors.size(); ++i)
    {
        if(armors[i].Number == targetNumber )
        {
            targetArmors.emplace_back(armors[i]);
        }
    }

    matchNum = targetArmors.size();

    return targetArmors;
}
//判断旋转方向
bool AntiTop::distinctOrientation(std::vector<ClassicalArmor> &last_armors,
                                  std::vector<ClassicalArmor> &now_armors) {
    //x坐标的差值
    //x坐标的差值

    float dx = 0;

    for (int i = 0; i < last_armors.size(); ++i) {
        dx += now_armors[i].RightLower.x - last_armors[i].RightLower.x;
    }

    return dx > 0;
}



//给装甲板编号
void AntiTop::giveIndexToArmor(std::vector<ClassicalArmor> &armors,
                               std::vector<double> R,
                               std::vector<int> *numberIndexs) {

    int number1;
    int number2;
    //长轴装甲板编号0，短轴装甲板编号1
    if (matchNum == 2)
    {
        number1= R[0] > R[1] ? 0 : 1;
        number2 = -number1 + 1;

        numberIndexs->emplace_back(number1);
        numberIndexs->emplace_back(number2);
    }
    //只有一块装甲板，则根据半径编号
    if (matchNum == 1 && IsLocked)
    {
        number1 = longRadius - R[0] > R[0] - shortRadius ? 0:1;
    }
}

//选择击打装甲板
ClassicalArmor AntiTop::chooseShootArmor(std::vector<ClassicalArmor> armors,
                                         float yaw1,
                                         float yaw2,
                                         int *index) {
    ClassicalArmor shootArmor;

    /** 装甲板装甲板状态
    * 0: 视野中仅有一块装甲板
    * 1: 未锁定装甲板且yaw1角小
    * 2: 未锁定装甲板且yaw2角小
    * 3: 已锁定装甲板且yaw1-阈值小于yaw2
    * 4: 已锁定装甲板且yaw2-阈值小于yaw1
    * 5: 锁定原来装甲板
    * @note 以上角度均为绝对值
    */
    int chooseState = -1;

    //未锁定状态，则直接判定状态为1/2
    if(!Ischoosearmor || last_state == 0)
    {
        chooseState = abs(yaw1) > abs(yaw2)? 2 : 1 ;
    }
        //处于已锁定状态，则按照chooseState注释编号
    else if ( last_state == 1 || last_state == 2)
    {
        if ( abs(yaw1) - anglethreshold < abs(yaw2))
            chooseState = 3;
        else if ( abs(yaw2) - anglethreshold < abs(yaw1))
            chooseState = 4;
        else
            chooseState = 5;
    }
    else
    {
        if (last_state == 4 && abs(yaw2) - abs(yaw1) > 20/57.3)
            chooseState = 3;
        else if (last_state == 3 && abs(yaw1) - abs(yaw2) > 20/57.3)
            chooseState = 4;
        else
            chooseState = last_state;
//        std::cout << chooseState <<std::endl;
    }

    //无论上方如何判断，若仅有一块装甲板，则直接锁定此装甲板
    if (matchNum == 1)
    {
        chooseState = 0;
    }
//    std::cout << yaw1 << "   " <<yaw2 <<std::endl;
    bool flag = abs(yaw1) <= abs(yaw2);
//    std::cout << chooseState <<std::endl;
    if (chooseState == 0 || chooseState == 1 || chooseState == 3) {
        *index = 0;
    }
    else if (chooseState == 2 || chooseState == 4) {
        *index = 1;
    }
    if (chooseState != 0) {
        float armor1 = armors[0].boxCenter.x;
        float armor2 = armors[1].boxCenter.x;
//    std::cout <<  *index << std::endl;
//    std::cout << armors[*index].boxCenter <<std::endl;
        //根据旋向判定锁定装甲板
//    else if (flag) {
//        *index = IsrotateRight ? 1 : 0;
//    }
//    else {
//        *index = IsrotateRight ? 0 : 1;
//    }
        if (*index = 0)
            shootArmor = armor1 > armor2 ? armors[1] : armors[0];
        else if (*index = 1)
            shootArmor = armor1 > armor2 ? armors[0] : armors[1];
    }
    else
        shootArmor = armors[0];
    if (!Ischoosearmor)
        Ischoosearmor = true;

    lockedShootArmor = shootArmor;
    last_state = chooseState;
    return shootArmor;
}

//判断上一帧指定装甲板到这一帧是哪块
//没写完
ClassicalArmor AntiTop::armorInherit(std::vector<ClassicalArmor> last_armors,

                                     ClassicalArmor armor) {

    float score0,score1;
    int orienscore0,orienscore1;
    int radiuscore0,radiuscore1;
    int distancescore0,distancescore1;
    ClassicalArmor armor1 = last_armors[0];
    ClassicalArmor armor2 = last_armors[1];
    float dx0 = armor.Center.x - armor1.Center.x;
    float dx1 = armor.Center.x - armor2.Center.x;

    if (IsrotateRight)
    {
        orienscore0 = dx0 > 0? 100:0;
        orienscore1 = dx1 > 0? 100:0;
    }
    else
    {
        orienscore0 = dx0 > 0? 0:100;
        orienscore1 = dx1 > 0? 0:100;
    }



}

//计算并返回车辆中心和长短轴半径
void AntiTop::computeCenter(std::vector<ClassicalArmor> armors,
                            std::vector<double>* R,
                            Eigen::Vector3d center1,
                            Eigen::Vector3d center2,
                            cv::Point2d *centerPoint,
                            float yaw1,float yaw2 ) {

    cv::Point2d centerLocation;

    //在陀螺仪坐标系下的xy二维平面上，yaw为负值时，斜率为-tan（1/yaw）；yaw为正值时，斜率为-tan（yaw）TODO 未验证
    yaw1 = yaw1 < 0 ?  yaw1: -yaw1;
    yaw2 = yaw2 < 0 ?  yaw2: -yaw2;
    float k1 = 1 / tan(yaw1);
    float k2 = 1 / tan(yaw2);
    //根据装甲板yaw和中心点计算车中心
    centerLocation.x = (k1 * center1(0,0) - k2 * center2(0,0) + center2(1,0) - center1(1,0)) / (k1 - k2);
    centerLocation.y = k1 * (centerLocation.x - center1(0,0)) + center1(1,0);
    centerPoint -> x = centerLocation.x;
    centerPoint -> y = centerLocation.y;
    //计算装甲板半径
    double r1 = sqrt(pow((center1(0,0)-centerLocation.x),2) + pow((center1(1,0)-centerLocation.y),2));
    double r2 = sqrt(pow((center2(0,0)-centerLocation.x),2) + pow((center2(1,0)-centerLocation.y),2));
    double lineLongth = sqrt(pow(center1.x() - center2.x(),2) + pow(center1.y() - center2.y() ,2 ));
    r1 = lineLongth * cos( CV_PI / 2 - abs(yaw1));
    r2 = lineLongth * cos(CV_PI / 2 - abs(yaw2));
//    std::cout << r1 << "  " << r2 << std::endl;

    longRadius = std::max(r1,r2);
    shortRadius = std::min(r1,r2);
    R -> emplace_back(r1);
    R -> emplace_back(r2);
}


//预测所有装甲板位姿
void AntiTop::predictAll(Eigen::Matrix<double,9,1> &Attitude,
                         std::vector<ClassicalArmor> &armors,
                         double predict_time,
                         std::vector<cv::Point2d> *centers,
                         std::vector<int> armorIndexs,
                         HuarayCameraModelParam &modelParam,
                         float yaw1 ,    float yaw2) {

    double x =Attitude(0, 0);
    double vx=Attitude(1, 0);
    double y =Attitude(2, 0);
    double vy=Attitude(3, 0);
    double w =Attitude(7, 0);



    double x_p = x + vx * predict_time;
    double y_p = y + vy * predict_time;

    if(matchNum == 1)
    {

        float radius0,radius1;

        radius0 = armorIndexs[0]? shortRadius:longRadius;
        radius0 = armorIndexs.size()? radius0:260;

        //
        cv::Point2d armorCPts;
        armorCPts.x = x_p + radius0 * cos(yaw1);
        armorCPts.y = y_p - radius0 * sin(yaw1);


        //TODO 把四个装甲板中心逆时针放到容器中
        //PredictorTool里面有将四块装甲板画出来的函数


    }


//    std::vector<Eigen::Vector3d> centerLocations ;
//    for (int i = 0; i < armors.size(); ++i) {
//
//        centerLocations.emplace_back(PredictorTool::getArmorCenterLocation(armors[i],modelParam));
//        cv::Point3f centerPt ;
//        PredictorTool::eigenVec3d2cvPt3f(centerLocations[i],&centerPt);
//        float yaw = Remap::findAccurateYaw(centerPt,modelParam,armors[i]);
//    }



}
