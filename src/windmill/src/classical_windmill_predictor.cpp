//
// Created by whales on 23-3-18.
//

#include "classical_windmill_predictor.h"

bool Fitting::run(ClassicalWindmillFan &armor, Eigen::Vector3d &LogoCenter, Eigen::Vector3d *nextPoint)
{
    std::lock_guard<std::mutex> lockGuard(predictMutex_);
    switch (runeMode)
    {
        case Mode::RUNE:
            WindmillPredict(armor, LogoCenter, nextPoint);
            break;
        case Mode::NORMAL_RUNE:
            NormalWindmillPredict(armor, LogoCenter, nextPoint);
            break;
    }
    return true;
}

void Fitting::CalWorldCoordinate(Eigen::Vector3d CameraCoordinate, Eigen::Vector3d LogoCameraCoordinate, double *realDistanceTarget)
{
    double Angle;
    // x1 * x2 + y1 * y2 + z1 * z2 / sqrt * sqrt
    double pointMultiplication  = CameraCoordinate.x() * LogoCameraCoordinate.x() + CameraCoordinate.y() * LogoCameraCoordinate.y() + CameraCoordinate.z() * LogoCameraCoordinate.z();
    double model = sqrt(CameraCoordinate.x() * CameraCoordinate.x() + CameraCoordinate.y() * CameraCoordinate.y() + CameraCoordinate.z() * CameraCoordinate.z());
    double modelLogo = sqrt(LogoCameraCoordinate.x() * LogoCameraCoordinate.x() + LogoCameraCoordinate.y() * LogoCameraCoordinate.y() + LogoCameraCoordinate.z() * LogoCameraCoordinate.z());
    Angle = acos(pointMultiplication / ( model * modelLogo));
    double ratio = 680 / sin(Angle);
    double AngleB = asin(7000 / ratio);
    double deltaAngle = M_PI - Angle - AngleB;
    *realDistanceTarget = sin(deltaAngle) * ratio;
}

double Fitting::calDistance(Eigen::Vector3d &Point_1, Eigen::Vector3d &Point_2)
{
    double n_x = Point_1.x() - Point_2.x();
    double n_y = Point_1.y() - Point_2.y();
    double n_z = Point_1.z() - Point_2.z();

    return sqrt(n_x * n_x + n_y * n_y + n_z * n_z);
}

void Fitting::LinearInterpolation(SpeedTime newData)
{
    if(fittingData.empty())
    {
        fittingData.emplace_back(newData);
        std::cout << "First angleSpeed: " << newData.angleSpeed << std::endl;
//        std::cout << "First fittingData was emplace back" << std::endl;
        return;
    }
    SpeedTime flag_data = fittingData[fittingData.size() - 1];                              //最新拟合数据
//    std::cout<< "timeInterval:" <<  - (double)timeInterval * 1000000000.0<<std::endl;
//    std::cout<< "newData.time - flag_data:" << ((double)newData.time - (double)flag_data.time) / 1000000 << std::endl;      // 时间差
    if ((double)newData.time - (double)flag_data.time - timeInterval * 1000000000.0 < 0)   // 若new_data与flag_data位于待拟合一侧时，返回
    {
//        std::cout << "Data time sampling does not meet the requirements" << std::endl;
        return;

    }

    double T = 1000000000 * timeInterval;           // 10ms
    double n = ((double)newData.time - (double)flag_data.time) / T;     // 现在的待预测与上一次的待预测的时间进行相减
//    std::cout << "n: "<< n<<std::endl;
    if (n > 50)                     // 若new_data与flag_data的时间相差500ms时，清理数据，重新开始10000000
    {
//        std::cout << "The gap is too large. Clear the data" << std::endl;
        clearData();
        return;
    }
// 此处需要进行测试，判断哪种方法是准确的，线性插值比较准确
// 一、
//    for (int i = 0; i < (int)n; i++)
//    {
//        double temp_T = T * (i + 1);
//        double delta = (double)newData.time - (double)flag_data.time - temp_T;          // 当前靶心的时间戳减去拟合成功的最后一个的时间戳再减去
//        SpeedTime temp = SpeedTime(newData.angleSpeed * (temp_T / (temp_T + delta)) + flag_data.angleSpeed * (delta / (temp_T + delta)), flag_data.time + (uint64_t)temp_T);
//        fittingData.push_back(temp);
//        std::cout << "Data has been fitted" << std::endl;
//
//    }

// 二、
    for(int i = 0; i < (int)n; i++)
    {
        double delta = (double)newData.time - fittingData[fittingData.size() - 1].time - T;
        SpeedTime temp = SpeedTime(newData.angleSpeed * (T / (T + delta)) + fittingData[fittingData.size() - 1].angleSpeed * (delta / (T + delta)),
                                   fittingData[fittingData.size() - 1].time + (uint64_t)T);
//        std::cout << "LinearAngle: " << temp.angleSpeed << std::endl;
        fittingData.push_back(temp);
//        std::cout << "Data has been fitted" << std::endl;
    }

//    std::cout << "fittingDataSize: " << fittingData.size() << std::endl;
}

double Fitting::calPalstance(ClassicalWindmillFan &armor_1, ClassicalWindmillFan &armor_2)
{
    // 两帧图像时间差
    double gapTime = (double)(armor_1.Timestamp - armor_2.Timestamp) / 1000000000.0;

    // 两帧图像角度差
    double angle_diff = armor_2.Angle - armor_1.Angle;
//    std::cout << "AngleDiff: " << angle_diff << std::endl;

    // 进行角度差调整
    if (armor_1.Angle > CV_PI / 2.0 && armor_2.Angle < -CV_PI / 2.0)
    {
        angle_diff = angle_diff + CV_PI * 2.0;
    }
    else if (armor_1.Angle < -CV_PI / 2.0 && armor_2.Angle > CV_PI / 2.0)
    {
        angle_diff = angle_diff - CV_PI * 2.0;
    }

//    std::cout << "Speed: " << angle_diff / gapTime << std::endl;
    // 返回角速度
//    std::cout << "angle: " << angle_diff << std::endl;
//    std::cout << "gaptime: " << gapTime << std::endl;
    double Speed = angle_diff / gapTime;
    if(isnan(Speed))
    {
        Speed = pastSpeed;
    }
    pastSpeed = Speed;
//    std::cout << "Speed: " << Speed << std::endl;
    return Speed;
}

double Fitting::calAngle(Eigen::Vector3d &armorCenter, Eigen::Vector3d &logoCenter)
{
    // 求出Logo与装甲板待击打点的向量；V = (x1 - x2, y1 - y2, z1 - z2)；方向Logo指向装甲板待击打点
    double n_x = armorCenter.x() - logoCenter.x();
    double n_y = armorCenter.y() - logoCenter.y();
    double n_z = armorCenter.z() - logoCenter.z();

    // 求上述向量与z轴夹角；采用叉乘进行计算
//    double Cross_x = n_y;
//    double Cross_y = -n_x;
//    double Cross_z = 0;
//    double CrossAngle = asin(sqrt(Cross_x * Cross_x + Cross_y * Cross_y + Cross_z * Cross_z) / sqrt(n_z * n_z + n_y * n_y + n_x * n_x));
    // 求上述向量与z轴夹角；Angle = V·Z / |V|·|Z|
    double Angle = acos(n_z / sqrt(n_z * n_z + n_y * n_y + n_x * n_x));

    // 通过与x轴向量相乘，增加角度的范围
    if(!IsPositive)
    {
        Angle = - Angle;
    }

//    std::cout << "CalAngle: " << Angle * 180 / M_PI << std::endl;
    return Angle;
}

Eigen::Vector3d Fitting::calNextPosition(Eigen::Vector3d &armorCenter,
                                         Eigen::Vector3d &logoCenter,
                                         double &rotate_angle)
{
    // 计算半径
    double radius = calDistance(armorCenter, logoCenter);
//    std::cout << "Radius:" << radius << std::endl;

    // 计算世界坐标系下的装甲板待击打点相对Logo的坐标
    double n_x = armorCenter.x() - logoCenter.x();
    double n_y = armorCenter.y() - logoCenter.y();
    double n_z = armorCenter.z() - logoCenter.z();
    Eigen::Vector3d relative_point(n_x, n_y, n_z);

    // 计算当前装甲板与z轴的夹角
    double relative_angle = calAngle(armorCenter, logoCenter);
    double next_angle;
    if(runeMode == Mode::NORMAL_RUNE || runeMode == Mode::RUNE)
    {
        if(relative_angle > 0)
        {
            if(relative_angle < M_PI / 2)
            {
                double deltaAngle = 5.0 * (radius + radius * cos(relative_angle)) / (2 * radius) * M_PI / 180;
                rotate_angle = rotate_angle + deltaAngle;
            }
            else if(relative_angle < (M_PI * 3 /4))
            {
                double deltaAngle = -10.0 * (radius + radius * cos(relative_angle)) / (2 * radius) * M_PI / 180;
                rotate_angle = rotate_angle + deltaAngle;
            }
            else
            {
                rotate_angle = rotate_angle + 5.0 * M_PI / 180;
            }
        }
        else
        {
            if(relative_angle > (- M_PI / 2))
            {
                double deltaAngle = 10.0 * (radius + radius * cos(relative_angle)) / (2 * radius) * M_PI / 180;
                rotate_angle = rotate_angle + deltaAngle;
            }
            else if(relative_angle > (-M_PI * 3 /4))
            {
                double deltaAngle = 10.0 * (radius + radius * cos(relative_angle)) / (2 * radius) * M_PI / 180;
                rotate_angle = rotate_angle + deltaAngle;
            }
            else
            {
                rotate_angle = rotate_angle + 5.0 * M_PI / 180;
            }
        }
    }
//    if(std::abs(rotate_angle) < (M_PI / 4))
//    {
//        double deltaAngle = 10 * (radius + radius * cos(relative_angle)) / (2 * radius) * M_PI / 180;
//        rotate_angle = rotate_angle + deltaAngle;
//    }
//    else
//    {
//        double deltaAngle = 5 * (radius + radius * cos(relative_angle)) / (2 * radius) * M_PI / 180;
//        rotate_angle = rotate_angle + deltaAngle;
//    }
//    std::cout << "rotate_angle: " << rotate_angle << std::endl;
//    std::cout << "IsColockWise: " << isClockwise << std::endl;

    if (isClockwise) // 顺时针运动
    {
//        next_angle = relative_angle + rotate_angle;
//        if (next_angle > CV_PI)
//            next_angle -= 2.0 * CV_PI;
        next_angle = relative_angle - rotate_angle;
        if (next_angle < -CV_PI)
            next_angle += 2.0 * CV_PI;
    }
    else
    {
        next_angle = relative_angle + rotate_angle;
        if (next_angle > CV_PI)
            next_angle -= 2.0 * CV_PI;
//        next_angle = relative_angle - rotate_angle;
//        if (next_angle < -CV_PI)
//            next_angle += 2.0 * CV_PI;
//        std::cout << "RotateAngle: " << rotate_angle * 180 / M_PI << std::endl;
//        std::cout << "NextAngle: " << next_angle * 180 / M_PI << std::endl;
    }
//    std::cout << "RotateAngle: " << rotate_angle * 180 / M_PI << std::endl;
//    std::cout << "NextAngle: " << next_angle * 180 / M_PI << std::endl;

    // 计算出当前在xoy平面上的投影与x轴的夹角、与y轴的夹角
    double nowAngle = acos(std::abs(n_x) / sqrt(n_x * n_x + n_y * n_y));      // 为正锐角
//    std::cout << "nowAngle: " << nowAngle << std::endl;
//    std::cout << "NX: " << n_x << std::endl;

    // 计算出预测后半径在xoy平面上的投影、x轴上的投影、y轴上的投影
    double projectionLength_xy = radius * sin(next_angle);              // 为负数
    double projectionLength_x = projectionLength_xy * cos(nowAngle);    // 为负数
    double projectionLength_y = projectionLength_xy * sin(nowAngle);

    // 通过预测出的偏转角，计算出预测点的世界坐标系
    Eigen::Vector3d nextPoint;
//    std::cout << "NY: " << n_y << std::endl;

    // X方向正负
    if(IsXPositive)
    {
        nextPoint.x() = projectionLength_x + logoCenter.x();
    }
    else
    {
        nextPoint.x() = -projectionLength_x + logoCenter.x();
    }

    // Y方向正负
    if(IsYPositive)
    {
        nextPoint.y() = projectionLength_y + logoCenter.y();
    }
    else
    {
        nextPoint.y() = -projectionLength_y + logoCenter.y();
    }

    // Z方向坐标
    nextPoint.z() = cos(next_angle) * radius + logoCenter.z();

    return nextPoint;
}

void Fitting::wFitting()
{
    double nMin = 1.884 / (2 * CV_PI) * N;
    double nMax = 2.000 / (2 * CV_PI) * N;
    double deltaN = (nMax - nMin) / WN;

    double maxIndex = -1;
    double maxValue = FLT_MIN;
    double value;

    for(unsigned int i = 0; i < WN + 1; i++)
    {
        double FittingN = nMin + deltaN * i;
//        std::cout << "FittingN: " << FittingN << std::endl;
        value = get_F(FittingN);
//        std::cout << "Value: " << value << std::endl;
        if(value > maxValue)
        {
            maxValue = value;
            maxIndex = (double)i;
        }
    }

    _w = (maxIndex * deltaN + nMin) / (double)N * 2 * CV_PI;
//    std::cout << "_W: " << _w << std::endl;
//    std::cout << "maxIndex: " << maxIndex << std::endl;
    _a = maxValue / N * 2;
//    std::cout << "Value: " << value << std::endl;

    if (_a > 1.045)
        _a = 1.045;
    else if (_a < 0.780)
        _a = 0.780;
//    std::cout << "A: " << _a << std::endl;
}

void Fitting::tFitting()
{
    double maxIndex = -1;
    double maxValue = FLT_MIN;
    double value;

    for(unsigned int i = 0; i < t0_N + 1 ; i++)
    {
        value = get_integral((double)i * max_T0 / t0_N);
        if(value > maxValue)
        {
            maxIndex = i;
            maxValue = value;
        }
    }

    _t0 = maxIndex * max_T0 / t0_N;
    startTime = fittingData[0].time;
}

void Fitting::clearData()
{
    fittingData.clear();
    buffArmor_.clear();
}

void Fitting::SteeringJudgment()
{
    double recordSize = fittingData.size();
    double clockwise = 0;
    double anticlockwise = 0;
    if(recordSize > 50)
    {
        for(unsigned int i = 0; i < recordSize; i++)
        {
            if(fittingData[i].angleSpeed > 0)
            {
                clockwise++;
            }
            else
            {
                anticlockwise++;
            }
        }
        if(clockwise > anticlockwise)
        {
            isClockwise = true;
        }
        else
        {
            isClockwise = false;
        }
        isDirection = true;
    }
}

void Fitting::FittingCurve()
{
    if(fittingData.empty())
        return;
    double recordSize = fittingData.size();
    if(fittingData[recordSize - 1].time - fittingData[0].time > (N - 1) * timeInterval * 1000000000 - 1)
    {
        wFitting();
        if(isnan(_a))
        {
            _a = 0.9;
            _w = 1.9;
            _t0 = 0.0;
            clearData();
            return;
        }

        tFitting();
        isCurve = true;
    }
//    std::cout << "w:  " << _w << "a:  " << _a << std::endl;
}

void Fitting::calOffsetAngle(uint64_t &time, double *deltaAngle)
{
    double gaptime = (double)(time - startTime) / 1000000000;

    // 积分后求角度差
    *deltaAngle = (_a / _w) * ( - cos(_w * (gaptime + _t0 + delayTime)) + cos(_w * (gaptime + _t0))) + (2.090 - _a) * delayTime;
//    std::cout << "DELTAANGLE: " << (*deltaAngle) * 180 / M_PI << std::endl;
}

void Fitting::WindmillPredict(ClassicalWindmillFan &armor, Eigen::Vector3d &logoCenter, Eigen::Vector3d *nextPoint)
{
    double deltaAngle;

    SteeringJudgment();
    if(isDirection)
    {
        FittingCurve();
        if(isCurve)
        {
            calOffsetAngle(armor.Timestamp, &deltaAngle);
//            deltaAngle = deltaAngle + 10.0 * M_PI /180 * (_a * sin(_w * (armor.Timestamp + _t0 + delayTime)) + 2.090 - _a) / 2.090;
//            double offsetDeltaAngle = 25.0 / 180 * std::abs((std::abs(armor.Angle) - M_PI));
//            deltaAngle = deltaAngle + offsetDeltaAngle;
            *nextPoint = calNextPosition(armor.EigenTargetCenter, logoCenter, deltaAngle);
        }
        else
        {
            deltaAngle = CV_PI / 3 * delayTime;
            *nextPoint = calNextPosition(armor.EigenTargetCenter, logoCenter, deltaAngle);
        }
    }

}

void Fitting::NormalWindmillPredict(ClassicalWindmillFan &armor, Eigen::Vector3d &logoCenter, Eigen::Vector3d *nextPoint)
{
    double deltaAngle;

    SteeringJudgment();
//    std::cout << "SteeringJudgment: " << isDirection << std::endl;
    if(isDirection)
    {
        deltaAngle = CV_PI / 3 * delayTime;
//        std::cout << "deltaAngle: " << deltaAngle << std::endl;
//        std::cout << "armorAngle: " << armor.Angle << std::endl;
//        double offsetDeltaAngle = 20.0 / 180 * std::abs((std::abs(armor.Angle) - M_PI));
//        std::cout << "offsetAngle: " << offsetDeltaAngle << std::endl;
//        deltaAngle = deltaAngle + offsetDeltaAngle;
//        std::cout << deltaAngle << std::endl;
        *nextPoint = calNextPosition(armor.EigenTargetCenter, logoCenter, deltaAngle);
    }

}

bool Fitting::JudgeCoordinate(Eigen::Vector3d &RightUpperPoint, Eigen::Vector3d &WorldLogoCoordinate)
{
    double n_x = RightUpperPoint.x() - WorldLogoCoordinate.x();
    double n_y = RightUpperPoint.y() - WorldLogoCoordinate.y();

    // 与X正半轴夹角
    double AngleX = acos(n_x / sqrt(n_x * n_x + n_y * n_y ));
    if(AngleX > M_PI / 2)
    {
        IsXPositive = false;
    }
    else
    {
        IsXPositive = true;
    }
    // 与Y正半轴夹角
    double AngleY = acos(n_y / sqrt(n_x * n_x + n_y * n_y ));
    if(AngleY > M_PI / 2)
    {
        IsYPositive = false;
    }
    else
    {
        IsYPositive = true;
    }
    std::cout << "IsX: " << IsXPositive << std::endl;
    std::cout << "IsY: " << IsYPositive << std::endl;
}

void Fitting::DataStatus(ClassicalWindmillFan &armor)
{
    switch(armorStatus)
    {
        case EClassicalStatus::First:

            // 当开始识别风车装甲板时，对装甲板缓存和角速度缓存进行清零操作
            if(armor.Timestamp)
                clearData();
            buffArmor_.emplace_back(armor);
//            std::cout << "First Windmill was recognized" << std::endl;
            break;

        case EClassicalStatus::Shoot:

            buffArmor_.emplace_back(armor);
//            std::cout << "Windmill was play Shoot status" << std::endl;
            // 通过逐差法求得角速度，若符合要求进行线性插值
            while(buffArmor_.size() > DN)
            {
                double deltaTime = (double)(armor.Timestamp - buffArmor_[0].Timestamp) /  1000000.0;
                double newDeltaTime;
//                std::cout << "deltaTime: " << deltaTime <<std::endl;
                // 一定条件符合后，进行速度线性插值
                if(deltaTime > 100)
                {
                    do
                    {
                        buffArmor_.erase(buffArmor_.begin());
                        newDeltaTime = (double)(armor.Timestamp - buffArmor_[0].Timestamp) / 1000000.0;
//                        std::cout << "buffArmor was erased" << std::endl;
//                        std::cout << "now deltaTime:" << newDeltaTime << std::endl;
//                        std::cout << "now: " << (newDeltaTime > 30.0) << std::endl;
                    }while(newDeltaTime > 100.0);
//                    std::cout << "Armor: " << armor.Angle << std::endl;
//                    std::cout << "ArmorAngle: " << buffArmor_[0].Angle << std::endl;
                    LinearInterpolation(SpeedTime(calPalstance(armor, buffArmor_[0]), (armor.Timestamp + buffArmor_[0].Timestamp) / 2));
//                    std::cout<< "return"<<std::endl;
                    break;
                }
                if(deltaTime >5)
                {
//                    std::cout << "bufferArmor was LinearInterpolation" << std::endl;
                    LinearInterpolation(SpeedTime(calPalstance(armor, buffArmor_[0]), (armor.Timestamp + buffArmor_[0].Timestamp) / 2));
                    break;
                }
                else
//                    std::cout << "Not qualified, continue to join" << std::endl;
                    break;
            }
            break;

        case EClassicalStatus::Lost:

            // 若存在掉帧现象，对当前缓存进行清零操作
            clearData();
            break;

        case EClassicalStatus::Invalid:

            // 若未识别到装甲板，进行？操作
            break;

    }

    // 实际采样数目大于预采样数目，清除第一帧
    while(fittingData.size() > N)
    {
        fittingData.erase(fittingData.begin());
    }
}

void Fitting::call_param() {

    std::cout << " A =  " << this->_a << std::endl;
    std::cout << " w =  " << this->_w << std::endl;
    std::cout << " t0 =  " << this->_t0 << std::endl;
}
