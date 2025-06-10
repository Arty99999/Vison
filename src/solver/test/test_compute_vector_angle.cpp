//
// Created by plutoli on 2022/2/8.
//

#include <cmath>
#include <iostream>

// 二维向量的点乘和叉乘参考网址：
// https://www.cnblogs.com/zhoug2020/p/7508722.html
// https://blog.csdn.net/qq_39534332/article/details/100170970
int main()
{
    double a_x = -1.0;
    double a_y = 1.0;
    double b_x = 1.0;
    double b_y = 1.0;
    double c_x = 1.0;
    double c_y = -1.0;

    double dotProduct1 = b_x * a_x + b_y * a_y;
    double crossProduct1 = b_x * a_y - b_y * a_x;
    double angle1 = atan2(crossProduct1, dotProduct1);

    double dotProduct2 = b_x * c_x + b_y * c_y;
    double crossProduct2 = b_x * c_y - b_y * c_x;
    double angle2 = atan2(crossProduct2, dotProduct2);

    std::cout << "angle1: " << angle1 << std::endl;
    std::cout << "angle2: " << angle2 << std::endl;

    return 0;
}
