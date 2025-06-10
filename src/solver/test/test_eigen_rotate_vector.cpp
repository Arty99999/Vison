//
// Created by plutoli on 2022/2/8.
//

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;
#define pi 3.14159265359

// 参考网址：https://www.cnblogs.com/lovebay/p/11215028.html
//         https://blog.csdn.net/qq_35097289/article/details/94002101
int main()
{
    cout << "##-------------------搞清旋转关系-------------------##" << endl;
    Eigen::Vector3d v1(1, 0, 0);
    Eigen::AngleAxisd angle_axis1(-pi / 2, Eigen::Vector3d(0, 0, 1));
    // Eigen::Vector3d rotated_v1 = angle_axis1.matrix().inverse()*v1;
    Eigen::Vector3d rotated_v1 = angle_axis1.matrix()*v1;
    cout << "旋转矩阵: " << endl << angle_axis1.matrix() << endl;
    cout << "(1, 0, 0)旋转后: " << endl << rotated_v1.transpose() << endl;
    cout << "------------------------------------------------------" << endl;

    return 0;
}