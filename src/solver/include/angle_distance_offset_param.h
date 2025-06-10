//
// Created by xcx on 22-11-21.
//

#ifndef CUBOT_BRAIN_ANGLE_DISTANCE_OFFSET_PARAM_H
#define CUBOT_BRAIN_ANGLE_DISTANCE_OFFSET_PARAM_H
#include <vector>
/**
 *  �Զ����ݾ�����нǶȲ����Ĳ���
 */
 class AngleDistanceOffset{
 public:

     int max_iter;                     ///< ʹ�õ��������pitch����������������
     int R_K_iter;                     ///< ���������������ĵ�������
     float stop_error;                 ///< ֹͣ��������С���(��λm)
     float k;                          ///< ������еĿ���Ħ����
     float g;                          ///< �������ٶ�
     float bullet_speed;               ///< ��������ٶ�(��λm/s) ��ʵ�����������ж�ȡ����

     /**
     * @brief ���캯��
     */
     AngleDistanceOffset();

     /**
      * @brief ��������
      */
     ~AngleDistanceOffset() = default;
 };
#endif //CUBOT_BRAIN_ANGLE_DISTANCE_OFFSET_PARAM_H
