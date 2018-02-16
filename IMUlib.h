//
// Created by artem on 15.02.18.
//

#ifndef PYIMUMODULEFORC_IMULIB_H
#define PYIMUMODULEFORC_IMULIB_H

#include <python3.5/Python.h>
#include <math.h>

#define PI 3.14159265358979f
#define G  9.80665f
#define GYROMEASERROR PI*(5.f/180.f)
#define BETA sqrt(3.f/4.f)*GYROMEASERROR
#define A_ADRESS   0x53
#define A_POWER_CTL 0x2D  //Power Control Register
#define A_DATA_FORMAT   0x31
#define A_POINT_DATA 0x32
#define G_ADRESS 0x68
#define G_DLPF_FS 0x16 // full-scale
#define G_PWR_MGM 0x3E //
#define G_POINT_DATA 0x1D

typedef struct quaternion       // структура квартерниона
{
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct vector           // структура вектора
{
    float x;
    float y;
    float z;
} Vector;

Vector quatToEulerAngle(Quaternion q);      // переводит квартернион в углы Эйлера
Quaternion scale(Quaternion q, float val);  // умножает все параметры квартерниона на число
Quaternion normalize(Quaternion q);         // нормализация квартерниона
Quaternion inverse(Quaternion q);           // инверсия квартерниона
Quaternion mul(Quaternion q1, Quaternion q2);   // умножение двух квартернионов
Quaternion summ(Quaternion q1, Quaternion q2);  // сумма двух квартернионов
Quaternion VecToQuat(Vector v);             // перевод вектора в квартернион
Quaternion updateFilterIterator(Quaternion q, Vector a, Vector w, float beta, float deltaT);    // фильтр
void HPFilterIterator(Quaternion inValue, Quaternion oldInValue, Quaternion* outValue, float deltaT, float RC);
void LPFilterIterator(Quaternion inValue, Quaternion* outValue, float Kp);


#endif //PYIMUMODULEFORC_IMULIB_H
