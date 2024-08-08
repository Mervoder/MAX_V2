/*
 * mahony.h
 *
 *  Created on: Aug 5, 2024
 *      Author: oguzk
 */

#ifndef MAHONY_H
#define MAHONY_H

typedef struct {
    float q0, q1, q2, q3; // Quaternion components representing orientation
    float roll, pitch, yaw; // Euler angles representing orientation
    float twoKp;  // 2 * proportional gain (Kp)
    float twoKi;  // 2 * integral gain (Ki)
    float integralFBx, integralFBy, integralFBz;  // Integral feedback terms
} MahonyAHRS;

void Mahony_Init(MahonyAHRS* mahony);
void Mahony_Update(MahonyAHRS* mahony, float gx, float gy, float gz, float ax, float ay, float az);

#endif
