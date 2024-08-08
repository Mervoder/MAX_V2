/*
 * mahony.c
 *
 *  Created on: Aug 5, 2024
 *      Author: oguzk
 */


#include "mahony.h"
#include <math.h>

#define SAMPLE_FREQ 33.3f  // Sample frequency in Hz
#define TWO_KP_DEFAULT (2.0f * 0.5f) // 2 * proportional gain
#define TWO_KI_DEFAULT (2.0f * 0.1f) // 2 * integral gain

// Auxiliary variables to avoid repeated arithmetic
static float recipNorm;
static float halfvx, halfvy, halfvz;
static float halfex, halfey, halfez;

void Mahony_Init(MahonyAHRS* mahony) {
    mahony->q0 = 1.0f;
    mahony->q1 = 0.0f;
    mahony->q2 = 0.0f;
    mahony->q3 = 0.0f;
    mahony->twoKp = TWO_KP_DEFAULT;
    mahony->twoKi = TWO_KI_DEFAULT;
    mahony->integralFBx = 0.0f;
    mahony->integralFBy = 0.0f;
    mahony->integralFBz = 0.0f;
}

void Mahony_Update(MahonyAHRS* mahony, float gx, float gy, float gz, float ax, float ay, float az) {
    // Normalize accelerometer measurement
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = mahony->q1 * mahony->q3 - mahony->q0 * mahony->q2;
    halfvy = mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3;
    halfvz = mahony->q0 * mahony->q0 - 0.5f + mahony->q3 * mahony->q3;

    // Error is cross product between estimated direction and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Apply integral feedback if enabled
    if (mahony->twoKi > 0.0f) {
        mahony->integralFBx += mahony->twoKi * halfex * (1.0f / SAMPLE_FREQ); // integral error scaled by Ki
        mahony->integralFBy += mahony->twoKi * halfey * (1.0f / SAMPLE_FREQ);
        mahony->integralFBz += mahony->twoKi * halfez * (1.0f / SAMPLE_FREQ);
        gx += mahony->integralFBx;  // apply integral feedback
        gy += mahony->integralFBy;
        gz += mahony->integralFBz;
    }
    else {
        mahony->integralFBx = 0.0f; // prevent integral windup
        mahony->integralFBy = 0.0f;
        mahony->integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += mahony->twoKp * halfex;
    gy += mahony->twoKp * halfey;
    gz += mahony->twoKp * halfez;

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / SAMPLE_FREQ));   // pre-multiply common factors
    gy *= (0.5f * (1.0f / SAMPLE_FREQ));
    gz *= (0.5f * (1.0f / SAMPLE_FREQ));
    float qa = mahony->q0;
    float qb = mahony->q1;
    float qc = mahony->q2;
    mahony->q0 += (-qb * gx - qc * gy - mahony->q3 * gz);
    mahony->q1 += (qa * gx + qc * gz - mahony->q3 * gy);
    mahony->q2 += (qa * gy - qb * gz + mahony->q3 * gx);
    mahony->q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    recipNorm = 1.0f / sqrtf(mahony->q0 * mahony->q0 + mahony->q1 * mahony->q1 + mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3);
    mahony->q0 *= recipNorm;
    mahony->q1 *= recipNorm;
    mahony->q2 *= recipNorm;
    mahony->q3 *= recipNorm;

    // Convert quaternion to Euler angles
    mahony->roll = atan2f(2.0f * (mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3), 1.0f - 2.0f * (mahony->q1 * mahony->q1 + mahony->q2 * mahony->q2)) * 180.0f / M_PI;
    mahony->pitch = asinf(2.0f * (mahony->q0 * mahony->q2 - mahony->q3 * mahony->q1)) * 180.0f / M_PI;
    mahony->yaw = atan2f(2.0f * (mahony->q0 * mahony->q3 + mahony->q1 * mahony->q2), 1.0f - 2.0f * (mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3)) * 180.0f / M_PI;
}
