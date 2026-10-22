#ifndef KALMAN_H
#define KALMAN_H

#include <stdio.h>

#define Kalman_Q 0.01f
#define Kalman_R 0.01f

typedef struct
{
    float x;  // 当前估计位置
    float p;  // 当前估计方差
    float q;  // 过程噪声方差
    float r;  // 测量噪声方差
} kalman_t;

extern kalman_t filter_x;
extern kalman_t filter_y;

void Kalman_Init(kalman_t *filter, float X, float P, float Q, float R);
void kalman_predict(kalman_t *filter, float velocity, float dt);
float kalman_update(kalman_t *filter, float measurement);

#endif
