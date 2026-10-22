#include "kalman.h"

kalman_t filter_x;
kalman_t filter_y;

void Kalman_Init(kalman_t *filter, float X, float P, float Q, float R)
{
    filter->p = P;
    filter->q = Q;
    filter->r = R;
    filter->x = X;
}

/**
 * x_prior = x + velocity * dt
 * P_prior = P + Q
 */
void kalman_predict(kalman_t *filter, float velocity, float dt)
{
    filter->x += velocity * (dt / 1000.0f);
    filter->p += filter->q;
}

/**
 *  @brief measurement 观测值
 */
float kalman_update(kalman_t *filter, float measurement)
{
    float innovation;        //残差
    float K;                 //卡尔曼增益

    innovation = measurement - filter->x;

    K = filter->p / (filter->p + filter->r);

    filter->x += K * innovation;        //计算最佳值
    filter->p = (1.0f - K) * filter->p; //更新方差

    return filter->x;
}
