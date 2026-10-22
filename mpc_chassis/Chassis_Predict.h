#ifndef CHASSIS_PREDICT_H
#define CHASSIS_PREDICT_H

typedef struct
{
    Chassis_speed world_speed;
    Chassis_speed body_speed;
    Wheel_speed Wheel_speed;
}Chassis_predoct;


typedef struct
{
    float vx;
    float vy;
    float w;
}Chassis_speed;

typedef struct 
{
    float lf;
    float rf;
    float rr;
    float lr;
}Wheel_speed;


#endif
