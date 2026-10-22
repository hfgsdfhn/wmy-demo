/**
 * @file Chassis_Control.c
 * @author your name (you@domain.com)
 * @brief 使用MPC控制器进行底盘实时曲线跟踪，参考中科大开源方案https://bbs.robomaster.com/wiki/8562105/566830?source=7
 * @details 
 * 
 * 状态量：X = {x,y,0,vx,vy,w}
 *      x,y:平面坐标
 *      0：yaw
 *      vx,vy:速度
 *      w:yaw角速度
 * 控制量 U = {Fx,Fy,Mz}
 *      Fx,Fy:力    Mz:扭矩
 * @version 0.1
 * @date 2026-10-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "Chassis_Control.h"

//MPC 预测
