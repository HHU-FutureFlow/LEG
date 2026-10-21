#ifndef _BALANCE_H
#define _BALANCE_H
#pragma once




//零点标定状态：大腿竖直，小腿伸到最长，此时大腿编码器值与小腿编码器的差值为0
//β=（两编码器差值）+ （31.17 * 2）*PI/180   单位为rad
//α= 大腿电机编码器值 - 0.357252  单位为rad


#define LEG_A (0.21f)
#define LEG_B (0.25f)
#define HALF_WHEEL_BASE (0.06f)
#define REDUCTION_RATIO (15.8f)

typedef struct
{
    //LQR用
    float theta;//该腿摆杆相对竖直倾⻆
    float dtheta;//该腿摆杆⻆速度
    float x;//该腿轮⼦⽔平位移
    float dx;//该腿轮⼦⽔平速度
    float phi;//机体俯仰⻆
    float dphi;//机体俯仰⻆速度



    //PID用
    float leg_length;//该腿摆杆长度


} balance_observation; //需要观测的状态


typedef struct
{
    float alpha;
    float beta;
    float dalpha;
    float dbeta;

} leg_Resolve;//解算要用的变量

typedef struct {
    float x;
    float dx;
} Wheel_data; 

typedef struct {
    float x;
    float z;
} Point2D; //二维坐标系

typedef struct {
   //LQR用
    float theta_t_target;//该腿摆杆相对竖直倾⻆目标值
    float dtheta_target;//该腿摆杆⻆速度
    float x_target;//该腿轮⼦⽔平位移
    float dx_target;//该腿轮⼦⽔平速度
    float phi_target;//机体俯仰⻆
    float dphi_target;//机体俯仰⻆速度


    //PID用
    float leg_length_target;//该腿摆杆长度
} balance_target; //目标值

typedef struct 
{
    float Tw;
    float Tp;
    float Tleg;
} control_data; //LQR的控制向量和PID输出力矩

typedef struct {
    float Tf;
    float Tb;
} leg_motor_torque; //腿部电机力矩





float leg_theta_calculate(leg_Resolve *leg,int L,int theta);
void BalanceObserve(balance_observation *observe);
void  Alpha_beta_calculate(float motor_big_leg,float motor_small_leg,float motor_big_speed,float motor_small_speed,leg_Resolve *leg);
float leg_theta_rate_calculate(leg_Resolve *leg);
void wheel_control(float wheel_angle,float wheel_w,Wheel_data *wheel);

#endif 
