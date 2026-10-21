#include "balance.h"
#include "chassis.h"
#include "math.h"


void  Alpha_beta_calculate(float motor_big_leg,float motor_small_leg,float motor_big_speed,float motor_small_speed,leg_Resolve *leg)
{
    leg->alpha = (motor_big_leg  - 0.357252);
    leg->beta = -(motor_big_leg - motor_small_leg) + ((31.17*2)*PI/180.0f)-0.019f;
    leg->dalpha = motor_big_speed;
    leg->dbeta =motor_big_speed - motor_small_speed;

}

float leg_theta_calculate(leg_Resolve *leg,int L,int theta)
{
    Point2D c,b,p,d,pd,bc;
    c.x=-LEG_A*sin(-leg->alpha);
    c.z=-LEG_A*cos(-leg->alpha);
    b.x= LEG_A*sin(leg->beta+leg->alpha);
    b.z=-LEG_A*cos(leg->beta+leg->alpha);
    bc.x = c.x-b.x;
    bc.z = c.z-b.z;
    p.x=(b.x+c.x)/2;
    p.z=(b.z+c.z)/2;
    float BC = sqrt(pow(b.x - c.x, 2) + pow(b.z - c.z, 2));
    float PD = sqrt(pow(LEG_B, 2) - pow((BC/2), 2));
    float k = PD/BC;
    pd.x = -k * bc.z ;
    pd.z = k * bc.x;
    d.x=pd.x+p.x;
    d.z=pd.z+p.z;//�ֶ�
    float l =  sqrt(pow(d.x, 2) + pow(d.z, 2));//�ȳ�
    float  Theta = atan2(d.x,-d.z);//�ڽ�

    if((L==1)  &&  (theta==0))
    {
        return l;
    }
    else if((L==0) && (theta==1))
    {
       return Theta;
    }

}

float leg_theta_rate_calculate(leg_Resolve *leg)
{
    return leg->dalpha - 0.5f * leg->dbeta;
}

void wheel_control(float wheel_angle_aps,float wheel_w_aps,Wheel_data *wheel)
{
    float wheel_angle_rad=wheel_angle_aps* PI/ 180.0f;
    float wheel_w_rad=wheel_w_aps* PI/ 180.0f;
    float x=wheel_angle_rad/REDUCTION_RATIO*HALF_WHEEL_BASE;
    float dx=wheel_w_rad/REDUCTION_RATIO*HALF_WHEEL_BASE;

    wheel->x=x;
    wheel->dx=dx;

}

void BalanceObserve(balance_observation *observe)
{

    observe->theta =leg_theta_calculate(&leg, 0, 1)+chassis_IMU_data->Pitch* 0.01745329252f;
    observe->dtheta =leg_theta_rate_calculate(&leg)+chassis_IMU_data->Gyro[0];
    observe->x =  wheel.x;
    observe->dx = wheel.dx;
    observe->phi = chassis_IMU_data->Roll* 0.01745329252f;
    observe->dphi =chassis_IMU_data->Gyro[1];
    observe->leg_length = leg_theta_calculate(&leg, 1, 0);
   
}

