#include "VMC.h"
#include "balance.h"
#include "chassis.h"
#include "math.h"






void VMC_Calc(control_data *control_datas,leg_motor_torque *leg_motor_torques,leg_Resolve *leg)
{
    
    float g=-0.5f*LEG_A*sin(0.5f*leg->beta)-(LEG_A*LEG_A*sin(0.5f*leg->beta)*cos(0.5f*leg->beta))/(2.0f*sqrt(LEG_B*LEG_B-LEG_A*LEG_A*sin(0.5f*leg->beta)*sin(0.5f*leg->beta)));
    
    float Tf= g*control_datas->Tleg + 0.5f*control_datas->Tp;
    float Tb= -g*control_datas->Tleg + 0.5f*control_datas->Tp;

    leg_motor_torques->Tf=Tf;
    leg_motor_torques->Tb=Tb;

    
  
}