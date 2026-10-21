#include "LQR.h"


float K[2][6]=
{
    { 14.5591,    1.1394 ,  -2.2194 , -1.3776 ,  -1.0008 ,  -0.2788},
    { 0.7588 ,   0.0713 ,  -0.5443,   -0.2561 ,  14.2407   , 2.2591} 

};//大腿竖直，小腿最长


float u[2];
float x[6];
void LQR_calc(balance_observation *observe,balance_target *balance_targets,control_data *control_datas)
{
    float x[6];
    float u[2];
    x[0]=observe->theta-balance_targets->theta_t_target;
    x[1]=observe->dtheta-balance_targets->dtheta_target;
    x[2]=observe->x-balance_targets->x_target;
    x[3]=observe->dx-balance_targets->dx_target;
    x[4]=observe->phi-balance_targets->phi_target;
    x[5]=observe->dphi-balance_targets->dphi_target;

    for (int i = 0; i < 2; i++) {
        u[i] = 0.0;
        for (int j = 0; j < 6; j++) {
            u[i] += K[i][j] * x[j];
        }
    }

    control_datas->Tw=-u[0];
    control_datas->Tp=-u[1];

}


