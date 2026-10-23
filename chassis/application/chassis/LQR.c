#include "LQR.h"


// float K[2][6]=
// {
//     {  16.3382  ,  1.1671  , 3.8455  , 1.6600  , 1.3472 ,  0.3284},
//     {     0.9049  ,  0.0725  , -0.9217  , -0.2619  , 20.0598  ,  2.3618} 

// };//大腿竖直，小腿最长


float K[2][6]=
{
    {  16.3382  ,  1.1671  , 3.8455  , 1.6600 , 1.3472 ,  0.3284},
    {     0  ,  0  , -0.9217  , -0.2619  , 0  ,  0}

};//大腿竖直，小腿最长

// float K[4][10]=
// {
//     {  },
//     {   },
//     {   },
//     {   }

// }


float u[2];
float x[6];
void LQR_calc(r_balance_observation *observe,r_balance_target *balance_targets,r_control_data *control_datas)
{
    float x[6];
    float u[2];
    x[0]=observe->r_theta-balance_targets->r_theta_t_target;
    x[1]=observe->r_dtheta-balance_targets->r_dtheta_target;
    x[2]=observe->r_x-balance_targets->r_x_target;
    x[3]=observe->r_dx-balance_targets->r_dx_target;
    x[4]=observe->r_phi-balance_targets->r_phi_target;
    x[5]=observe->r_dphi-balance_targets->r_dphi_target;

    for (int i = 0; i < 2; i++) {
        u[i] = 0.0;
        for (int j = 0; j < 6; j++) {
            u[i] += K[i][j] * x[j];
        }
    }

    control_datas->r_Tw=-u[0];
    // control_datas->r_Tp=-u[1];

}




void l_LQR_calc(l_balance_observation *observe,l_balance_target *balance_targets,l_control_data *control_datas)
{
    float x[6];
    float u[2];
    x[0]=observe->l_theta-balance_targets->l_theta_t_target;
    x[1]=observe->l_dtheta-balance_targets->l_dtheta_target;
    x[2]=observe->l_x-balance_targets->l_x_target;
    x[3]=observe->l_dx-balance_targets->l_dx_target;
    x[4]=observe->l_phi-balance_targets->l_phi_target;
    x[5]=observe->l_dphi-balance_targets->l_dphi_target;

    for (int i = 0; i < 2; i++) {
        u[i] = 0.0;
        for (int j = 0; j < 6; j++) {
            u[i] += K[i][j] * x[j];
        }
    }

    control_datas->l_Tw=-u[0];
    // control_datas->l_Tp=-u[1];

}



// float x[10];
// float u[4];

// void LQR_calc(r_balance_observation *observe,r_balance_target *balance_targets,r_control_data *control_datas)
// {
//     float x[10];
//     float u[4];
//     x[0]=observe->r_theta-balance_targets->r_theta_t_target;
//     x[1]=observe->r_dtheta-balance_targets->r_dtheta_target;
//     x[2]=observe->r_x-balance_targets->r_x_target;
//     x[3]=observe->r_dx-balance_targets->r_dx_target;
//     x[4]=observe->r_phi-balance_targets->r_phi_target;
//     x[5]=observe->r_dphi-balance_targets->r_dphi_target;
//     x[6]=observe->r_leg_length-balance_targets->r_leg_length_target;
//     x[7]=0;
//     x[8]=0;
//     x[9]=0;




//     for (int i = 0; i < 4; i++) {
//         u[i] = 0.0;
//         for (int j = 0; j < 10; j++) {
//             u[i] += K[i][j] * x[j];
//         }
//     }

//     control_datas->r_Tw=-u[0];
//     control_datas->r_Tp=-u[1];

// }




