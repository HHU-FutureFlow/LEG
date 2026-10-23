#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"
#include "n630.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
# include "dmmotor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "LQR.h"
#include "controller.h"
#include "VMC.h"


static DMMotorInstance *front_8009_motor,*behind_8009_motor;
static DJIMotorInstance *motor_left, *motor_right;
static PIDInstance leg_length_pid;
attitude_t *chassis_IMU_data;
leg_Resolve leg;
Wheel_data wheel;
balance_observation balance_observe;
balance_target targets;
control_data control_datas;
leg_motor_torque leg_motor_torques;

void ChassisInit()
{
    chassis_IMU_data = INS_Init();

    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,            
        },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    config.can_init_config.tx_id = 2;
    config.can_init_config.rx_id = 2;
    front_8009_motor = DMMotorInit(&config);

    config.can_init_config.tx_id = 3;
    config.can_init_config.rx_id = 3;
    behind_8009_motor = DMMotorInit(&config);




    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .current_PID = {
                .Kp = 5.0f,
                .Ki = 1.5f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.3f,
            },
        },
        .controller_setting_init_config = {

            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

        },

        .motor_type = M3508,

    };

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

    motor_left = PowerControlInit(&chassis_motor_config);



    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

    motor_right = PowerControlInit(&chassis_motor_config);




    static PID_Init_Config_s leg_length_pid_config = 

    {
        .Kp = 60.0f,  
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 9000.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 0.0f,
    };


    PIDInit(&leg_length_pid, &leg_length_pid_config);


    
}



void ChassisTask()
{
    
    // DMMotorSetMode(DM_CMD_MOTOR_MODE, front_8009_motor);
    // DMMotorSetMode(DM_CMD_MOTOR_MODE, behind_8009_motor);
    SetPowerLimit(1000000.0f);
    float theta_f=front_8009_motor->measure.position;
    float theta_b=behind_8009_motor->measure.position;
    float speed_f=front_8009_motor->measure.velocity;
    float speed_b=behind_8009_motor->measure.velocity;
    float wheel_angle=motor_left->measure.total_angle;
    float wheel_w=motor_left->measure.speed_aps;


    targets.theta_t_target=0;
    targets.dtheta_target=0;
    targets.x_target=0;
    targets.dx_target=0;
    targets.phi_target=0;
    targets.dphi_target=0;
    targets.leg_length_target = 0.40482f;


    Alpha_beta_calculate(theta_f,theta_b,speed_f,speed_b,&leg);
    wheel_control(wheel_angle,wheel_w,&wheel);
    BalanceObserve(&balance_observe);
    LQR_calc(&balance_observe,&targets,&control_datas);
    control_datas.Tleg= PIDCalculate(&leg_length_pid,balance_observe.leg_length,targets.leg_length_target);
    VMC_Calc(&control_datas,&leg_motor_torques,&leg);

    // DMMotorSetRef(front_8009_motor,leg_motor_torques.Tf );
    // DMMotorSetRef(behind_8009_motor,leg_motor_torques.Tb );



    //  DMMotorSetRef(front_8009_motor,0.01f );
    //  DMMotorSetRef(behind_8009_motor,0.01f);


    DJIMotorSetRef( motor_right ,3318.833*control_datas.Tw);




    DMMotorSetCtrlMode(front_8009_motor, DM_CTRL_POSITION); 
    DMMotorSetRef(front_8009_motor, -0.6735f);
    DMMotorSetCtrlMode(behind_8009_motor, DM_CTRL_POSITION); 
    DMMotorSetRef(behind_8009_motor, -0.04177f);




}

