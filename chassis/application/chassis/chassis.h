#ifndef CHASSIS_H
#define CHASSIS_H
#include "ins_task.h"
#include "balance.h"

extern attitude_t *chassis_IMU_data;
extern leg_Resolve leg;
extern Wheel_data wheel;
extern balance_observation balance_observe;
extern balance_target targets;
extern control_data control_datas;
extern leg_motor_torque leg_motor_torques;
/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

#endif // CHASSIS_H