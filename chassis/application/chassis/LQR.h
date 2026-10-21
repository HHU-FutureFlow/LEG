#ifndef _LQR_H
#define _LQR_H

#include "chassis.h"
#include "balance.h"

void LQR_calc(balance_observation *observe,balance_target *balance_targets,control_data *control_datas);


#endif
