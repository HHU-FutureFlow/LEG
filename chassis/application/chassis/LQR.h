#ifndef _LQR_H
#define _LQR_H

#include "chassis.h"
#include "balance.h"

void LQR_calc(r_balance_observation *observe,r_balance_target *balance_targets,r_control_data *control_datas);
void l_LQR_calc(l_balance_observation *observe,l_balance_target *balance_targets,l_control_data *control_datas);


#endif
