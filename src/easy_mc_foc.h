#ifndef _EASY_MC_FOC_H_
#define _EASY_MC_FOC_H_

#include <stddef.h>
#include <math.h>

#include "easy_mc_pid.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

typedef enum
{
    MOTOR_PID_TYPE_IQ = 0,
    MOTOR_PID_TYPE_ID = 1,
    MOTOR_PID_TYPE_SPEED = 2,
    MOTOR_PID_TYPE_POSITION = 3,
    MOTOR_PID_TYPE_POSITION_SPEED_LIMIT = 4,
} easy_mc_pid_type_t;

easy_mc_pid_t *easy_mc_foc_get_pid_controller(easy_mc_pid_type_t type);

void easy_mc_foc_step(void);
void easy_mc_foc_init(void);

#endif // _EASY_MC_FOC_H_
