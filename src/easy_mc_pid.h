#ifndef _EASY_MC_PID_H_
#define _EASY_MC_PID_H_

#include <stddef.h>
#include <math.h>

typedef struct
{
    float Kp;           // Proportional gain coefficient
    float Ki;           // Integral gain coefficient
    float Kd;           // Derivative gain coefficient
    float integral;     // Accumulated value of integral term
    float prev_error;   // Previous error value
    float integral_max; // Integral term saturation limit ±value
    float output_max;   // Output saturation limit ±value

    float err;   // Error term output, for debugging
    float p_out; // Proportional term output, for debugging
    float i_out; // Integral term output, for debugging
    float d_out; // Derivative term output, for debugging
} easy_mc_pid_t;

void easy_mc_pid_history_init(easy_mc_pid_t *pid);
void easy_mc_pid_init(easy_mc_pid_t *pid, float kp, float ki, float kd);
float easy_mc_pid_process(float value_meas, float value_ref, easy_mc_pid_t *pid);
float easy_mc_pi_process(float value_meas, float value_ref, easy_mc_pid_t *pid);

#endif // _EASY_MC_PID_H_
