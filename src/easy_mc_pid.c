#include "easy_mc_pid.h"

#include "easy_mc_common.h"

#include <string.h>

#ifndef CLAMP
#define CLAMP(x, min, max) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
#endif

void easy_mc_pid_history_init(easy_mc_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    pid->err = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
}

void easy_mc_pid_init(easy_mc_pid_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki * EASY_MC_SAMPLE_TIME;
    pid->Kd = kd / EASY_MC_SAMPLE_TIME;

    easy_mc_pid_history_init(pid);
}

float easy_mc_pid_process(float value_meas, float value_ref, easy_mc_pid_t *pid)
{
    // 1. Calculate the error
    float error = value_ref - value_meas;

    // 2. Calculate the proportional term
    float p_term = pid->Kp * error;

    // 3. Calculate the integral term with anti-windup protection
    float integral_term = pid->integral + error * pid->Ki;
    integral_term = CLAMP(integral_term, -pid->integral_max, pid->integral_max);

    // 4. Calculate the derivative term with low-pass filtering
    float derivative = (error - pid->prev_error);
    float derivative_term = pid->Kd * derivative;

    // 5. Calculate the PID output
    float output = p_term + integral_term + derivative_term;

    // 6. Limit the output
    output = CLAMP(output, -pid->output_max, pid->output_max);

    // 7. Update the state variables
    pid->integral = integral_term; // Note that this stores the integral term after multiplying by Ki and T
    pid->prev_error = error;

    // 8. Update memory information
    pid->err = error;
    pid->p_out = p_term;
    pid->i_out = integral_term;
    pid->d_out = derivative_term;

    return output;
}

float easy_mc_pi_process(float value_meas, float value_ref, easy_mc_pid_t *pid)
{
    // 1. Calculate the error
    float error = value_ref - value_meas;

    // 2. Calculate the proportional term
    float p_term = pid->Kp * error;

    // 3. Calculate the integral term with anti-windup protection
    float integral_term = pid->integral + error * pid->Ki;
    integral_term = CLAMP(integral_term, -pid->integral_max, pid->integral_max);

    // 4. Calculate the PI output
    float output = p_term + integral_term;

    // 5. Limit the output
    output = CLAMP(output, -pid->output_max, pid->output_max);

    // 6. Update the state variables
    pid->integral = integral_term; // Note that this stores the integral term after multiplying by Ki and T
    pid->prev_error = error;

    // 7. Update memory information
    pid->err = error;
    pid->p_out = p_term;
    pid->i_out = integral_term;

    return output;
}
