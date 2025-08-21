#ifndef _EASY_MC_FILTER_LOW_PASS_H_
#define _EASY_MC_FILTER_LOW_PASS_H_

#include <stddef.h>
#include <math.h>

typedef struct
{
    float fc;         // Cut off frequency, unit Hz
    float Ts;         // Sampling period, unit second
    float alpha;      // Filter coefficient
    float out_last;   // Last output value
    float first_flag; // First flag
} easy_mc_filter_low_pass_t;

void easy_mc_filter_low_pass_init(easy_mc_filter_low_pass_t *pid, float fc, float Ts);
float easy_mc_filter_low_pass_process(easy_mc_filter_low_pass_t *filter, float value);

#endif // _EASY_MC_FILTER_LOW_PASS_H_
