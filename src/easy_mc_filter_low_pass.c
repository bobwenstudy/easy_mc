#include "easy_mc_filter_low_pass.h"

#include "easy_mc_common.h"

#include <string.h>

// https://blog.csdn.net/qq_37662088/article/details/125075600
void easy_mc_filter_low_pass_init(easy_mc_filter_low_pass_t *filter, float fc, float Ts)
{
    float b = 2.0f * EASY_MC_MATH_PI * fc * Ts;

    filter->fc = fc;
    filter->Ts = Ts;
    filter->alpha = b / (b + 1);
    filter->out_last = 0.0f;
    filter->first_flag = 1;
}

float easy_mc_filter_low_pass_process(easy_mc_filter_low_pass_t *filter, float value)
{
    float out;
    float out_last = filter->out_last;
    float alpha = filter->alpha;

    // first flag
    if (filter->first_flag == 1)
    {
        filter->first_flag = 0;
        out_last = value;
    }

    // process
    out = out_last + alpha * (value - out_last);

    // update out_last
    filter->out_last = out;

    return out;
}
