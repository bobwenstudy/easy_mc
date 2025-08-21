#ifndef _EASY_MC_ABZ_ENCODER_H_
#define _EASY_MC_ABZ_ENCODER_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "easy_mc_filter_low_pass.h"

typedef struct
{
    float angle; // unit: radian
    float angle_prev;
    float position; // unit: radian

    // speed monitoring
    float speed; // unit: RPM
    int32_t last_count;
    uint32_t last_tick;
    easy_mc_filter_low_pass_t speed_filter;

    uint8_t zero_flag;
} easy_mc_abz_encoder_t;

extern easy_mc_abz_encoder_t abz_encoder_handle;

void easy_mc_abz_encoder_init(void);
void easy_mc_abz_encoder_update(void);

#endif