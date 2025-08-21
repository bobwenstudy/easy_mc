#include "easy_mc_abz_encoder.h"
#include <math.h>
#include <string.h>

#include "easy_mc.h"

easy_mc_abz_encoder_t abz_encoder_handle;

void easy_mc_abz_encoder_init_speed_monitor(void)
{
    abz_encoder_handle.last_count = 0;
    abz_encoder_handle.last_tick = 0;

    // Initialize the speed filter
    // easy_mc_filter_low_pass_init(&abz_encoder_handle.speed_filter, 100.0f, 0.001f); // 100Hz cutoff frequency, 1ms sampling period
    easy_mc_filter_low_pass_init(&abz_encoder_handle.speed_filter, 150.0f, 0.001f); // 150Hz cutoff frequency, 1ms sampling period
}

void easy_mc_abz_encoder_init(void)
{
    memset(&abz_encoder_handle, 0, sizeof(abz_encoder_handle));

    easy_mc_abz_encoder_init_speed_monitor();

    easy_mc_hw_encoder_set_zero();
}

static void easy_mc_abz_encoder_cal_speed(uint16_t current_count)
{
    float delta_t;
    int32_t delta_cnt;
    uint32_t current_tick = EASY_MC_TIME_MS_GET();
    float speed;

    if (current_tick == abz_encoder_handle.last_tick)
    {
        return;
    }

    // calculate the time difference in seconds
    delta_t = (float)(current_tick - abz_encoder_handle.last_tick);
    // calculate the count difference
    delta_cnt = (current_count - abz_encoder_handle.last_count);

    // handle count overflow
    if (delta_cnt > (EASY_MC_PULSE_NBR / 2))
    {
        delta_cnt -= EASY_MC_PULSE_NBR;
    }
    else if (delta_cnt < -(EASY_MC_PULSE_NBR / 2))
    {
        delta_cnt += EASY_MC_PULSE_NBR;
    }

    // calculate the speed in RPM
    speed = (delta_cnt * 60.0f * 1000.0f) / (delta_t * (float)EASY_MC_PULSE_NBR);

    // update last count and tick
    abz_encoder_handle.last_count = current_count;
    abz_encoder_handle.last_tick = current_tick;

    abz_encoder_handle.speed = easy_mc_filter_low_pass_process(&abz_encoder_handle.speed_filter, speed);
}

void easy_mc_abz_encoder_update(void)
{
    // Get the current count from the encoder
    uint16_t current_count = easy_mc_hw_encoder_get_count();
    // Calculate the angle in radians
    float angle_mechanical = (float)current_count * (EASY_MC_MATH_2PI / EASY_MC_PULSE_NBR);

    // Calculate the position.
    float d_angle = angle_mechanical - abz_encoder_handle.angle_prev;
    // Calculate rotations
    // Check if the angle change is greater than 80% of a circle (0.8f*2PI) to detect overflow, if overflow, add or subtract 1 to full_rotations.
    if (fabsf(d_angle) > (0.8f * EASY_MC_MATH_2PI))
    {
        d_angle += (d_angle > 0) ? -EASY_MC_MATH_2PI : EASY_MC_MATH_2PI;
    }
    // update previous angle
    abz_encoder_handle.angle_prev = angle_mechanical;

    // update position
    abz_encoder_handle.position += d_angle;

    // update angle
    abz_encoder_handle.angle = angle_mechanical;

    easy_mc_abz_encoder_cal_speed(current_count);
}
