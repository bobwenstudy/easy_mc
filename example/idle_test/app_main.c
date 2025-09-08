#include <stdlib.h>
#include <math.h>

#include "easy_mc.h"

#define TEST_MODE_IQ_IDLE                   0
#define TEST_MODE_UQ_IDLE                   1
#define TEST_MODE_SPEED_IDLE                2
#define TEST_MODE_POSITION_IDLE             3
#define TEST_MODE_POSITION_SPEED_LIMIT_IDLE 4

#define TEST_MODE TEST_MODE_IQ_IDLE

#define TEST_SPEED_LIMIT 50.0f

void app_main(void)
{
    EASY_MC_LOG_INF("idle_test, test mode: %d\r\n", TEST_MODE);
    easy_mc_init();

    // Set theta mode to encoder ABZ
    easy_mc_user_set_theta_abz();

#if TEST_MODE == TEST_MODE_IQ_IDLE
    easy_mc_user_set_i_ref(0, 0);
#elif TEST_MODE == TEST_MODE_UQ_IDLE
    easy_mc_user_set_u_ref(0, 0);
#elif TEST_MODE == TEST_MODE_SPEED_IDLE
    easy_mc_user_set_speed_ref(0);
#elif TEST_MODE == TEST_MODE_POSITION_IDLE
    easy_mc_user_set_position_ref(EASY_MC_MATH_2PI * 0);
    // easy_mc_user_set_position_angle_ref(360.0f * 0);
#elif TEST_MODE == TEST_MODE_POSITION_SPEED_LIMIT_IDLE
    easy_mc_user_set_position_ref_limit_speed(EASY_MC_MATH_2PI * 0, TEST_SPEED_LIMIT);
#endif
}

#define TEST_POSITION_OFFSET_CHANGE 0
#if TEST_POSITION_OFFSET_CHANGE
static uint32_t last_tick = 0;
#endif
void app_main_loop(void)
{
    easy_mc_debug_polling_state_change();

#if TEST_POSITION_OFFSET_CHANGE
    if (HAL_GetTick() - last_tick > 5000)
    {
        last_tick = HAL_GetTick();

#if TEST_MODE == TEST_MODE_POSITION_SPEED_LIMIT_IDLE
        easy_mc_user_set_position_angle_offset_ref_limit_speed(360.0f * 0.5f, TEST_SPEED_LIMIT);
#else
        // easy_mc_user_set_position_offset_ref(EASY_MC_MATH_2PI * 0.1f);
        easy_mc_user_set_position_angle_offset_ref(360.0f * 0.1f);
#endif
    }
#endif
}
