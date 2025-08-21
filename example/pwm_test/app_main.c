#include <stdlib.h>
#include <math.h>

#include "easy_mc.h"

void easy_mc_isr_task_low_frequency_user_handle(void)
{
}

void easy_mc_isr_task_high_frequency_user_handle(void)
{
    easy_mc_hw_debug_io_control(0, 1);

    easy_mc_hw_debug_io_control(0, 0);
}

void app_main(void)
{
    EASY_MC_LOG_INF("pwm_test\r\n");
    uint32_t test_unit = EASY_MC_PWM_PERIOD_CYCLES / 10;
    easy_mc_hw_init();

    easy_mc_hw_start();

    easy_mc_hw_set_u_v_w_count(test_unit, test_unit * 2, test_unit * 3);
}

void app_main_loop(void)
{
}
