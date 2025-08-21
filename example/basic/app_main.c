#include <stdlib.h>
#include <math.h>

#include "easy_mc.h"

void app_main(void)
{
    EASY_MC_LOG_INF("basic\r\n");
    easy_mc_init();

    // Set theta mode to encoder ABZ
    easy_mc_user_set_theta_abz();

    easy_mc_user_set_speed_ref(100);
}

void app_main_loop(void)
{
    easy_mc_debug_polling_state_change();
}
