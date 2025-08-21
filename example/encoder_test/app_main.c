#include <stdlib.h>
#include <math.h>

#include "easy_mc.h"

struct foc_test_vofa_data
{
    float current_count;
    float angle;
    float position;
    float z_trigger;

    unsigned char tail[4];
};

static volatile int is_z_triggered = 0;
void encoder_wave_test(void)
{
    easy_mc_abz_encoder_update();
    uint16_t current_count = easy_mc_hw_encoder_get_count();
    float angle = easy_mc_get_angle();
    float position = easy_mc_get_position();

    struct foc_test_vofa_data vofa_data = {.tail = {0x00, 0x00, 0x80, 0x7f}};

    // Set data to vofa_data
    vofa_data.current_count = current_count;
    vofa_data.angle = angle;
    vofa_data.position = position;

    vofa_data.z_trigger = is_z_triggered;
    is_z_triggered = 0;

    easy_mc_hw_vofa_debug_out((uint8_t *)&vofa_data, sizeof(vofa_data));

    HAL_Delay(1);
}

void easy_mc_isr_task_low_frequency_user_handle(void)
{
}

void easy_mc_isr_task_high_frequency_user_handle(void)
{
}

void easy_mc_isr_set_zero_user_handle(void)
{
    // EASY_MC_LOG_INF("z_triggered, is_z_triggered = %d\r\n", is_z_triggered);
    is_z_triggered = 1;
}

void app_main(void)
{
    EASY_MC_LOG_INF("encoder_test\r\n");
    easy_mc_hw_init();

    easy_mc_abz_encoder_init();
}

void app_main_loop(void)
{
    encoder_wave_test();
}
