#ifndef _EASY_MC_H_
#define _EASY_MC_H_

#include <stdio.h>
#include <stdint.h>

#include "easy_mc_config.h"

#include "easy_mc_common.h"
#include "easy_mc_debug.h"

#include "easy_mc_abz_encoder.h"

#include "easy_mc_foc.h"
#include "easy_mc_pid.h"
#include "easy_mc_filter_low_pass.h"

#include "easy_mc_user_interface.h"

/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

extern easy_mc_motor_control_user_command_t motor_control_user_command;
extern easy_mc_motor_control_t motor_control;

void easy_mc_debug_polling_state_change(void);
void easy_mc_init(void);
void easy_mc_deinit(void);
void easy_mc_isr_set_zero(void);
void easy_mc_adc_debug_data(void);
bool easy_mc_adc_offset_init(void);
bool easy_mc_check_enable(void);
int easy_mc_get_status(void);
void easy_mc_control(bool status);
void easy_mc_set_v_bus(float v_bus);
void easy_mc_set_i_bus(float i_bus);
float easy_mc_get_speed(void);
float easy_mc_get_angle(void);
float easy_mc_get_position(void);
float easy_mc_get_position_degree(void);
void easy_mc_isr_task_low_frequency(void);
void easy_mc_isr_task_high_frequency(void);

// api need to be implemented by hardware driver
void easy_mc_hw_1ms_task(void);
void easy_mc_hw_debug_io_control(int io_index, int is_set);
uint16_t easy_mc_hw_encoder_get_count(void);
void easy_mc_hw_encoder_set_zero(void);
void easy_mc_hw_vofa_debug_out(uint8_t *data, uint16_t size);
void easy_mc_hw_set_u_v_w_count(uint16_t u_count, uint16_t v_count, uint16_t w_count);
void easy_mc_hw_start(void);
void easy_mc_hw_stop(void);
void easy_mc_hw_init(void);
void easy_mc_hw_deinit(void);

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif

#endif /* _EASY_MC_H_ */
