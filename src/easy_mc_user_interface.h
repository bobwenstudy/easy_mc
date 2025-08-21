#ifndef _EASY_MC_USER_INTERFACE_H_
#define _EASY_MC_USER_INTERFACE_H_

#include <stdio.h>
#include <stdint.h>

#include "easy_mc_config.h"

#include "easy_mc_common.h"
#include "easy_mc_debug.h"

#include "easy_mc_abz_encoder.h"

#include "easy_mc_foc.h"

/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif
void easy_mc_user_interface_init(void); // Initialize the user interface for motor control

// Enable or disable motor control
void easy_mc_user_motor_control(uint8_t is_enable);

// Open-loop torque control
void easy_mc_user_set_u_ref(float uq_ref, float ud_ref); // Set the reference values for uq and ud in open-loop torque control

// Speed loop control
void easy_mc_user_set_speed_ref(float speed_ref); // Set the reference speed for the speed loop

// Position loop control
void easy_mc_user_set_position_ref(float position_ref);                    // Set the reference position for the position loop
void easy_mc_user_set_position_angle_ref(float position_angle_ref);        // Set the reference position angle for the position loop
void easy_mc_user_set_position_offset_ref(float position_ref);             // Set the position offset reference for the position loop
void easy_mc_user_set_position_angle_offset_ref(float position_angle_ref); // Set the position angle offset reference for the position loop

// Current loop control
void easy_mc_user_set_i_ref(float iq_ref, float id_ref); // Set the reference values for iq and id in current loop control

// Current loop control with position limit
void easy_mc_user_set_i_ref_limit_position(float iq_ref, float id_ref,
                                           float position_ref); // Set the reference values for iq and id with position limit in current loop control

// Angle control mode - Incremental mode
void easy_mc_user_set_theta_inc(float arc_inc); // Set the incremental angle for angle control mode

// Angle control mode - ABZ mode
void easy_mc_user_set_theta_abz(void); // Set the angle for angle control mode using ABZ encoder

// Angle control mode - Fixed mode
void easy_mc_user_set_theta_fix(float fix_arc); // Set the fixed angle for angle control mode

// Handle user commands
void easy_mc_user_handle_command(void); // Process and handle user commands

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif

#endif /* _EASY_MC_USER_INTERFACE_H_ */
