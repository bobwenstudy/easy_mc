#include "easy_mc_abz_encoder.h"
#include <string.h>

#include "easy_mc.h"

easy_mc_motor_control_user_command_t motor_control_user_command;

static int easy_mc_user_check_allow_control(void)
{
    if (motor_control.state != MOTOR_STATE_ENABLE)
    {
        return 0;
    }

    return 1;
}

void easy_mc_user_motor_control(uint8_t is_enable)
{
    motor_control_user_command.params_control_enable_is_set = 0;
    motor_control_user_command.params_is_control_enable = is_enable;
    motor_control_user_command.params_control_enable_is_set = 1;
}

void easy_mc_user_set_u_ref(float uq_ref, float ud_ref)
{
    motor_control_user_command.params_control_is_set = 0;
    motor_control_user_command.params_u_qd_ref.q = uq_ref;
    motor_control_user_command.params_u_qd_ref.d = ud_ref;
    motor_control_user_command.params_control_mode = MOTOR_CONTROL_MODE_UQ;
    motor_control_user_command.params_control_is_set = 1;
}

void easy_mc_user_set_i_ref(float iq_ref, float id_ref)
{
    motor_control_user_command.params_control_is_set = 0;
    motor_control_user_command.params_i_qd_ref.q = iq_ref;
    motor_control_user_command.params_i_qd_ref.d = id_ref;
    motor_control_user_command.params_control_mode = MOTOR_CONTROL_MODE_IQ;
    motor_control_user_command.params_control_is_set = 1;
}

void easy_mc_user_set_i_ref_limit_position(float iq_ref, float id_ref, float position_limit)
{
    motor_control_user_command.params_control_is_set = 0;
    motor_control_user_command.params_i_qd_ref.q = iq_ref;
    motor_control_user_command.params_i_qd_ref.d = id_ref;
    motor_control_user_command.params_position_limit = position_limit;
    motor_control_user_command.params_control_mode = MOTOR_CONTROL_MODE_IQ_WITH_POSITION_LIMIT;
    motor_control_user_command.params_control_is_set = 1;
}

void easy_mc_user_set_speed_ref(float speed_ref)
{
    motor_control_user_command.params_control_is_set = 0;
    motor_control_user_command.params_speed_ref = speed_ref;
    motor_control_user_command.params_control_mode = MOTOR_CONTROL_MODE_SPEED;
    motor_control_user_command.params_control_is_set = 1;
}

void easy_mc_user_set_position_ref(float position_ref)
{
    motor_control_user_command.params_control_is_set = 0;
    motor_control_user_command.params_position_ref = position_ref;
    motor_control_user_command.params_control_mode = MOTOR_CONTROL_MODE_POSITION;
    motor_control_user_command.params_control_is_set = 1;
}

void easy_mc_user_set_position_angle_ref(float position_angle_ref)
{
    easy_mc_user_set_position_ref((position_angle_ref) * (EASY_MC_MATH_2PI / 360.0f));
}

void easy_mc_user_set_position_offset_ref(float position_ref)
{
    easy_mc_user_set_position_ref(position_ref + abz_encoder_handle.position);
}

void easy_mc_user_set_position_angle_offset_ref(float position_angle_ref)
{
    easy_mc_user_set_position_ref((position_angle_ref) * (EASY_MC_MATH_2PI / 360.0f) + abz_encoder_handle.position);
}

void easy_mc_user_set_theta_inc(float arc_inc)
{
    motor_control_user_command.params_theta_is_set = 0;
    motor_control_user_command.params_theta_inc = arc_inc;
    motor_control_user_command.params_theta_mode = MOTOR_THETA_MODE_OPEN_INC;
    motor_control_user_command.params_theta_is_set = 1;
}

void easy_mc_user_set_theta_abz(void)
{
    motor_control_user_command.params_theta_is_set = 0;
    motor_control_user_command.params_theta_mode = MOTOR_THETA_MODE_ENCODER_ABZ;
    motor_control_user_command.params_theta_is_set = 1;
}

void easy_mc_user_set_theta_fix(float fix_arc)
{
    motor_control_user_command.params_theta_is_set = 0;
    motor_control_user_command.params_theta_fix = fix_arc;
    motor_control_user_command.params_theta_mode = MOTOR_THETA_MODE_FIX;
    motor_control_user_command.params_theta_is_set = 1;
}

static void easy_mc_handle_user_control_command(void)
{
    if (!motor_control_user_command.params_control_is_set)
    {
        return;
    }

    if (!easy_mc_user_check_allow_control())
    {
        return;
    }

    motor_control.control_mode = motor_control_user_command.params_control_mode;
    switch (motor_control_user_command.params_control_mode)
    {
    case MOTOR_CONTROL_MODE_UQ:
        motor_control.uq_ref = motor_control_user_command.params_u_qd_ref.q;
        motor_control.ud_ref = motor_control_user_command.params_u_qd_ref.d;
        break;
    case MOTOR_CONTROL_MODE_IQ:
        motor_control.iq_ref = motor_control_user_command.params_i_qd_ref.q;
        motor_control.id_ref = motor_control_user_command.params_i_qd_ref.d;
        break;
    case MOTOR_CONTROL_MODE_SPEED:
        motor_control.speed_ref = motor_control_user_command.params_speed_ref;
        break;
    case MOTOR_CONTROL_MODE_POSITION:
        motor_control.position_ref = motor_control_user_command.params_position_ref;
        break;
    case MOTOR_CONTROL_MODE_IQ_WITH_POSITION_LIMIT:
        motor_control.iq_ref = motor_control_user_command.params_i_qd_ref.q;
        motor_control.id_ref = motor_control_user_command.params_i_qd_ref.d;
        motor_control.position_limit = motor_control_user_command.params_position_limit;
        break;

    default:
        return;
    }

    motor_control_user_command.params_control_is_set = 0;
}

static void easy_mc_handle_user_theta_command(void)
{
    if (!motor_control_user_command.params_theta_is_set)
    {
        return;
    }
    if (!easy_mc_user_check_allow_control())
    {
        return;
    }

    motor_control.theta_mode = motor_control_user_command.params_theta_mode;
    motor_control.theta_inc = motor_control_user_command.params_theta_inc;
    motor_control.theta_fix = motor_control_user_command.params_theta_fix;

    motor_control_user_command.params_theta_is_set = 0;
}

static void easy_mc_handle_user_enable_command(void)
{
    if (!motor_control_user_command.params_control_enable_is_set)
    {
        return;
    }

    if ((motor_control.state != MOTOR_STATE_ENABLE) && (motor_control.state != MOTOR_STATE_DISABLE))
    {
        goto _user_enable_end;
    }

    easy_mc_control(motor_control_user_command.params_is_control_enable);

_user_enable_end:
    motor_control_user_command.params_control_enable_is_set = 0;
}

void easy_mc_user_handle_command(void)
{
    easy_mc_handle_user_enable_command();
    easy_mc_handle_user_control_command();
    easy_mc_handle_user_theta_command();
}

void easy_mc_user_interface_init(void)
{
    memset(&motor_control_user_command, 0, sizeof(motor_control_user_command));
}
