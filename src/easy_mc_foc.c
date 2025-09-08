#include "easy_mc_foc.h"

#include "easy_mc_common.h"

#include "easy_mc.h"
#include "easy_mc_pid.h"

#include <string.h>

// 电角度求解
static float electrical_angle(float shaft_angle)
{
    return (shaft_angle * EASY_MC_CONFIG_POLE_PAIRS);
}

// Follow https://zhuanlan.zhihu.com/p/414721065.
static void SVPWM(float u_alpha, float u_beta, float v_dc, float t_s, float time_array[3])
{
    float X, Y, Z;
    float Tfirst, Tsecond;
    float Vref1, Vref2, Vref3;
    float Ta, Tb, Tc;
    float Tcm1, Tcm2, Tcm3;
    float temp0, temp1, temp2, temp3;
    int N;

    // EASY_MC_LOG_DBG("u_alpha = %f, u_beta = %f, v_dc = %f, t_s = %f\n"
    //     , u_alpha, u_beta, v_dc, t_s);

    // sqrt(3)/2*u_alpha
    temp0 = EASY_MC_MATH_SQRT3_2 * u_alpha;
    temp1 = u_beta / 2;

    // EASY_MC_LOG_DBG("temp0 = %f, temp1 = %f\n"
    //     , temp0, temp1);

    Vref1 = u_beta;
    Vref2 = temp0 - temp1;
    Vref3 = -temp0 - temp1;

    // EASY_MC_LOG_DBG("Vref1 = %f, Vref2 = %f, Vref3 = %f\n", Vref1, Vref2, Vref3);

    // N = 4C + 2B + A
    N = ((Vref3 > 0) << 2) + ((Vref2 > 0) << 1) + ((Vref1 > 0));

    // EASY_MC_LOG_DBG("N = %d\n", N);

    temp2 = EASY_MC_MATH_SQRT3 * t_s / v_dc;
    // EASY_MC_LOG_DBG("temp2 = %f\n"
    //     , temp2);

    X = temp2 * u_beta;
    Y = temp2 * (temp0 + temp1);
    Z = temp2 * (-temp0 + temp1);

    // EASY_MC_LOG_DBG("X = %f, Y = %f, Z = %f\n", X, Y, Z);

    switch (N)
    {
    case 1:
        Tfirst = Z;
        Tsecond = Y;
        break;
    case 2:
        Tfirst = Y;
        Tsecond = -X;
        break;
    case 3:
        Tfirst = -Z;
        Tsecond = X;
        break;
    case 4:
        Tfirst = -X;
        Tsecond = Z;
        break;
    case 5:
        Tfirst = X;
        Tsecond = -Y;
        break;
    case 6:
        Tfirst = -Y;
        Tsecond = -Z;
        break;
    default:
        // N = 0, 7
        Tfirst = 0;
        Tsecond = 0;
        break;
    }

    temp3 = (Tfirst + Tsecond);
    if (temp3 > t_s)
    {
        // EASY_MC_LOG_DBG("error, Tfirst = %f, Tsecond = %f, temp3 = %f\n"
        //     , Tfirst, Tsecond, temp3);
        Tfirst = t_s * Tfirst / temp3;
        Tsecond = t_s * Tsecond / temp3;
    }

    Ta = (t_s - Tfirst - Tsecond) / 4;
    Tb = Ta + Tfirst / 2;
    Tc = Tb + Tsecond / 2;

    // EASY_MC_LOG_DBG("Tfirst = %f, Tsecond = %f, Ta = %f, Tb = %f, Tc = %f\n"
    //     , Tfirst, Tsecond, Ta, Tb, Tc);

    switch (N)
    {
    case 1:
        Tcm1 = Tb;
        Tcm2 = Ta;
        Tcm3 = Tc;
        break;
    case 2:
        Tcm1 = Ta;
        Tcm2 = Tc;
        Tcm3 = Tb;
        break;
    case 3:
        Tcm1 = Ta;
        Tcm2 = Tb;
        Tcm3 = Tc;
        break;
    case 4:
        Tcm1 = Tc;
        Tcm2 = Tb;
        Tcm3 = Ta;
        break;
    case 5:
        Tcm1 = Tc;
        Tcm2 = Ta;
        Tcm3 = Tb;
        break;
    case 6:
        Tcm1 = Tb;
        Tcm2 = Tc;
        Tcm3 = Ta;
        break;
    default:
        // N = 0, 7
        Tcm1 = 0;
        Tcm2 = 0;
        Tcm3 = 0;
        break;
    }

    time_array[0] = Tcm1;
    time_array[1] = Tcm2;
    time_array[2] = Tcm3;
}

// Global PID Controller.
static easy_mc_pid_t iq_pid = {
        .Kp = EASY_MC_CONFIG_PID_CURRENT_KP,
        .Ki = EASY_MC_CONFIG_PID_CURRENT_KI * EASY_MC_SAMPLE_TIME,
        .Kd = 0.0f / EASY_MC_SAMPLE_TIME,
        .integral = 0.0f,
        .prev_error = 0.0f,
        .integral_max = EASY_MC_SVPWM_MAX_VOLTAGE,
        .output_max = EASY_MC_SVPWM_MAX_VOLTAGE,
        .p_out = 0.0f,
        .i_out = 0.0f,
        .d_out = 0.0f,
};

static easy_mc_pid_t id_pid = {
        .Kp = EASY_MC_CONFIG_PID_CURRENT_KP,
        .Ki = EASY_MC_CONFIG_PID_CURRENT_KI * EASY_MC_SAMPLE_TIME,
        .Kd = 0.0f / EASY_MC_SAMPLE_TIME,
        .integral = 0.0f,
        .prev_error = 0.0f,
        .integral_max = EASY_MC_SVPWM_MAX_VOLTAGE,
        .output_max = EASY_MC_SVPWM_MAX_VOLTAGE,
        .p_out = 0.0f,
        .i_out = 0.0f,
        .d_out = 0.0f,
};

static easy_mc_pid_t speed_pid = {
        .Kp = EASY_MC_CONFIG_PID_SPEED_KP,
        .Ki = EASY_MC_CONFIG_PID_SPEED_KI * EASY_MC_SAMPLE_TIME,
        .Kd = 0.0f / EASY_MC_SAMPLE_TIME,
        .integral = 0.0f,
        .prev_error = 0.0f,
        .integral_max = EASY_MC_CONFIG_CURRENT_MAX,
        .output_max = EASY_MC_CONFIG_CURRENT_MAX,
        .p_out = 0.0f,
        .i_out = 0.0f,
        .d_out = 0.0f,
};

static easy_mc_pid_t position_pid = {
        .Kp = EASY_MC_CONFIG_PID_POSITION_KP,
        .Ki = EASY_MC_CONFIG_PID_POSITION_KI * EASY_MC_SAMPLE_TIME,
        .Kd = 0.0f / EASY_MC_SAMPLE_TIME,
        .integral = 0.0f,
        .prev_error = 0.0f,
        .integral_max = EASY_MC_CONFIG_CURRENT_MAX,
        .output_max = EASY_MC_CONFIG_CURRENT_MAX,
        .p_out = 0.0f,
        .i_out = 0.0f,
        .d_out = 0.0f,
};

static easy_mc_pid_t position_speed_limit_pid = {
        .Kp = EASY_MC_CONFIG_PID_POSITION_SPEED_LIMIT_KP,
        .Ki = EASY_MC_CONFIG_PID_POSITION_SPEED_LIMIT_KI * EASY_MC_SAMPLE_TIME,
        .Kd = 0.0f / EASY_MC_SAMPLE_TIME,
        .integral = 0.0f,
        .prev_error = 0.0f,
        .integral_max = 100,
        .output_max = 100,
        .p_out = 0.0f,
        .i_out = 0.0f,
        .d_out = 0.0f,
};

easy_mc_pid_t *easy_mc_foc_get_pid_controller(easy_mc_pid_type_t type)
{
    switch (type)
    {
    case MOTOR_PID_TYPE_IQ:
        return &iq_pid;
    case MOTOR_PID_TYPE_ID:
        return &id_pid;
    case MOTOR_PID_TYPE_SPEED:
        return &speed_pid;
    case MOTOR_PID_TYPE_POSITION:
        return &position_pid;
    case MOTOR_PID_TYPE_POSITION_SPEED_LIMIT:
        return &position_speed_limit_pid;
    default:
        return NULL;
    }
}

static void easy_mc_foc_iqd_loop_step(float angle_el)
{
    // Clark Transform
    // float i_alpha = motor_control.ia*2/3-(motor_control.ib+motor_control.ic)/3;
    // float i_alpha = motor_control.ia; // 优化: https://www.aoske.cn/2025/03/08/clark%e5%8f%98%e6%8d%a2/
    // float i_beta = (motor_control.ib-motor_control.ic)*sqrt(3)/3;

    // Clark Transform - optimized
    // float i_alpha = 0.666666687F * motor_control.ia - (motor_control.ib + motor_control.ic) * 0.333333343F;
    float i_alpha = motor_control.ia;
    float i_beta = (motor_control.ib - motor_control.ic) * 0.577350259F;

    // Park Transform
    // float id = i_alpha*cos(angle_el)+i_beta*sin(angle_el);
    // float iq = -i_alpha*sin(angle_el)+i_beta*cos(angle_el);

    // Park Transform - optimized
    float id = i_alpha * motor_control.cos_theta + i_beta * motor_control.sin_theta;
    float iq = i_beta * motor_control.cos_theta - i_alpha * motor_control.sin_theta;

    motor_control.i_alpha = i_alpha;
    motor_control.i_beta = i_beta;
    motor_control.iq = iq;
    motor_control.id = id;

    // PID, change id/iq to ud/uq
    motor_control.ud_ref = easy_mc_pi_process(motor_control.id, motor_control.id_ref, &id_pid);
    motor_control.uq_ref = easy_mc_pi_process(motor_control.iq, motor_control.iq_ref, &iq_pid);
}

static void easy_mc_foc_speed_loop_step(float angle_el)
{
    motor_control.id_ref = 0;
    motor_control.iq_ref = easy_mc_pid_process(abz_encoder_handle.speed, motor_control.speed_ref, &speed_pid);
    easy_mc_foc_iqd_loop_step(angle_el);
}

static void easy_mc_foc_position_loop_step(float angle_el)
{
    motor_control.id_ref = 0;
    motor_control.iq_ref = easy_mc_pid_process(abz_encoder_handle.position, motor_control.position_ref, &position_pid);
    easy_mc_foc_iqd_loop_step(angle_el);
}

static void easy_mc_foc_position_with_speed_limit_loop_step(float angle_el)
{
    float speed_ref = easy_mc_pid_process(abz_encoder_handle.position, motor_control.position_ref, &position_speed_limit_pid);
    motor_control.id_ref = 0;
    motor_control.iq_ref = easy_mc_pid_process(abz_encoder_handle.speed, speed_ref, &speed_pid);
    easy_mc_foc_iqd_loop_step(angle_el);
}

static void easy_mc_foc_iqd_with_position_limit_loop_step(float angle_el)
{
    if (motor_control.position_limit != 0)
    {
        if (abz_encoder_handle.position > 0)
        {
            if (abz_encoder_handle.position > motor_control.position_limit)
            {
                if (motor_control.iq_ref > 0)
                {
                    motor_control.iq_ref = 0;
                }
                if (motor_control.id_ref > 0)
                {
                    motor_control.id_ref = 0;
                }
            }
        }
        else
        {
            if (abz_encoder_handle.position < -motor_control.position_limit)
            {
                if (motor_control.iq_ref < 0)
                {
                    motor_control.iq_ref = 0;
                }
                if (motor_control.id_ref < 0)
                {
                    motor_control.id_ref = 0;
                }
            }
        }
    }

    easy_mc_foc_iqd_loop_step(angle_el);
}

void easy_mc_foc_step(void)
{
    // calculate electrical angle
    float angle_el = electrical_angle(motor_control.theta);

    motor_control.sin_theta = EASY_MC_MATH_SIN(angle_el);
    motor_control.cos_theta = EASY_MC_MATH_COS(angle_el);

    switch (motor_control.control_mode)
    {
    case MOTOR_CONTROL_MODE_UQ:
        break;
    case MOTOR_CONTROL_MODE_IQ:
        easy_mc_foc_iqd_loop_step(angle_el);
        break;
    case MOTOR_CONTROL_MODE_SPEED:
        easy_mc_foc_speed_loop_step(angle_el);
        break;
    case MOTOR_CONTROL_MODE_POSITION:
        easy_mc_foc_position_loop_step(angle_el);
        break;
    case MOTOR_CONTROL_MODE_IQ_WITH_POSITION_LIMIT:
        easy_mc_foc_iqd_with_position_limit_loop_step(angle_el);
        break;
    case MOTOR_CONTROL_MODE_POSITION_SPEED_LIMIT:
        easy_mc_foc_position_with_speed_limit_loop_step(angle_el);
        break;
    default:
        return;
    }

    // In-Park Transform, https://zhuanlan.zhihu.com/p/12003413490
    float u_alpha = motor_control.ud_ref * motor_control.cos_theta - motor_control.uq_ref * motor_control.sin_theta;
    float u_beta = motor_control.ud_ref * motor_control.sin_theta + motor_control.uq_ref * motor_control.cos_theta;

    motor_control.u_alpha = u_alpha;
    motor_control.u_beta = u_beta;

    SVPWM(u_alpha, u_beta, EASY_MC_CONFIG_VBUS_VOLTAGE, EASY_MC_PWM_PERIOD_CYCLES * 2, motor_control.tcm);

    easy_mc_hw_set_u_v_w_count(motor_control.tcm[0], motor_control.tcm[1], motor_control.tcm[2]);
}

void easy_mc_foc_init(void)
{
    //   memset(&motor_control, 0, sizeof(motor_control));
    easy_mc_pid_history_init(&iq_pid);
    easy_mc_pid_history_init(&id_pid);
    easy_mc_pid_history_init(&speed_pid);
    easy_mc_pid_history_init(&position_pid);
}
