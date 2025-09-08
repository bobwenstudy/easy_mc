#include <string.h>
#include "easy_mc_abz_encoder.h"

#include "easy_mc.h"

#include "easy_mc_user_interface.h"

easy_mc_motor_control_t motor_control;

static easy_mc_filter_low_pass_t ia_filter;
static easy_mc_filter_low_pass_t ib_filter;
static easy_mc_filter_low_pass_t ic_filter;

void easy_mc_adc_debug_data(void);

__EASY_MC_WEAK__ void easy_mc_isr_task_high_frequency_user_handle(void)
{
}

__EASY_MC_WEAK__ void easy_mc_isr_task_low_frequency_user_handle(void)
{
}

__EASY_MC_WEAK__ void easy_mc_isr_set_zero_user_handle(void)
{
}

const char *get_mc_motor_status_str(easy_mc_state_t state)
{
    switch (state)
    {
    case MOTOR_STATE_INIT:
        return "MOTOR_STATE_INIT";
    case MOTOR_STATE_ADC_INIT:
        return "MOTOR_STATE_ADC_INIT";
    case MOTOR_STATE_ELECTRIC_ZERO_INIT:
        return "MOTOR_STATE_ELECTRIC_ZERO_INIT";
    case MOTOR_STATE_MECHANICAL_ZERO_INIT:
        return "MOTOR_STATE_MECHANICAL_ZERO_INIT";
    case MOTOR_STATE_ENABLE:
        return "MOTOR_STATE_ENABLE";
    case MOTOR_STATE_DISABLE:
        return "MOTOR_STATE_DISABLE";
    default:
        return "UNKNOWN";
    }
}

const char *get_mc_control_state_str(easy_mc_control_mode_t state)
{
    switch (state)
    {
    case MOTOR_CONTROL_MODE_UQ:
        return "MOTOR_CONTROL_MODE_UQ";
    case MOTOR_CONTROL_MODE_IQ:
        return "MOTOR_CONTROL_MODE_IQ";
    case MOTOR_CONTROL_MODE_SPEED:
        return "MOTOR_CONTROL_MODE_SPEED";
    case MOTOR_CONTROL_MODE_POSITION:
        return "MOTOR_CONTROL_MODE_POSITION";
    case MOTOR_CONTROL_MODE_IQ_WITH_POSITION_LIMIT:
        return "MOTOR_CONTROL_MODE_IQ_WITH_POSITION_LIMIT";
    default:
        return "UNKNOWN";
    }
}

const char *get_mc_theta_state_str(easy_mc_theta_mode_t state)
{
    switch (state)
    {
    case MOTOR_THETA_MODE_OPEN_INC:
        return "MOTOR_THETA_MODE_OPEN_INC";
    case MOTOR_THETA_MODE_ENCODER_ABZ:
        return "MOTOR_THETA_MODE_ENCODER_ABZ";
    case MOTOR_THETA_MODE_FIX:
        return "MOTOR_THETA_MODE_FIX";
    default:
        return "UNKNOWN";
    }
}

static int mc_motor_status_last = -1;
static int mc_control_mode_last = -1;
static int mc_theta_mode_last = -1;

void easy_mc_debug_polling_state_change(void)
{
    if (mc_motor_status_last != motor_control.state)
    {
        EASY_MC_LOG_INF("Motor state change to %s\r\n", get_mc_motor_status_str(motor_control.state));
        if (mc_motor_status_last != MOTOR_STATE_ELECTRIC_ZERO_INIT && mc_motor_status_last != MOTOR_STATE_MECHANICAL_ZERO_INIT)
        {
            easy_mc_adc_debug_data();
        }
        mc_motor_status_last = motor_control.state;
    }
    if (mc_control_mode_last != motor_control.control_mode)
    {
        EASY_MC_LOG_INF("Motor control mode change to %s\r\n", get_mc_control_state_str(motor_control.control_mode));
        mc_control_mode_last = motor_control.control_mode;
    }

    if (mc_theta_mode_last != motor_control.theta_mode)
    {
        EASY_MC_LOG_INF("Motor theta mode change to %s\r\n", get_mc_theta_state_str(motor_control.theta_mode));
        mc_theta_mode_last = motor_control.theta_mode;
    }
}

static void motor_init_params(void)
{
    // motor_control.id_ref = 0;
    // motor_control.iq_ref = 0;

    // motor_control.ud_ref = 0;
    // motor_control.uq_ref = 0;

    // motor_control.Speed_ref = 0;
    // motor_control.torque_ref = 0;
    // motor_control.position_ref = 0;
#if EASY_MC_CONFIG_CURRENT_FILTER_END_FREQ > 0
    easy_mc_filter_low_pass_init(&ia_filter, EASY_MC_CONFIG_CURRENT_FILTER_END_FREQ, 1.0f / EASY_MC_CONFIG_PWM_FREQUENCY);
    easy_mc_filter_low_pass_init(&ib_filter, EASY_MC_CONFIG_CURRENT_FILTER_END_FREQ, 1.0f / EASY_MC_CONFIG_PWM_FREQUENCY);
    easy_mc_filter_low_pass_init(&ic_filter, EASY_MC_CONFIG_CURRENT_FILTER_END_FREQ, 1.0f / EASY_MC_CONFIG_PWM_FREQUENCY);
#endif
}

static void motor_stop(void)
{
    EASY_MC_LOG_DBG("motor_stop\r\n");
    motor_init_params();

    easy_mc_foc_init();

    easy_mc_hw_stop();
}

static void motor_start(void)
{
    EASY_MC_LOG_DBG("motor_start\r\n");

    motor_init_params();

    easy_mc_foc_init();

    easy_mc_hw_start();
}

void easy_mc_init(void)
{
    // Initialize the motor control structure to 0
    memset(&motor_control, 0, sizeof(motor_control));

    // Initialize motor state
    motor_control.state = MOTOR_STATE_INIT;
    // Initialize bus voltage
    motor_control.v_bus = EASY_MC_CONFIG_VBUS_VOLTAGE;

    // Initialize user control interface
    easy_mc_user_interface_init();

    // Initialize encoder
    easy_mc_abz_encoder_init();
    // Initialize parameters
    motor_init_params();

    // Initialize hardware
    easy_mc_hw_init();

    // Set motor state to ADC initialization state
    motor_control.state = MOTOR_STATE_ADC_INIT;

    // Print current configuration
    EASY_MC_LOG_DBG("Motor init, VBUS: %fV, PWM: %dHz, RS: %fohm, LS: %fH, PID_I:(Kp:%f, Ki:%f)\r\n", EASY_MC_CONFIG_VBUS_VOLTAGE, EASY_MC_CONFIG_PWM_FREQUENCY,
                    EASY_MC_CONFIG_RS, EASY_MC_CONFIG_LS, EASY_MC_CONFIG_PID_CURRENT_KP, EASY_MC_CONFIG_PID_CURRENT_KI);
}

void easy_mc_deinit(void)
{
    motor_stop();

    easy_mc_hw_deinit();
}

void easy_mc_isr_set_zero(void)
{
#if EASY_MC_CONFIG_DEBUG_CUSTOM_IRQ_HANDLE
    easy_mc_isr_set_zero_user_handle();
#else
    // EASY_MC_LOG_INF("easy_mc_isr_set_zero\n");
    if (abz_encoder_handle.zero_flag == 0)
    {
        uint16_t current_count;
        if (motor_control.state != MOTOR_STATE_MECHANICAL_ZERO_INIT)
        {
            return;
        }
        easy_mc_abz_encoder_update();

        // reset positiom
        abz_encoder_handle.position = 0;

        abz_encoder_handle.zero_flag = 1;

        // Update motor status
        motor_control.state = MOTOR_STATE_ENABLE;

        // easy_mc_user_set_theta_abz();
        motor_control.theta_mode = MOTOR_THETA_MODE_ENCODER_ABZ;

        // easy_mc_user_motor_control
        motor_control.control_mode = MOTOR_CONTROL_MODE_IQ;
        motor_control.iq_ref = 0;
        motor_control.id_ref = 0;
    }
#endif
}

#define EASY_MC_ADC_OFFSET_CALC_MAX_CNT 20
static uint32_t adc_offset_array_a[EASY_MC_ADC_OFFSET_CALC_MAX_CNT];
static uint32_t adc_offset_array_b[EASY_MC_ADC_OFFSET_CALC_MAX_CNT];
static uint32_t adc_offset_array_c[EASY_MC_ADC_OFFSET_CALC_MAX_CNT];

void easy_mc_adc_debug_data(void)
{
    for (int i = 0; i < EASY_MC_ADC_OFFSET_CALC_MAX_CNT; i++)
    {
        EASY_MC_LOG_DBG("[%d], ADC_A:%lu, ADC_B:%lu, ADC_C:%lu\r\n", i, adc_offset_array_a[i], adc_offset_array_b[i], adc_offset_array_c[i]);
    }
    EASY_MC_LOG_DBG("ia_adc_offset: %u, ib_adc_offset: %u, ic_adc_offset: %u\r\n", motor_control.ia_adc_offset, motor_control.ib_adc_offset,
                    motor_control.ic_adc_offset);
}

bool easy_mc_adc_offset_init(void)
{
    uint32_t sum_a, sum_b, sum_c;
    uint32_t cnt = motor_control.adc_offset_cnt;

    if (cnt < EASY_MC_ADC_OFFSET_CALC_MAX_CNT)
    {
        adc_offset_array_a[cnt] = motor_control.ia_adc;
        adc_offset_array_b[cnt] = motor_control.ib_adc;
        adc_offset_array_c[cnt] = motor_control.ic_adc;
    }
    else
    {
        sum_a = 0;
        sum_b = 0;
        sum_c = 0;
        for (int i = 0; i < EASY_MC_ADC_OFFSET_CALC_MAX_CNT; i++)
        {
            sum_a += adc_offset_array_a[i];
            sum_b += adc_offset_array_b[i];
            sum_c += adc_offset_array_c[i];
        }
        motor_control.ia_adc_offset = sum_a / EASY_MC_ADC_OFFSET_CALC_MAX_CNT;
        motor_control.ib_adc_offset = sum_b / EASY_MC_ADC_OFFSET_CALC_MAX_CNT;
        motor_control.ic_adc_offset = sum_c / EASY_MC_ADC_OFFSET_CALC_MAX_CNT;

        return true;
    }

    motor_control.adc_offset_cnt++;

    return false;
}

static float find_max_index(float arr[], int size)
{
    int max_index = 0;
    for (int i = 1; i < size; i++)
    {
        if (arr[i] > arr[max_index])
        {
            max_index = i;
        }
    }
    return max_index;
}

static void easy_mc_enable_process(void)
{
    static uint16_t ia_adc, ib_adc, ic_adc, IbusU16;
    float ia, ib, ic, IBus;
    float sensor_angle = 0;

    switch (motor_control.theta_mode)
    {
    case MOTOR_THETA_MODE_OPEN_INC:
        sensor_angle = motor_control.theta_last + motor_control.theta_inc;
        if (sensor_angle > EASY_MC_MATH_2PI)
        {
            sensor_angle -= EASY_MC_MATH_2PI;
        }
        else if (sensor_angle < -EASY_MC_MATH_2PI)
        {
            sensor_angle += EASY_MC_MATH_2PI;
        }
        motor_control.theta_last = sensor_angle;
        break;
    case MOTOR_THETA_MODE_ENCODER_ABZ:
        sensor_angle = abz_encoder_handle.angle;
        break;
    case MOTOR_THETA_MODE_FIX:
        sensor_angle = motor_control.theta_fix;
        break;
    }

    // Current Convert
#if EASY_MC_CONFIG_CURRENT_FILTER_END_FREQ > 0
    ia_adc = easy_mc_filter_low_pass_process(&ia_filter, motor_control.ia_adc);
    ib_adc = easy_mc_filter_low_pass_process(&ib_filter, motor_control.ib_adc);
    ic_adc = easy_mc_filter_low_pass_process(&ic_filter, motor_control.ic_adc);
#else
    ia_adc = motor_control.ia_adc;
    ib_adc = motor_control.ib_adc;
    ic_adc = motor_control.ic_adc;
#endif

    ia = (ia_adc - motor_control.ia_adc_offset) * EASY_MC_CONFIG_ADC_CONV_VALUE;
    ib = (ib_adc - motor_control.ib_adc_offset) * EASY_MC_CONFIG_ADC_CONV_VALUE;
    ic = (ic_adc - motor_control.ic_adc_offset) * EASY_MC_CONFIG_ADC_CONV_VALUE;

    motor_control.ia_origin = ia;
    motor_control.ib_origin = ib;
    motor_control.ic_origin = ic;
    // Consider the duty cycle of the current in one phase is too low, causing sampling current jitter. Synthesize this current with the other two phases.
    // https://blog.csdn.net.xingsongyu/article/details/120302894
    int max_tcm_index = find_max_index(motor_control.tcm, 3);
    if (max_tcm_index == 0)
    {
        ia = -(ib + ic);
    }
    else if (max_tcm_index == 1)
    {
        ib = -(ia + ic);
    }
    else if (max_tcm_index == 2)
    {
        ic = -(ia + ib);
    }

    // Calculate three-phase current
    motor_control.ia = ia;
    motor_control.ib = ib;
    motor_control.ic = ic;

    motor_control.theta = sensor_angle;
    // Run FOC model
    easy_mc_foc_step();

    // Update motor debug information
    g_easy_mc_debug_vofa_data.ia = ia;
    g_easy_mc_debug_vofa_data.ib = ib;
    g_easy_mc_debug_vofa_data.ic = ic;
    g_easy_mc_debug_vofa_data.angle = abz_encoder_handle.angle;
    g_easy_mc_debug_vofa_data.id_ref = motor_control.id_ref;
    g_easy_mc_debug_vofa_data.id = motor_control.id;
    g_easy_mc_debug_vofa_data.iq_ref = motor_control.iq_ref;
    g_easy_mc_debug_vofa_data.iq = motor_control.iq;
    g_easy_mc_debug_vofa_data.ud = motor_control.ud_ref;
    g_easy_mc_debug_vofa_data.uq = motor_control.uq_ref;
    g_easy_mc_debug_vofa_data.u_alpha = motor_control.u_alpha;
    g_easy_mc_debug_vofa_data.u_beta = motor_control.u_beta;
    g_easy_mc_debug_vofa_data.tcm1 = motor_control.tcm[0];
    g_easy_mc_debug_vofa_data.tcm2 = motor_control.tcm[1];
    g_easy_mc_debug_vofa_data.tcm3 = motor_control.tcm[2];
    g_easy_mc_debug_vofa_data.speed = abz_encoder_handle.speed;
    g_easy_mc_debug_vofa_data.speed_ref = motor_control.speed_ref;
    g_easy_mc_debug_vofa_data.position = abz_encoder_handle.position;
    g_easy_mc_debug_vofa_data.position_ref = motor_control.position_ref;
}

void easy_mc_high_freq_task_loop(void)
{
    switch (motor_control.state)
    {
    case MOTOR_STATE_INIT:
        break;
    case MOTOR_STATE_ADC_INIT:
        if (easy_mc_adc_offset_init())
        {
            // switch next state
            motor_control.state = MOTOR_STATE_ELECTRIC_ZERO_INIT;

            motor_control.is_zero_increase_mode = 1;

            // start align zero process
            // clear abz encoder zero flag
            abz_encoder_handle.zero_flag = 0;

            // easy_mc_user_set_theta_fix
            motor_control.theta_fix = 0.0f;
            motor_control.theta_mode = MOTOR_THETA_MODE_FIX;
            // easy_mc_user_set_u_ref
            motor_control.uq_ref = 0.0f;
            motor_control.ud_ref = 0.0f;
            motor_control.control_mode = MOTOR_CONTROL_MODE_UQ;

            // start motor
            motor_start();
        }
        break;
    case MOTOR_STATE_ELECTRIC_ZERO_INIT:
        if (motor_control.is_zero_increase_mode)
        {
            motor_control.ud_ref += EASY_MC_ZERO_ALIGN_UD_INCREASE;
            // Check if the motor is ready to zero position
            if (motor_control.ud_ref >= EASY_MC_CONFIG_ZERO_ALIGN_MAX_VOLTAGE)
            {
                // reach electric zero.
                easy_mc_abz_encoder_init();

                motor_control.is_zero_increase_mode = 0;
            }
        }
        else
        {
            motor_control.ud_ref -= EASY_MC_ZERO_ALIGN_UD_INCREASE * 2;
            if (motor_control.ud_ref <= 0)
            {
#if EASY_MC_CONFIG_MECHANICAL_ALIGN_ENABLE
                // switch next state
                motor_control.state = MOTOR_STATE_MECHANICAL_ZERO_INIT;

                // 2s timeout work
                motor_control.work_timeout_cnt = EASY_MC_ZERO_ALIGN_SPEED_MAX_COUNT;

                motor_control.theta_mode = MOTOR_THETA_MODE_ENCODER_ABZ;

                motor_control.speed_ref = EASY_MC_CONFIG_ENCODER_ZERO_ALIGN_SPEED_RPM;
                motor_control.control_mode = MOTOR_CONTROL_MODE_SPEED;
#else
                // reset positiom
                abz_encoder_handle.position = 0;

                abz_encoder_handle.zero_flag = 1;

                // Update motor status
                motor_control.state = MOTOR_STATE_ENABLE;

                // easy_mc_user_set_theta_abz();
                motor_control.theta_mode = MOTOR_THETA_MODE_ENCODER_ABZ;

                // easy_mc_user_motor_control
                motor_control.control_mode = MOTOR_CONTROL_MODE_IQ;
                motor_control.iq_ref = 0;
                motor_control.id_ref = 0;
#endif
            }
        }
        easy_mc_enable_process();
        break;
    case MOTOR_STATE_MECHANICAL_ZERO_INIT:
        if (motor_control.work_timeout_cnt-- <= 0)
        {
            // TODO: timeout work.
            easy_mc_isr_set_zero();
        }
        easy_mc_enable_process();
        break;
    case MOTOR_STATE_ENABLE:
        easy_mc_enable_process();
        break;
    case MOTOR_STATE_DISABLE:
        break;
    default:
        break;
    }
}

bool easy_mc_check_enable(void)
{
    return (motor_control.state == MOTOR_STATE_ENABLE) ? true : false;
}

int easy_mc_get_state(void)
{
    return motor_control.state;
}

void easy_mc_control(bool status)
{
    if (status == true && !easy_mc_check_enable())
    {
        motor_start();
        motor_control.state = MOTOR_STATE_ENABLE;
    }
    else if (status == false && easy_mc_check_enable())
    {
        motor_stop();
        motor_control.state = MOTOR_STATE_DISABLE;
    }
}

void easy_mc_set_v_bus(float v_bus)
{
    motor_control.v_bus = v_bus;
    g_easy_mc_debug_vofa_data.v_bus = v_bus;
}

void easy_mc_set_i_bus(float i_bus)
{
    motor_control.i_bus = i_bus;
    g_easy_mc_debug_vofa_data.i_bus = i_bus;
}

float easy_mc_get_speed(void)
{
    return abz_encoder_handle.speed;
}

float easy_mc_get_angle(void)
{
    return abz_encoder_handle.angle;
}

float easy_mc_get_position(void)
{
    return abz_encoder_handle.position;
}

float easy_mc_get_position_degree(void)
{
    return abz_encoder_handle.position * (180.0f / EASY_MC_MATH_PI);
}

void easy_mc_isr_task_low_frequency(void)
{
#if EASY_MC_CONFIG_DEBUG_CUSTOM_IRQ_HANDLE
    easy_mc_isr_task_low_frequency_user_handle();
#else
    easy_mc_hw_debug_io_control(1, 1);

    // handle user control/theta command
    easy_mc_user_handle_command();

    // do some hardware task
    easy_mc_hw_1ms_task();

    // other debug
    // g_easy_mc_debug_vofa_data.debug_1 = easy_mc_hw_encoder_get_count();
    // g_easy_mc_debug_vofa_data.debug_2 = easy_mc_abz_encoder_cal_sensor_angle() / (2.0f * EASY_MC_MATH_PI) * 360.0f;

    // g_easy_mc_debug_vofa_data.gpio_output_u = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != 0) ? 1.0f : 0.0f;
    // g_easy_mc_debug_vofa_data.gpio_output_v = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) != 0) ? 3.0f : 2.0f;
    // g_easy_mc_debug_vofa_data.gpio_output_w = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) != 0) ? 5.0f : 4.0f;

    // easy_mc_pid_t* pid = easy_mc_foc_get_pid_controller(1);
    // g_easy_mc_debug_vofa_data.debug_0 = pid->err;
    // g_easy_mc_debug_vofa_data.debug_1 = pid->p_out;
    // g_easy_mc_debug_vofa_data.debug_2 = pid->i_out;
    // g_easy_mc_debug_vofa_data.debug_3 = pid->d_out;

    // g_easy_mc_debug_vofa_data.debug_2 = motor_control.ibus;

    // g_easy_mc_debug_vofa_data.debug_0 = motor_control.ibus;
    // g_easy_mc_debug_vofa_data.debug_1 = motor_control.ia_origin;
    // g_easy_mc_debug_vofa_data.debug_2 = motor_control.ib_origin;
    // g_easy_mc_debug_vofa_data.debug_3 = motor_control.ic_origin;

    g_easy_mc_debug_vofa_data.debug_0 = motor_control.state;
    g_easy_mc_debug_vofa_data.debug_1 = motor_control.control_mode;

    easy_mc_vofa_polling_send_data();
    easy_mc_hw_debug_io_control(1, 0);
#endif
}

// every adc event
void easy_mc_isr_task_high_frequency(void)
{
#if EASY_MC_CONFIG_DEBUG_CUSTOM_IRQ_HANDLE
    easy_mc_isr_task_high_frequency_user_handle();
#else
    easy_mc_hw_debug_io_control(0, 1);

    // update encoder info
    easy_mc_abz_encoder_update();

    // handle user control/theta command
    easy_mc_user_handle_command();

    easy_mc_high_freq_task_loop();
    easy_mc_hw_debug_io_control(0, 0);
#endif
}
