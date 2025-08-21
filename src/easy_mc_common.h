#ifndef _EASY_MC_COMMON_H_
#define _EASY_MC_COMMON_H_

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "easy_mc_config.h"

#if EASY_MC_CONFIG_ENABLE_ARM_DSP
#include <arm_math.h>
#endif

/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef EASY_MC_LOG_IMPL
#error "EASY_MC_LOG_IMPL not defined"
#endif

#ifndef EASY_MC_TIME_MS_GET
#error "EASY_MC_TIME_MS_GET not defined"
#endif

#define EASY_MC_LOG_IMPL_LEVEL_NONE 0U
#define EASY_MC_LOG_IMPL_LEVEL_ERR  1U
#define EASY_MC_LOG_IMPL_LEVEL_WRN  2U
#define EASY_MC_LOG_IMPL_LEVEL_INF  3U
#define EASY_MC_LOG_IMPL_LEVEL_DBG  4U

#if EASY_MC_CONFIG_DEBUG_LOG_LEVEL >= EASY_MC_LOG_IMPL_LEVEL_ERR
#define EASY_MC_LOG_ERR EASY_MC_LOG_IMPL
#else
#define EASY_MC_LOG_ERR(fmt, ...)
#endif
#if EASY_MC_CONFIG_DEBUG_LOG_LEVEL >= EASY_MC_LOG_IMPL_LEVEL_WRN
#define EASY_MC_LOG_WRN EASY_MC_LOG_IMPL
#else
#define EASY_MC_LOG_WRN(fmt, ...)
#endif
#if EASY_MC_CONFIG_DEBUG_LOG_LEVEL >= EASY_MC_LOG_IMPL_LEVEL_INF
#define EASY_MC_LOG_INF EASY_MC_LOG_IMPL
#else
#define EASY_MC_LOG_INF(fmt, ...)
#endif
#if EASY_MC_CONFIG_DEBUG_LOG_LEVEL >= EASY_MC_LOG_IMPL_LEVEL_DBG
#define EASY_MC_LOG_DBG EASY_MC_LOG_IMPL
#else
#define EASY_MC_LOG_DBG(fmt, ...)
#endif

/* Apparently this is needed by several Windows compilers */
// #if !defined(__MACH__)
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif /* NULL */
// #endif /* ! Mac OS X - breaks precompiled headers */

/**
 * \brief           Get maximal value between 2 values
 * \param[in]       x: First value
 * \param[in]       y: Second value
 * \retval          Maximal value
 * \hideinitializer
 */
#define EASY_MC_MAX(x, y) ((x) > (y) ? (x) : (y))

/**
 * \brief           Get minimal value between 2 values
 * \param[in]       x: First value
 * \param[in]       y: Second value
 * \retval          Minimal value
 * \hideinitializer
 */
#define EASY_MC_MIN(x, y) ((x) < (y) ? (x) : (y))

/**
 * \brief           Get absolute value of input
 * \param[in]       x: Input value
 * \retval          Absolute value of input
 * \hideinitializer
 */
#define EASY_MC_ABS(x) ((x) >= 0 ? (x) : -(x))

/**
 * \brief           Limit input value between min and max values
 * \param[in]       x: Input value
 * \param[in]       min: Minimum value
 * \param[in]       max: Maximum value
 * \retval          Limited value
 */
#define EASY_MC_LIMIT_MIN_MAX(_x, _min, _max) EASY_MC_MAX(EASY_MC_MIN((_x), (_max)), (_min))

/**
 * \brief           Limit input value between 2 values
 * \param[in]       x: Input value
 * \param[in]       value0: First value
 * \param[in]       value1: Second value
 * \retval          Limited value
 */
#define EASY_MC_LIMIT_CHECK_MIN_MAX(_x, _value0, _value1) EASY_MC_LIMIT_MIN_MAX(_x, EASY_MC_MIN(_value0, _value1), EASY_MC_MAX(_value0, _value1))

#define EASY_MC_SWAP(a, b) (a = (a) + (b), b = (a) - (b), a = (a) - (b))

#define EASY_MC_ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define EASY_MC_UNUSED(_x) (void)(_x)

#define __EASY_MC_WEAK__ __attribute__((weak))

#define __EASY_MC_STATIC_INLINE__ static inline

#define EASY_MC_MATH_PI  3.14159265358979323846f // pi
#define EASY_MC_MATH_2PI 6.28318530717958647693f // 2pi

#define EASY_MC_MATH_SQRT3   1.7320508075688f
#define EASY_MC_MATH_SQRT3_2 0.8660254037844f

#if EASY_MC_CONFIG_ENABLE_ARM_DSP
#define EASY_MC_MATH_COS(_phase) arm_cos_f32(_phase)
#define EASY_MC_MATH_SIN(_phase) arm_sin_f32(_phase)
#else
#define EASY_MC_MATH_COS(_phase) cosf(_phase)
#define EASY_MC_MATH_SIN(_phase) sinf(_phase)
#endif

#define EASY_MC_DEG_TO_RAD(_deg) ((_deg) * (EASY_MC_MATH_PI / 180.0f))
#define EASY_MC_RAD_TO_DEG(_rad) ((_rad) * (180.0f / EASY_MC_MATH_PI))

#define EASY_MC_SPEED_RPM_TO_DEG(_speed) ((_speed / 60.0f / EASY_MC_CONFIG_PWM_FREQUENCY) * 360.0f)
#define EASY_MC_SPEED_RPM_TO_RAD(_speed) ((_speed / 60.0f / EASY_MC_CONFIG_PWM_FREQUENCY) * (2 * EASY_MC_MATH_PI))

// Control Timeing parameters
#define EASY_MC_SAMPLE_TIME (1.0f / (float)EASY_MC_CONFIG_PWM_FREQUENCY)

// Reference: https://zhuanlan.zhihu.com/p/454914546 & https://www.bilibili.com/video/BV1MC4y137CT
#define EASY_MC_CURRENT_PI_BANDWIDTH (EASY_MC_CONFIG_PWM_FREQUENCY / 4)
#define EASY_MC_CURRENT_PI_COEX (EASY_MC_CONFIG_CURRENT_MAX / EASY_MC_CONFIG_VBUS_VOLTAGE)

#define EASY_MC_PULSE_NBR         (4 * EASY_MC_CONFIG_ENCODER_PPR)
#define EASY_MC_PWM_PERIOD_CYCLES (((EASY_MC_CONFIG_ADV_TIM_CLK_MHz * 1000000) / EASY_MC_CONFIG_PWM_FREQUENCY) / 2)

// SVPWM max voltage is VBUS/sqrt(3)
#define EASY_MC_SVPWM_MAX_VOLTAGE (EASY_MC_CONFIG_VBUS_VOLTAGE / EASY_MC_MATH_SQRT3)

// Zero align index
#define EASY_MC_ZERO_ALIGN_MAX_COUNT   (EASY_MC_CONFIG_ENCODER_ZERO_ALIGN_TIMEOUT_MS * EASY_MC_CONFIG_PWM_FREQUENCY / 1000.0f)
#define EASY_MC_ZERO_ALIGN_UD_INCREASE (EASY_MC_CONFIG_ZERO_ALIGN_MAX_VOLTAGE / EASY_MC_ZERO_ALIGN_MAX_COUNT)

#define EASY_MC_ZERO_ALIGN_SPEED_MAX_COUNT   (EASY_MC_CONFIG_ENCODER_ZERO_ALIGN_SPEED_TIMEOUT_MS * EASY_MC_CONFIG_PWM_FREQUENCY / 1000.0f)
#define EASY_MC_ZERO_ALIGN_SPEED_UD_INCREASE (EASY_MC_CONFIG_ZERO_ALIGN_MAX_VOLTAGE / EASY_MC_ZERO_ALIGN_SPEED_MAX_COUNT)

#define MOTOR_CONVERT_CURRENT_TO_FORCE(_current) (_current / EASY_MC_CONFIG_CURRENT_MAX)
#define MOTOR_CONVERT_FORCE_TO_CURRENT(_force)   (_force * EASY_MC_CONFIG_CURRENT_MAX)

typedef enum
{
    MOTOR_STATE_INIT = 0,
    MOTOR_STATE_ADC_INIT = 1,
    MOTOR_STATE_ELECTRIC_ZERO_INIT = 2,
    MOTOR_STATE_MECHANICAL_ZERO_INIT = 3,
    MOTOR_STATE_ENABLE = 4,
    MOTOR_STATE_DISABLE = 5,
} easy_mc_state_t;

typedef enum
{
    MOTOR_CONTROL_MODE_UQ = 0,
    MOTOR_CONTROL_MODE_IQ = 1,
    MOTOR_CONTROL_MODE_SPEED = 2,
    MOTOR_CONTROL_MODE_POSITION = 3,
    MOTOR_CONTROL_MODE_IQ_WITH_POSITION_LIMIT = 4,
} easy_mc_control_mode_t;

typedef enum
{
    MOTOR_THETA_MODE_OPEN_INC = 0,
    MOTOR_THETA_MODE_ENCODER_ABZ = 1,
    MOTOR_THETA_MODE_FIX = 2,
} easy_mc_theta_mode_t;

typedef struct
{
    float q;
    float d;
} easy_mc_qd_t;

typedef struct
{
    easy_mc_state_t state; // state machine

    easy_mc_control_mode_t control_mode; // control mode

    easy_mc_theta_mode_t theta_mode; // theta mode
    float theta_last;                // last theta value, unit radian
    float theta_inc;                 // cycle increment, unit radian
    float theta_fix;                 // fixed angle, unit radian

    int is_zero_increase_mode; // work timeout counter, unit: cycle
    int32_t work_timeout_cnt;  // work timeout counter, unit: cycle

    uint16_t ia_adc, ib_adc, ic_adc; // Current ADC value

    // ADC offset
    uint32_t adc_offset_cnt;
    uint16_t ia_adc_offset;
    uint16_t ib_adc_offset;
    uint16_t ic_adc_offset;

    // Control params
    float ia_origin;
    float ib_origin;
    float ic_origin;

    float ia;
    float ib;
    float ic;
    float i_bus;
    float v_bus;
    float i_alpha;
    float i_beta;
    float id_ref;
    float id;
    float iq_ref;
    float iq;
    float u_alpha;
    float u_beta;

    float theta;
    float sin_theta;
    float cos_theta;

    float position_limit; // Position limit, unit: radian

    float position_ref; // position reference, unit: radian
    float speed_ref;    // speed reference, unit: rpm
    float ud_ref;
    float uq_ref;

    // tcm1, tcm2, tcm3
    float tcm[3];

} easy_mc_motor_control_t;

typedef struct
{
    // User api params
    uint8_t params_control_is_set;        // is user control params set
    uint8_t params_theta_is_set;          // is user theta params set 是否设置了用户角度参数
    uint8_t params_control_enable_is_set; // is user control enable set

    // params in params_control_is_set
    easy_mc_control_mode_t params_control_mode;
    easy_mc_qd_t params_i_qd_ref;
    float params_position_limit;
    float params_speed_ref;
    float params_position_ref;
    easy_mc_qd_t params_u_qd_ref;

    // params in params_theta_is_set
    easy_mc_theta_mode_t params_theta_mode; // theta mode
    float params_theta_inc;                 // theta increment in one cycle, unit radian
    float params_theta_fix;                 // fixed angle, unit radian

    // params in params_control_enable_is_set
    uint8_t params_is_control_enable; // is enable/disable motor control
} easy_mc_motor_control_user_command_t;

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif

#endif /* _EASY_MC_COMMON_H_ */
