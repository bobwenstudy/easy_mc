#ifndef _EASY_MC_CONFIG_DEFAULT_H_
#define _EASY_MC_CONFIG_DEFAULT_H_

/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * Motor parameters
 * Number of pole pairs
 */
#ifndef EASY_MC_CONFIG_POLE_PAIRS
#define EASY_MC_CONFIG_POLE_PAIRS 4
#endif

/**
 * Motor parameters
 * Speed max, unit: rpm.
 */
#ifndef EASY_MC_CONFIG_SPEED_MAX
#define EASY_MC_CONFIG_SPEED_MAX 300.0f
#endif

/**
 * Motor parameters
 * Current max, unit: A.
 */
#ifndef EASY_MC_CONFIG_CURRENT_MAX
#define EASY_MC_CONFIG_CURRENT_MAX 3.0f
#endif

/**
 * Motor parameters
 * Max voltage of the ZERO align, unit: V.
 */
#ifndef EASY_MC_CONFIG_ZERO_ALIGN_MAX_VOLTAGE
#define EASY_MC_CONFIG_ZERO_ALIGN_MAX_VOLTAGE (EASY_MC_SVPWM_MAX_VOLTAGE / 4)
#endif

/**
 * Motor parameters
 * Enable mechanical align, 1: enable, 0: disable.
 */
#ifndef EASY_MC_CONFIG_MECHANICAL_ALIGN_ENABLE
#define EASY_MC_CONFIG_MECHANICAL_ALIGN_ENABLE 1
#endif

/**
 * Motor parameters
 * Rs value of the motor, unit: Ohm.
 */
#ifndef EASY_MC_CONFIG_RS
#define EASY_MC_CONFIG_RS 2.851f
#endif

/**
 * Motor parameters
 * Ls value of the motor, unit: H.
 */
#ifndef EASY_MC_CONFIG_LS
#define EASY_MC_CONFIG_LS 0.003221f
#endif

/**
 * Motor parameters
 * Magnetic flux constant value of the motor.
 * M = Vpp/2/(2*PI)/sqrt(3)/F
 * M = (23.4V)/2/(2*PI)/sqrt(3)/(10.3Hz) = (23.4V)/ 21.765591999418080224f / (10.3Hz)
 * ref: https://zhuanlan.zhihu.com/p/25447946579
 */
#ifndef EASY_MC_CONFIG_MAGNETIC_FLUX_CONSTANT
#define EASY_MC_CONFIG_MAGNETIC_FLUX_CONSTANT 0.10437780237f
#endif

/**
 * Encoder parameters
 * Number of pulses per revolution
 */
#ifndef EASY_MC_CONFIG_ENCODER_PPR
#define EASY_MC_CONFIG_ENCODER_PPR 2500
#endif

/**
 * Encoder parameters
 * Zero align timeout, unit: ms.
 */
#ifndef EASY_MC_CONFIG_ENCODER_ZERO_ALIGN_TIMEOUT_MS
#define EASY_MC_CONFIG_ENCODER_ZERO_ALIGN_TIMEOUT_MS 500
#endif

/**
 * Encoder parameters
 * Zero align speed, unit: rpm.
 */
#ifndef EASY_MC_CONFIG_ENCODER_ZERO_ALIGN_SPEED_RPM
#define EASY_MC_CONFIG_ENCODER_ZERO_ALIGN_SPEED_RPM 50
#endif

/**
 * Encoder parameters
 * Zero align speed timeout, unit: ms.
 */
#ifndef EASY_MC_CONFIG_ENCODER_ZERO_ALIGN_SPEED_TIMEOUT_MS
#define EASY_MC_CONFIG_ENCODER_ZERO_ALIGN_SPEED_TIMEOUT_MS 5000
#endif

/**
 * ADC parameters
 * ADC conversion value
 * 3.3 / (4096(ADC 12bit) * 0.005(RSHUNT) * Gain(7.333))
 */
#ifndef EASY_MC_CONFIG_ADC_CONV_VALUE
#define EASY_MC_CONFIG_ADC_CONV_VALUE (0.02197265625f)
#endif

/**
 * ADC parameters
 * ADC conversion value
 * 3.3 / (4096(ADC 12bit) * 0.005(RSHUNT) * Gain(10.476f))
 */
#ifndef EASY_MC_CONFIG_ADC_CONV_VALUE_IBUS
#define EASY_MC_CONFIG_ADC_CONV_VALUE_IBUS (0.015381139032f)
#endif

/**
 * PWM parameters
 * PWM CLK frequency
 */
#ifndef EASY_MC_CONFIG_ADV_TIM_CLK_MHz
#define EASY_MC_CONFIG_ADV_TIM_CLK_MHz (160)
#endif

/**
 * PWM parameters
 * PWM frequency
 */
#ifndef EASY_MC_CONFIG_PWM_FREQUENCY
#define EASY_MC_CONFIG_PWM_FREQUENCY 10000
#endif

/**
 * VBus parameters
 * VBus voltage
 */
#ifndef EASY_MC_CONFIG_VBUS_VOLTAGE
#define EASY_MC_CONFIG_VBUS_VOLTAGE 24.0f
#endif

/**
 * PID parameters
 * Current controller, Kp, need the RS and Ls value right. Reference: https://www.bilibili.com/video/BV1MC4y137CT
 */
#ifndef EASY_MC_CONFIG_PID_CURRENT_KP
#define EASY_MC_CONFIG_PID_CURRENT_KP (EASY_MC_CONFIG_LS * EASY_MC_CURRENT_PI_BANDWIDTH * EASY_MC_CURRENT_PI_COEX)
#endif

/**
 * PID parameters
 * Current controller, Ki, need the RS and Ls value right. https://www.bilibili.com/video/BV1MC4y137CT
 */
#ifndef EASY_MC_CONFIG_PID_CURRENT_KI
#define EASY_MC_CONFIG_PID_CURRENT_KI (EASY_MC_CONFIG_RS * EASY_MC_CURRENT_PI_BANDWIDTH * EASY_MC_CURRENT_PI_COEX)
#endif

/**
 * PID parameters
 * Speed controller, Kp, use speed/current relation to calculate.
 */
#ifndef EASY_MC_CONFIG_PID_SPEED_KP
#define EASY_MC_CONFIG_PID_SPEED_KP 0.015f
#endif

/**
 * PID parameters
 * Speed controller, Ki
 */
#ifndef EASY_MC_CONFIG_PID_SPEED_KI
#define EASY_MC_CONFIG_PID_SPEED_KI 0.025f
#endif

/**
 * PID parameters
 * Position controller, Kp
 */
#ifndef EASY_MC_CONFIG_PID_POSITION_KP
#define EASY_MC_CONFIG_PID_POSITION_KP 0.637f
#endif

/**
 * PID parameters
 * Position controller, Ki
 */
#ifndef EASY_MC_CONFIG_PID_POSITION_KI
#define EASY_MC_CONFIG_PID_POSITION_KI 0.1f
#endif

/**
 * PID parameters
 * Position speed limit controller, Kp
 */
#ifndef EASY_MC_CONFIG_PID_POSITION_SPEED_LIMIT_KP
#define EASY_MC_CONFIG_PID_POSITION_SPEED_LIMIT_KP 50.0f
#endif

/**
 * PID parameters
 * Position speed limit controller, Ki
 */
#ifndef EASY_MC_CONFIG_PID_POSITION_SPEED_LIMIT_KI
#define EASY_MC_CONFIG_PID_POSITION_SPEED_LIMIT_KI 50.0f
#endif

/**
 * Motor Current Filter parameters
 * Motor Current Filter end frequence, unit: Hz.
 */
#ifndef EASY_MC_CONFIG_CURRENT_FILTER_END_FREQ
#define EASY_MC_CONFIG_CURRENT_FILTER_END_FREQ 200
#endif

/**
 * Compiler options.
 * Enable arm dsp.
 */
#ifndef EASY_MC_CONFIG_ENABLE_ARM_DSP
#define EASY_MC_CONFIG_ENABLE_ARM_DSP 0
#endif

/**
 * Debug options.
 * For user process irq task.
 */
#ifndef EASY_MC_CONFIG_DEBUG_CUSTOM_IRQ_HANDLE
#define EASY_MC_CONFIG_DEBUG_CUSTOM_IRQ_HANDLE 0
#endif

/**
 * Debug options.
 * For log level. EASY_MC_LOG_IMPL_LEVEL_NONE, EASY_MC_LOG_IMPL_LEVEL_ERR, EASY_MC_LOG_IMPL_LEVEL_WRN, EASY_MC_LOG_IMPL_LEVEL_INF, EASY_MC_LOG_IMPL_LEVEL_DBG
 */
#ifndef EASY_MC_CONFIG_DEBUG_LOG_LEVEL
#define EASY_MC_CONFIG_DEBUG_LOG_LEVEL EASY_MC_LOG_IMPL_LEVEL_DBG
#endif

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif

#endif /* _EASY_TOOLS_CONFIG_DEFAULT_H_ */
