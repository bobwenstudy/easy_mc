#ifndef _APP_EASY_MC_CONFIG_H_
#define _APP_EASY_MC_CONFIG_H_

#include "main.h"

/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

#define EASY_MC_TIME_MS_GET HAL_GetTick
#define EASY_MC_LOG_IMPL    printf

#define EASY_MC_CONFIG_ENABLE_ARM_DSP 1
#if 0
#define EASY_MC_CONFIG_POLE_PAIRS  7

// Encoder parameters
#define EASY_MC_CONFIG_ENCODER_PPR 5120
#else

// #define EASY_MC_CONFIG_PID_CURRENT_KP 1.0f
// #define EASY_MC_CONFIG_PID_CURRENT_KI 50.0f

// #define EASY_MC_CONFIG_PID_POSITION_KP 10.0f
// #define EASY_MC_CONFIG_PID_POSITION_KI 0.1f
#endif

// #define EASY_MC_CONFIG_DEBUG_LOG_LEVEL EASY_MC_LOG_IMPL_LEVEL_INF

/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif

#endif /* _APP_EASY_MC_CONFIG_H_ */
