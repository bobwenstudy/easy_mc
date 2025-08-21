#include <string.h>
#include <stdio.h>
#include <time.h>
#include <assert.h>
#include <stdbool.h>

#include <math.h>

#include "easy_mc.h"
#include "main.h"

static int is_hw_start = 0;
static int is_hw_init = 0;

#define TIM_MOTOR_PWM_OUT htim1
#define TIM_ENCODER_COUNT htim3
#define UART_VOFA_DEBUG   huart4

#define DEBUG_IO_2_Pin       GPIO_PIN_11
#define DEBUG_IO_2_GPIO_Port GPIOC
#define DEBUG_IO_3_Pin       GPIO_PIN_11
#define DEBUG_IO_3_GPIO_Port GPIOB

static int adc_ibus_offset = -1;

static uint16_t adc1_data[2];
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // EASY_MC_LOG_DBG("HAL_ADC_ConvCpltCallback\n");
    if (hadc->Instance == ADC1)
    {
        float vbus = (float)adc1_data[0] / 4096 * 3.3f * (2.7f + 47.0f) / 2.7f;
        float ibus = 0;
        if (adc_ibus_offset < 0)
        {
            adc_ibus_offset = adc1_data[1];
        }
        else
        {
            ibus = (adc_ibus_offset - adc1_data[1]) * EASY_MC_CONFIG_ADC_CONV_VALUE_IBUS;
        }

        easy_mc_set_v_bus(vbus);
        easy_mc_set_i_bus(ibus);
    }
}

void easy_mc_hw_1ms_task(void)
{
    // EASY_MC_LOG_DBG("easy_mc_hw_1ms_task\n");
    if (is_hw_init == 0)
    {
        return;
    }

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_data, sizeof(adc1_data) / sizeof(uint16_t));
}

void easy_mc_hw_debug_io_control(int io_index, int is_set)
{
    switch (io_index)
    {
    case 0:
        HAL_GPIO_WritePin(DEBUG_IO_0_GPIO_Port, DEBUG_IO_0_Pin, is_set ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    case 1:
        HAL_GPIO_WritePin(DEBUG_IO_1_GPIO_Port, DEBUG_IO_1_Pin, is_set ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    case 2:
        HAL_GPIO_WritePin(DEBUG_IO_2_GPIO_Port, DEBUG_IO_2_Pin, is_set ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    case 3:
        HAL_GPIO_WritePin(DEBUG_IO_3_GPIO_Port, DEBUG_IO_3_Pin, is_set ? GPIO_PIN_SET : GPIO_PIN_RESET);
        break;
    default:
        break;
    }
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);
    if (hadc == &hadc1)
    {
        motor_control.ia_adc = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        motor_control.ib_adc = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        motor_control.ic_adc = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

        easy_mc_isr_task_high_frequency();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    UNUSED(GPIO_Pin);

    if (M1_ENCODER_Z_Pin == GPIO_Pin)
    {
        // Handler motor zero interrupt
        easy_mc_isr_set_zero();
    }
}

uint16_t easy_mc_hw_encoder_get_count(void)
{
    return __HAL_TIM_GET_COUNTER(&TIM_ENCODER_COUNT);
}

void easy_mc_hw_encoder_set_zero(void)
{
    __HAL_TIM_SET_COUNTER(&TIM_ENCODER_COUNT, 0);
}

static uint8_t transmit_buffer[1024];
void easy_mc_hw_vofa_debug_out(uint8_t *data, uint16_t size)
{
    // HAL_UART_Transmit_DMA(&UART_VOFA_DEBUG, data, size);
    // HAL_UART_Transmit(&UART_VOFA_DEBUG, data, size, 1000);

    // Wait last DMA transmit complete
    int offset = 0;
    int len = size;
    while (len > 0)
    {
        uint16_t len_tx = 0;
        if (len > sizeof(transmit_buffer))
        {
            len_tx = sizeof(transmit_buffer);
        }
        else
        {
            len_tx = len;
        }

        while (HAL_UART_GetState(&UART_VOFA_DEBUG) != HAL_UART_STATE_READY)
        {
        }
        memcpy(transmit_buffer, data + offset, len_tx);
        if (HAL_UART_Transmit_DMA(&UART_VOFA_DEBUG, transmit_buffer + offset, len_tx) != HAL_OK)
        {
            break;
        }
        offset += len_tx;
        len -= len_tx;
    }
}

void easy_mc_hw_set_u_v_w_count(uint16_t u_count, uint16_t v_count, uint16_t w_count)
{
    TIM1->CCR1 = u_count;
    TIM1->CCR2 = v_count;
    TIM1->CCR3 = w_count;
}

void easy_mc_hw_start(void)
{
    if (is_hw_start)
    {
        return;
    }
    // EASY_MC_LOG_DBG("easy_mc_hw_start()\n");
    // UVW Start PWM output
    HAL_TIM_PWM_Start(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_3);

    is_hw_start = 1;
}

void easy_mc_hw_stop(void)
{
    if (!is_hw_start)
    {
        return;
    }
    // EASY_MC_LOG_DBG("easy_mc_hw_stop()\n");
    // UVW Stop PWM output
    HAL_TIM_PWM_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_3);

    is_hw_start = 0;
}

void easy_mc_hw_init(void)
{
    if (is_hw_init)
    {
        return;
    }
    // EASY_MC_LOG_DBG("easy_mc_hw_init()\n");
    // Calibrate ADC, also known as self-calibration, calibrate both ADC1 and ADC2 here
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    // Clear ADC flags
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOS);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_EOC);
    // Enable ADC injected conversion
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2); // ADC2 starts without interrupt, results are obtained directly in adc1's interrupt, will it get the previous result?

    // Initialize TIM1 and enable PWM output
    TIM1->ARR = EASY_MC_PWM_PERIOD_CYCLES;
    TIM1->CCR4 = EASY_MC_PWM_PERIOD_CYCLES - 1;
    HAL_TIM_Base_Start(&TIM_MOTOR_PWM_OUT);
    HAL_TIM_PWM_Start(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_4);

    // Initialize ABZ encoder
    HAL_TIM_Encoder_Start(&TIM_ENCODER_COUNT, TIM_CHANNEL_ALL);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DEBUG_IO_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DEBUG_IO_2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DEBUG_IO_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DEBUG_IO_3_GPIO_Port, &GPIO_InitStruct);

    // Test IO output
    easy_mc_hw_debug_io_control(0, 1);
    easy_mc_hw_debug_io_control(0, 0);
    easy_mc_hw_debug_io_control(1, 1);
    easy_mc_hw_debug_io_control(1, 0);
    easy_mc_hw_debug_io_control(2, 1);
    easy_mc_hw_debug_io_control(2, 0);
    easy_mc_hw_debug_io_control(3, 1);
    easy_mc_hw_debug_io_control(3, 0);

    is_hw_init = 1;
}

void easy_mc_hw_deinit(void)
{
    if (!is_hw_init)
    {
        return;
    }
    // EASY_MC_LOG_DBG("easy_mc_hw_deinit()\n");
    // Stop ADC injected conversion
    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);

    // Stop PWM output
    HAL_TIM_Base_Stop(&TIM_MOTOR_PWM_OUT);
    HAL_TIM_PWM_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_4);

    // Stop UVW PWM output
    HAL_TIM_PWM_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&TIM_MOTOR_PWM_OUT, TIM_CHANNEL_3);

    // Stop ABZ encoder
    HAL_TIM_Encoder_Stop(&TIM_ENCODER_COUNT, TIM_CHANNEL_ALL);

    is_hw_init = 0;
}
