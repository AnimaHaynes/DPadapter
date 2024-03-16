#include "pwm_driver.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include <string.h> // memcpy
#include <stdlib.h> // malloc

TIM_HandleTypeDef pwm_htim;
int duty_scalar;

int pwm_init()
{
    duty_scalar =  SystemCoreClock / PWM_SWITCHING_FREQUENCY / 4;
    // Mostly copied from HAL
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    pwm_htim.Instance = TIM1;
    pwm_htim.Init.Prescaler = 0;
    pwm_htim.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    pwm_htim.Init.Period = duty_scalar * 2;
    pwm_htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pwm_htim.Init.RepetitionCounter = 0;
    pwm_htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&pwm_htim) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&pwm_htim, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&pwm_htim) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&pwm_htim, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&pwm_htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&pwm_htim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&pwm_htim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 8;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&pwm_htim, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
      
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE8     ------> TIM1_CH1N
    PE9     ------> TIM1_CH1
    PE10     ------> TIM1_CH2N
    PE11     ------> TIM1_CH2
    PE12     ------> TIM1_CH3N
    PE13     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                        |GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    return 0;
}

int pwm_set_frequency(int freq)
{
    return 0;
}

int pwm_set_duty(float duty1, float duty2, float duty3)
{
    // pwm_htim.Instance->CCR1 = sine_ref[(counter) % 100];
    // pwm_htim.Instance->CCR2 = sine_ref[(counter+33) % 100];
    // pwm_htim.Instance->CCR3 = sine_ref[(counter+67) % 100];
    pwm_htim.Instance->CCR1 = (int)(duty1 * duty_scalar);
    pwm_htim.Instance->CCR2 = (int)(duty2 * duty_scalar);
    pwm_htim.Instance->CCR3 = (int)(duty3 * duty_scalar);
    return 0;
}

int pwm_start()
{
    int ret = 0;
    ret |= HAL_TIM_PWM_Start(&pwm_htim, TIM_CHANNEL_1);
    ret |= HAL_TIMEx_PWMN_Start(&pwm_htim, TIM_CHANNEL_1);
    ret |= HAL_TIM_PWM_Start(&pwm_htim, TIM_CHANNEL_2);
    ret |= HAL_TIMEx_PWMN_Start(&pwm_htim, TIM_CHANNEL_2);
    ret |= HAL_TIM_PWM_Start(&pwm_htim, TIM_CHANNEL_3);
    ret |= HAL_TIMEx_PWMN_Start(&pwm_htim, TIM_CHANNEL_3);
    return ret;
}

int pwm_stop()
{
    int ret = 0;
    ret |= HAL_TIM_PWM_Stop(&pwm_htim, TIM_CHANNEL_1);
    ret |= HAL_TIMEx_PWMN_Stop(&pwm_htim, TIM_CHANNEL_1);
    ret |= HAL_TIM_PWM_Stop(&pwm_htim, TIM_CHANNEL_2);
    ret |= HAL_TIMEx_PWMN_Stop(&pwm_htim, TIM_CHANNEL_2);
    ret |= HAL_TIM_PWM_Stop(&pwm_htim, TIM_CHANNEL_3);
    ret |= HAL_TIMEx_PWMN_Stop(&pwm_htim, TIM_CHANNEL_3);
    return ret;
}
