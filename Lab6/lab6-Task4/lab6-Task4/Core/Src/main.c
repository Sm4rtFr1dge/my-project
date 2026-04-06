/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Standalone Task 4 RPM Code (No CubeMX Dependencies)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <stdint.h>

/* ========================================================================= */
/* VARIABLES & DEFINES                                                       */
/* ========================================================================= */
#define PPR 330.0f

UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

volatile uint32_t ic_val1 = 0;
volatile uint32_t ic_val2 = 0;
volatile uint32_t diff = 0;
volatile uint8_t is_first = 0;
volatile float frequency = 0;
volatile float motor_rpm = 0.0;

/* ========================================================================= */
/* PRINTF REDIRECT TO UART                                                   */
/* ========================================================================= */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ========================================================================= */
/* HARDWARE INTERRUPT (TIM2 CHANNEL 2 on PA1)                                */
/* ========================================================================= */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        if (is_first == 0)
        {
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); 
            is_first = 1;
        }
        else
        {
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

            if (ic_val2 > ic_val1)
                diff = ic_val2 - ic_val1;
            else
                diff = (0xFFFFFFFF - ic_val1) + ic_val2;

            if (diff != 0) {
                // 1 MHz Clock (PSC=47 on 48MHz SysTick)
                frequency = 1000000.0f / (float)diff; 
                motor_rpm = (60.0f * frequency) / PPR; 
            }

            // Keep the chain going for smooth readings!
            ic_val1 = ic_val2; 
        }
    }
}

// Emergency Interrupt Catcher

/* ========================================================================= */
/* INITIALIZATION FUNCTIONS (Manually Forced Hardware Setup)                   */
/* ========================================================================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Force 48MHz System Clock
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}

void Standalone_Hardware_Init(void)
{
    // 1. Enable Clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 2. Setup Motor Direction Pins (PE8, PE12)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8 | GPIO_PIN_12, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // 3. Setup Encoder Pin (PA1 -> TIM2_CH2)
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 4. Setup PWM Pin (PA6 -> TIM3_CH1)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 5. Setup UART Pins (PA9 TX, PA10 RX)
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 6. Initialize UART 115200
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    // 7. Initialize TIM2 (Input Capture)
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 47; // 48MHz / 48 = 1MHz Clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    HAL_TIM_Base_Init(&htim2);

    TIM_IC_InitTypeDef sConfigIC = {0};
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);

    // Enable TIM2 Hardware Interrupt
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    // 8. Initialize TIM3 (PWM Motor Speed)
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 47;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
}

/* ========================================================================= */
/* MAIN LOOP                                                                 */
/* ========================================================================= */
int main(void)
{
    // Boot up the microcontroller
    HAL_Init();
    SystemClock_Config();
    
    // Force our custom hardware initialization
    Standalone_Hardware_Init();

    // 1. Start the Input Capture on PA1
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

    // 2. Start the Motor PWM on PA6
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    
    // 3. Set Speed to 50% (500 / 999)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);  
    
    // 4. Move Right Motor Forward (PE8 High, PE12 Low)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);

    // Say Hello!
    printf("\r\n--- SYSTEM BOOT: MOTOR RUNNING ---\r\n");

    while (1)
    {
        // Print the real-time RPM every 0.5 seconds
        printf("Frequency: %d Hz | RPM: %d\r\n", (int)frequency, (int)motor_rpm);
        HAL_Delay(500);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}