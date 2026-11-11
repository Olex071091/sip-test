/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx_ll_tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern volatile uint32_t msTicks;
uint16_t treshold = 1687;
uint16_t ADC_value = 0;
uint8_t state = '0';
#define HYSTERESIS 2      // запас в отсчётах АЦП (примерно 1,5 °C)

// ============================================================
// ТАБЛИЦА СЕГМЕНТОВ ДЛЯ ИНДИКАТОРА С ОБЩИМ АНОДОМ
// 0 = сегмент ГОРИТ, 1 = сегмент НЕ ГОРИТ
// Порядок битов: (dp) g f e d c b a
// ============================================================
const uint8_t segmentCodes[10] = {
    0b11000000, // 0
    0b11111001, // 1
    0b10100100, // 2
    0b10110000, // 3
    0b10011001, // 4
    0b10010010, // 5
    0b10000010, // 6
    0b11111000, // 7
    0b10000000, // 8
    0b10010000  // 9
};

// ============================================================
// ПИНЫ ДЛЯ РЕГИСТРА СДВИГА 74HC595
// ============================================================
#define SHIFT_PORT      GPIOA
#define SER_PIN         LL_GPIO_PIN_0   // Данные (SER / DS) - пин 14
#define SRCLK_PIN       LL_GPIO_PIN_1   // Такт сдвига (SRCLK / SHCP) - пин 11
#define RCLK_PIN        LL_GPIO_PIN_4   // Защелка (RCLK / STCP) - пин 12

// ============================================================
// ПИНЫ ДЛЯ УПРАВЛЕНИЯ ОБЩИМИ АНОДАМИ (ВЫБОР РАЗРЯДА)
// ============================================================
#define MUX_PORT        GPIOB
#define DIGIT_1_PIN     LL_GPIO_PIN_8   // Сотни
#define DIGIT_2_PIN     LL_GPIO_PIN_9   // Десятки
#define DIGIT_3_PIN     LL_GPIO_PIN_10  // Единицы

// Массив пинов для удобного перебора
const uint16_t digitPins[3] = {DIGIT_1_PIN, DIGIT_2_PIN, DIGIT_3_PIN};

// ============================================================
// ПЕРЕМЕННЫЕ ДЛЯ ДИНАМИЧЕСКОЙ ИНДИКАЦИИ
// ============================================================
volatile uint8_t displayDigits[3] = {0, 0, 0}; // [0] - сотни, [1] - десятки, [2] - единицы
volatile uint8_t currentDigit = 0;              // Какой разряд сейчас активен (0, 1 или 2)

// Флаг для синхронизации (опционально)
volatile uint8_t displayUpdated = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Error_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>

// ============================================================
// ОТПРАВКА ОДНОГО БАЙТА В РЕГИСТР СДВИГА 74HC595
// ============================================================
void sendByteToShiftRegister(uint8_t data)
{
    // Отправляем биты, начиная со старшего (бит 7)
    for (int i = 7; i >= 0; i--)
    {
        // 1. Устанавливаем бит данных
        if (data & (1 << i))
        {
            LL_GPIO_SetOutputPin(SHIFT_PORT, SER_PIN);   // HIGH
        }
        else
        {
            LL_GPIO_ResetOutputPin(SHIFT_PORT, SER_PIN); // LOW
        }

        // 2. Подаем тактовый импульс (сдвигаем бит)
        LL_GPIO_ResetOutputPin(SHIFT_PORT, SRCLK_PIN);
        // Небольшая задержка для стабильности (опционально)
        for (volatile int d = 0; d < 5; d++);
        LL_GPIO_SetOutputPin(SHIFT_PORT, SRCLK_PIN);
    }

    // 3. Защелкиваем данные (копируем на выходы)
    LL_GPIO_ResetOutputPin(SHIFT_PORT, RCLK_PIN);
    for (volatile int d = 0; d < 5; d++);
    LL_GPIO_SetOutputPin(SHIFT_PORT, RCLK_PIN);
}

// ============================================================
// УСТАНОВКА ЧИСЛА ДЛЯ ОТОБРАЖЕНИЯ НА ИНДИКАТОРЕ
// ============================================================
void setDisplayNumber(uint16_t number)
{
    if (number > 999) number = 999;

    displayDigits[0] = number / 100;          // Сотни
    displayDigits[1] = (number % 100) / 10;   // Десятки
    displayDigits[2] = number % 10;           // Единицы
}

// ============================================================
// ПЕРЕКЛЮЧЕНИЕ РАЗРЯДОВ (ВЫЗЫВАЕТСЯ ИЗ ПРЕРЫВАНИЯ)
// ============================================================
void switchDigit(void)
{
    // 1. Гасим все разряды (выключаем общие аноды) для PNP: HIGH = транзистор закрыт
    LL_GPIO_SetOutputPin(MUX_PORT, DIGIT_1_PIN | DIGIT_2_PIN | DIGIT_3_PIN);

    // 2. Отправляем код текущей цифры в регистр сдвига
    uint8_t code = segmentCodes[displayDigits[currentDigit]];
    // Если текущий разряд — десятки (индекс 1), включаем десятичную точку
        if (currentDigit == 1)
        {
            code &= ~0x80;  // Сбрасываем бит 7 (включаем точку, т.к. 0 = горит)
        }

    sendByteToShiftRegister(code);

    // 3. Включаем текущий разряд
    LL_GPIO_ResetOutputPin(MUX_PORT, digitPins[currentDigit]);

    // 4. Переходим к следующему разряду
    currentDigit++;
    if (currentDigit >= 3) currentDigit = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint16_t TS_CAL1 = *((uint16_t*)0x1FFFF7B8);
  uint16_t TS_CAL2 = *((uint16_t*)0x1FFFF7C2);
  float temperature_celsius;
  (void)temperature_celsius;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  uint32_t adcSum = 0, adcCnt = 0, lastUpdate = 0;
  LL_SYSTICK_EnableIT();      // разрешаем прерывание SysTick (msTicks++)
  LL_ADC_Enable(ADC1);                           //TODO: enable ADC
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0) {} //wait for ADC "ready" flag

  // Запускаем таймер TIM2 в режиме прерываний с использованием LL
  LL_TIM_EnableIT_UPDATE(TIM2);   // Разрешаем прерывание по событию обновления
  LL_TIM_EnableCounter(TIM2);     // Запускаем счетчик таймера

  // Устанавливаем HIGH на пинах разрядов, все индикаторы погашены
  LL_GPIO_SetOutputPin(MUX_PORT, DIGIT_1_PIN | DIGIT_2_PIN | DIGIT_3_PIN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    /* USER CODE BEGIN 3 */
	    LL_ADC_REG_StartConversion(ADC1);
	    while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0) {}
	    ADC_value = LL_ADC_REG_ReadConversionData12(ADC1);
	    adcSum += ADC_value;
	    adcCnt++;

	    if (msTicks - lastUpdate >= 500)       // раз в 500 мс
	    {
	      lastUpdate = msTicks;
	      float adcAvg = (float)adcSum / (float)adcCnt;
	      adcSum = 0; adcCnt = 0;

	      temperature_celsius = ((110.0f - 30.0f) * (adcAvg - TS_CAL1) / (TS_CAL2 - TS_CAL1)) + 30.0f;
	      if (temperature_celsius < 0.0f) temperature_celsius = 0.0f;
	      setDisplayNumber((uint16_t)(temperature_celsius * 10.0f));

	      // Сравнение с порогом с гистерезисом
	      if (adcAvg > (float)(treshold + HYSTERESIS))
	      {
	        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
	        state = '1';
	      }
	      else if (adcAvg < (float)(treshold - HYSTERESIS))
	      {
	        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
	        state = '0';
	      }

	      // Отправка по кнопке (теперь тоже раз в 500 мс)
	      if (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13) == 0)
	      {
	        while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
	        LL_USART_TransmitData8(USART2, state);
	        while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
	        LL_USART_TransmitData8(USART2, '\r');
	        while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
	        LL_USART_TransmitData8(USART2, '\n');
	      }
	    }
	    /* USER CODE END 3 */



  /* USER CODE END 3 */
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}
