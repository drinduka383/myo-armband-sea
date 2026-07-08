/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F446RE ESCON serial-to-DAC test
  ******************************************************************************
  *
  * Command protocol on USART2 / ST-LINK VCP, 115200 8N1:
  *   STOP or 0     -> PA4 = 0 V,  PC0 = LOW, LD2 = OFF
  *   RUN  or 1     -> PA4 = about 1.0 V, PC0 = HIGH, LD2 = ON
  *   P 0..100      -> scale RUN level
  *   STATUS or S   -> report DAC / enable state
  *
  * Commands are newline-terminated. Boot, parse errors, UART errors, and stale
  * commands all force STOP.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);

/* Private defines -----------------------------------------------------------*/
#define DAC_ZERO_VALUE        0U
#define DAC_RUN_VALUE         1241U
#define COMMAND_TIMEOUT_MS    3000U
#define UART_BAUDRATE         115200U
#define LINE_BUFFER_LEN       32U

/* Private variables ---------------------------------------------------------*/
static uint16_t g_dac_code = 0U;
static uint32_t g_last_command_ms = 0U;

/* Private function prototypes -----------------------------------------------*/
static void MX_USART2_Minimal_Init(void);
static void UART2_WriteText(const char *text);
static uint8_t UART2_TryReadByte(uint8_t *byte);
static uint8_t UART2_HadError(void);
static void ApplyOutput(uint16_t dac_code);
static void StopOutput(void);
static void SetPercent(unsigned percent);
static void ProcessCommand(char *line);
static void SafeStopOutputsRegisters(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint8_t byte = 0U;
  char line[LINE_BUFFER_LEN];
  size_t used = 0U;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DAC_Init();
  BSP_LED_Init(LED2);
  MX_USART2_Minimal_Init();

  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  StopOutput();
  g_last_command_ms = HAL_GetTick();
  UART2_WriteText("READY STOP\r\n");

  while (1)
  {
    while (UART2_TryReadByte(&byte) != 0U)
    {
      if ((byte == '\r') || (byte == '\n'))
      {
        if (used != 0U)
        {
          line[used] = '\0';
          ProcessCommand(line);
          used = 0U;
          g_last_command_ms = HAL_GetTick();
        }
      }
      else if (used + 1U < sizeof(line))
      {
        line[used++] = (char)byte;
      }
      else
      {
        used = 0U;
        StopOutput();
        UART2_WriteText("ERR command too long; STOP\r\n");
        g_last_command_ms = HAL_GetTick();
      }
    }

    if (UART2_HadError() != 0U)
    {
      used = 0U;
      StopOutput();
      UART2_WriteText("ERR UART; STOP\r\n");
      g_last_command_ms = HAL_GetTick();
    }

    if ((g_dac_code != 0U) && ((HAL_GetTick() - g_last_command_ms) > COMMAND_TIMEOUT_MS))
    {
      StopOutput();
      UART2_WriteText("TIMEOUT STOP\r\n");
      g_last_command_ms = HAL_GetTick();
    }
  }
}

static void MX_USART2_Minimal_Init(void)
{
  uint32_t pclk1_hz;

  __HAL_RCC_USART2_CLK_ENABLE();

  USART2->CR1 = 0U;
  USART2->CR2 = 0U;
  USART2->CR3 = 0U;

  pclk1_hz = HAL_RCC_GetPCLK1Freq();
  USART2->BRR = (pclk1_hz + (UART_BAUDRATE / 2U)) / UART_BAUDRATE;
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void UART2_WriteText(const char *text)
{
  while (*text != '\0')
  {
    while ((USART2->SR & USART_SR_TXE) == 0U)
    {
    }

    USART2->DR = (uint8_t)(*text);
    text++;
  }

  while ((USART2->SR & USART_SR_TC) == 0U)
  {
  }
}

static uint8_t UART2_TryReadByte(uint8_t *byte)
{
  if ((USART2->SR & USART_SR_RXNE) == 0U)
  {
    return 0U;
  }

  *byte = (uint8_t)USART2->DR;
  return 1U;
}

static uint8_t UART2_HadError(void)
{
  if ((USART2->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) == 0U)
  {
    return 0U;
  }

  (void)USART2->SR;
  (void)USART2->DR;
  return 1U;
}

static void ApplyOutput(uint16_t dac_code)
{
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_code);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (dac_code == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  if (dac_code == 0U)
  {
    BSP_LED_Off(LED2);
  }
  else
  {
    BSP_LED_On(LED2);
  }

  g_dac_code = dac_code;
}

static void StopOutput(void)
{
  ApplyOutput(DAC_ZERO_VALUE);
}

static void SetPercent(unsigned percent)
{
  uint16_t scaled;

  scaled = (uint16_t)((DAC_RUN_VALUE * percent + 50U) / 100U);
  ApplyOutput(scaled);
}

static void ProcessCommand(char *line)
{
  char *end;
  char *number;
  unsigned long percent;
  char ack[48];
  char *cursor = line;

  while (isspace((unsigned char)*cursor) != 0)
  {
    cursor++;
  }

  for (char *p = cursor; *p != '\0'; ++p)
  {
    *p = (char)toupper((unsigned char)*p);
  }

  end = cursor + strlen(cursor);
  while ((end > cursor) && (isspace((unsigned char)end[-1]) != 0))
  {
    *--end = '\0';
  }

  if ((strcmp(cursor, "0") == 0) || (strcmp(cursor, "STOP") == 0))
  {
    StopOutput();
    UART2_WriteText("ACK STOP\r\n");
    return;
  }

  if ((strcmp(cursor, "1") == 0) || (strcmp(cursor, "RUN") == 0))
  {
    ApplyOutput(DAC_RUN_VALUE);
    UART2_WriteText("ACK RUN DAC=1241 ENABLE=1\r\n");
    return;
  }

  if ((strcmp(cursor, "S") == 0) || (strcmp(cursor, "STATUS") == 0))
  {
    snprintf(ack, sizeof(ack), "ACK STATUS DAC=%u ENABLE=%u\r\n", g_dac_code, (g_dac_code != 0U) ? 1U : 0U);
    UART2_WriteText(ack);
    return;
  }

  if (cursor[0] == 'P')
  {
    number = cursor + 1;
    while (isspace((unsigned char)*number) != 0)
    {
      number++;
    }

    percent = strtoul(number, &end, 10);
    while (isspace((unsigned char)*end) != 0)
    {
      end++;
    }

    if ((end == number) || (*end != '\0') || (percent > 100UL))
    {
      StopOutput();
      UART2_WriteText("ERR invalid percentage; STOP\r\n");
      return;
    }

    SetPercent((unsigned)percent);
    snprintf(ack, sizeof(ack), "ACK P=%lu DAC=%u ENABLE=%u\r\n", percent, g_dac_code, (g_dac_code != 0U) ? 1U : 0U);
    UART2_WriteText(ack);
    return;
  }

  StopOutput();
  UART2_WriteText("ERR unknown command; STOP\r\n");
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};

  hdac.Instance = DAC;

  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void SafeStopOutputsRegisters(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;

  GPIOC->MODER = (GPIOC->MODER & ~(3UL << (0U * 2U))) | (1UL << (0U * 2U));
  GPIOA->MODER = (GPIOA->MODER & ~(3UL << (5U * 2U))) | (1UL << (5U * 2U));

  GPIOC->BSRR = ((uint32_t)GPIO_PIN_0 << 16U);
  GPIOA->BSRR = ((uint32_t)GPIO_PIN_5 << 16U);
  DAC->DHR12R1 = 0U;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  SafeStopOutputsRegisters();
  __disable_irq();

  while (1)
  {
    GPIOA->BSRR = GPIO_PIN_5;
    for (volatile uint32_t delay = 0U; delay < 200000U; ++delay)
    {
    }

    GPIOA->BSRR = ((uint32_t)GPIO_PIN_5 << 16U);
    for (volatile uint32_t delay = 0U; delay < 200000U; ++delay)
    {
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
