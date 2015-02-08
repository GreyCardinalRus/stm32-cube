/**
  ******************************************************************************
  * @file    ADC/ADC_LowPower/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   This example describes how to use Timer to convert continuously data.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup ADC_LowPower
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MESSAGE1   "STM32F0xx CortexM0" 
#define MESSAGE2   "STM32072B-EVAL" 
#define MESSAGE3   "Turn RV3(PC.00)"

/* Uncomment the line below if you will use the ADC Low Power features */ 
#define ADC_LOWPOWER

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef        sConfig;


/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle;
 
__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedVoltage = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void Display_Init(void);
static void Display(void);
static void ADC_Config(void);
static void TIM_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
 /* This sample code shows how to convert an analog input and read the converted
    data using DMA transfer.
    To proceed, 4 steps are required: */
   
  /* STM32F0xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Initialize LED1 & LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);

  /* Configure the system clock to 48 MHz */
  SystemClock_Config();

  /* LCD Display init  */
  Display_Init();
  
  /* Tamper push-button configuration */
  BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_GPIO);

  /* Configure ADC1  */
  ADC_Config();

  /* Configure TIM3  */
  TIM_Config();  

  /* Infinite loop */
  while (1)
  {
    /* Press Tamper push-button for STM32072B-EVAL  to get the converted data */
    while(BSP_PB_GetState(BUTTON_TAMPER) != RESET);
    
    /* Get ADC1 converted data */
    ADC1ConvertedValue = HAL_ADC_GetValue(&AdcHandle);
    
    /* Compute the voltage */
    ADC1ConvertedVoltage = (ADC1ConvertedValue *3300)/0xFFF;
    
    /* Display converted data on the LCD */
    Display();
  }
}

/**
  * @brief  ADC1 configuration
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  /*##-1- Configure the ADC peripheral #######################################*/
  AdcHandle.Instance                   = ADCx;

  /* Configure the ADCx in continous mode withe a resolution equal to 12 bits*/
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_BACKWARD;
  AdcHandle.Init.ContinuousConvMode    = ENABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.EOCSelection          = EOC_SEQ_CONV;
  AdcHandle.Init.Overrun               = OVR_DATA_PRESERVED;
#if defined(ADC_LOWPOWER)
  /* Enable the wait conversion mode */    
  AdcHandle.Init.LowPowerAutoWait      = ENABLE;
  /* Enable the Auto power off mode */
  AdcHandle.Init.LowPowerAutoPowerOff  = ENABLE;
#else
  /* Disable the wait conversion mode */    
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;
  /* Disable the Auto power off mode */
  AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
#endif

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* Initiliazation Error */
    Error_Handler();
  }

  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
  /*##-3- Configure ADC regular channel ######################################*/
  /* Convert the ADCx_CHANNEL with 28.5 Cycles as sampling time */
  sConfig.Channel      = ADCx_CHANNEL;
  sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
  
  /* Enable only OVR Interrupt */
  __HAL_ADC_ENABLE_IT(&AdcHandle, (ADC_IT_OVR));

  /* ADC1 regular Software Start Conv */ 
  HAL_ADC_Start(&AdcHandle);
}

/**
  * @brief  TIM3 configuration
  * @param  None
  * @retval None
  */
static void TIM_Config(void)
{
  TIM_MasterConfigTypeDef  sMasterConfig =  {0};
  
  TimHandle.Instance               = TIMx;
  TimHandle.Init.Period            = 0xFF;
  TimHandle.Init.Prescaler         = 0;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);

  /*##-2- tarts the TIM Output Compare signal generation. ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}
/**
  * @brief  Display ADC converted value on LCD
  * @param  None
  * @retval None
  */
static void Display(void)
{
  uint32_t v=0,mv=0;
  uint8_t text[50];

  v=(ADC1ConvertedVoltage)/1000;
  mv = (ADC1ConvertedVoltage%1000)/100;
  sprintf((char*)text,"V(Pot) = %d,%d V",v,mv);
  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_DisplayStringAt(0, 145, (uint8_t*)text, CENTER_MODE); 
}

/**
  * @brief  Display Init (LCD)
  * @param  None
  * @retval None
  */
static void Display_Init(void)
{
  /* Initialize the LCD */
  BSP_LCD_Init();

  /* Clear the LCD */ 
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  
  /* Set LCD Demo description */
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), 80);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE); 
  BSP_LCD_SetFont(&Font24);
  BSP_LCD_DisplayStringAt(0, 0, MESSAGE1, CENTER_MODE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, 30, (uint8_t*)"ADC Low Power Mode", CENTER_MODE); 
  BSP_LCD_DisplayStringAt(0, 45, (uint8_t*)"Press Tamper push-button", CENTER_MODE); 
  BSP_LCD_DisplayStringAt(0, 60, MESSAGE3, CENTER_MODE); 

   /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);  
  BSP_LCD_DrawRect(10, 90, BSP_LCD_GetXSize() - 20, BSP_LCD_GetYSize()- 100);
  BSP_LCD_DrawRect(11, 91, BSP_LCD_GetXSize() - 22, BSP_LCD_GetYSize()- 102);

  BSP_LCD_SetFont(&Font24);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE); 
}

/**
  * @brief  Error ADC callback.
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  BSP_LED_On(LED1);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler(); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    Error_Handler(); 
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
