/**
  ******************************************************************************
  * @file    RCC/RCC_CRS_Synchonization_IT/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   This example describes how to use the RCC HAL API to configure the
  *          system clock (SYSCLK) and modify the clock settings on run time.
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

/** @addtogroup RCC_CRS_Synchonization_IT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RCC_CRSInitTypeDef CRSInitStruct = {0};

/* Variable to save SYNC status*/
__IO uint32_t CRSSyncStatus = 0;

/* Private function prototypes -----------------------------------------------*/
static void CRS_Init(void);
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F0xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Low Level Initialization
     */

  RCC_CRSSynchroInfoTypeDef syncinfo = {0};
  
  HAL_Init();

  /* Configure LED1, LED2, LED4 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);  
  BSP_LED_Init(LED4);  
  BSP_LED_Init(LED3);

  /* Enable HSE oscillator and configure the PLL to reach the max system frequency (48 MHz)
     when using HSE oscillator as PLL clock source. */
  SystemClock_Config();

  /* CRS initialization (enable HSI48 and LSE oscillators and then enable CRS clock */
  /* Enable RCC_CRS interuption*/
  CRS_Init();
  
  /* HSI48 Synchronization without synchronization frequency (source set to GPIO */
  /* which is not configured)*/
  CRSInitStruct.Prescaler             = RCC_CRS_SYNC_DIV1;
  CRSInitStruct.Source                = RCC_CRS_SYNC_SOURCE_GPIO;
  CRSInitStruct.Polarity              = RCC_CRS_SYNC_POLARITY_FALLING;
  /* F(LSE)=32.768kHz then Reload=(f(Target)/f(LSE))-1= 0x5B7*/
  CRSInitStruct.ReloadValue           = __HAL_RCC_CRS_CALCULATE_RELOADVALUE(48000000,32768);
  CRSInitStruct.ErrorLimitValue       = RCC_CRS_ERRORLIMIT_DEFAULT;
  /* Change the HSI trimming value to see the automatic calibration performed by CRS */
  CRSInitStruct.HSI48CalibrationValue = 0x00;

  /* Start automatic synchronization*/
  HAL_RCCEx_CRSConfig(&CRSInitStruct);

  /* Enable RCC_CRS Interuptions */
   __HAL_RCC_CRS_ENABLE_IT(RCC_CRS_IT_SYNCOK | RCC_CRS_IT_SYNCWARN | RCC_CRS_IT_ERR | RCC_CRS_IT_ESYNC);

  /* Simulate a SYNC EVENT which will raise a SYNC_MISS EVENT */ 
  HAL_RCCEx_CRSSoftwareSynchronizationGenerate();

  /* Infinite loop */
  while (1)
  {
    /* Perform a new configuration to LSE when SYNC MISS is detected. */
    if (CRSSyncStatus == 2)
    {
      /* Delay to see the LED power on switch from LED4 to LED1 */
      HAL_Delay(1000);
      
      /* Retrieve synchronization information if needed */
      HAL_RCCEx_CRSGetSynchronizationInfo(&syncinfo);
    
      /* Change source to LSE and restart synchronization */
      CRSInitStruct.Source                = RCC_CRS_SYNC_SOURCE_LSE;
      HAL_RCCEx_CRSConfig(&CRSInitStruct);

      /* Enable RCC_CRS Interuptions */
      __HAL_RCC_CRS_ENABLE_IT(RCC_CRS_IT_SYNCOK | RCC_CRS_IT_SYNCWARN | RCC_CRS_IT_ERR | RCC_CRS_IT_ESYNC);
   }
  }
}

/**
  * @brief  Handle CRS interuption
  * @param  none 
  * @retval none
  */
void CRS_HandlerIT(void)
{
  /* Check CRS SYNCOK flag  */
  if ((__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCOK)) && (__HAL_RCC_CRS_GET_IT_SOURCE(RCC_CRS_IT_SYNCOK)))
  {
    /* Power on LED1 */
    BSP_LED_On(LED1);
    /* Power off LED2*/
    BSP_LED_Off(LED2);
      /* Power off LED4*/
    BSP_LED_Off(LED4);

    CRSSyncStatus = 0;
    /* Clear CRS SYNC event OK bit */
    __HAL_RCC_CRS_CLEAR_IT(RCC_CRS_FLAG_SYNCOK);
  }

  /* Check CRS SYNCWARN flag  */
  if ((__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCWARN)) && (__HAL_RCC_CRS_GET_IT_SOURCE(RCC_CRS_IT_SYNCWARN)))
  {
    /* Power off LED1 */
    BSP_LED_Off(LED1);
    /* Power on LED2*/
    BSP_LED_On(LED2);
     /* Power off LED4*/
    BSP_LED_Off(LED4);

    CRSSyncStatus = 1;

    /* Clear CRS SYNCWARN bit */
    __HAL_RCC_CRS_CLEAR_IT(RCC_CRS_FLAG_SYNCWARN);
  }

  /* Check CRS Error flag  */
  if ((__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCERR)) && (__HAL_RCC_CRS_GET_IT_SOURCE(RCC_CRS_IT_SYNCERR)))
  {
    /* Power off LED1 */
    BSP_LED_Off(LED1);
    /* Power off LED2*/
    BSP_LED_Off(LED2);
    /* Power on LED4*/
    BSP_LED_On(LED4);

    CRSSyncStatus = 2;

    /* Clear CRS Error bit */
    __HAL_RCC_CRS_CLEAR_IT(RCC_CRS_FLAG_SYNCERR);
  }

  /* Check CRS SYNC Missed flag  */
  if ((__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_SYNCMISS)) && (__HAL_RCC_CRS_GET_IT_SOURCE(RCC_CRS_IT_ERR)))
  {
    /* Power off LED1 */
    BSP_LED_Off(LED1);
    /* Power off LED2*/
    BSP_LED_Off(LED2);
    /* Power on LED4*/
    BSP_LED_On(LED4);

    CRSSyncStatus = 2;
    
    /* Clear CRS SYNC Missed bit */
    __HAL_RCC_CRS_CLEAR_IT(RCC_CRS_FLAG_SYNCMISS);
  }
  
  /* Check CRS Expected SYNC flag  */
  if ((__HAL_RCC_CRS_GET_FLAG(RCC_CRS_FLAG_ESYNC)) && (__HAL_RCC_CRS_GET_IT_SOURCE(RCC_CRS_IT_ESYNC)))
  {
    /* frequency error counter reached a zero value */
    __HAL_RCC_CRS_CLEAR_IT(RCC_CRS_FLAG_ESYNC);
  }
}


static void CRS_Init(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct =  {0};
  
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  if (RCC_OscInitStruct.HSI48State != RCC_HSI48_ON)
  {
    /* Enable HSI48 and LSE Oscillator*/
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }
  
  /* Enable CRS clock*/
  __CRS_CLK_ENABLE();

  /* Enable and set RCC/CRS Interrupt*/
  HAL_NVIC_SetPriority(RCC_CRS_IRQn, 0x03, 0x00);
  HAL_NVIC_EnableIRQ(RCC_CRS_IRQn);

  /* Output HSI48 MCO pin(PA8) */
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCOSOURCE_HSI48, RCC_MCO_DIV1);
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
