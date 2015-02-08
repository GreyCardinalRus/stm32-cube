/**
  ******************************************************************************
  * @file    USB_Device/HID_Standalone/Src/main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-March-2014
  * @brief   USB device HID demo main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"  

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;
extern PCD_HandleTypeDef hpcd;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F2xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
 
  /* Configure the system clock to 120 MHz */
  SystemClock_Config();
  
  /* Change Systick priority: should be higher than Key button interrupt */
  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
  
  /* Configure LED1, LED2, LED3 and LED4 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  
  /* Initialize Joystick */
  BSP_JOY_Init(JOY_MODE_GPIO);  
  
  /* Configure Key button for remote wakeup */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  
  /* Init Device Library */
  USBD_Init(&USBD_Device, &HID_Desc, 0);
  
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, &USBD_HID);
  
  /* Start Device Process */
  USBD_Start(&USBD_Device);

  while (1);

}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 120000000
  *            HCLK(Hz)                       = 120000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 240
  *            PLL_P                          = 2
  *            PLL_Q                          = 5
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

/**
  * @brief  Toggle LEDs to shows user input state.   
  * @param  None
  * @retval None
  */
void Toggle_Leds(void)
{
  static uint32_t ticks;
  
  if(ticks++ == 100)
  {
    BSP_LED_Toggle(LED1);
    BSP_LED_Toggle(LED2);
    BSP_LED_Toggle(LED3);
    BSP_LED_Toggle(LED4);
    ticks = 0;
  }  
}

/**
  * @brief  GPIO EXTI Callback function
  *         Handle remote-wakeup through key button
  * @param  GPIO_Pin
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    if ((USBD_Device.dev_remote_wakeup == 1)&&(USBD_Device.dev_state == USBD_STATE_SUSPENDED))
    {
      if ((&hpcd)->Init.low_power_enable)
      {
        /* Reset SLEEPDEEP bit of Cortex System Control Register */
        SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));  
    
        /* Configures system clock after wake-up from STOP: enable HSE, PLL and select 
        PLL as system clock source (HSE and PLL are disabled in STOP mode) */
    
        __HAL_RCC_HSE_CONFIG(RCC_HSE_ON);
    
        /* Wait till HSE is ready */  
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
        {}
    
        /* Enable the main PLL. */
        __HAL_RCC_PLL_ENABLE();
    
        /* Wait till PLL is ready */  
        while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
        {}
    
        /* Select PLL as SYSCLK */
        MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYSCLKSOURCE_PLLCLK);
    
        /* Wait till system clock switch to PLL */
        while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL)
        {}
      }
      
      /* ungate PHY clock */
       __HAL_PCD_UNGATE_PHYCLOCK((&hpcd));
     
      /* Activate Remote wakeup */
      HAL_PCD_ActiveRemoteWakeup((&hpcd));
      /* remote wakeup delay */
      HAL_Delay(10);
      /* Desactivate Remote wakeup */
      HAL_PCD_DeActiveRemoteWakeup((&hpcd));
      
      /* change state to configured */
      USBD_Device.dev_state = USBD_STATE_CONFIGURED;
      
      /* change remote_wakeup feature to 0*/
      USBD_Device.dev_remote_wakeup=0;
    }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
