/**
  @page semaphore demonstration Readme file
 
  @verbatim
  ******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
  * @file   FreeRTOS/FreeRTOS_Semaphore/readme.txt
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014 
  * @brief   Description of the STM32F0xx CMSIS RTOS semaphore example.
  ******************************************************************************
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
  @endverbatim

@par Description

This directory contains a set of sources files that implement an example that
uses semaphores with CMSIS RTOS API. 

This example creates two threads that toggle LEDs through a shared semaphore.

The first thread which has the highest priority obtains the semaphore and 
toggles the LED1 every 200 ms. After 5 seconds it releases the semaphore and 
suspends itself. The low priority thread can execute now, obtains the 
semaphore and resumes execution of the first thread; 
as it has the highest priority, the first thread tries to obtain the semaphore 
but fails because the semaphore is already taken by the low priority thread, 
which toggles the LED2 for 5 seconds before releasing the semaphore and 
beginning a new cycle.

In case of error, LED1 is turned on and remains on.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents

    - FreeRTOS/FreeRTOS_Semaphore/Inc/main.h                main config file
    - FreeRTOS/FreeRTOS_Semaphore/Inc/stm32f0xx_hal_conf.h  HAL Configuration file
    - FreeRTOS/FreeRTOS_Semaphore/Inc/stm32f0xx_it.h        Header for stm32f0xx_it.c
    - FreeRTOS/FreeRTOS_Semaphore/Inc/FreeRTOSConfig.h      FreeRTOS Configuration file
    - FreeRTOS/FreeRTOS_Semaphore/Src/main.c                main program file
    - FreeRTOS/FreeRTOS_Semaphore/Src/stm32f0xx_it.c        STM32 Interrupt handlers

@par Hardware and Software environment

  - This example runs on STM32F072VB devices.
    
  - This example has been tested with a STM32072B-EVAL board embedding
    a STM32072B device and can be easily tailored to any other supported device 
    and development board.
    
@par How to use it ?

In order to make the program work, you must do the following:
  1. Load the demonstration code in the Flash memory (see below)
  2. please Ensure that LEDs toggle like described above

  In order to load the demonstration code, you have do the following:
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
