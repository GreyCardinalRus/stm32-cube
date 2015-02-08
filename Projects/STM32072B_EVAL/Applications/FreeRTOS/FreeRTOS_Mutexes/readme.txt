/**
  @page FreeRTOS mutexes application
 
  @verbatim
  ******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
  * @file    FreeRTOS/FreeRTOS_Mutexes/readme.txt
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   Description of the STM32F0xx CMSIS RTOS mutexes example.
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

This directory contains a set of sources files that implement an example
that uses mutexes with CMSIS RTOS API. 

This example creates three threads with different priorities 
and accessing the same mutex.

MutexHighPriorityThread() has the highest priority so executes 
first to access he mutex and sleeps for a short period to let the lower 
priority threads execute.  When it has completed its demo functionality,
it gives the mutex back before suspending itself. 
At that point, LED1 toggles.
 
MutexMeduimPriorityThread() attempts to access the mutex by performing
a blocking 'wait'.  This thread blocks when the mutex is already taken 
by the high priority thread. It does not unblock until the highest 
priority thread  has released the mutex, and it does not actually run until 
the highest priority thread has suspended itself.
When it eventually obtains the mutex, it only gives the mutex back
prior to suspending itself as well.  At this point both the high and medium
priority threads are suspended and LED2 toggles.

MutexLowPriorityThread() runs at the idle priority.  It spins round
a tight loop attempting to obtain the mutex with a non-blocking call.  As
the lowest priority thread, it doesn't successfully obtain the mutex until
both high and medium priority threads are suspended.  Once it eventually 
obtains the mutex, it first resumes both suspended threads (and LED4 toggles
at that time) prior to giving the mutex back - resulting in the low priority 
thread temporarily inheriting the highest thread priority.       


@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents

    - FreeRTOS/FreeRTOS_Mutexes/Inc/main.h                main config file
    - FreeRTOS/FreeRTOS_Mutexes/Inc/stm32f0xx_hal_conf.h  HAL Configuration file
    - FreeRTOS/FreeRTOS_Mutexes/Inc/stm32f0xx_it.h        Header for stm32f0xx_it.c
    - FreeRTOS/FreeRTOS_Mutexes/Inc/FreeRTOSConfig.h      FreeRTOS Configuration file
    - FreeRTOS/FreeRTOS_Mutexes/Src/main.c                main program file
    - FreeRTOS/FreeRTOS_Mutexes/Src/stm32f0xx_it.c        STM32 Interrupt handlers

@par Hardware and Software environment

  - This example runs on STM32F072VB devices.
    
  - This example has been tested with a STM32072B-EVAL board embedding
    a STM32072B device and can be easily tailored to any other supported device 
    and development board.

@par How to use it ?

In order to make the program work, you must do the following:
  1. Load the demonstration code in the Flash memory (see below)
  2. Add the following variables to LiveWatch (HighPriorityThreadCycles, 
     MediumPriorityThreadCycles, LowPriorityThreadCycles) 
     All three variables must remain equals as long as
     LED1, LED2 and LED4 should toggle;
     LED3 is turned on in case of error
     
  In order to load the demonstration code, you have do the following:
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
