/**
  @page LwIP TCP Echo Server Application
  
  @verbatim
  ******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
  * @file    LwIP/LwIP_TCP_Echo_Server/readme.txt 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-March-2014
  * @brief   Description of the LwIP TCP Echo Server Application.
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

@par Example Description 

This example guides STM32Cube HAL API users to run TCP Echo Server application 
based on Raw API of LwIP TCP/IP stack

To run this application, On the remote PC, open a command prompt window.
(In Windows, select Start > All Programs > Accessories > Command Prompt.)
At the command prompt, enter:
  C:\>echotool IP_address /p tcp /r 7 /n 15 /t 2 /d Testing LwIP TCP echo server

where:
    – IP_address is the actual board’s IP address. By default, the following 
    static IP address is used: 192.168.0.10
    – /p transport layer protocol used for communication (TCP)
    – /r is the actual remote port on the echo server (echo port)
    – /n is the number of echo requests (for example, 15)
    – /t is the connection timeout in seconds (for example, 2)
    – /d is the message to be sent for echo 

STM32 Eval board LEDs are used for the following purpose:
  + LED1: ethernet cable is connected.
  + LED2: ethernet cable is not connected.

Note: In this application the Ethernet Link ISR need the System tick interrupt 
to configure the Ethernet MAC, so the Ethernet Link interrupt priority must be 
set lower (numerically greater) than the Systick interrupt priority to ensure 
that the System tick increments while executing the Ethernet Link ISR.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 

  - LwIP/LwIP_TCP_Echo_Server/Inc/app_ethernet.h          header of app_ethernet.c file
  - LwIP/LwIP_TCP_Echo_Server/Inc/ethernetif.h            header for ethernetif.c file
  - LwIP/LwIP_TCP_Echo_Server/Inc/stm32f2xx_hal_conf.h    HAL configuration file
  - LwIP/LwIP_TCP_Echo_Server/Inc/stm32f2xx_it.h          STM32 interrupt handlers header file
  - LwIP/LwIP_TCP_Echo_Server/Inc/main.h                  Header for main.c module
  - LwIP/LwIP_TCP_Echo_Server/Inc/lwipopts.h              LwIP stack configuration options
  - LwIP/LwIP_TCP_Echo_Server/Inc/tcp_echoserver.h        Header for tcp echoserver application
  - LwIP/LwIP_TCP_Echo_Server/Src/app_ethernet.c          Ethernet specefic module
  - LwIP/LwIP_TCP_Echo_Server/Src/stm32f2xx_it.c          STM32 interrupt handlers
  - LwIP/LwIP_TCP_Echo_Server/Src/main.c                  Main program
  - LwIP/LwIP_TCP_Echo_Server/Src/system_stm32f2xx.c      STM32F2xx system source file
  - LwIP/LwIP_TCP_Echo_Server/Src/ethernetif.c            Interfacing LwIP to ETH driver
  - LwIP/LwIP_TCP_Echo_Server/Src/tcp_echoserver.c        tcp echoserver application


@par Hardware and Software environment

  - This example runs on STM32F207xx/STM32F217xx Devices.
    
  - This example has been tested with the following environments:
     - STM322xG-EVAL RevC board   
     - echotool: (http://bansky.net/echotool/) is used as echo client that sends
       data to the server and checking whether they came back      
      
  - STM322xG-EVAL Set-up
    - Connect the eval board to remote PC (through a crossover ethernet cable)
      or to your local network (through a straight ethernet cable)
    - STM322xG-EVAL jumpers setting
        +==========================================================================================+
        +  Jumper |       MII mode configuration            |     RMII mode configuration(*)       +
        +==========================================================================================+
        +  JP5    | 2-3 provide 25MHz clock by MCO(PA8)     |  Not fitted                          +
        +         | 1-2 provide 25MHz clock by ext. Crystal |                                      +
        + -----------------------------------------------------------------------------------------+
        +  JP6    |          2-3                            |  1-2                                 +
        + -----------------------------------------------------------------------------------------+
        +  JP8    |          Open                           |  Close                               +
        + -----------------------------------------------------------------------------------------+
        +  JP22   | 1-2: RS232 is enabled                                                          +
        +==========================================================================================+
    (*) User have to provide the 50 MHz clock by soldering a 50 MHz oscillator (ref SM7745HEV-50.0M or
        equivalent) on the U3 footprint located under CN3 and also removing jumper on JP5. This oscillator
        is not provided with the board. 
        For more details, please refer to STM3220G-EVAL evaluation board User manual (UM1057).
    @Note: the default setting is MII mode, to change it to RMII mode, assign the "ETH_MEDIA_INTERFACE_RMII" 
    value to the "EthHandle.Init.MediaInterface" field in ethernetif.c file. 
    
  - Remote PC Set-up
    - Configure a static IP address for your remote PC 
      for example 192.168.0.11 

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
