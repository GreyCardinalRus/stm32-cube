/**
  @page BSP  Example on how to use the BSP drivers
  
  @verbatim
  ******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
  * @file    BSP/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   Description of the BSP example.
  ******************************************************************************
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
  @endverbatim

@par Example Description 

This example provides a description of how to use the different BSP drivers. 

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.
Then the SystemClock_Config() function is used to configure the system
clock (SYSCLK) to run at 48 MHz.

This example shows how to use the different functionalities of LCD, SD card, 
eeprom, temperatue sensor and joystick by switching 
between all tests using key button. 

 ** Push the User button to start first Test.  
Blue Led (LD4) will blink between each test.Press user key to start another test:

 ** JOSTICK **
Use the joystick button to move a pointer inside a rectangle 
(up/down/right/left) and change the pointer color(select).

 ** LCD **
This example shows how to use the different LCD features to display string
with different fonts, to display different shapes and to draw a bitmap.

 ** SD **
This example shows how to erase, write and read the SD card and also 
how to detect the presence of the card.

 ** EEPROM **
This example show how to read and write data in EEPROM I2C M24LR64 connected on STM32072B-EVAL
   * The I2C RF EEPROM memory (M24LR64) is available thru the connector CN2.

 ** TSENSOR **
This example show how to read a Temperature using the temperature sensor.


 ** CEC **
This example shows how to initialize and detect the devices connected on the HDMI-CEC 
according the device type choosen. It displays the received messages. The board can be configured as 
ROOT or not and with the use of HDMI-DDC or not.

@note Connect HDMI cable to HDMI_SOURCE and with another device which support HDMI connection.

</!\ Be careful : HDMI-CEC has same I2C address than RF EEPROM ANT7-M24LR-A /!\>

 ** LCD LOG **
This example show how to use the LCD log features. 

@par Directory contents 

  - BSP/Src/main.c                 Main program
  - BSP/Src/system_stm32f0xx.c     STM32F0xx system clock configuration file
  - BSP/Src/stm32f0xx_it.c         Interrupt handlers 
  - BSP/Src/joystick.c             joystick feature
  - BSP/Src/lcd.c                  LCD drawing features
  - BSP/Src/log.c                  LCD Log firmware functions
  - BSP/Src/sd.c                   SD features
  - BSP/Src/eeprom.c               EEPROM Read/Write features
  - BSP/Src/tsensor.c              Temperature Sensor features
  - BSP/Src/tsensor.c              Temperature Sensor features
  - BSP/Src/audio_play.c	   Audio Playback features
  - BSP/AudioFile/audio.bin        Audio wave extract.
  - BSP/Inc/main.h                 Main program header file  
  - BSP/Inc/stm32f0xx_conf.h       HAL Configuration file
  - BSP/Inc/stm32f0xx_it.h         Interrupt handlers header file
  - BSP/Inc/lcd_log_conf.h         lcd_log configuration template file
  - BSP/Inc/stlogo.h               Image used for BSP example
        
@par Hardware and Software environment  

  - This example runs on STM32F072VB devices.
    
  - This example has been tested with STMicroelectronics STM32072B-EVAL
    evaluation boards and can be easily tailored to any other supported device 
    and development board.
  
@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
