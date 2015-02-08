/**
  ******************************************************************************
  * @file    k_conifg.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    09-Sept-2014
  * @brief   Header for k_config.c file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __K_CONFIG_H
#define __K_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
#define EVAL_BOARD   "STM32091C-EVAL"
#define DEMO_VERSION "Version 1.1.0"
#define DEMO_DATE    "03 - October - 2014"   

#define DEMO_INFO1   "MCD Application Team"
#define DEMO_INFO2   "COPYRIGHT 2014"   
#define DEMO_INFO3   "STMicroelectronics"
   
#define CHOOSEFILE_MAXLEN   255
   
typedef enum{
 MODULE_MAIN_APP,
 MODULE_LOWPOWER,
 MODULE_8UART,
 MODULE_TSENSOR,
 MODULE_LOG_DEMO,
 MODULE_VIEWBMP,
 MODULE_FILESBRO,
 MODULE_SYSTEM_INFO,
 MODULE_MAX,
 MODULE_NONE = 0xFF
} MODULES_INFO;

#define countof(a) (sizeof(a) / sizeof(*(a)))
/* Exported functions ------------------------------------------------------- */ 

#ifdef __cplusplus
}
#endif

#endif /*__K_LOG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
