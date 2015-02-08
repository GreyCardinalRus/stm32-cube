/**
  ******************************************************************************
  * @file    images_broswer_app.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    09-Sept-2014   
  * @brief   thermometer system information.
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
#define __IMAGEBROWSER_APP_C

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stdio.h"
#include "k_config.h"
#include "k_module.h"
#include "k_menu.h"
#include "k_storage.h"

/** @addtogroup 8UART_MODULE
  * @{
  */

/** @defgroup SYSTEM_INFO
  * @brief system info routines 
  * @{
  */

/* Private typedef ----------------------------------------------------------*/    
/* Private constants ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
KMODULE_RETURN _ImageBrowserDemoExec(void);

void ImageBrowserDemo(void);
void ImageBrowserMenuUserAction(uint8_t sel);
void ImageBrowserUserInformation(int16_t tempvalue);
void ImageBrowserUserHeader(void);

/* Private Variable ----------------------------------------------------------*/
const tMenuItem ImageBrowserMenuMenuItems[] =
{
    {NULL, 14, 30, TYPE_EXEC, MODULE_NONE, ImageBrowserDemo, ImageBrowserMenuUserAction, NULL, NULL },
};

const tMenu ImageBrowserMenu = {
  NULL, ImageBrowserMenuMenuItems, countof(ImageBrowserMenuMenuItems), TYPE_EXEC, 1, 1 };

/* used to exit application */
static __IO uint8_t user_event=0;
static __IO uint8_t user_action=0;

/* Private typedef -----------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
const K_ModuleItem_Typedef ModuleImageBrowser =
{
  MODULE_TSENSOR,
  NULL,
  _ImageBrowserDemoExec,
  NULL,
  NULL
};

/**
  * @brief  Run the 8 uart application 
  * @param  None.
  * @note   run and display information about the uart transaction.  
  * @retval None.
  */
KMODULE_RETURN _ImageBrowserDemoExec(void)
{
  /* Prepare the main MMI */
  kMenu_Execute(ImageBrowserMenu);
  
  /* Execute the app 8uart */
  /* App initialization    */
  return KMODULE_OK;
}


/**
  * @brief  Run the image browser 
  * @param  None.
  * @note   run and display image accordng the user action.  
  * @retval None.
  */
void ImageBrowserDemo(void)
{
  uint8_t  filename[15];
  uint8_t  PathImage[50];
  
  /* Lecture du folder source */
  kStorage_GetDirectoryFiles((uint8_t *)"USER", KSTORAGE_FINDFIRST, filename, (uint8_t *)"BMP");
  
  /* Affichage de la premiere image */
  sprintf((char*)PathImage,"USER/%s",filename);
  kStorage_OpenFileDrawPixel(0,0,PathImage);
  
  /* reset the user action event handler */
  user_action = 0;
  
  /* Wait User event                    */
  /*       JOY_LEFT  : next picture     */
  /*       JOY_RIGHT : next picture     */
  /*       JOY_SEL   : exit application */
  do
  {
    while(user_action == 0);
    
    if((user_event == JOY_RIGHT) || (user_event == JOY_LEFT))
    {
     
      if(kStorage_GetDirectoryFiles((uint8_t *)"USER", ((user_event == JOY_LEFT) ? KSTORAGE_FINDPREV : KSTORAGE_FINDNEXT) , filename, (uint8_t *)"BMP") == KSTORAGE_NOERROR)
      {
        /* display the picture */
        sprintf((char *)PathImage,"USER/%s",filename);
        kStorage_OpenFileDrawPixel(0,0,PathImage);
      }
    }
    
    /* exit on UP or Down */
    if((user_event == JOY_UP) || (user_event == JOY_DOWN))
    {
      user_event = JOY_SEL;
    }

    /* reset the user action event handler */
    user_action = 0;
  } 
  while(user_event != JOY_SEL);
  
  /* Close the find */
  kStorage_GetDirectoryFiles((uint8_t *)"USER", KSTORAGE_FINDCLOSE, filename, (uint8_t *)"BMP");
}

/**
  * @brief  Get User action 
  * @param  sel : User selection (JOY_SEL,...)
  * @note   This example is the only way to get user information.  
  * @retval None
  */
void ImageBrowserMenuUserAction(uint8_t sel)
{
  if (user_action == 0 )
  {
    user_action = 1;
    user_event = sel;
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
