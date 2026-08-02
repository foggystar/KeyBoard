/**
  ******************************************************************************
  * @file    usbd_keyboard.h
  * @author  MCD Application Team
  * @brief   Header file for the usbd_keyboard_core.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_KEYBOARD_H
#define __USB_KEYBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_KEYBOARD
  * @brief This file is the Header file for usbd_keyboard.c
  * @{
  */


/** @defgroup USBD_KEYBOARD_Exported_Defines
  * @{
  */
#define KEYBOARD_EPIN_ADDR                 0x81U
#define KEYBOARD_EPIN_SIZE                 0x08U

#define USB_KEYBOARD_CONFIG_DESC_SIZ       34U
#define USB_KEYBOARD_DESC_SIZ              9U
#define HID_KEYBOARD_REPORT_DESC_SIZE    63U

#define KEYBOARD_DESCRIPTOR_TYPE           0x21U
#define KEYBOARD_REPORT_DESC               0x22U

#ifndef KEYBOARD_HS_BINTERVAL
#define KEYBOARD_HS_BINTERVAL            0x07U
#endif /* HID_HS_BINTERVAL */

#ifndef KEYBOARD_FS_BINTERVAL
#define KEYBOARD_FS_BINTERVAL            0x0AU
#endif /* HID_FS_BINTERVAL */

#define KEYBOARD_REQ_SET_PROTOCOL          0x0BU
#define KEYBOARD_REQ_GET_PROTOCOL          0x03U

#define KEYBOARD_REQ_SET_IDLE              0x0AU
#define KEYBOARD_REQ_GET_IDLE              0x02U

#define KEYBOARD_REQ_SET_REPORT            0x09U
#define KEYBOARD_REQ_GET_REPORT            0x01U
/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef USBD_KEYBOARD;
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t USBD_KEYBOARD_SendReport(USBD_HandleTypeDef *pdev,
                            uint8_t *report,
                            uint16_t len);

uint32_t USBD_KEYBOARD_GetPollingInterval(USBD_HandleTypeDef *pdev);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_KEYBOARD_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
