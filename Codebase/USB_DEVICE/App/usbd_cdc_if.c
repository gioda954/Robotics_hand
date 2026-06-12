/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include <string.h>

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
#define CDC_COMMAND_BUFFER_SIZE  48U
#define CDC_TX_TIMEOUT_MS        100U
#define CDC_ROTATION_MICRO_PER_ROTATION 1000000UL
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
static char cdc_command_buffer[CDC_COMMAND_BUFFER_SIZE];
static uint32_t cdc_command_len = 0U;
static volatile uint8_t cdc_stop_command_pending = 0U;
static volatile uint8_t cdc_grab_command_pending = 0U;
static volatile uint8_t cdc_grabpid_command_pending = 0U;
static volatile uint8_t cdc_light_command_pending = 0U;
static volatile uint8_t cdc_hard_command_pending = 0U;
static volatile uint8_t cdc_uneven_command_pending = 0U;
static volatile uint8_t cdc_yeah_command_pending = 0U;
static volatile uint8_t cdc_release_command_pending = 0U;
static volatile uint8_t cdc_setup_command_pending = 0U;
static volatile uint8_t cdc_adjust_command_pending = 0U;
static volatile uint8_t cdc_adjust_motor_number = 0U;
static volatile int8_t cdc_adjust_direction_sign = 1;
static volatile uint32_t cdc_adjust_rotation_micro = 0U;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static void CDC_ParseReceivedByte(char c);
static uint8_t CDC_CommandEquals(const char *command, const char *expected);
static uint8_t CDC_TryParseAdjustCommand(const char *command);
static const char *CDC_SkipSpaces(const char *text);
static uint8_t CDC_ParseUnsigned(const char **text, uint32_t *value);
static uint8_t CDC_ParseRotationMicro(const char **text, uint32_t *rotation_micro);
static void CDC_ClearQueuedMotionCommands(void);

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  for (uint32_t i = 0U; i < *Len; i++)
  {
    CDC_ParseReceivedByte((char)Buf[i]);
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
static void CDC_ParseReceivedByte(char c)
{
  if ((c == '\r') || (c == '\n'))
  {
    cdc_command_buffer[cdc_command_len] = '\0';
    cdc_command_len = 0U;

    if (CDC_CommandEquals(cdc_command_buffer, "STOP") != 0U)
    {
      cdc_stop_command_pending = 1U;
    }
    else if (CDC_CommandEquals(cdc_command_buffer, "grab") != 0U)
    {
      cdc_grab_command_pending = 1U;
    }
    else if (CDC_CommandEquals(cdc_command_buffer, "grabpid") != 0U)
    {
      cdc_grabpid_command_pending = 1U;
    }
    else if (CDC_CommandEquals(cdc_command_buffer, "light") != 0U)
    {
      cdc_light_command_pending = 1U;
    }
    else if (CDC_CommandEquals(cdc_command_buffer, "hard") != 0U)
    {
      cdc_hard_command_pending = 1U;
    }
    else if (CDC_CommandEquals(cdc_command_buffer, "uneven") != 0U)
    {
      cdc_uneven_command_pending = 1U;
    }
    else if (CDC_CommandEquals(cdc_command_buffer, "yeah") != 0U)
    {
      cdc_yeah_command_pending = 1U;
    }
    else if ((CDC_CommandEquals(cdc_command_buffer, "release") != 0U) ||
             (CDC_CommandEquals(cdc_command_buffer, "relase") != 0U))
    {
      cdc_release_command_pending = 1U;
    }
    else if (CDC_CommandEquals(cdc_command_buffer, "setup") != 0U)
    {
      cdc_setup_command_pending = 1U;
    }
    else
    {
      (void)CDC_TryParseAdjustCommand(cdc_command_buffer);
    }
  }
  else if ((c == '\b') || (c == 0x7F))
  {
    if (cdc_command_len > 0U)
    {
      cdc_command_len--;
    }
  }
  else if (cdc_command_len < (CDC_COMMAND_BUFFER_SIZE - 1U))
  {
    cdc_command_buffer[cdc_command_len] = c;
    cdc_command_len++;
  }
  else
  {
    cdc_command_len = 0U;
  }
}

static uint8_t CDC_CommandEquals(const char *command, const char *expected)
{
  while ((*command != '\0') && (*expected != '\0'))
  {
    char a = *command;
    char b = *expected;

    if ((a >= 'A') && (a <= 'Z'))
    {
      a = (char)(a - 'A' + 'a');
    }

    if ((b >= 'A') && (b <= 'Z'))
    {
      b = (char)(b - 'A' + 'a');
    }

    if (a != b)
    {
      return 0U;
    }

    command++;
    expected++;
  }

  return ((*command == '\0') && (*expected == '\0')) ? 1U : 0U;
}

static uint8_t CDC_TryParseAdjustCommand(const char *command)
{
  const char *cursor = CDC_SkipSpaces(command);
  uint32_t motor_number;
  uint32_t rotation_micro;
  int8_t direction_sign;

  if (((cursor[0] != 'a') && (cursor[0] != 'A')) ||
      ((cursor[1] != 'd') && (cursor[1] != 'D')) ||
      ((cursor[2] != 'j') && (cursor[2] != 'J')) ||
      ((cursor[3] != ' ') && (cursor[3] != '\t')))
  {
    return 0U;
  }

  cursor = CDC_SkipSpaces(&cursor[3]);
  if (CDC_ParseUnsigned(&cursor, &motor_number) == 0U)
  {
    return 0U;
  }

  cursor = CDC_SkipSpaces(cursor);
  if (*cursor == '+')
  {
    direction_sign = 1;
  }
  else if (*cursor == '-')
  {
    direction_sign = -1;
  }
  else
  {
    return 0U;
  }
  cursor++;

  cursor = CDC_SkipSpaces(cursor);
  if (CDC_ParseRotationMicro(&cursor, &rotation_micro) == 0U)
  {
    return 0U;
  }

  cursor = CDC_SkipSpaces(cursor);
  if (*cursor != '\0')
  {
    return 0U;
  }

  if (motor_number > 255U)
  {
    return 0U;
  }

  cdc_adjust_motor_number = (uint8_t)motor_number;
  cdc_adjust_direction_sign = direction_sign;
  cdc_adjust_rotation_micro = rotation_micro;
  cdc_adjust_command_pending = 1U;

  return 1U;
}

static const char *CDC_SkipSpaces(const char *text)
{
  while ((*text == ' ') || (*text == '\t'))
  {
    text++;
  }

  return text;
}

static uint8_t CDC_ParseUnsigned(const char **text, uint32_t *value)
{
  const char *cursor = *text;
  uint32_t parsed = 0U;
  uint8_t found_digit = 0U;

  while ((*cursor >= '0') && (*cursor <= '9'))
  {
    parsed = (parsed * 10U) + (uint32_t)(*cursor - '0');
    cursor++;
    found_digit = 1U;
  }

  if (found_digit == 0U)
  {
    return 0U;
  }

  *text = cursor;
  *value = parsed;
  return 1U;
}

static uint8_t CDC_ParseRotationMicro(const char **text, uint32_t *rotation_micro)
{
  const char *cursor = *text;
  uint32_t whole = 0U;
  uint32_t fraction = 0U;
  uint32_t scale = CDC_ROTATION_MICRO_PER_ROTATION / 10U;
  uint8_t found_digit = 0U;

  while ((*cursor >= '0') && (*cursor <= '9'))
  {
    whole = (whole * 10U) + (uint32_t)(*cursor - '0');
    cursor++;
    found_digit = 1U;
  }

  if (*cursor == '.')
  {
    cursor++;
    while ((*cursor >= '0') && (*cursor <= '9'))
    {
      if (scale > 0U)
      {
        fraction += ((uint32_t)(*cursor - '0') * scale);
        scale /= 10U;
      }
      cursor++;
      found_digit = 1U;
    }
  }

  if (found_digit == 0U)
  {
    return 0U;
  }

  *text = cursor;
  *rotation_micro = (whole * CDC_ROTATION_MICRO_PER_ROTATION) + fraction;
  return 1U;
}

static void CDC_ClearQueuedMotionCommands(void)
{
  cdc_grab_command_pending = 0U;
  cdc_grabpid_command_pending = 0U;
  cdc_light_command_pending = 0U;
  cdc_hard_command_pending = 0U;
  cdc_uneven_command_pending = 0U;
  cdc_yeah_command_pending = 0U;
  cdc_setup_command_pending = 0U;
  cdc_adjust_command_pending = 0U;
}

uint8_t USB_CDC_PollGrabCommand(void)
{
  uint8_t pending;

  __disable_irq();
  pending = cdc_grab_command_pending;
  cdc_grab_command_pending = 0U;
  __enable_irq();

  return pending;
}

uint8_t USB_CDC_PollStopCommand(void)
{
  uint8_t pending;

  __disable_irq();
  pending = cdc_stop_command_pending;
  cdc_stop_command_pending = 0U;
  if (pending != 0U)
  {
    cdc_release_command_pending = 0U;
    CDC_ClearQueuedMotionCommands();
  }
  __enable_irq();

  return pending;
}

uint8_t USB_CDC_PollGrabPidCommand(void)
{
  uint8_t pending;

  __disable_irq();
  pending = cdc_grabpid_command_pending;
  cdc_grabpid_command_pending = 0U;
  __enable_irq();

  return pending;
}

uint8_t USB_CDC_PollLightCommand(void)
{
  uint8_t pending;

  __disable_irq();
  pending = cdc_light_command_pending;
  cdc_light_command_pending = 0U;
  __enable_irq();

  return pending;
}

uint8_t USB_CDC_PollHardCommand(void)
{
  uint8_t pending;

  __disable_irq();
  pending = cdc_hard_command_pending;
  cdc_hard_command_pending = 0U;
  __enable_irq();

  return pending;
}

uint8_t USB_CDC_PollUnevenCommand(void)
{
  uint8_t pending;

  __disable_irq();
  pending = cdc_uneven_command_pending;
  cdc_uneven_command_pending = 0U;
  __enable_irq();

  return pending;
}

uint8_t USB_CDC_PollYeahCommand(void)
{
  uint8_t pending;

  __disable_irq();
  pending = cdc_yeah_command_pending;
  cdc_yeah_command_pending = 0U;
  __enable_irq();

  return pending;
}

uint8_t USB_CDC_PollReleaseCommand(void)
{
  uint8_t pending;

  __disable_irq();
  pending = cdc_release_command_pending;
  cdc_release_command_pending = 0U;
  if (pending != 0U)
  {
    CDC_ClearQueuedMotionCommands();
  }
  __enable_irq();

  return pending;
}

uint8_t USB_CDC_PollSetupCommand(void)
{
  uint8_t pending;

  __disable_irq();
  pending = cdc_setup_command_pending;
  cdc_setup_command_pending = 0U;
  __enable_irq();

  return pending;
}

uint8_t USB_CDC_PollAdjustCommand(uint8_t *motor_number, int8_t *direction_sign, uint32_t *rotation_micro)
{
  uint8_t pending;

  __disable_irq();
  pending = cdc_adjust_command_pending;
  if (pending != 0U)
  {
    *motor_number = cdc_adjust_motor_number;
    *direction_sign = cdc_adjust_direction_sign;
    *rotation_micro = cdc_adjust_rotation_micro;
    cdc_adjust_command_pending = 0U;
  }
  __enable_irq();

  return pending;
}

void USB_CDC_SendString(const char *text)
{
  const uint8_t *src = (const uint8_t *)text;
  uint16_t len = (uint16_t)strlen(text);
  uint32_t start_tick;
  USBD_CDC_HandleTypeDef *hcdc;

  if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) || (hUsbDeviceFS.pClassData == NULL))
  {
    return;
  }

  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  while (len > 0U)
  {
    uint16_t chunk_len = len;

    if (chunk_len > APP_TX_DATA_SIZE)
    {
      chunk_len = APP_TX_DATA_SIZE;
    }

    start_tick = HAL_GetTick();
    while (hcdc->TxState != 0U)
    {
      if ((HAL_GetTick() - start_tick) > CDC_TX_TIMEOUT_MS)
      {
        return;
      }
    }

    memcpy(UserTxBufferFS, src, chunk_len);
    if (CDC_Transmit_FS(UserTxBufferFS, chunk_len) != USBD_OK)
    {
      return;
    }

    src += chunk_len;
    len -= chunk_len;
  }
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
