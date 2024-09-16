/*******************************************************************************
  * @file    stm32wlxx_hal_subghz.c
  * @author  MCD Application Team
  * @brief   SUBGHZ HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the SUBGHZ peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and Errors functions
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
 @verbatim
 ==============================================================================
                       ##### How to use this driver #####
 ==============================================================================
 [..]
    The SUBGHZ HAL driver can be used as follows:

    (#) Declare a SUBGHZ_HandleTypeDef handle structure, for example:
        SUBGHZ_HandleTypeDef hUserSubghz;

    (#) Initialize the SUBGHZ low level resources by implementing the @ref HAL_SUBGHZ_MspInit() API:
        (##) PWR configuration
            (+++) Enable the SUBGHZSPI interface clock
            (+++) Enable wakeup signal of the Radio peripheral
        (##) NVIC configuration:
            (+++) Enable the NVIC Radio IRQ ITs for CPU1 (EXTI 44)
            (+++) Configure the Radio interrupt priority

    (#) Initialize the SUBGHZ handle and SUBGHZSPI SPI registers by calling the @ref HAL_SUBGHZ_Init(&hUserSubghz),
        configures also the low level Hardware (GPIO, CLOCK, NVIC...etc) by calling
        the customized @ref HAL_SUBGHZ_MspInit() API.

    (#) For SUBGHZ IO operations, polling operation modes is available within this driver :

    *** Polling mode IO operation      ***
    =====================================
    [..]
      (+) Set and execute a command in blocking mode using @ref HAL_SUBGHZ_ExecSetCmd()
      (+) Get a status blocking mode using @ref HAL_SUBGHZ_ExecGetCmd()
      (+) Write a Data Buffer in blocking mode using @ref HAL_SUBGHZ_WriteBuffer()
      (+) Read a Data Buffer  in blocking mode using @ref HAL_SUBGHZ_ReadBuffer()
      (+) Write Registers (more than 1 byte) in blocking mode using @ref HAL_SUBGHZ_WriteRegisters()
      (+) Read Registers (more than 1 byte) in blocking mode using @ref HAL_SUBGHZ_ReadRegisters()
      (+) Write Register (1 byte) in blocking mode using @ref HAL_SUBGHZ_WriteRegister()
      (+) Read Register (1 byte) in blocking mode using @ref HAL_SUBGHZ_ReadRegister()

    *** SUBGHZ HAL driver macros list ***
    =====================================
    [..]
      (+) @ref __HAL_SUBGHZ_RESET_HANDLE_STATE: Reset the SUBGHZ handle state
  */

/* Includes ------------------------------------------------------------------*/
#include "subghz.h"
#include "subghz_support.h"
#include "mprintf.h"
#include "pin_defs.h"
#include "stm32wlxx_ll_gpio.h"

#include "stm32wlxx_hal_subghz.h"
#include "stm32wlxx_ll_exti.h"
#include "stm32wlxx_ll_pwr.h"
#include "stm32wlxx_ll_rcc.h"
#include "stm32_assert.h"
#include <stddef.h>

/** @addtogroup STM32WLxx_HAL_Driver
  * @{
  */

/** @defgroup SUBGHZ SUBGHZ
  * @brief SUBGHZ HAL module driver
  * @{
  */
#define HAL_SUBGHZ_MODULE_ENABLED
#ifdef HAL_SUBGHZ_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup SUBGHZ_Private_Constants SUBGHZ Private Constants
  * @{
  */
#define SUBGHZ_DEFAULT_TIMEOUT     100U    /* HAL Timeout in ms               */
#define SUBGHZ_DUMMY_DATA          0xFFU   /* SUBGHZSPI Dummy Data use for Tx */
#define SUBGHZ_DEEP_SLEEP_ENABLE   1U      /* SUBGHZ Radio in Deep Sleep      */
#define SUBGHZ_DEEP_SLEEP_DISABLE  0U      /* SUBGHZ Radio not in Deep Sleep  */

/* SystemCoreClock dividers. Corresponding to time execution of while loop.   */
#define SUBGHZ_DEFAULT_LOOP_TIME   ((SystemCoreClock*28U)>>19U)
#define SUBGHZ_RFBUSY_LOOP_TIME    ((SystemCoreClock*24U)>>20U)
#define SUBGHZ_NSS_LOOP_TIME       ((SystemCoreClock*24U)>>16U)
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup SUBGHZ_Private_Functions SUBGHZ Private Functions
  * @{
  */
void              SUBGHZSPI_Init(uint32_t BaudratePrescaler);
void              SUBGHZSPI_DeInit(void);
HAL_StatusTypeDef SUBGHZSPI_Transmit(SUBGHZ_HandleTypeDef *hsubghz, uint8_t Data);
HAL_StatusTypeDef SUBGHZSPI_Receive(SUBGHZ_HandleTypeDef *hsubghz, uint8_t *pData);
HAL_StatusTypeDef SUBGHZ_WaitOnBusy(SUBGHZ_HandleTypeDef *hsubghz);
HAL_StatusTypeDef SUBGHZ_CheckDeviceReady(SUBGHZ_HandleTypeDef *hsubghz);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SUBGHZ_Exported_Functions SUBGHZ Exported Functions
  * @{
  */

/** @defgroup SUBGHZ_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the SUBGHZ peripheral:

      (+) User must implement HAL_SUBGHZ_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, IT and NVIC ).

      (+) Call the function HAL_SUBGHZ_Init() to configure SUBGHZSPI peripheral
          and initialize SUBGHZ Handle.

      (+) Call the function HAL_SUBGHZ_DeInit() to restore the default configuration
          of the SUBGHZ peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the SUBGHZ according to the specified parameters
  *         in the SUBGHZ_HandleTypeDef and initialize the associated handle.
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the handle information for SUBGHZ module.
  * @note   In case of exiting from Standby mode, before calling this function,
  *         set the state to HAL_SUBGHZ_STATE_RESET_RF_READY with __HAL_SUBGHZ_RESET_HANDLE_STATE_RF_READY
  *         to avoid the reset of Radio peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SUBGHZ_Init(SUBGHZ_HandleTypeDef *hsubghz)
{
  HAL_StatusTypeDef status;
  __IO uint32_t count;
  HAL_SUBGHZ_StateTypeDef subghz_state;

  /* Check the hsubghz handle allocation */
  if (hsubghz == NULL)
  {
    status = HAL_ERROR;
    return status;
  }
  else
  {
    status = HAL_OK;
  }

  assert_param(IS_SUBGHZSPI_BAUDRATE_PRESCALER(hsubghz->Init.BaudratePrescaler));

  subghz_state = hsubghz->State;
  if ((subghz_state == HAL_SUBGHZ_STATE_RESET) ||
      (subghz_state == HAL_SUBGHZ_STATE_RESET_RF_READY))
  {
    /* Allocate lock resource and initialize it */
    hsubghz->Lock = HAL_UNLOCKED;

    // Init the low level hardware : GPIO, CLOCK, NVIC...

#if defined(CM0PLUS)
    /* Enable EXTI 44 : Radio IRQ ITs for CPU2 */
    LL_C2_EXTI_EnableIT_32_63(LL_EXTI_LINE_44);
#else
    /* Enable EXTI 44 : Radio IRQ ITs for CPU1 */
    LL_EXTI_EnableIT_32_63(LL_EXTI_LINE_44);
#endif /* CM0PLUS */
  }

  if (subghz_state == HAL_SUBGHZ_STATE_RESET)
  {
    /* Reinitialize Radio peripheral only if SUBGHZ is in full RESET state */
    hsubghz->State = HAL_SUBGHZ_STATE_BUSY;

    /* De-asserts the reset signal of the Radio peripheral */
    LL_RCC_RF_DisableReset();

    /* Verify that Radio in reset status flag is set */
    count  = SUBGHZ_DEFAULT_TIMEOUT * SUBGHZ_DEFAULT_LOOP_TIME;

    do
    {
      if (count == 0U)
      {
        status  = HAL_ERROR;
        hsubghz->ErrorCode = HAL_SUBGHZ_ERROR_TIMEOUT;
        break;
      }
      count--;
    } while (LL_RCC_IsRFUnderReset() != 0UL);

    /* Asserts the reset signal of the Radio peripheral */
    LL_PWR_UnselectSUBGHZSPI_NSS();

#if defined(CM0PLUS)
    /* Enable wakeup signal of the Radio peripheral */
    LL_C2_PWR_SetRadioBusyTrigger(LL_PWR_RADIO_BUSY_TRIGGER_WU_IT);
#else
    /* Enable wakeup signal of the Radio peripheral */
    LL_PWR_SetRadioBusyTrigger(LL_PWR_RADIO_BUSY_TRIGGER_WU_IT);
#endif /* CM0PLUS */
  }

  /* Clear Pending Flag */
  LL_PWR_ClearFlag_RFBUSY();

  if (status == HAL_OK)
  {
    /* Initialize SUBGHZSPI Peripheral */
    SUBGHZSPI_Init(hsubghz->Init.BaudratePrescaler);

    hsubghz->DeepSleep = SUBGHZ_DEEP_SLEEP_ENABLE;
    hsubghz->ErrorCode = HAL_SUBGHZ_ERROR_NONE;
  }

  hsubghz->State = HAL_SUBGHZ_STATE_READY;

  return status;
}

/**
  * @brief  De-Initialize the SUBGHZ peripheral.
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the handle information for SUBGHZ module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SUBGHZ_DeInit(SUBGHZ_HandleTypeDef *hsubghz)
{
  HAL_StatusTypeDef status;
  __IO uint32_t count;

  /* Check the SUBGHZ handle allocation */
  if (hsubghz == NULL)
  {
    status = HAL_ERROR;
    return status;
  }
  else
  {
    status = HAL_OK;
  }

  hsubghz->State = HAL_SUBGHZ_STATE_BUSY;

  /* DeInitialize SUBGHZSPI Peripheral */
  SUBGHZSPI_DeInit();

  // DeInit the low level hardware: GPIO, CLOCK, NVIC...

#if defined(CM0PLUS)
  /* Disable EXTI 44 : Radio IRQ ITs for CPU2 */
  LL_C2_EXTI_DisableIT_32_63(LL_EXTI_LINE_44);

  /* Disable wakeup signal of the Radio peripheral */
  LL_C2_PWR_SetRadioBusyTrigger(LL_PWR_RADIO_BUSY_TRIGGER_NONE);
#else
  /* Disable EXTI 44 : Radio IRQ ITs for CPU1 */
  LL_EXTI_DisableIT_32_63(LL_EXTI_LINE_44);

  /* Disable wakeup signal of the Radio peripheral */
  LL_PWR_SetRadioBusyTrigger(LL_PWR_RADIO_BUSY_TRIGGER_NONE);
#endif /* CM0PLUS */

  /* Clear Pending Flag */
  LL_PWR_ClearFlag_RFBUSY();

  /* Re-asserts the reset signal of the Radio peripheral */
  LL_RCC_RF_EnableReset();

  /* Verify that Radio in reset status flag is set */
  count  = SUBGHZ_DEFAULT_TIMEOUT * SUBGHZ_DEFAULT_LOOP_TIME;

  do
  {
    if (count == 0U)
    {
      status  = HAL_ERROR;
      hsubghz->ErrorCode = HAL_SUBGHZ_ERROR_TIMEOUT;
      break;
    }
    count--;
  } while (LL_RCC_IsRFUnderReset() != 1UL);

  hsubghz->ErrorCode = HAL_SUBGHZ_ERROR_NONE;
  hsubghz->State     = HAL_SUBGHZ_STATE_RESET;

  /* Release Lock */
  __HAL_UNLOCK(hsubghz);

  return status;
}

/**
  * @}
  */

/** @defgroup SUBGHZ_Exported_Functions_Group2 IO operation functions
  *  @brief   Data transfers functions
  *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the SUBGHZ
    data transfers.

    [..] The SUBGHZ supports Read and Write operation:

    (#) There are four modes of transfer:
       (++) Set operation: The Set Command operation is performed in polling mode.
            The HAL status of command processing is returned by the same function
            after finishing transfer.
       (++) Get operation: The Get Status operation is performed using polling mode
            These API update buffer in parameter to retrieve status of command.
            These API return the HAL status
       (++) Write operation: The write operation is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) Read operation: The read operation is performed using polling mode
            These APIs return the HAL status.

    (#) Blocking mode functions are :
        (++) HAL_SUBGHZ_ExecSetCmd(
        (++) HAL_SUBGHZ_ExecGetCmd()
        (++) HAL_SUBGHZ_WriteBuffer()
        (++) HAL_SUBGHZ_ReadBuffer()
        (++) HAL_SUBGHZ_WriteRegisters()
        (++) HAL_SUBGHZ_ReadRegisters()
        (++) HAL_SUBGHZ_WriteRegister()
        (++) HAL_SUBGHZ_ReadRegister()

@endverbatim
  * @{
  */

/**
  * @brief  Write data buffer at an Address to configurate the peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the configuration information for the specified SUBGHZ.
  * @param  Address register to configurate
  * @param  pBuffer pointer to a data buffer
  * @param  Size    amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SUBGHZ_WriteRegisters(SUBGHZ_HandleTypeDef *hsubghz,
                                            uint16_t Address,
                                            uint8_t *pBuffer,
                                            uint16_t Size)
{
  HAL_StatusTypeDef status;

  if (hsubghz->State == HAL_SUBGHZ_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsubghz);

    hsubghz->State = HAL_SUBGHZ_STATE_BUSY;

    (void)SUBGHZ_CheckDeviceReady(hsubghz);

    /* NSS = 0 */
    LL_PWR_SelectSUBGHZSPI_NSS();

    (void)SUBGHZSPI_Transmit(hsubghz, SUBGHZ_RADIO_WRITE_REGISTER);
    (void)SUBGHZSPI_Transmit(hsubghz, (uint8_t)((Address & 0xFF00U) >> 8U));
    (void)SUBGHZSPI_Transmit(hsubghz, (uint8_t)(Address & 0x00FFU));

    for (uint16_t i = 0U; i < Size; i++)
    {
      (void)SUBGHZSPI_Transmit(hsubghz, pBuffer[i]);
    }

    /* NSS = 1 */
    LL_PWR_UnselectSUBGHZSPI_NSS();

    (void)SUBGHZ_WaitOnBusy(hsubghz);

    if (hsubghz->ErrorCode != HAL_SUBGHZ_ERROR_NONE)
    {
      status = HAL_ERROR;
    }
    else
    {
      status = HAL_OK;
    }

    hsubghz->State = HAL_SUBGHZ_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsubghz);

    return status;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Read data register at an Address in the peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the configuration information for the specified SUBGHZ.
  * @param  Address register to configurate
  * @param  pBuffer pointer to a data buffer
  * @param  Size    amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SUBGHZ_ReadRegisters(SUBGHZ_HandleTypeDef *hsubghz,
                                           uint16_t Address,
                                           uint8_t *pBuffer,
                                           uint16_t Size)
{
  HAL_StatusTypeDef status;
  uint8_t *pData = pBuffer;

  if (hsubghz->State == HAL_SUBGHZ_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsubghz);

    (void)SUBGHZ_CheckDeviceReady(hsubghz);

    /* NSS = 0 */
    LL_PWR_SelectSUBGHZSPI_NSS();

    (void)SUBGHZSPI_Transmit(hsubghz, SUBGHZ_RADIO_READ_REGISTER);
    (void)SUBGHZSPI_Transmit(hsubghz, (uint8_t)((Address & 0xFF00U) >> 8U));
    (void)SUBGHZSPI_Transmit(hsubghz, (uint8_t)(Address & 0x00FFU));
    (void)SUBGHZSPI_Transmit(hsubghz, 0U);

    for (uint16_t i = 0U; i < Size; i++)
    {
      (void)SUBGHZSPI_Receive(hsubghz, (pData));
      pData++;
    }

    /* NSS = 1 */
    LL_PWR_UnselectSUBGHZSPI_NSS();

    (void)SUBGHZ_WaitOnBusy(hsubghz);

    if (hsubghz->ErrorCode != HAL_SUBGHZ_ERROR_NONE)
    {
      status = HAL_ERROR;
    }
    else
    {
      status = HAL_OK;
    }

    hsubghz->State = HAL_SUBGHZ_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsubghz);

    return status;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Write one data at an Address to configurate the peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the configuration information for the specified SUBGHZ.
  * @param  Address register to configurate
  * @param  Value data
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SUBGHZ_WriteRegister(SUBGHZ_HandleTypeDef *hsubghz,
                                           uint16_t Address,
                                           uint8_t Value)
{
  return (HAL_SUBGHZ_WriteRegisters(hsubghz, Address, &Value, 1U));
}

/**
  * @brief  Read data register at an Address in the peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the configuration information for the specified SUBGHZ.
  * @param  Address register to configurate
  * @param  pValue pointer to a data
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SUBGHZ_ReadRegister(SUBGHZ_HandleTypeDef *hsubghz,
                                          uint16_t Address,
                                          uint8_t *pValue)
{
  return (HAL_SUBGHZ_ReadRegisters(hsubghz, Address, pValue, 1U));
}

/**
  * @brief  Send a command to configure the peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the configuration information for the specified SUBGHZ.
  * @param  Command configuration for peripheral
  * @param  pBuffer pointer to a data buffer
  * @param  Size    amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SUBGHZ_ExecSetCmd(SUBGHZ_HandleTypeDef *hsubghz,
                                        SUBGHZ_RadioSetCmd_t Command,
                                        uint8_t *pBuffer,
                                        uint16_t Size)
{
  HAL_StatusTypeDef status;

  /* LORA Modulation not available on STM32WLx4xx devices */
  assert_param(IS_SUBGHZ_MODULATION_SUPPORTED(Command, pBuffer[0U]));

  if (hsubghz->State == HAL_SUBGHZ_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsubghz);

    /* Need to wakeup Radio if already in Sleep at startup */
    (void)SUBGHZ_CheckDeviceReady(hsubghz);

    if ((Command == RADIO_SET_SLEEP) || (Command == RADIO_SET_RXDUTYCYCLE))
    {
      hsubghz->DeepSleep = SUBGHZ_DEEP_SLEEP_ENABLE;
    }
    else
    {
      hsubghz->DeepSleep = SUBGHZ_DEEP_SLEEP_DISABLE;
    }

    /* NSS = 0 */
    LL_PWR_SelectSUBGHZSPI_NSS();

    (void)SUBGHZSPI_Transmit(hsubghz, (uint8_t)Command);

    for (uint16_t i = 0U; i < Size; i++)
    {
      (void)SUBGHZSPI_Transmit(hsubghz, pBuffer[i]);
    }

    /* NSS = 1 */
    LL_PWR_UnselectSUBGHZSPI_NSS();

    if (Command != RADIO_SET_SLEEP)
    {
      (void)SUBGHZ_WaitOnBusy(hsubghz);
    }

    if (hsubghz->ErrorCode != HAL_SUBGHZ_ERROR_NONE)
    {
      status = HAL_ERROR;
    }
    else
    {
      status = HAL_OK;
    }

    hsubghz->State = HAL_SUBGHZ_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsubghz);

    return status;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Retrieve a status from the peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the configuration information for the specified SUBGHZ.
  * @param  Command configuration for peripheral
  * @param  pBuffer pointer to a data buffer
  * @param  Size    amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SUBGHZ_ExecGetCmd(SUBGHZ_HandleTypeDef *hsubghz,
                                        SUBGHZ_RadioGetCmd_t Command,
                                        uint8_t *pBuffer,
                                        uint16_t Size)
{
  HAL_StatusTypeDef status;
  uint8_t *pData = pBuffer;

  if (hsubghz->State == HAL_SUBGHZ_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsubghz);

    (void)SUBGHZ_CheckDeviceReady(hsubghz);

    /* NSS = 0 */
    LL_PWR_SelectSUBGHZSPI_NSS();

    (void)SUBGHZSPI_Transmit(hsubghz, (uint8_t)Command);

    /* Use to flush the Status (First byte) receive from SUBGHZ as not use */
    // (void)SUBGHZSPI_Transmit(hsubghz, 0x00U);

    for (uint16_t i = 0U; i < Size; i++)
    {
      (void)SUBGHZSPI_Receive(hsubghz, (pData));
      pData++;
    }

    /* NSS = 1 */
    LL_PWR_UnselectSUBGHZSPI_NSS();

    (void)SUBGHZ_WaitOnBusy(hsubghz);

    if (hsubghz->ErrorCode != HAL_SUBGHZ_ERROR_NONE)
    {
      status = HAL_ERROR;
    }
    else
    {
      status = HAL_OK;
    }

    hsubghz->State = HAL_SUBGHZ_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsubghz);

    return status;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Write data buffer inside payload of peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the configuration information for the specified SUBGHZ.
  * @param  Offset  Offset inside payload
  * @param  pBuffer pointer to a data buffer
  * @param  Size    amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SUBGHZ_WriteBuffer(SUBGHZ_HandleTypeDef *hsubghz,
                                         uint8_t Offset,
                                         uint8_t *pBuffer,
                                         uint16_t Size)
{
  HAL_StatusTypeDef status;

  if (hsubghz->State == HAL_SUBGHZ_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsubghz);

    (void)SUBGHZ_CheckDeviceReady(hsubghz);

    /* NSS = 0 */
    LL_PWR_SelectSUBGHZSPI_NSS();

    (void)SUBGHZSPI_Transmit(hsubghz, SUBGHZ_RADIO_WRITE_BUFFER);
    (void)SUBGHZSPI_Transmit(hsubghz, Offset);

    for (uint16_t i = 0U; i < Size; i++)
    {
      (void)SUBGHZSPI_Transmit(hsubghz, pBuffer[i]);
    }
    /* NSS = 1 */
    LL_PWR_UnselectSUBGHZSPI_NSS();

    (void)SUBGHZ_WaitOnBusy(hsubghz);

    if (hsubghz->ErrorCode != HAL_SUBGHZ_ERROR_NONE)
    {
      status = HAL_ERROR;
    }
    else
    {
      status = HAL_OK;
    }

    hsubghz->State = HAL_SUBGHZ_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsubghz);

    return status;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Read data buffer inside payload of peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the configuration information for the specified SUBGHZ.
  * @param  Offset  Offset inside payload
  * @param  pBuffer pointer to a data buffer
  * @param  Size    amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SUBGHZ_ReadBuffer(SUBGHZ_HandleTypeDef *hsubghz,
                                        uint8_t Offset,
                                        uint8_t *pBuffer,
                                        uint16_t Size)
{
  HAL_StatusTypeDef status;
  uint8_t *pData = pBuffer;

  if (hsubghz->State == HAL_SUBGHZ_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hsubghz);

    (void)SUBGHZ_CheckDeviceReady(hsubghz);

    /* NSS = 0 */
    LL_PWR_SelectSUBGHZSPI_NSS();

    (void)SUBGHZSPI_Transmit(hsubghz, SUBGHZ_RADIO_READ_BUFFER);
    (void)SUBGHZSPI_Transmit(hsubghz, Offset);
    // (void)SUBGHZSPI_Transmit(hsubghz, 0x00U);

    for (uint16_t i = 0U; i < Size; i++)
    {
      (void)SUBGHZSPI_Receive(hsubghz, (pData));
      pData++;
    }

    /* NSS = 1 */
    LL_PWR_UnselectSUBGHZSPI_NSS();

    (void)SUBGHZ_WaitOnBusy(hsubghz);

    if (hsubghz->ErrorCode != HAL_SUBGHZ_ERROR_NONE)
    {
      status = HAL_ERROR;
    }
    else
    {
      status = HAL_OK;
    }

    hsubghz->State = HAL_SUBGHZ_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hsubghz);

    return status;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Handle SUBGHZ interrupt request.
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *               the configuration information for the specified SUBGHZ module.
  * @retval None
  */
void HAL_SUBGHZ_IRQHandler(SUBGHZ_HandleTypeDef *hsubghz)
{
  uint8_t tmpisr[3U] = {0U};
  uint16_t itsource;

  /* Retrieve Interrupts from SUBGHZ Irq Register */
  HAL_SUBGHZ_ExecGetCmd(hsubghz, RADIO_GET_IRQSTATUS, tmpisr, 3U);
  itsource = tmpisr[1U];
  itsource = (itsource << 8U) | tmpisr[2U];

  printf_("irqstatus = %#04x\r\n", itsource);

  /* Clear SUBGHZ Irq Register */
  HAL_SUBGHZ_ExecSetCmd(hsubghz, RADIO_CLR_IRQSTATUS, tmpisr+1, 2U);

  // if there is an error, don't do anything else
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_ERROR) != RESET)
  {
    // if you need more info about the error source, look at the packet status
    return;
  }

  /* Packet transmission completed Interrupt */
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_TXDONE) != RESET)
  {
    // do something
  }

  /* Packet received Interrupt */
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_RXDONE) != RESET)
  {
    // do something
    uint8_t packet_status[4];
    // subghz_radio_getstatus();
    subghz_radio_getPacketStatus(packet_status, true);
    LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    subghz_read_rx_buffer();
  }

  /* Preamble Detected Interrupt */
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_PREAMBLE_DETECTED) != RESET)
  {
    // do something
  }

  /*  Valid sync word detected Interrupt */
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_SYNCWORD_VALID) != RESET)
  {
    // do something
  }

  /* Rx or Tx Timeout Interrupt */
  if (SUBGHZ_CHECK_IT_SOURCE(itsource, SUBGHZ_IRQ_RX_TX_TIMEOUT) != RESET)
  {
    // do something
  }
}

/**
  * @}
  */

/** @defgroup SUBGHZ_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   SUBGHZ control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SUBGHZ.
     (+) HAL_SUBGHZ_GetState() API can be helpful to check in run-time the state of the SUBGHZ peripheral
     (+) HAL_SUBGHZ_GetError() check in run-time Errors occurring during communication
@endverbatim
  * @{
  */

/**
  * @brief  Return the SUBGHZ handle state.
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the handle information for SUBGHZ module.
  * @retval SUBGHZ state
  */
HAL_SUBGHZ_StateTypeDef HAL_SUBGHZ_GetState(SUBGHZ_HandleTypeDef *hsubghz)
{
  /* Return SUBGHZ handle state */
  return hsubghz->State;
}

/**
  * @brief  Return the SUBGHZ error code.
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the handle information for SUBGHZ module.
  * @retval SUBGHZ error code in bitmap format
  */
uint32_t HAL_SUBGHZ_GetError(SUBGHZ_HandleTypeDef *hsubghz)
{
  /* Return SUBGHZ ErrorCode */
  return hsubghz->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup SUBGHZ_Private_Functions
  * @brief   Private functions
  * @{
  */

/**
  * @brief  Initializes the SUBGHZSPI peripheral
  * @param  BaudratePrescaler SPI Baudrate prescaler
  * @retval None
  */
void SUBGHZSPI_Init(uint32_t BaudratePrescaler)
{
  /* Check the parameters */
  assert_param(IS_SUBGHZ_ALL_INSTANCE(SUBGHZSPI));

  /* Disable SUBGHZSPI Peripheral */
  CLEAR_BIT(SUBGHZSPI->CR1, SPI_CR1_SPE);

  /*----------------------- SPI CR1 Configuration ----------------------------*
   *             SPI Mode: Master                                             *
   *   Communication Mode: 2 lines (Full-Duplex)                              *
   *       Clock polarity: Low                                                *
   *                phase: 1st Edge                                           *
   *       NSS management: Internal (Done with External bit inside PWR        *
   *  Communication speed: BaudratePrescaler                                  *
   *            First bit: MSB                                                *
   *      CRC calculation: Disable                                            *
   *--------------------------------------------------------------------------*/
  WRITE_REG(SUBGHZSPI->CR1, (SPI_CR1_MSTR | SPI_CR1_SSI | BaudratePrescaler | SPI_CR1_SSM));

  /*----------------------- SPI CR2 Configuration ----------------------------*
   *            Data Size: 8bits                                              *
   *              TI Mode: Disable                                            *
   *            NSS Pulse: Disable                                            *
   *    Rx FIFO Threshold: 8bits                                              *
   *--------------------------------------------------------------------------*/
  WRITE_REG(SUBGHZSPI->CR2, (SPI_CR2_FRXTH |  SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2));

  /* Enable SUBGHZSPI Peripheral */
  SET_BIT(SUBGHZSPI->CR1, SPI_CR1_SPE);
}

/**
  * @brief  DeInitializes the SUBGHZSPI peripheral
  * @retval None
  */
void  SUBGHZSPI_DeInit(void)
{
  /* Check the parameters */
  assert_param(IS_SUBGHZ_ALL_INSTANCE(SUBGHZSPI));

  /* Disable SUBGHZSPI Peripheral */
  CLEAR_BIT(SUBGHZSPI->CR1, SPI_CR1_SPE);
}

/**
  * @brief  Transmit data through SUBGHZSPI peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the handle information for SUBGHZ module.
  * @param  Data  data to transmit
  * @retval HAL status
  */
HAL_StatusTypeDef SUBGHZSPI_Transmit(SUBGHZ_HandleTypeDef *hsubghz,
                                     uint8_t Data)
{
  HAL_StatusTypeDef status = HAL_OK;
  __IO uint32_t count;

  /* Handle Tx transmission from SUBGHZSPI peripheral to Radio ****************/
  /* Initialize Timeout */
  count = SUBGHZ_DEFAULT_TIMEOUT * SUBGHZ_DEFAULT_LOOP_TIME;

  /* Wait until TXE flag is set */
  do
  {
    if (count == 0U)
    {
      status = HAL_ERROR;
      hsubghz->ErrorCode = HAL_SUBGHZ_ERROR_TIMEOUT;
      break;
    }
    count--;
  } while (READ_BIT(SUBGHZSPI->SR, SPI_SR_TXE) != (SPI_SR_TXE));

  /* Transmit Data*/
#if defined (__GNUC__)
  __IO uint8_t *spidr = ((__IO uint8_t *)&SUBGHZSPI->DR);
  *spidr = Data;
#else
  *((__IO uint8_t *)&SUBGHZSPI->DR) = Data;
#endif /* __GNUC__ */

  /* Handle Rx transmission from SUBGHZSPI peripheral to Radio ****************/
  /* Initialize Timeout */
  count = SUBGHZ_DEFAULT_TIMEOUT * SUBGHZ_DEFAULT_LOOP_TIME;

  /* Wait until RXNE flag is set */
  do
  {
    if (count == 0U)
    {
      status = HAL_ERROR;
      hsubghz->ErrorCode = HAL_SUBGHZ_ERROR_TIMEOUT;
      break;
    }
    count--;
  } while (READ_BIT(SUBGHZSPI->SR, SPI_SR_RXNE) != (SPI_SR_RXNE));

  /* Flush Rx data */
  READ_REG(SUBGHZSPI->DR);

  return status;
}

/**
  * @brief  Receive data through SUBGHZSPI peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the handle information for SUBGHZ module.
  * @param  pData  pointer on data to receive
  * @retval HAL status
  */
HAL_StatusTypeDef SUBGHZSPI_Receive(SUBGHZ_HandleTypeDef *hsubghz,
                                    uint8_t *pData)
{
  HAL_StatusTypeDef status = HAL_OK;
  __IO uint32_t count;

  /* Handle Tx transmission from SUBGHZSPI peripheral to Radio ****************/
  /* Initialize Timeout */
  count = SUBGHZ_DEFAULT_TIMEOUT * SUBGHZ_DEFAULT_LOOP_TIME;

  /* Wait until TXE flag is set */
  do
  {
    if (count == 0U)
    {
      status = HAL_ERROR;
      hsubghz->ErrorCode = HAL_SUBGHZ_ERROR_TIMEOUT;
      break;
    }
    count--;
  } while (READ_BIT(SUBGHZSPI->SR, SPI_SR_TXE) != (SPI_SR_TXE));

  /* Transmit Data*/
#if defined (__GNUC__)
  __IO uint8_t *spidr = ((__IO uint8_t *)&SUBGHZSPI->DR);
  *spidr = SUBGHZ_DUMMY_DATA;
#else
  *((__IO uint8_t *)&SUBGHZSPI->DR) = SUBGHZ_DUMMY_DATA;
#endif /* __GNUC__ */

  /* Handle Rx transmission from SUBGHZSPI peripheral to Radio ****************/
  /* Initialize Timeout */
  count = SUBGHZ_DEFAULT_TIMEOUT * SUBGHZ_DEFAULT_LOOP_TIME;

  /* Wait until RXNE flag is set */
  do
  {
    if (count == 0U)
    {
      status = HAL_ERROR;
      hsubghz->ErrorCode = HAL_SUBGHZ_ERROR_TIMEOUT;
      break;
    }
    count--;
  } while (READ_BIT(SUBGHZSPI->SR, SPI_SR_RXNE) != (SPI_SR_RXNE));

  /* Retrieve pData */
  *pData = (uint8_t)(READ_REG(SUBGHZSPI->DR));

  return status;
}

/**
  * @brief  Check if peripheral is ready
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the handle information for SUBGHZ module.
  * @retval HAL status
  */
HAL_StatusTypeDef SUBGHZ_CheckDeviceReady(SUBGHZ_HandleTypeDef *hsubghz)
{
  __IO uint32_t count;

  /* Wakeup radio in case of sleep mode: Select-Unselect radio */
  if (hsubghz->DeepSleep == SUBGHZ_DEEP_SLEEP_ENABLE)
  {
    /* Initialize NSS switch Delay */
    count  = SUBGHZ_NSS_LOOP_TIME;

    /* NSS = 0; */
    LL_PWR_SelectSUBGHZSPI_NSS();

    /* Wait Radio wakeup */
    do
    {
      count--;
    } while (count != 0UL);

    /* NSS = 1 */
    LL_PWR_UnselectSUBGHZSPI_NSS();
  }
  return (SUBGHZ_WaitOnBusy(hsubghz));
}

/**
  * @brief  Wait busy flag low from peripheral
  * @param  hsubghz pointer to a SUBGHZ_HandleTypeDef structure that contains
  *         the handle information for SUBGHZ module.
  * @retval HAL status
  */
HAL_StatusTypeDef SUBGHZ_WaitOnBusy(SUBGHZ_HandleTypeDef *hsubghz)
{
  HAL_StatusTypeDef status;
  __IO uint32_t count;
  uint32_t mask;

  status = HAL_OK;
  count  = SUBGHZ_DEFAULT_TIMEOUT * SUBGHZ_RFBUSY_LOOP_TIME;

  /* Wait until Busy signal is set */
  do
  {
    mask = LL_PWR_IsActiveFlag_RFBUSYMS();

    if (count == 0U)
    {
      status  = HAL_ERROR;
      hsubghz->ErrorCode = HAL_SUBGHZ_ERROR_RF_BUSY;
      break;
    }
    count--;
  } while ((LL_PWR_IsActiveFlag_RFBUSYS()& mask) == 1UL);

  return status;
}
/**
  * @}
  */

#endif /* HAL_SUBGHZ_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */
