/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "EEP_STM32H7.h"

/* Includes **************************************************************************************************/
#include "eeprom.h"

/**************************************************************************************************************
 Declaration : EEP_Init
 Parameters  : None
 Return Value: HAL Status
 Description : Initialize the EEPROM emulation
**************************************************************************************************************/
HAL_StatusTypeDef EEP_Init(void)
{
  if (HAL_FLASH_Unlock() != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  if (EE_Init() != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  if (HAL_FLASH_Lock() != HAL_OK) {
    ErrorHandler(__FILE__, __LINE__);
  }

  return HAL_OK;
}

/**************************************************************************************************************
 Declaration : EEP_DeInit
 Parameters  : None
 Return Value: HAL Status
 Description : De-Initialize the EEPROM emulation
**************************************************************************************************************/
HAL_StatusTypeDef EEP_DeInit(void)
{
  return HAL_ERROR;
}

/**************************************************************************************************************
 Declaration : EEP_ReadVariable
 Parameters  : Address - Variable address
 Parameters  : Data    - Global variable contains the read variable value
 Return Value: HAL Status
 Description : Returns the last stored variable data
**************************************************************************************************************/
HAL_StatusTypeDef EEP_ReadVariable(uint16_t Address, uint32_t *Data)
{
  return EE_ReadVariable(Address, Data);
}

/**************************************************************************************************************
 Declaration : EEP_WriteVariable
 Parameters  : Address - Variable address
 Parameters  : Data    - 32 bit data to be written
 Return Value: HAL Status
 Description : Writes/updates variable data in EEPROM
**************************************************************************************************************/
HAL_StatusTypeDef EEP_WriteVariable(uint16_t Address, uint32_t Data)
{
  HAL_StatusTypeDef Status;

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return HAL_ERROR;
  }

  Status = EE_WriteVariable(Address, Data);

  if (HAL_FLASH_Lock() != HAL_OK) {
    return HAL_ERROR;
  }

  return Status;
}
