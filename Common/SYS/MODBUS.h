/**************************************************************************************************************
 File  :
 Author:
 Date  :
 Brief :
**************************************************************************************************************/

/* __MODBUS_H__ **********************************************************************************************/
#ifndef __MODBUS_H__
#define __MODBUS_H__

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes **************************************************************************************************/
#include "MAIN.h"

/* Includes **************************************************************************************************/
#include "BSP_STM32H7.h"
    
/* Functions *************************************************************************************************/
extern uint16_t MODBUS_ReplyToQuery(uint8_t *buf, uint16_t len, uint16_t size);

extern __weak bool     MODBUS_GetStatus  (uint8_t num, uint16_t adr);
extern __weak uint16_t MODBUS_GetRegister(uint8_t num, uint16_t adr);

extern __weak void     MODBUS_SetStatus  (uint8_t num, uint16_t adr, bool     val);
extern __weak void     MODBUS_SetRegister(uint8_t num, uint16_t adr, uint16_t val);

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  }
#endif

/* __MODBUS_H__ *************************************************************************************************/
#endif
