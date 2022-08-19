/**************************************************************************************************************
 File  :
 Author:
 Date  :
 Brief :
**************************************************************************************************************/

/* __FIFO_H__ ************************************************************************************************/
#ifndef __FIFO_H__
#define __FIFO_H__

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes **************************************************************************************************/
#include "MAIN.h"

/* Typedef ***************************************************************************************************/
typedef struct {
  volatile uint8_t *buf;
  volatile uint32_t size;
  volatile uint32_t in;
  volatile uint32_t out;
} FIFO_QueueTypeDef;

/* Functions *************************************************************************************************/
extern bool FIFO_InitQueue (FIFO_QueueTypeDef *QueueStruct,
                            volatile uint8_t  *StoreSpaces,
                            uint32_t           SizeOfSpace);
extern bool FIFO_FlushQueue(FIFO_QueueTypeDef *QueueStruct);

extern uint32_t FIFO_GetCount    (FIFO_QueueTypeDef const *QueueStruct);
extern uint32_t FIFO_GetAvailable(FIFO_QueueTypeDef const *QueueStruct);

extern bool FIFO_IsEmpty(FIFO_QueueTypeDef const *QueueStruct);
extern bool FIFO_IsFull (FIFO_QueueTypeDef const *QueueStruct);

extern int FIFO_GetQueue (FIFO_QueueTypeDef *QueueStruct);
extern int FIFO_PutQueue (FIFO_QueueTypeDef *QueueStruct, int DataByte);
extern int FIFO_PeekQueue(FIFO_QueueTypeDef const *QueueStruct);

extern bool FIFO_PushQueue(FIFO_QueueTypeDef *QueueStruct, const uint8_t *DataBuf, uint32_t DataLen);
extern bool FIFO_PullQueue(FIFO_QueueTypeDef *QueueStruct, uint8_t *DataBuf, uint32_t DataLen);

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  }
#endif

/* __FIFO_H__ ************************************************************************************************/
#endif
