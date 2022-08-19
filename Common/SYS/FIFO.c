/**************************************************************************************************************
 File  :
 Author:
 Date  :
 Brief :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "FIFO.h"

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
bool FIFO_InitQueue(FIFO_QueueTypeDef *QueueStruct, volatile uint8_t *StoreSpaces, uint32_t SizeOfSpace)
{
  if (!QueueStruct || !StoreSpaces || !SizeOfSpace) {
    assert(false);
    return false;
  }

  QueueStruct->buf  = StoreSpaces;
  QueueStruct->size = SizeOfSpace;

  QueueStruct->in  = 0;
  QueueStruct->out = 0;

  return true;
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
bool FIFO_FlushQueue(FIFO_QueueTypeDef *QueueStruct)
{
  if (!QueueStruct) {
    assert(false);
    return false;
  }

  uint32_t in = QueueStruct->in; /* used to avoid volatile decision */
  QueueStruct->out = in;

  return true;
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
uint32_t FIFO_GetCount(FIFO_QueueTypeDef const *QueueStruct)
{
  if (!QueueStruct) {
    assert(false);
    return false;
  }

  uint32_t in  = QueueStruct->in ; /* used to avoid volatile decision */
  uint32_t out = QueueStruct->out; /* used to avoid volatile decision */

  return in - out;
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
uint32_t FIFO_GetAvailable(FIFO_QueueTypeDef const *QueueStruct)
{
  return (QueueStruct ? QueueStruct->size - FIFO_GetCount(QueueStruct) : 0);
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
bool FIFO_IsEmpty(FIFO_QueueTypeDef const *QueueStruct)
{
  return (QueueStruct ? (FIFO_GetCount(QueueStruct) == 0) : true);
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
bool FIFO_IsFull(FIFO_QueueTypeDef const *QueueStruct)
{
  return (QueueStruct ? (FIFO_GetCount(QueueStruct) == QueueStruct->size) : true);
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
int FIFO_GetQueue(FIFO_QueueTypeDef *QueueStruct)
{
  if (FIFO_IsEmpty(QueueStruct)) {
    return EOF;
  }

  return QueueStruct->buf[QueueStruct->out++ % QueueStruct->size];
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
int FIFO_PeekQueue(FIFO_QueueTypeDef const *QueueStruct)
{
  if (FIFO_IsEmpty(QueueStruct)) {
    return EOF;
  }

  return QueueStruct->buf[QueueStruct->out % QueueStruct->size];
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
int FIFO_PutQueue(FIFO_QueueTypeDef *QueueStruct, int DataByte)
{
  if (FIFO_IsFull(QueueStruct)) {
    return EOF;
  }

  return QueueStruct->buf[QueueStruct->in++ % QueueStruct->size] = DataByte;
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
bool FIFO_PushQueue(FIFO_QueueTypeDef *QueueStruct, const uint8_t *DataBuf, uint32_t DataLen)
{
  if (!QueueStruct || !DataBuf || !DataLen) {
    assert(false);
    return false;
  }

  if (FIFO_GetAvailable(QueueStruct) < DataLen) {
    return false;
  }

  while (DataLen--) {
    QueueStruct->buf[QueueStruct->in++ % QueueStruct->size] = *DataBuf++;
  }

  return true;
}

/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
bool FIFO_PullQueue(FIFO_QueueTypeDef *QueueStruct, uint8_t *DataBuf, uint32_t DataLen)
{
  if (!QueueStruct || !DataBuf || !DataLen) {
    assert(false);
    return false;
  }

  if (FIFO_GetCount(QueueStruct) < DataLen) {
    return false;
  }

  while (DataLen--) {
    *DataBuf++ = QueueStruct->buf[QueueStruct->out++ % QueueStruct->size];
  }

  return true;
}
