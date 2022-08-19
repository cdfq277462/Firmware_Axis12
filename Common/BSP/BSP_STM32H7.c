/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "BSP_STM32H7.h"

/**************************************************************************************************************
 Declaration : osRtxErrorNotify
 Parameters  : code     , The code to identify the error condition
 Parameters  : object_id, A reference to any RTX object to identify the object that caused the issue
 Return Value: None
 Description : OS Error Callback function
**************************************************************************************************************/
uint32_t osRtxErrorNotify(uint32_t code, void *object_id)
{
  switch (code) {
    case osRtxErrorStackUnderflow    : ErrorHandler(__FILE__, __LINE__); break;
    case osRtxErrorISRQueueOverflow  : ErrorHandler(__FILE__, __LINE__); break;
    case osRtxErrorTimerQueueOverflow: ErrorHandler(__FILE__, __LINE__); break;
    case osRtxErrorClibSpace         : ErrorHandler(__FILE__, __LINE__); break;
    case osRtxErrorClibMutex         : ErrorHandler(__FILE__, __LINE__); break;
    default                          : ErrorHandler(__FILE__, __LINE__); break;
  }

  /* Infinite loop will never happen */
  while (true) { __NOP(); };
}

/**************************************************************************************************************
 Declaration : net_sys_error
 Parameters  : error, fatal error code
 Return Value: None
 Description : Network system error handler
**************************************************************************************************************/
#if defined(RTE_Network_Core) && defined(CORE_CM4)
void net_sys_error(NET_ERROR error)
{
  switch (error) {
    case NET_ERROR_MEM_ALLOC  : ErrorHandler(__FILE__, __LINE__); break;
    case NET_ERROR_MEM_FREE   : ErrorHandler(__FILE__, __LINE__); break;
    case NET_ERROR_MEM_CORRUPT: ErrorHandler(__FILE__, __LINE__); break;
    case NET_ERROR_CONFIG     : ErrorHandler(__FILE__, __LINE__); break;
    case NET_ERROR_UDP_ALLOC  : ErrorHandler(__FILE__, __LINE__); break;
    case NET_ERROR_TCP_ALLOC  : ErrorHandler(__FILE__, __LINE__); break;
    case NET_ERROR_TCP_STATE  : ErrorHandler(__FILE__, __LINE__); break;
    default                   : ErrorHandler(__FILE__, __LINE__); break;
  }

  /* Infinite loop will never happen */
  while (true) { __NOP(); };
}
#endif