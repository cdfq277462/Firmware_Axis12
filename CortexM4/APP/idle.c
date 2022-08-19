/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "IDLE.h"

/* Variables *************************************************************************************************/
/**************************************************************************************************************
 Declaration :
 Parameters  :
 Return Value:
 Description :
**************************************************************************************************************/
static uint32_t udp_cb_func(int32_t socket, const NET_ADDR *addr, const uint8_t *buf, uint32_t len)
{
  return 0;
}

/**************************************************************************************************************
 Declaration : app_idle_task
 Parameters  : argument, Arbitrary user data set on osThreadNew
 Return Value: None
 Description : application idle thread
**************************************************************************************************************/
__NO_RETURN void app_idle_task(void *argument)
{
  int32_t socket = -1;
  uint8_t *buf;
  const uint32_t len = 64;
  const NET_ADDR addr = { NET_ADDR_IP4, 6052, 192, 6, 1, 100 };

  /* Get the currently running thread */
  app_idle_tid = osThreadGetId();
  ASSERT(app_idle_tid != NULL);

  /* Allocate free UDP socket and open */
  socket = netUDP_GetSocket(udp_cb_func);
  VERIFY(netUDP_Open(socket, 6052) == netOK);

  /* Start thread infinite loop */
  while (true) {
    /* Notify main thread activity flag */
    VERIFY(osThreadFlagsSet(app_main_tid, (uint32_t) argument) != osFlagsError);

    /* Please do something */
    if (FIFO_GetCount(UDP_QueueStruct) < len) {
      continue;
    }

    buf = netUDP_GetBuffer(len);
    if (buf == 0) { continue; };

   VERIFY(FIFO_PullQueue(UDP_QueueStruct, buf, len));
   VERIFY(netUDP_Send(socket, &addr, buf, len) == netOK);

    /* Passes the control to the next thread */
    VERIFY(osThreadYield() == osOK);
  }

  /* Never happen */
  ErrorHandler(__FILE__, __LINE__);
}
