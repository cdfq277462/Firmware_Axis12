/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "SOCK.h"
#include "FIFO.h"
#include "MODBUS.h"

/* Variables *************************************************************************************************/
static int32_t tcp_soc[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };
static int32_t udp_soc = -1;

static volatile uint8_t  TCP_QueueSpaces[8][1024];
static FIFO_QueueTypeDef TCP_QueueStruct[8];

/**************************************************************************************************************
 Declaration : udp_cb_func
 Parameters  : socket, UDP socket handle
 Parameters  : addr  , Pointer to the structure containing the remote IP address and port number
 Parameters  : buf   , Pointer to buffer containing the received data
 Parameters  : len   , Number of bytes in the received packet
 Return Value: Currently not used. Reserved for future use
 Description : UDP Event callback function
**************************************************************************************************************/
static uint32_t udp_cb_func(int32_t socket, const NET_ADDR *addr, const uint8_t *buf, uint32_t len)
{
 /* Check the parameters  */
  assert_param(buf && len);

  /* Data received */
  if (len == 0x00) { return 0; }
  if (buf == NULL) { return 0; }

  uint8_t *ptr = netUDP_GetBuffer(512);
  if (ptr == NULL) { return 0; }

  memcpy(ptr, buf, len);
  len = MODBUS_ReplyToQuery(ptr + 6, len - 6, 512 - 6);

  if (len) {
    ptr[5] = len;
    len += 6;
  }

  VERIFY(netUDP_Send(socket, addr, ptr, len) == netOK);

  /* Return function status */
  return 0;
}

/**************************************************************************************************************
 Declaration : tcp_cb_func
 Parameters  : socket, TCP socket handle.
 Parameters  : event , Event type as shown in the table below (netTCP_Event).
 Parameters  : addr  , Pointer to the structure containing the remote IP address and port number.
 Parameters  : buf   , Pointer to buffer containing the received data.
 Parameters  : len   , Number of bytes in the received packet.
 Return Value: Integer Value used to decide whether to accept or reject an incoming connection
 Description : TCP Event callback function
**************************************************************************************************************/
static uint32_t tcp_cb_func(int32_t socket, netTCP_Event event, const NET_ADDR *addr,
                                                    const uint8_t *buf, uint32_t len)
{
  /* Connect request received in server mode */
  if (event == netTCP_EventConnect) {
    VERIFY(osThreadFlagsSet(app_sock_tid, TCP_EVENT_FLAG_CONNECT) != osFlagsError);
    return 1;
  }

  /* Connection established */
  else if (event == netTCP_EventEstablished) {
    VERIFY(osThreadFlagsSet(app_sock_tid, TCP_EVENT_FLAG_ESTABLISHED) != osFlagsError);
  }

  /* Connection was properly closed */
  else if (event == netTCP_EventClosed) {
    VERIFY(osThreadFlagsSet(app_sock_tid, TCP_EVENT_FLAG_CLOSED) != osFlagsError);
  }

  /* Connection is for some reason aborted */
  else if (event == netTCP_EventAborted) {
    VERIFY(osThreadFlagsSet(app_sock_tid, TCP_EVENT_FLAG_ABORTED) != osFlagsError);
  }

  /* Previously sent data acknowledged */
  else if (event == netTCP_EventACK) {
    VERIFY(osThreadFlagsSet(app_sock_tid, TCP_EVENT_FLAG_ACK) != osFlagsError);
  }

  /* Data received */
  else if (event == netTCP_EventData) {
    if (socket == tcp_soc[0]) {
      if (!FIFO_PushQueue(&TCP_QueueStruct[0], (void *) buf, len)) {
        VERIFY(netTCP_Abort(socket) == netOK);
      }
    } else
    if (socket == tcp_soc[1]) {
      if (!FIFO_PushQueue(&TCP_QueueStruct[1], (void *) buf, len)) {
        VERIFY(netTCP_Abort(socket) == netOK);
      }
    } else
    if (socket == tcp_soc[2]) {
      if (!FIFO_PushQueue(&TCP_QueueStruct[2], (void *) buf, len)) {
        VERIFY(netTCP_Abort(socket) == netOK);
      }
    } else
    if (socket == tcp_soc[3]) {
      if (!FIFO_PushQueue(&TCP_QueueStruct[3], (void *) buf, len)) {
        VERIFY(netTCP_Abort(socket) == netOK);
      }
    } else
    if (socket == tcp_soc[4]) {
      if (!FIFO_PushQueue(&TCP_QueueStruct[4], (void *) buf, len)) {
        VERIFY(netTCP_Abort(socket) == netOK);
      }
    } else
    if (socket == tcp_soc[5]) {
      if (!FIFO_PushQueue(&TCP_QueueStruct[5], (void *) buf, len)) {
        VERIFY(netTCP_Abort(socket) == netOK);
      }
    } else
    if (socket == tcp_soc[6]) {
      if (!FIFO_PushQueue(&TCP_QueueStruct[6], (void *) buf, len)) {
        VERIFY(netTCP_Abort(socket) == netOK);
      }
    } else
    if (socket == tcp_soc[7]) {
      if (!FIFO_PushQueue(&TCP_QueueStruct[7], (void *) buf, len)) {
        VERIFY(netTCP_Abort(socket) == netOK);
      }
    }

    VERIFY(osThreadFlagsSet(app_sock_tid, TCP_EVENT_FLAG_DATA) != osFlagsError);
  }

  /* Return function status */
  return 0;
}

/**************************************************************************************************************
 Declaration : tcp_sock_send
 Parameters  : socket, socket handle obtained with netTCP_GetSocket
 Parameters  : buf   , buffer containing the data
 Parameters  : len   , length of data in bytes
 Return Value: Status code that indicates the execution status of the function
 Description : Send a data packet to remote node
**************************************************************************************************************/
static netStatus tcp_sock_send(int32_t socket, uint8_t *buf, uint32_t len)
{
  /* Check the parameters */
  assert_param(socket >= 0);
  assert_param(buf != NULL);

  if (socket < 0 || !buf || !len) {
    return netError;
  }

  /* Check if TCP socket can send data */
  if (!netTCP_SendReady(socket)) {
    return netError;
  }

  /* Allocate memory for TCP send buffer */
  uint8_t *tmp = (void *) netTCP_GetBuffer(len);

  if (!tmp) {
    return netError;
  }
  else {
    memcpy(tmp, buf, len);
  }

  /* Must call RL_TCP netTCP_Send anyway */
  return netTCP_Send(socket, tmp, len);
}

/**************************************************************************************************************
 Declaration : app_sock_task
 Parameters  : argument, Arbitrary user data set on osThreadNew
 Return Value: None
 Description : application socket thread
**************************************************************************************************************/
__NO_RETURN void app_sock_task(void *argument)
{
  size_t len = 0;
  uint8_t buf[512];

  /* Get the currently running thread */
  app_sock_tid = osThreadGetId();
  ASSERT(app_sock_tid != NULL);

  /* Allocate free UDP socket and open */
  udp_soc = netUDP_GetSocket(udp_cb_func);
  VERIFY(netUDP_Open(udp_soc, 502) == netOK);

  /* Allocate free TCP socket and set options */
  tcp_soc[0] = netTCP_GetSocket(tcp_cb_func);
  tcp_soc[1] = netTCP_GetSocket(tcp_cb_func);
  tcp_soc[2] = netTCP_GetSocket(tcp_cb_func);
  tcp_soc[3] = netTCP_GetSocket(tcp_cb_func);
  tcp_soc[4] = netTCP_GetSocket(tcp_cb_func);
  tcp_soc[5] = netTCP_GetSocket(tcp_cb_func);
  tcp_soc[6] = netTCP_GetSocket(tcp_cb_func);
  tcp_soc[7] = netTCP_GetSocket(tcp_cb_func);

  VERIFY(netTCP_SetOption(tcp_soc[0], netTCP_OptionTimeout    , 4) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[1], netTCP_OptionTimeout    , 4) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[2], netTCP_OptionTimeout    , 4) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[3], netTCP_OptionTimeout    , 4) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[4], netTCP_OptionTimeout    , 4) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[5], netTCP_OptionTimeout    , 4) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[6], netTCP_OptionTimeout    , 4) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[7], netTCP_OptionTimeout    , 4) == netOK);

  VERIFY(netTCP_SetOption(tcp_soc[0], netTCP_OptionKeepAlive  , 1) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[1], netTCP_OptionKeepAlive  , 1) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[2], netTCP_OptionKeepAlive  , 1) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[3], netTCP_OptionKeepAlive  , 1) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[4], netTCP_OptionKeepAlive  , 1) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[5], netTCP_OptionKeepAlive  , 1) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[6], netTCP_OptionKeepAlive  , 1) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[7], netTCP_OptionKeepAlive  , 1) == netOK);

  VERIFY(netTCP_SetOption(tcp_soc[0], netTCP_OptionDelayedACK , 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[1], netTCP_OptionDelayedACK , 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[2], netTCP_OptionDelayedACK , 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[3], netTCP_OptionDelayedACK , 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[4], netTCP_OptionDelayedACK , 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[5], netTCP_OptionDelayedACK , 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[6], netTCP_OptionDelayedACK , 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[7], netTCP_OptionDelayedACK , 0) == netOK);

  VERIFY(netTCP_SetOption(tcp_soc[0], netTCP_OptionFlowControl, 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[1], netTCP_OptionFlowControl, 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[2], netTCP_OptionFlowControl, 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[3], netTCP_OptionFlowControl, 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[4], netTCP_OptionFlowControl, 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[5], netTCP_OptionFlowControl, 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[6], netTCP_OptionFlowControl, 0) == netOK);
  VERIFY(netTCP_SetOption(tcp_soc[7], netTCP_OptionFlowControl, 0) == netOK);

  /* Initializes the TCP FIFO Queue */
  VERIFY(FIFO_InitQueue(&TCP_QueueStruct[0], TCP_QueueSpaces[0], sizeof(TCP_QueueSpaces[0])));
  VERIFY(FIFO_InitQueue(&TCP_QueueStruct[1], TCP_QueueSpaces[1], sizeof(TCP_QueueSpaces[1])));
  VERIFY(FIFO_InitQueue(&TCP_QueueStruct[2], TCP_QueueSpaces[2], sizeof(TCP_QueueSpaces[2])));
  VERIFY(FIFO_InitQueue(&TCP_QueueStruct[3], TCP_QueueSpaces[3], sizeof(TCP_QueueSpaces[3])));
  VERIFY(FIFO_InitQueue(&TCP_QueueStruct[4], TCP_QueueSpaces[4], sizeof(TCP_QueueSpaces[4])));
  VERIFY(FIFO_InitQueue(&TCP_QueueStruct[5], TCP_QueueSpaces[5], sizeof(TCP_QueueSpaces[5])));
  VERIFY(FIFO_InitQueue(&TCP_QueueStruct[6], TCP_QueueSpaces[6], sizeof(TCP_QueueSpaces[6])));
  VERIFY(FIFO_InitQueue(&TCP_QueueStruct[7], TCP_QueueSpaces[7], sizeof(TCP_QueueSpaces[7])));

  /* Start thread infinite loop */
  while (true) {
    /* Notify main thread activity flag */
    VERIFY(osThreadFlagsSet(app_main_tid, (uint32_t) argument) != osFlagsError);

    /* Open TCP socket for incoming connection */
    if (netTCP_GetState(tcp_soc[0]) == netTCP_StateCLOSED ||
        netTCP_GetState(tcp_soc[0]) == netTCP_StateUNUSED) {
      VERIFY(FIFO_FlushQueue(&TCP_QueueStruct[0]));
      VERIFY(netTCP_Listen(tcp_soc[0], 502) == netOK);
    }
    if (netTCP_GetState(tcp_soc[1]) == netTCP_StateCLOSED ||
        netTCP_GetState(tcp_soc[1]) == netTCP_StateUNUSED) {
      VERIFY(FIFO_FlushQueue(&TCP_QueueStruct[1]));
      VERIFY(netTCP_Listen(tcp_soc[1], 502) == netOK);
    }
    if (netTCP_GetState(tcp_soc[2]) == netTCP_StateCLOSED ||
        netTCP_GetState(tcp_soc[2]) == netTCP_StateUNUSED) {
      VERIFY(FIFO_FlushQueue(&TCP_QueueStruct[2]));
      VERIFY(netTCP_Listen(tcp_soc[2], 502) == netOK);
    }
    if (netTCP_GetState(tcp_soc[3]) == netTCP_StateCLOSED ||
        netTCP_GetState(tcp_soc[3]) == netTCP_StateUNUSED) {
      VERIFY(FIFO_FlushQueue(&TCP_QueueStruct[3]));
      VERIFY(netTCP_Listen(tcp_soc[3], 502) == netOK);
    }
    if (netTCP_GetState(tcp_soc[4]) == netTCP_StateCLOSED ||
        netTCP_GetState(tcp_soc[4]) == netTCP_StateUNUSED) {
      VERIFY(FIFO_FlushQueue(&TCP_QueueStruct[4]));
      VERIFY(netTCP_Listen(tcp_soc[4], 502) == netOK);
    }
    if (netTCP_GetState(tcp_soc[5]) == netTCP_StateCLOSED ||
        netTCP_GetState(tcp_soc[5]) == netTCP_StateUNUSED) {
      VERIFY(FIFO_FlushQueue(&TCP_QueueStruct[5]));
      VERIFY(netTCP_Listen(tcp_soc[5], 502) == netOK);
    }
    if (netTCP_GetState(tcp_soc[6]) == netTCP_StateCLOSED ||
        netTCP_GetState(tcp_soc[6]) == netTCP_StateUNUSED) {
      VERIFY(FIFO_FlushQueue(&TCP_QueueStruct[6]));
      VERIFY(netTCP_Listen(tcp_soc[6], 502) == netOK);
    }
    if (netTCP_GetState(tcp_soc[7]) == netTCP_StateCLOSED ||
        netTCP_GetState(tcp_soc[7]) == netTCP_StateUNUSED) {
      VERIFY(FIFO_FlushQueue(&TCP_QueueStruct[7]));
      VERIFY(netTCP_Listen(tcp_soc[7], 502) == netOK);
    }

    /* Wait for the thread flag to be set */
    osThreadFlagsWait(TCP_EVENT_FLAG_MASK, osFlagsWaitAny, 1000);

    /* TCP socket response modbus message */
    if (netTCP_SendReady(tcp_soc[0]) && FIFO_GetCount(&TCP_QueueStruct[0]) >= 12) {
      len = FIFO_GetCount(&TCP_QueueStruct[0]);
      VERIFY(FIFO_PullQueue(&TCP_QueueStruct[0], buf, len));

      len = MODBUS_ReplyToQuery(buf + 6, len - 6, sizeof(buf) - 6);
      buf[5] = len;

      if (len && tcp_sock_send(tcp_soc[0], buf, len + 6) != netOK) {
        VERIFY(netTCP_Abort(tcp_soc[0]) == netOK);
      }
    }
    if (netTCP_SendReady(tcp_soc[1]) && FIFO_GetCount(&TCP_QueueStruct[1]) >= 12) {
      len = FIFO_GetCount(&TCP_QueueStruct[1]);
      VERIFY(FIFO_PullQueue(&TCP_QueueStruct[1], buf, len));

      len = MODBUS_ReplyToQuery(buf + 6, len - 6, sizeof(buf) - 6);
      buf[5] = len;

      if (len && tcp_sock_send(tcp_soc[1], buf, len + 6) != netOK) {
        VERIFY(netTCP_Abort(tcp_soc[1]) == netOK);
      }
    }
    if (netTCP_SendReady(tcp_soc[2]) && FIFO_GetCount(&TCP_QueueStruct[2]) >= 12) {
      len = FIFO_GetCount(&TCP_QueueStruct[2]);
      VERIFY(FIFO_PullQueue(&TCP_QueueStruct[2], buf, len));

      len = MODBUS_ReplyToQuery(buf + 6, len - 6, sizeof(buf) - 6);
      buf[5] = len;

      if (len && tcp_sock_send(tcp_soc[2], buf, len + 6) != netOK) {
        VERIFY(netTCP_Abort(tcp_soc[2]) == netOK);
      }
    }
    if (netTCP_SendReady(tcp_soc[3]) && FIFO_GetCount(&TCP_QueueStruct[3]) >= 12) {
      len = FIFO_GetCount(&TCP_QueueStruct[3]);
      VERIFY(FIFO_PullQueue(&TCP_QueueStruct[3], buf, len));

      len = MODBUS_ReplyToQuery(buf + 6, len - 6, sizeof(buf) - 6);
      buf[5] = len;

      if (len && tcp_sock_send(tcp_soc[3], buf, len + 6) != netOK) {
        VERIFY(netTCP_Abort(tcp_soc[3]) == netOK);
      }
    }
    if (netTCP_SendReady(tcp_soc[4]) && FIFO_GetCount(&TCP_QueueStruct[4]) >= 12) {
      len = FIFO_GetCount(&TCP_QueueStruct[4]);
      VERIFY(FIFO_PullQueue(&TCP_QueueStruct[4], buf, len));

      len = MODBUS_ReplyToQuery(buf + 6, len - 6, sizeof(buf) - 6);
      buf[5] = len;

      if (len && tcp_sock_send(tcp_soc[4], buf, len + 6) != netOK) {
        VERIFY(netTCP_Abort(tcp_soc[4]) == netOK);
      }
    }
    if (netTCP_SendReady(tcp_soc[5]) && FIFO_GetCount(&TCP_QueueStruct[5]) >= 12) {
      len = FIFO_GetCount(&TCP_QueueStruct[5]);
      VERIFY(FIFO_PullQueue(&TCP_QueueStruct[5], buf, len));

      len = MODBUS_ReplyToQuery(buf + 6, len - 6, sizeof(buf) - 6);
      buf[5] = len;

      if (len && tcp_sock_send(tcp_soc[5], buf, len + 6) != netOK) {
        VERIFY(netTCP_Abort(tcp_soc[5]) == netOK);
      }
    }
    if (netTCP_SendReady(tcp_soc[6]) && FIFO_GetCount(&TCP_QueueStruct[6]) >= 12) {
      len = FIFO_GetCount(&TCP_QueueStruct[6]);
      VERIFY(FIFO_PullQueue(&TCP_QueueStruct[6], buf, len));

      len = MODBUS_ReplyToQuery(buf + 6, len - 6, sizeof(buf) - 6);
      buf[5] = len;

      if (len && tcp_sock_send(tcp_soc[6], buf, len + 6) != netOK) {
        VERIFY(netTCP_Abort(tcp_soc[6]) == netOK);
      }
    }
    if (netTCP_SendReady(tcp_soc[7]) && FIFO_GetCount(&TCP_QueueStruct[7]) >= 12) {
      len = FIFO_GetCount(&TCP_QueueStruct[7]);
      VERIFY(FIFO_PullQueue(&TCP_QueueStruct[7], buf, len));

      len = MODBUS_ReplyToQuery(buf + 6, len - 6, sizeof(buf) - 6);
      buf[5] = len;

      if (len && tcp_sock_send(tcp_soc[7], buf, len + 6) != netOK) {
        VERIFY(netTCP_Abort(tcp_soc[7]) == netOK);
      }
    }

    /* Passes the control to the next thread */
    VERIFY(osThreadYield() == osOK);
  }

  /* Never happen */
  ErrorHandler(__FILE__, __LINE__);
}
