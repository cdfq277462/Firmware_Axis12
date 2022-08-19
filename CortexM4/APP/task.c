/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* Includes **************************************************************************************************/
#include "TASK.h"
#include "SOCK.h"
#include "TEST.h"
#include "IDLE.h"

/* Variables *************************************************************************************************/
const osThreadAttr_t app_main_attr = { .name = "main", .stack_size = 2048, .priority = osPriorityNormal1 };
const osThreadAttr_t app_sock_attr = { .name = "sock", .stack_size = 2048, .priority = osPriorityNormal1 };
const osThreadAttr_t app_test_attr = { .name = "test", .stack_size = 2048, .priority = osPriorityNormal1 };
const osThreadAttr_t app_idle_attr = { .name = "idle", .stack_size = 2048, .priority = osPriorityNormal  };

/* Variables *************************************************************************************************/
osThreadId_t app_main_tid = NULL;
osThreadId_t app_sock_tid = NULL;
osThreadId_t app_test_tid = NULL;
osThreadId_t app_idle_tid = NULL;

/**************************************************************************************************************
 Declaration : netDHCP_Notify
 Parameters  : if_id , Interface identification (class and number).
 Parameters  : option, DHCP option code
 Parameters  : val   , pointer to option value
 Parameters  : len   , length of option value in bytes
 Return Value: None
 Description : Notify the user of DHCP event or extended DHCP option
**************************************************************************************************************/
void netDHCP_Notify(uint32_t if_id, uint8_t option, const uint8_t *val, uint32_t len)
{
  if (NET_DHCP_OPTION_IP_ADDRESS == option) {
    netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionIP4_Address       , LocalMS.IP_Address  , NET_ADDR_IP4_LEN);
    netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionIP4_SubnetMask    , LocalMS.SubnetMask  , NET_ADDR_IP4_LEN);
    netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionIP4_DefaultGateway, LocalMS.DefGateWay  , NET_ADDR_IP4_LEN);
    netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionIP4_PrimaryDNS    , LocalMS.PrimaryDNS  , NET_ADDR_IP4_LEN);
    netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionIP4_SecondaryDNS  , LocalMS.SecondaryDNS, NET_ADDR_IP4_LEN);
    netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionMAC_Address       , LocalMS.ETH_Address , NET_ADDR_ETH_LEN);
  }
}

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
  const NET_ADDR broadcast_addr = { NET_ADDR_IP4, addr->port, 255, 255, 255, 255 };
  const size_t size = 0x0100;

  /* Packet length */
  if (len != size) {
    return 0;
  }

  /* Product information */
  else if (buf[0] != 'G' || buf[1] != 'o' || buf[2] != 'r' ||
           buf[3] != 'd' || buf[4] != 'e' || buf[5] != 'n') {
    return 0;
  }

  /* Query Command */
  else if (buf[6] == 0x00) {
    LOCALMI *LocalMI = (LOCALMI *) netUDP_GetBuffer(size);

    if (LocalMI == NULL) {
      return 0;
    }
    else {
      memset(LocalMI, 0, sizeof(LOCALMI));
    }

    memcpy(LocalMI->ProductInfo, "Gorden\x80", 7);
    memcpy(LocalMI->ProductName, "STM32H7\x0", 8);

    memcpy(LocalMI->ETH_Address, LocalMS.ETH_Address, NET_ADDR_ETH_LEN);
    memcpy(&LocalMI->LocalMS, &LocalMS, sizeof(LOCALMS));

    LocalMI->FW_Version = FW_VERSION;
    VERIFY(netUDP_Send(socket, &broadcast_addr, (uint8_t *) LocalMI, size) == netOK);
  }

  /* MAC Address */
  else if (memcmp(((LOCALMI *) buf)->ETH_Address, LocalMS.ETH_Address, NET_ADDR_ETH_LEN)) {
    return 0;
  }

  /* Setup Command */
  else if (buf[6] == 0x10) {
#if 0
    struct tm tm;
    time_t timer;

    RTC_TimeTypeDef RTC_TimeStruct;
    RTC_DateTypeDef RTC_DateStruct;

    VERIFY(RTC_GetTime(&RTC_TimeStruct, RTC_FORMAT_BIN) == HAL_OK);
    VERIFY(RTC_GetDate(&RTC_DateStruct, RTC_FORMAT_BIN) == HAL_OK);

    RTC_DateStruct.Year    = min(max(((LOCALMI *) buf)->RTC_Date[0], 0), 99);
    RTC_DateStruct.Month   = min(max(((LOCALMI *) buf)->RTC_Date[1], 1), 12);
    RTC_DateStruct.Date    = min(max(((LOCALMI *) buf)->RTC_Date[2], 1), 31);

    RTC_TimeStruct.Hours   = min(max(((LOCALMI *) buf)->RTC_Time[0], 0), 23);
    RTC_TimeStruct.Minutes = min(max(((LOCALMI *) buf)->RTC_Time[1], 0), 59);
    RTC_TimeStruct.Seconds = min(max(((LOCALMI *) buf)->RTC_Time[2], 0), 59);

    tm.tm_mday = RTC_DateStruct.Date;
    tm.tm_mon  = RTC_DateStruct.Month - 1;
    tm.tm_year = RTC_DateStruct.Year + 100;

    tm.tm_sec  = RTC_TimeStruct.Seconds;
    tm.tm_min  = RTC_TimeStruct.Minutes;
    tm.tm_hour = RTC_TimeStruct.Hours;

    timer = mktime(&tm);
    _localtime_r(&timer, &tm);

    tm.tm_wday = tm.tm_wday ? tm.tm_wday : 7;
    RTC_DateStruct.WeekDay = max(min(tm.tm_wday, 7), 1);

    VERIFY(RTC_SetTime(&RTC_TimeStruct, RTC_FORMAT_BIN) == HAL_OK);
    VERIFY(RTC_SetDate(&RTC_DateStruct, RTC_FORMAT_BIN) == HAL_OK);
#endif

    memcpy(&LocalMS, &((LOCALMI *) buf)->LocalMS, sizeof(LocalMS));
#if 0
    VERIFY(LMS_Save() == SUCCESS);
#endif

    NVIC_SystemReset();
  }

  /* Return status */
  return 0;
}

/**************************************************************************************************************
 Declaration : app_main_task
 Parameters  : argument, Arbitrary user data set on osThreadNew
 Return Value: None
 Description : application main thread
**************************************************************************************************************/
__NO_RETURN void app_main_task(void *argument)
{
  /* Wait for the peripheral to ready */
  while (osKernelGetTickCount() < 100);

  /* Get the currently running thread */
  app_main_tid = osThreadGetId();
  ASSERT(app_main_tid != NULL);

  /* Load local machine settings */
  VERIFY(LMS_Load(&LocalMS) == SUCCESS);

  /* Initialize the Adc peripherals */
  VERIFY(ADC_Init() == HAL_OK);

  /* Initialize the EEP peripherals */
  VERIFY(EEP_Init() == HAL_OK);

  EEP_ReadVariable(0, (uint32_t *) &(TMC1->R.RESERVE_19)); /* REF_OFFSET_POSITION */
  EEP_ReadVariable(1, (uint32_t *) &(TMC2->R.RESERVE_19)); /* REF_OFFSET_POSITION */
  EEP_ReadVariable(2, (uint32_t *) &(TMC3->R.RESERVE_19)); /* REF_OFFSET_POSITION */
  EEP_ReadVariable(3, (uint32_t *) &(TMC4->R.RESERVE_19)); /* REF_OFFSET_POSITION */
  EEP_ReadVariable(4, (uint32_t *) &(TMC5->R.RESERVE_19)); /* REF_OFFSET_POSITION */
  EEP_ReadVariable(5, (uint32_t *) &(TMC6->R.RESERVE_19)); /* REF_OFFSET_POSITION */

  TMC1->W.RESERVE_19 = TMC1->M.RESERVE_19 = TMC1->R.RESERVE_19; /* REF_OFFSET_POSITION */
  TMC2->W.RESERVE_19 = TMC2->M.RESERVE_19 = TMC2->R.RESERVE_19; /* REF_OFFSET_POSITION */
  TMC3->W.RESERVE_19 = TMC3->M.RESERVE_19 = TMC3->R.RESERVE_19; /* REF_OFFSET_POSITION */
  TMC4->W.RESERVE_19 = TMC4->M.RESERVE_19 = TMC4->R.RESERVE_19; /* REF_OFFSET_POSITION */
  TMC5->W.RESERVE_19 = TMC5->M.RESERVE_19 = TMC5->R.RESERVE_19; /* REF_OFFSET_POSITION */
  TMC6->W.RESERVE_19 = TMC6->M.RESERVE_19 = TMC6->R.RESERVE_19; /* REF_OFFSET_POSITION */

  /* Initialize the ETH peripherals */
  VERIFY(netInitialize() == netOK);

  netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionIP4_Address       , LocalMS.IP_Address  , NET_ADDR_IP4_LEN);
  netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionIP4_SubnetMask    , LocalMS.SubnetMask  , NET_ADDR_IP4_LEN);
  netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionIP4_DefaultGateway, LocalMS.DefGateWay  , NET_ADDR_IP4_LEN);
  netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionIP4_PrimaryDNS    , LocalMS.PrimaryDNS  , NET_ADDR_IP4_LEN);
  netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionIP4_SecondaryDNS  , LocalMS.SecondaryDNS, NET_ADDR_IP4_LEN);
  netIF_GetOption(NET_IF_CLASS_ETH, netIF_OptionMAC_Address       , LocalMS.ETH_Address , NET_ADDR_ETH_LEN);

  /* Create the application threads */
  VERIFY(osThreadNew(app_sock_task, (void *) 0x00000001UL, &app_sock_attr) != NULL);
  VERIFY(osThreadNew(app_test_task, (void *) 0x00000002UL, &app_test_attr) != NULL);
  VERIFY(osThreadNew(app_idle_task, (void *) 0x00000004UL, &app_idle_attr) != NULL);

  /* Allocate a free UDP socket and open */
  VERIFY(netUDP_Open(netUDP_GetSocket(udp_cb_func), 1208) == netOK);

  /* Start thread infinite loop */
  while (true) {
    /* Check that all threads are alive */
    if ((osThreadFlagsGet() & 0x03) == 0x03) {
      VERIFY(osThreadFlagsClear(0x03) != osFlagsError);
    }

    /* Parameter has changed record value */
    if (TMC1->W.RESERVE_19 != TMC1->M.RESERVE_19) { /* #1 REF_OFFSET_POSITION */
      if (TMC1->R.RESERVE_19 == 0 || TMC1->W.RESERVE_19 == 0) {
        EEP_WriteVariable(0, TMC1->W.RESERVE_19);
        TMC1->M.RESERVE_19 = TMC1->W.RESERVE_19;
        TMC1->R.RESERVE_19 = TMC1->W.RESERVE_19;
      }
    } /* #1 REF_OFFSET_POSITION */

    if (TMC2->W.RESERVE_19 != TMC2->M.RESERVE_19) { /* #2 REF_OFFSET_POSITION */
      if (TMC2->R.RESERVE_19 == 0 || TMC2->W.RESERVE_19 == 0) {
        EEP_WriteVariable(1, TMC2->W.RESERVE_19);
        TMC2->M.RESERVE_19 = TMC2->W.RESERVE_19;
        TMC2->R.RESERVE_19 = TMC2->W.RESERVE_19;
      }
    } /* #2 REF_OFFSET_POSITION */

    if (TMC3->W.RESERVE_19 != TMC3->M.RESERVE_19) { /* #3 REF_OFFSET_POSITION */
      if (TMC3->R.RESERVE_19 == 0 || TMC3->W.RESERVE_19 == 0) {
        EEP_WriteVariable(2, TMC3->W.RESERVE_19);
        TMC3->M.RESERVE_19 = TMC3->W.RESERVE_19;
        TMC3->R.RESERVE_19 = TMC3->W.RESERVE_19;
      }
    } /* #3 REF_OFFSET_POSITION */

    if (TMC4->W.RESERVE_19 != TMC4->M.RESERVE_19) { /* #4 REF_OFFSET_POSITION */
      if (TMC4->R.RESERVE_19 == 0 || TMC4->W.RESERVE_19 == 0) {
        EEP_WriteVariable(3, TMC4->W.RESERVE_19);
        TMC4->M.RESERVE_19 = TMC4->W.RESERVE_19;
        TMC4->R.RESERVE_19 = TMC4->W.RESERVE_19;
      }
    } /* #4 REF_OFFSET_POSITION */

    if (TMC5->W.RESERVE_19 != TMC5->M.RESERVE_19) { /* #5 REF_OFFSET_POSITION */
      if (TMC5->R.RESERVE_19 == 0 || TMC5->W.RESERVE_19 == 0) {
        EEP_WriteVariable(4, TMC5->W.RESERVE_19);
        TMC5->M.RESERVE_19 = TMC5->W.RESERVE_19;
        TMC5->R.RESERVE_19 = TMC5->W.RESERVE_19;
      }
    } /* #5 REF_OFFSET_POSITION */

    if (TMC6->W.RESERVE_19 != TMC6->M.RESERVE_19) { /* #6 REF_OFFSET_POSITION */
      if (TMC6->R.RESERVE_19 == 0 || TMC6->W.RESERVE_19 == 0) {
        EEP_WriteVariable(5, TMC6->W.RESERVE_19);
        TMC6->M.RESERVE_19 = TMC6->W.RESERVE_19;
        TMC6->R.RESERVE_19 = TMC6->W.RESERVE_19;
      }
    } /* #6 REF_OFFSET_POSITION */

    /* LED flashes every half second */
    LED_Toggle(LED1);
    VERIFY(osDelay(500) == osOK);

    /* Pass control to the next thread */
    VERIFY(osThreadYield() == osOK);
  }

  /* Never happen */
  ErrorHandler(__FILE__, __LINE__);
}
