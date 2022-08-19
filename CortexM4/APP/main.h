/**************************************************************************************************************
 File  :
 Author:
 Date  :
 Brief :
**************************************************************************************************************/

/* Define to prevent recursive inclusion *********************************************************************/
#ifndef __MAIN_H__
#define __MAIN_H__

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif

/* Standard include file *************************************************************************************/
#include <STDBOOL.h>
#include <ASSERT.h>
#include <STDLIB.h>
#include <STDIO.h>
#include <STRING.h>
#include <LIMITS.h>

/* STMicroelectronics include file ***************************************************************************/
#include "STM32H7XX_HAL.h"

/* Keil MDK compiler include file ****************************************************************************/
#include "CMSIS_OS2.h"
#include "RTX_OS.h"

#include "RL_NET.h"
#include "RL_NET_LIB.h"

/* Version definition ****************************************************************************************/
#define FW_VERSION MAKELONG(MAKEWORD(0, 0), MAKEWORD(0, 0))

/* Base address of the Flash sectors Bank 1 ******************************************************************/
#define ADDR_FLASH_SECTOR_0_BANK1 ((uint32_t) 0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1 ((uint32_t) 0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1 ((uint32_t) 0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1 ((uint32_t) 0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1 ((uint32_t) 0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1 ((uint32_t) 0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1 ((uint32_t) 0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1 ((uint32_t) 0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 ******************************************************************/
#define ADDR_FLASH_SECTOR_0_BANK2 ((uint32_t) 0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2 ((uint32_t) 0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2 ((uint32_t) 0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2 ((uint32_t) 0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2 ((uint32_t) 0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2 ((uint32_t) 0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2 ((uint32_t) 0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2 ((uint32_t) 0x081E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Type definition data **************************************************************************************/
typedef signed   char  CHAR;
typedef unsigned char  BYTE;

typedef signed   short SHRT;
typedef unsigned short WORD;

typedef signed   long  LONG;
typedef unsigned long  DWORD;

typedef   signed long  S32;
typedef unsigned long  U32;

typedef   signed char  S08;
typedef unsigned char  U08;

typedef   signed short S16;
typedef unsigned short U16;

/* Local machine setup control structure *********************************************************************/
#pragma pack(1)
typedef struct LocalMS {
  uint8_t  ETH_Address[NET_ADDR_ETH_LEN];
  uint8_t  Reserved1[2];

  uint8_t  IP_Address[NET_ADDR_IP4_LEN];
  uint8_t  SubnetMask[NET_ADDR_IP4_LEN];
  uint8_t  DefGateWay[NET_ADDR_IP4_LEN];
  uint8_t  PrimaryDNS[NET_ADDR_IP4_LEN];
  uint8_t  SecondaryDNS[NET_ADDR_IP4_LEN];

  uint8_t  DHCP_Enable;
  uint8_t  Reserved2[3];

  uint8_t  DeviceName[NET_HOSTNAME_LEN];
  uint32_t DeviceID;

  uint8_t  TCP_HostIP[NET_ADDR_IP4_LEN];
  uint16_t TCP_HostPort;
  uint8_t  Reserved3[2];
  uint8_t  Reserved4[4];
} LOCALMS;
#pragma pack()

/* Local machine information control structure ***************************************************************/
#pragma pack(1)
typedef struct LocalMI {
  uint8_t  ProductInfo[NET_HOSTNAME_LEN];
  uint8_t  ETH_Address[NET_ADDR_ETH_LEN];

  uint8_t  ProductName[NET_HOSTNAME_LEN];
  uint32_t FW_Version;

  uint8_t  RTC_Date[3]; // YY/MM/DD
  uint8_t  RTC_Time[3]; // HH:MM:SS

  uint8_t  Reserved[16];
  LOCALMS  LocalMS;
} LOCALMI;
#pragma pack()

/* Enable or disable the GPIO peripheral clock ***************************************************************/
#define __HAL_RCC_GPIO_CLK_ENABLE(GPIOx)                         \
  do {                                                           \
    if ((GPIOx) == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); } else \
    if ((GPIOx) == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); } else \
    if ((GPIOx) == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); } else \
    if ((GPIOx) == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); } else \
    if ((GPIOx) == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); } else \
    if ((GPIOx) == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); } else \
    if ((GPIOx) == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); } else \
    if ((GPIOx) == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }      \
    if ((GPIOx) == GPIOI) { __HAL_RCC_GPIOI_CLK_ENABLE(); }      \
  } while (0U)

#define __HAL_RCC_GPIO_CLK_DISABLE(GPIOx)                         \
  do {                                                            \
    if ((GPIOx) == GPIOA) { __HAL_RCC_GPIOA_CLK_DISABLE(); } else \
    if ((GPIOx) == GPIOB) { __HAL_RCC_GPIOB_CLK_DISABLE(); } else \
    if ((GPIOx) == GPIOC) { __HAL_RCC_GPIOC_CLK_DISABLE(); } else \
    if ((GPIOx) == GPIOD) { __HAL_RCC_GPIOD_CLK_DISABLE(); } else \
    if ((GPIOx) == GPIOE) { __HAL_RCC_GPIOE_CLK_DISABLE(); } else \
    if ((GPIOx) == GPIOF) { __HAL_RCC_GPIOF_CLK_DISABLE(); } else \
    if ((GPIOx) == GPIOG) { __HAL_RCC_GPIOG_CLK_DISABLE(); } else \
    if ((GPIOx) == GPIOH) { __HAL_RCC_GPIOH_CLK_DISABLE(); }      \
    if ((GPIOx) == GPIOI) { __HAL_RCC_GPIOI_CLK_DISABLE(); }      \
  } while (0U)

/* Support for absolute placement of variables ***************************************************************/
#ifndef __MEMORY_AT
  /* Macro __MEMORY_AT is used to place ZI data at specific address. */
  #if defined(__CC_ARM)
    #define __MEMORY_AT__(x) __attribute__((section(".ARM.__AT_"#x)))
  #elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6060100)
    #define __MEMORY_AT__(x) __attribute__((section(".bss.ARM.__at_"#x)))
  #else
    /* Define __MEMORY_AT to create sections used by the linker script */
    #error "Objects defined with __MEMORY_AT(x) should be placed at specified address with scatter-loading."
  #endif
  #define __MEMORY_AT(x) __MEMORY_AT__(x)
#endif

/* Supports absolute specification of variables **************************************************************/
#define REG8(x)  (*((volatile unsigned char  *)(x)))
#define REG16(x) (*((volatile unsigned short *)(x)))
#define REG32(x) (*((volatile unsigned long  *)(x)))

/* Retrieves the low or high order byte from the specified value *********************************************/
#define LOBYTE(w) ((BYTE)((((WORD)(w)) >> 0x00) & 0xFF))
#define HIBYTE(w) ((BYTE)((((WORD)(w)) >> 0x08) & 0xFF))

/* Retrieves the low or high order word from the specified value *********************************************/
#define LOWORD(l) ((WORD)((((DWORD)(l)) >> 0x00) & 0xFFFF))
#define HIWORD(l) ((WORD)((((DWORD)(l)) >> 0x10) & 0xFFFF))

/* Creates the WORD or LONG value by concatenating the specified values **************************************/
#define MAKEWORD(a, b) ((U16)(((U08)(((U32)(a)) & 0x00FF)) | ((U16)((U08)(((U32)(b)) & 0x00FF))) << 0x08))
#define MAKELONG(a, b) ((S32)(((U16)(((U32)(a)) & 0xFFFF)) | ((U32)((U16)(((U32)(b)) & 0xFFFF))) << 0x10))

/* Count the number of elements in the static configuration array ********************************************/
#define countof(array) (sizeof(array) / sizeof(array[0]))

/* Return the largest or smallest value **********************************************************************/
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))

/* Trace and verify and assert functions during debugging ****************************************************/
#define ASSERT(f) assert(f)

#ifdef NDEBUG
  #define TRACE(format, ...) ((void) NULL)
  #define VERIFY(f) ((void)(f))
#else
  #define TRACE(format, ...) printf(format, ##__VA_ARGS__)
  #define VERIFY(f) ASSERT(f)
#endif

#ifdef USE_FULL_ASSERT
  extern void assert_failed(uint8_t *file, uint32_t line);
#endif

#ifndef NDEBUG
  extern void __aeabi_assert(const char *expr, const char *file, int line);
#endif

/* This function is executed in case of error occurrence *****************************************************/
extern void ErrorHandler(const char *file, int line);

/* Override the default functions ****************************************************************************/
extern void              HAL_Delay  (uint32_t Delay);
extern uint32_t          HAL_GetTick(void);
extern HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);

/* Exported functions ****************************************************************************************/
ErrorStatus LMS_Load(struct LocalMS *pLocalMS);
ErrorStatus LMS_Save(struct LocalMS *pLocalMS);

/* Exported variables ****************************************************************************************/
extern struct LocalMS LocalMS;

/* Variables *************************************************************************************************/
extern int32_t TMC_Registers[2][6][256];

/* Constants *************************************************************************************************/
#define TCP_EVENT_FLAG_CONNECT     0x00000100U
#define TCP_EVENT_FLAG_ESTABLISHED 0x00000200U
#define TCP_EVENT_FLAG_CLOSED      0x00000400U
#define TCP_EVENT_FLAG_ABORTED     0x00000800U
#define TCP_EVENT_FLAG_ACK         0x00001000U
#define TCP_EVENT_FLAG_DATA        0x00002000U

#define UART1_RECEIVE_COMPLETED    0x00004000U
#define UART1_TRANSMIT_COMPLETED   0x00008000U
#define UART2_RECEIVE_COMPLETED    0x00010000U
#define UART2_TRANSMIT_COMPLETED   0x00020000U
#define UART3_RECEIVE_COMPLETED    0x00040000U
#define UART3_TRANSMIT_COMPLETED   0x00080000U
#define UART4_RECEIVE_COMPLETED    0x00100000U
#define UART4_TRANSMIT_COMPLETED   0x00200000U
#define UART5_RECEIVE_COMPLETED    0x00400000U
#define UART5_TRANSMIT_COMPLETED   0x00800000U
#define UART6_RECEIVE_COMPLETED    0x01000000U
#define UART6_TRANSMIT_COMPLETED   0x02000000U
#define UART7_RECEIVE_COMPLETED    0x04000000U
#define UART7_TRANSMIT_COMPLETED   0x08000000U
#define UART8_RECEIVE_COMPLETED    0x10000000U
#define UART8_TRANSMIT_COMPLETED   0x20000000U

/* Constants *************************************************************************************************/
#define TCP_EVENT_FLAG_MASK (TCP_EVENT_FLAG_CONNECT     | \
                             TCP_EVENT_FLAG_ESTABLISHED | \
                             TCP_EVENT_FLAG_CLOSED      | \
                             TCP_EVENT_FLAG_ABORTED     | \
                             TCP_EVENT_FLAG_ACK         | \
                             TCP_EVENT_FLAG_DATA)

#define UART1_TRANSFER_COMPLETED (UART1_RECEIVE_COMPLETED | UART1_TRANSMIT_COMPLETED)
#define UART2_TRANSFER_COMPLETED (UART2_RECEIVE_COMPLETED | UART2_TRANSMIT_COMPLETED)
#define UART3_TRANSFER_COMPLETED (UART3_RECEIVE_COMPLETED | UART3_TRANSMIT_COMPLETED)
#define UART4_TRANSFER_COMPLETED (UART4_RECEIVE_COMPLETED | UART4_TRANSMIT_COMPLETED)
#define UART5_TRANSFER_COMPLETED (UART5_RECEIVE_COMPLETED | UART5_TRANSMIT_COMPLETED)
#define UART6_TRANSFER_COMPLETED (UART6_RECEIVE_COMPLETED | UART6_TRANSMIT_COMPLETED)
#define UART7_TRANSFER_COMPLETED (UART7_RECEIVE_COMPLETED | UART7_TRANSMIT_COMPLETED)
#define UART8_TRANSFER_COMPLETED (UART8_RECEIVE_COMPLETED | UART8_TRANSMIT_COMPLETED)

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  }
#endif

/* __MAIN_H__ ************************************************************************************************/
#endif
