
/*
 * Auto generated Run-Time-Environment Configuration File
 *      *** Do not modify ! ***
 *
 * Project: 'CortexM4' 
 * Target:  'Release' 
 */

#ifndef RTE_COMPONENTS_H
#define RTE_COMPONENTS_H


/*
 * Define the Device Header File: 
 */
#define CMSIS_device_header "stm32h7xx.h"

/* ARM::CMSIS:RTOS2:Keil RTX5:Library:5.5.3 */
#define RTE_CMSIS_RTOS2                 /* CMSIS-RTOS2 */
        #define RTE_CMSIS_RTOS2_RTX5            /* CMSIS-RTOS2 Keil RTX5 */
/* Keil.ARM Compiler::Compiler:Event Recorder:DAP:1.5.0 */
#define RTE_Compiler_EventRecorder
          #define RTE_Compiler_EventRecorder_DAP
/* Keil.ARM Compiler::Compiler:I/O:STDOUT:EVR:1.2.0 */
#define RTE_Compiler_IO_STDOUT          /* Compiler I/O: STDOUT */
          #define RTE_Compiler_IO_STDOUT_EVR      /* Compiler I/O: STDOUT EVR */
/* Keil.MDK-Plus::Network:CORE:IPv4 Release:7.16.0 */
#define RTE_Network_Core                /* Network Core */
          #define RTE_Network_IPv4                /* Network IPv4 Stack */
          #define RTE_Network_Release             /* Network Release Version */
/* Keil.MDK-Plus::Network:Interface:ETH:7.16.0 */
#define RTE_Network_Interface_ETH_0     /* Network Interface ETH 0 */

/* Keil.MDK-Plus::Network:Socket:TCP:7.16.0 */
#define RTE_Network_Socket_TCP          /* Network Socket TCP */
/* Keil.MDK-Plus::Network:Socket:UDP:7.16.0 */
#define RTE_Network_Socket_UDP          /* Network Socket UDP */
/* Keil::CMSIS Driver:Ethernet PHY:LAN8742A:1.3.0 */
#define RTE_Drivers_PHY_LAN8742A        /* Driver PHY LAN8742A */
/* Keil::Device:STM32Cube HAL:ADC:1.9.0 */
#define RTE_DEVICE_HAL_ADC
/* Keil::Device:STM32Cube HAL:Common:1.9.0 */
#define RTE_DEVICE_HAL_COMMON
/* Keil::Device:STM32Cube HAL:Cortex:1.9.0 */
#define RTE_DEVICE_HAL_CORTEX
/* Keil::Device:STM32Cube HAL:DMA:1.9.0 */
#define RTE_DEVICE_HAL_DMA
/* Keil::Device:STM32Cube HAL:ETH:1.9.0 */
#define RTE_DEVICE_HAL_ETH
/* Keil::Device:STM32Cube HAL:Flash:1.9.0 */
#define RTE_DEVICE_HAL_FLASH
/* Keil::Device:STM32Cube HAL:GPIO:1.9.0 */
#define RTE_DEVICE_HAL_GPIO
/* Keil::Device:STM32Cube HAL:HSEM:1.9.0 */
#define RTE_DEVICE_HAL_HSEM
/* Keil::Device:STM32Cube HAL:PWR:1.9.0 */
#define RTE_DEVICE_HAL_PWR
/* Keil::Device:STM32Cube HAL:RCC:1.9.0 */
#define RTE_DEVICE_HAL_RCC
/* Keil::Device:STM32Cube HAL:TIM:1.9.0 */
#define RTE_DEVICE_HAL_TIM
/* Keil::Device:STM32Cube HAL:UART:1.9.0 */
#define RTE_DEVICE_HAL_UART
/* Keil::Device:Startup:1.9.0 */
#define RTE_DEVICE_STARTUP_STM32H7XX    /* Device Startup for STM32H7 */


#endif /* RTE_COMPONENTS_H */
