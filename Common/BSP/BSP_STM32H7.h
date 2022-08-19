/**************************************************************************************************************
 File   :
 Author :
 Date   :
 Brief  :
**************************************************************************************************************/

/* STM32H745/STM32H755/STM32H747/STM32H757 Memory map and default device memory area attributes ***************
ITCM        0x00000000 - 0x0000FFFF 400Mhz  64KB
SRAM1       0x10000000 - 0x1001FFFF 200Mhz 128KB
SRAM2       0x10020000 - 0x1003FFFF 200Mhz 128KB
SRAM3       0x10040000 - 0x10047FFF 200Mhz  32KB
DTCM        0x20000000 - 0x2001FFFF 400Mhz 128KB
AXI SRAM    0x24000000 - 0x2407FFFF 200Mhz 512KB
SRAM1       0x30000000 - 0x3001FFFF 200Mhz 128KB
SRAM2       0x30020000 - 0x3003FFFF 200Mhz 128KB
SRAM3       0x30040000 - 0x30047FFF 200Mhz  32KB
SRAM4       0x38000000 - 0x3800FFFF 200Mhz  64KB
Backup SRAM 0x38800000 - 0x38800FFF 200Mhz   4KB
**************************************************************************************************************/

/* Define to prevent recursive inclusion *********************************************************************/
#ifndef __BSP_STM32H7_H__
#define __BSP_STM32H7_H__

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes **************************************************************************************************/
#include "MAIN.h"
#include "FIFO.h"

/* Constants *************************************************************************************************/
#define HSEM_ID_0 (0U) /* HW Semaphore 0, six axis position writing */
#define HSEM_ID_1 (1U) /* HW Semaphore 1, TMC-4671 register writing */
#define HSEM_ID_2 (2U) /* HW Semaphore 2 */
#define HSEM_ID_3 (3U) /* HW Semaphore 3 */
#define HSEM_ID_4 (4U) /* HW Semaphore 4 */
#define HSEM_ID_5 (5U) /* HW Semaphore 5 */
#define HSEM_ID_6 (6U) /* HW Semaphore 6 */
#define HSEM_ID_7 (7U) /* HW Semaphore 7 */

/* Typedef ***************************************************************************************************/
typedef union {
  int32_t Registers[1024];

  struct {
    int32_t RESERVE_00[256]                   ; /* FREE */
    int32_t CHIPINFO_DATA                     ; /* 0x00 */
    int32_t CHIPINFO_ADDR                     ; /* 0x01 */
    int32_t ADC_RAW_DATA                      ; /* 0x02 */
    int32_t ADC_RAW_ADDR                      ; /* 0x03 */
    int32_t dsADC_MCFG_B_MCFG_A               ; /* 0x04 */
    int32_t dsADC_MCLK_A                      ; /* 0x05 */
    int32_t dsADC_MCLK_B                      ; /* 0x06 */
    int32_t dsADC_MDEC_B_MDEC_A               ; /* 0x07 */
    int32_t ADC_I1_SCALE_OFFSET               ; /* 0x08 */
    int32_t ADC_I0_SCALE_OFFSET               ; /* 0x09 */
    int32_t ADC_I_SELECT                      ; /* 0x0A */
    int32_t ADC_I1_I0_EXT                     ; /* 0x0B */
    int32_t DS_ANALOG_INPUT_STAGE_CFG         ; /* 0x0C */
    int32_t AENC_0_SCALE_OFFSET               ; /* 0x0D */
    int32_t AENC_1_SCALE_OFFSET               ; /* 0x0E */
    int32_t AENC_2_SCALE_OFFSET               ; /* 0x0F */
    int32_t RESERVE_01                        ; /* 0x10 */
    int32_t AENC_SELECT                       ; /* 0x11 */
    int32_t ADC_IWY_IUX                       ; /* 0x12 */
    int32_t ADC_IV                            ; /* 0x13 */
    int32_t RESERVE_02                        ; /* 0x14 */
    int32_t AENC_WY_UX                        ; /* 0x15 */
    int32_t AENC_VN                           ; /* 0x16 */
    int32_t PWM_POLARITIES                    ; /* 0x17 */
    int32_t PWM_MAXCNT                        ; /* 0x18 */
    int32_t PWM_BBM_H_BBM_L                   ; /* 0x19 */
    int32_t PWM_SV_CHOP                       ; /* 0x1A */
    int32_t MOTOR_TYPE_N_POLE_PAIRS           ; /* 0x1B */
    int32_t PHI_E_EXT                         ; /* 0x1C */
    int32_t PHI_M_EXT                         ; /* 0x1D */
    int32_t POSITION_EXT                      ; /* 0x1E */
    int32_t OPENLOOP_MODE                     ; /* 0x1F */
    int32_t OPENLOOP_ACCELERATION             ; /* 0x20 */
    int32_t OPENLOOP_VELOCITY_TARGET          ; /* 0x21 */
    int32_t OPENLOOP_VELOCITY_ACTUAL          ; /* 0x22 */
    int32_t OPENLOOP_PHI                      ; /* 0x23 */
    int32_t UQ_UD_EXT                         ; /* 0x24 */
    int32_t ABN_DECODER_MODE                  ; /* 0x25 */
    int32_t ABN_DECODER_PPR                   ; /* 0x26 */
    int32_t ABN_DECODER_COUNT                 ; /* 0x27 */
    int32_t ABN_DECODER_COUNT_N               ; /* 0x28 */
    int32_t ABN_DECODER_PHI_E_PHI_M_OFFSET    ; /* 0x29 */
    int32_t ABN_DECODER_PHI_E_PHI_M           ; /* 0x2A */
    int32_t RESERVE_03                        ; /* 0x2B */
    int32_t ABN_2_DECODER_MODE                ; /* 0x2C */
    int32_t ABN_2_DECODER_PPR                 ; /* 0x2D */
    int32_t ABN_2_DECODER_COUNT               ; /* 0x2E */
    int32_t ABN_2_DECODER_COUNT_N             ; /* 0x2F */
    int32_t ABN_2_DECODER_PHI_M_OFFSET        ; /* 0x30 */
    int32_t ABN_2_DECODER_PHI_M               ; /* 0x31 */
    int32_t HALL_MODE                         ; /* 0x33 */
    int32_t RESERVE_04                        ; /* 0x32 */
    int32_t HALL_POSITION_060_000             ; /* 0x34 */
    int32_t HALL_POSITION_180_120             ; /* 0x35 */
    int32_t HALL_POSITION_300_240             ; /* 0x36 */
    int32_t HALL_PHI_E_PHI_M_OFFSET           ; /* 0x37 */
    int32_t HALL_DPHI_MAX                     ; /* 0x38 */
    int32_t HALL_PHI_E_INTERPOLATED_PHI_E     ; /* 0x39 */
    int32_t HALL_PHI_M                        ; /* 0x3A */
    int32_t AENC_DECODER_MODE                 ; /* 0x3B */
    int32_t AENC_DECODER_N_MASK_N_THRESHOLD   ; /* 0x3C */
    int32_t AENC_DECODER_PHI_A_RAW            ; /* 0x3D */
    int32_t AENC_DECODER_PHI_A_OFFSET         ; /* 0x3E */
    int32_t AENC_DECODER_PHI_A                ; /* 0x3F */
    int32_t AENC_DECODER_PPR                  ; /* 0x40 */
    int32_t AENC_DECODER_COUNT                ; /* 0x41 */
    int32_t AENC_DECODER_COUNT_N              ; /* 0x42 */
    int32_t RESERVE_05                        ; /* 0x43 */
    int32_t RESERVE_06                        ; /* 0x44 */
    int32_t AENC_DECODER_PHI_E_PHI_M_OFFSET   ; /* 0x45 */
    int32_t AENC_DECODER_PHI_E_PHI_M          ; /* 0x46 */
    int32_t AENC_DECODER_POSITION             ; /* 0x47 */
    int32_t RESERVE_07                        ; /* 0x48 */
    int32_t RESERVE_08                        ; /* 0x49 */
    int32_t RESERVE_09                        ; /* 0x4A */
    int32_t RESERVE_10                        ; /* 0x4B */
    int32_t RESERVE_11                        ; /* 0x4C */
    int32_t CONFIG_DATA                       ; /* 0x4D */
    int32_t CONFIG_ADDR                       ; /* 0x4E */
    int32_t RESERVE_12                        ; /* 0x4F */
    int32_t VELOCITY_SELECTION                ; /* 0x50 */
    int32_t POSITION_SELECTION                ; /* 0x51 */
    int32_t PHI_E_SELECTION                   ; /* 0x52 */
    int32_t PHI_E                             ; /* 0x53 */
    int32_t PID_FLUX_P_FLUX_I                 ; /* 0x54 */
    int32_t RESERVE_13                        ; /* 0x55 */
    int32_t PID_TORQUE_P_TORQUE_I             ; /* 0x56 */
    int32_t RESERVE_14                        ; /* 0x57 */
    int32_t PID_VELOCITY_P_VELOCITY_I         ; /* 0x58 */
    int32_t RESERVE_15                        ; /* 0x59 */
    int32_t PID_POSITION_P_POSITION_I         ; /* 0x5A */
    int32_t RESERVE_16                        ; /* 0x5B */
    int32_t PID_TORQUE_FLUX_TARGET_DDT_LIMITS ; /* 0x5C */
    int32_t PIDOUT_UQ_UD_LIMITS               ; /* 0x5D */
    int32_t PID_TORQUE_FLUX_LIMITS            ; /* 0x5E */
    int32_t PID_ACCELERATION_LIMIT            ; /* 0x5F */
    int32_t PID_VELOCITY_LIMIT                ; /* 0x60 */
    int32_t PID_POSITION_LIMIT_LOW            ; /* 0x61 */
    int32_t PID_POSITION_LIMIT_HIGH           ; /* 0x62 */
    int32_t MODE_RAMP_MODE_MOTION             ; /* 0x63 */
    int32_t PID_TORQUE_FLUX_TARGET            ; /* 0x64 */
    int32_t PID_TORQUE_FLUX_OFFSET            ; /* 0x65 */
    int32_t PID_VELOCITY_TARGET               ; /* 0x66 */
    int32_t PID_VELOCITY_OFFSET               ; /* 0x67 */
    int32_t PID_POSITION_TARGET               ; /* 0x68 */
    int32_t PID_TORQUE_FLUX_ACTUAL            ; /* 0x69 */
    int32_t PID_VELOCITY_ACTUAL               ; /* 0x6A */
    int32_t PID_POSITION_ACTUAL               ; /* 0x6B */
    int32_t PID_ERROR_DATA                    ; /* 0x6C */
    int32_t PID_ERROR_ADDR                    ; /* 0x6D */
    int32_t INTERIM_DATA                      ; /* 0x6E */
    int32_t INTERIM_ADDR                      ; /* 0x6F */
    int32_t RESERVE_17                        ; /* 0x70 */
    int32_t RESERVE_18                        ; /* 0x71 */
    int32_t RESERVE_19                        ; /* 0x72 */
    int32_t RESERVE_20                        ; /* 0x73 */
    int32_t WATCHDOG_CFG                      ; /* 0x74 */
    int32_t ADC_VM_LIMITS                     ; /* 0x75 */
    int32_t INPUTS_RAW                        ; /* 0x76 */
    int32_t OUTPUTS_RAW                       ; /* 0x77 */
    int32_t STEP_WIDTH                        ; /* 0x78 */
    int32_t UART_BPS                          ; /* 0x79 */
    int32_t UART_ADDRS                        ; /* 0x7A */
    int32_t GPIO_dsADCI_CONFIG                ; /* 0x7B */
    int32_t STATUS_FLAGS                      ; /* 0x7C */
    int32_t STATUS_MASK                       ; /* 0x7D */
    int32_t RESERVE_21                        ; /* 0x7E */
    int32_t RESERVE_22                        ; /* 0x7F */
  } M; /* Middle Values */

  struct {
    int32_t RESERVE_0[128]                    ; /* FREE */
    int32_t CHIPINFO_DATA                     ; /* 0x00 */
    int32_t CHIPINFO_ADDR                     ; /* 0x01 */
    int32_t ADC_RAW_DATA                      ; /* 0x02 */
    int32_t ADC_RAW_ADDR                      ; /* 0x03 */
    int32_t dsADC_MCFG_B_MCFG_A               ; /* 0x04 */
    int32_t dsADC_MCLK_A                      ; /* 0x05 */
    int32_t dsADC_MCLK_B                      ; /* 0x06 */
    int32_t dsADC_MDEC_B_MDEC_A               ; /* 0x07 */
    int32_t ADC_I1_SCALE_OFFSET               ; /* 0x08 */
    int32_t ADC_I0_SCALE_OFFSET               ; /* 0x09 */
    int32_t ADC_I_SELECT                      ; /* 0x0A */
    int32_t ADC_I1_I0_EXT                     ; /* 0x0B */
    int32_t DS_ANALOG_INPUT_STAGE_CFG         ; /* 0x0C */
    int32_t AENC_0_SCALE_OFFSET               ; /* 0x0D */
    int32_t AENC_1_SCALE_OFFSET               ; /* 0x0E */
    int32_t AENC_2_SCALE_OFFSET               ; /* 0x0F */
    int32_t RESERVE_01                        ; /* 0x10 */
    int32_t AENC_SELECT                       ; /* 0x11 */
    int32_t ADC_IWY_IUX                       ; /* 0x12 */
    int32_t ADC_IV                            ; /* 0x13 */
    int32_t RESERVE_02                        ; /* 0x14 */
    int32_t AENC_WY_UX                        ; /* 0x15 */
    int32_t AENC_VN                           ; /* 0x16 */
    int32_t PWM_POLARITIES                    ; /* 0x17 */
    int32_t PWM_MAXCNT                        ; /* 0x18 */
    int32_t PWM_BBM_H_BBM_L                   ; /* 0x19 */
    int32_t PWM_SV_CHOP                       ; /* 0x1A */
    int32_t MOTOR_TYPE_N_POLE_PAIRS           ; /* 0x1B */
    int32_t PHI_E_EXT                         ; /* 0x1C */
    int32_t PHI_M_EXT                         ; /* 0x1D */
    int32_t POSITION_EXT                      ; /* 0x1E */
    int32_t OPENLOOP_MODE                     ; /* 0x1F */
    int32_t OPENLOOP_ACCELERATION             ; /* 0x20 */
    int32_t OPENLOOP_VELOCITY_TARGET          ; /* 0x21 */
    int32_t OPENLOOP_VELOCITY_ACTUAL          ; /* 0x22 */
    int32_t OPENLOOP_PHI                      ; /* 0x23 */
    int32_t UQ_UD_EXT                         ; /* 0x24 */
    int32_t ABN_DECODER_MODE                  ; /* 0x25 */
    int32_t ABN_DECODER_PPR                   ; /* 0x26 */
    int32_t ABN_DECODER_COUNT                 ; /* 0x27 */
    int32_t ABN_DECODER_COUNT_N               ; /* 0x28 */
    int32_t ABN_DECODER_PHI_E_PHI_M_OFFSET    ; /* 0x29 */
    int32_t ABN_DECODER_PHI_E_PHI_M           ; /* 0x2A */
    int32_t RESERVE_03                        ; /* 0x2B */
    int32_t ABN_2_DECODER_MODE                ; /* 0x2C */
    int32_t ABN_2_DECODER_PPR                 ; /* 0x2D */
    int32_t ABN_2_DECODER_COUNT               ; /* 0x2E */
    int32_t ABN_2_DECODER_COUNT_N             ; /* 0x2F */
    int32_t ABN_2_DECODER_PHI_M_OFFSET        ; /* 0x30 */
    int32_t ABN_2_DECODER_PHI_M               ; /* 0x31 */
    int32_t HALL_MODE                         ; /* 0x33 */
    int32_t RESERVE_04                        ; /* 0x32 */
    int32_t HALL_POSITION_060_000             ; /* 0x34 */
    int32_t HALL_POSITION_180_120             ; /* 0x35 */
    int32_t HALL_POSITION_300_240             ; /* 0x36 */
    int32_t HALL_PHI_E_PHI_M_OFFSET           ; /* 0x37 */
    int32_t HALL_DPHI_MAX                     ; /* 0x38 */
    int32_t HALL_PHI_E_INTERPOLATED_PHI_E     ; /* 0x39 */
    int32_t HALL_PHI_M                        ; /* 0x3A */
    int32_t AENC_DECODER_MODE                 ; /* 0x3B */
    int32_t AENC_DECODER_N_MASK_N_THRESHOLD   ; /* 0x3C */
    int32_t AENC_DECODER_PHI_A_RAW            ; /* 0x3D */
    int32_t AENC_DECODER_PHI_A_OFFSET         ; /* 0x3E */
    int32_t AENC_DECODER_PHI_A                ; /* 0x3F */
    int32_t AENC_DECODER_PPR                  ; /* 0x40 */
    int32_t AENC_DECODER_COUNT                ; /* 0x41 */
    int32_t AENC_DECODER_COUNT_N              ; /* 0x42 */
    int32_t RESERVE_05                        ; /* 0x43 */
    int32_t RESERVE_06                        ; /* 0x44 */
    int32_t AENC_DECODER_PHI_E_PHI_M_OFFSET   ; /* 0x45 */
    int32_t AENC_DECODER_PHI_E_PHI_M          ; /* 0x46 */
    int32_t AENC_DECODER_POSITION             ; /* 0x47 */
    int32_t RESERVE_07                        ; /* 0x48 */
    int32_t RESERVE_08                        ; /* 0x49 */
    int32_t RESERVE_09                        ; /* 0x4A */
    int32_t RESERVE_10                        ; /* 0x4B */
    int32_t RESERVE_11                        ; /* 0x4C */
    int32_t CONFIG_DATA                       ; /* 0x4D */
    int32_t CONFIG_ADDR                       ; /* 0x4E */
    int32_t RESERVE_12                        ; /* 0x4F */
    int32_t VELOCITY_SELECTION                ; /* 0x50 */
    int32_t POSITION_SELECTION                ; /* 0x51 */
    int32_t PHI_E_SELECTION                   ; /* 0x52 */
    int32_t PHI_E                             ; /* 0x53 */
    int32_t PID_FLUX_P_FLUX_I                 ; /* 0x54 */
    int32_t RESERVE_13                        ; /* 0x55 */
    int32_t PID_TORQUE_P_TORQUE_I             ; /* 0x56 */
    int32_t RESERVE_14                        ; /* 0x57 */
    int32_t PID_VELOCITY_P_VELOCITY_I         ; /* 0x58 */
    int32_t RESERVE_15                        ; /* 0x59 */
    int32_t PID_POSITION_P_POSITION_I         ; /* 0x5A */
    int32_t RESERVE_16                        ; /* 0x5B */
    int32_t PID_TORQUE_FLUX_TARGET_DDT_LIMITS ; /* 0x5C */
    int32_t PIDOUT_UQ_UD_LIMITS               ; /* 0x5D */
    int32_t PID_TORQUE_FLUX_LIMITS            ; /* 0x5E */
    int32_t PID_ACCELERATION_LIMIT            ; /* 0x5F */
    int32_t PID_VELOCITY_LIMIT                ; /* 0x60 */
    int32_t PID_POSITION_LIMIT_LOW            ; /* 0x61 */
    int32_t PID_POSITION_LIMIT_HIGH           ; /* 0x62 */
    int32_t MODE_RAMP_MODE_MOTION             ; /* 0x63 */
    int32_t PID_TORQUE_FLUX_TARGET            ; /* 0x64 */
    int32_t PID_TORQUE_FLUX_OFFSET            ; /* 0x65 */
    int32_t PID_VELOCITY_TARGET               ; /* 0x66 */
    int32_t PID_VELOCITY_OFFSET               ; /* 0x67 */
    int32_t PID_POSITION_TARGET               ; /* 0x68 */
    int32_t PID_TORQUE_FLUX_ACTUAL            ; /* 0x69 */
    int32_t PID_VELOCITY_ACTUAL               ; /* 0x6A */
    int32_t PID_POSITION_ACTUAL               ; /* 0x6B */
    int32_t PID_ERROR_DATA                    ; /* 0x6C */
    int32_t PID_ERROR_ADDR                    ; /* 0x6D */
    int32_t INTERIM_DATA                      ; /* 0x6E */
    int32_t INTERIM_ADDR                      ; /* 0x6F */
    int32_t RESERVE_17                        ; /* 0x70 */
    int32_t RESERVE_18                        ; /* 0x71 */
    int32_t RESERVE_19                        ; /* 0x72 */
    int32_t RESERVE_20                        ; /* 0x73 */
    int32_t WATCHDOG_CFG                      ; /* 0x74 */
    int32_t ADC_VM_LIMITS                     ; /* 0x75 */
    int32_t INPUTS_RAW                        ; /* 0x76 */
    int32_t OUTPUTS_RAW                       ; /* 0x77 */
    int32_t STEP_WIDTH                        ; /* 0x78 */
    int32_t UART_BPS                          ; /* 0x79 */
    int32_t UART_ADDRS                        ; /* 0x7A */
    int32_t GPIO_dsADCI_CONFIG                ; /* 0x7B */
    int32_t STATUS_FLAGS                      ; /* 0x7C */
    int32_t STATUS_MASK                       ; /* 0x7D */
    int32_t RESERVE_21                        ; /* 0x7E */
    int32_t RESERVE_22                        ; /* 0x7F */
  } W; /* Target Values */

  struct {
    int32_t CHIPINFO_DATA                     ; /* 0x00 */
    int32_t CHIPINFO_ADDR                     ; /* 0x01 */
    int32_t ADC_RAW_DATA                      ; /* 0x02 */
    int32_t ADC_RAW_ADDR                      ; /* 0x03 */
    int32_t dsADC_MCFG_B_MCFG_A               ; /* 0x04 */
    int32_t dsADC_MCLK_A                      ; /* 0x05 */
    int32_t dsADC_MCLK_B                      ; /* 0x06 */
    int32_t dsADC_MDEC_B_MDEC_A               ; /* 0x07 */
    int32_t ADC_I1_SCALE_OFFSET               ; /* 0x08 */
    int32_t ADC_I0_SCALE_OFFSET               ; /* 0x09 */
    int32_t ADC_I_SELECT                      ; /* 0x0A */
    int32_t ADC_I1_I0_EXT                     ; /* 0x0B */
    int32_t DS_ANALOG_INPUT_STAGE_CFG         ; /* 0x0C */
    int32_t AENC_0_SCALE_OFFSET               ; /* 0x0D */
    int32_t AENC_1_SCALE_OFFSET               ; /* 0x0E */
    int32_t AENC_2_SCALE_OFFSET               ; /* 0x0F */
    int32_t RESERVE_01                        ; /* 0x10 */
    int32_t AENC_SELECT                       ; /* 0x11 */
    int32_t ADC_IWY_IUX                       ; /* 0x12 */
    int32_t ADC_IV                            ; /* 0x13 */
    int32_t RESERVE_02                        ; /* 0x14 */
    int32_t AENC_WY_UX                        ; /* 0x15 */
    int32_t AENC_VN                           ; /* 0x16 */
    int32_t PWM_POLARITIES                    ; /* 0x17 */
    int32_t PWM_MAXCNT                        ; /* 0x18 */
    int32_t PWM_BBM_H_BBM_L                   ; /* 0x19 */
    int32_t PWM_SV_CHOP                       ; /* 0x1A */
    int32_t MOTOR_TYPE_N_POLE_PAIRS           ; /* 0x1B */
    int32_t PHI_E_EXT                         ; /* 0x1C */
    int32_t PHI_M_EXT                         ; /* 0x1D */
    int32_t POSITION_EXT                      ; /* 0x1E */
    int32_t OPENLOOP_MODE                     ; /* 0x1F */
    int32_t OPENLOOP_ACCELERATION             ; /* 0x20 */
    int32_t OPENLOOP_VELOCITY_TARGET          ; /* 0x21 */
    int32_t OPENLOOP_VELOCITY_ACTUAL          ; /* 0x22 */
    int32_t OPENLOOP_PHI                      ; /* 0x23 */
    int32_t UQ_UD_EXT                         ; /* 0x24 */
    int32_t ABN_DECODER_MODE                  ; /* 0x25 */
    int32_t ABN_DECODER_PPR                   ; /* 0x26 */
    int32_t ABN_DECODER_COUNT                 ; /* 0x27 */
    int32_t ABN_DECODER_COUNT_N               ; /* 0x28 */
    int32_t ABN_DECODER_PHI_E_PHI_M_OFFSET    ; /* 0x29 */
    int32_t ABN_DECODER_PHI_E_PHI_M           ; /* 0x2A */
    int32_t RESERVE_03                        ; /* 0x2B */
    int32_t ABN_2_DECODER_MODE                ; /* 0x2C */
    int32_t ABN_2_DECODER_PPR                 ; /* 0x2D */
    int32_t ABN_2_DECODER_COUNT               ; /* 0x2E */
    int32_t ABN_2_DECODER_COUNT_N             ; /* 0x2F */
    int32_t ABN_2_DECODER_PHI_M_OFFSET        ; /* 0x30 */
    int32_t ABN_2_DECODER_PHI_M               ; /* 0x31 */
    int32_t HALL_MODE                         ; /* 0x33 */
    int32_t RESERVE_04                        ; /* 0x32 */
    int32_t HALL_POSITION_060_000             ; /* 0x34 */
    int32_t HALL_POSITION_180_120             ; /* 0x35 */
    int32_t HALL_POSITION_300_240             ; /* 0x36 */
    int32_t HALL_PHI_E_PHI_M_OFFSET           ; /* 0x37 */
    int32_t HALL_DPHI_MAX                     ; /* 0x38 */
    int32_t HALL_PHI_E_INTERPOLATED_PHI_E     ; /* 0x39 */
    int32_t HALL_PHI_M                        ; /* 0x3A */
    int32_t AENC_DECODER_MODE                 ; /* 0x3B */
    int32_t AENC_DECODER_N_MASK_N_THRESHOLD   ; /* 0x3C */
    int32_t AENC_DECODER_PHI_A_RAW            ; /* 0x3D */
    int32_t AENC_DECODER_PHI_A_OFFSET         ; /* 0x3E */
    int32_t AENC_DECODER_PHI_A                ; /* 0x3F */
    int32_t AENC_DECODER_PPR                  ; /* 0x40 */
    int32_t AENC_DECODER_COUNT                ; /* 0x41 */
    int32_t AENC_DECODER_COUNT_N              ; /* 0x42 */
    int32_t RESERVE_05                        ; /* 0x43 */
    int32_t RESERVE_06                        ; /* 0x44 */
    int32_t AENC_DECODER_PHI_E_PHI_M_OFFSET   ; /* 0x45 */
    int32_t AENC_DECODER_PHI_E_PHI_M          ; /* 0x46 */
    int32_t AENC_DECODER_POSITION             ; /* 0x47 */
    int32_t RESERVE_07                        ; /* 0x48 */
    int32_t RESERVE_08                        ; /* 0x49 */
    int32_t RESERVE_09                        ; /* 0x4A */
    int32_t RESERVE_10                        ; /* 0x4B */
    int32_t RESERVE_11                        ; /* 0x4C */
    int32_t CONFIG_DATA                       ; /* 0x4D */
    int32_t CONFIG_ADDR                       ; /* 0x4E */
    int32_t RESERVE_12                        ; /* 0x4F */
    int32_t VELOCITY_SELECTION                ; /* 0x50 */
    int32_t POSITION_SELECTION                ; /* 0x51 */
    int32_t PHI_E_SELECTION                   ; /* 0x52 */
    int32_t PHI_E                             ; /* 0x53 */
    int32_t PID_FLUX_P_FLUX_I                 ; /* 0x54 */
    int32_t RESERVE_13                        ; /* 0x55 */
    int32_t PID_TORQUE_P_TORQUE_I             ; /* 0x56 */
    int32_t RESERVE_14                        ; /* 0x57 */
    int32_t PID_VELOCITY_P_VELOCITY_I         ; /* 0x58 */
    int32_t RESERVE_15                        ; /* 0x59 */
    int32_t PID_POSITION_P_POSITION_I         ; /* 0x5A */
    int32_t RESERVE_16                        ; /* 0x5B */
    int32_t PID_TORQUE_FLUX_TARGET_DDT_LIMITS ; /* 0x5C */
    int32_t PIDOUT_UQ_UD_LIMITS               ; /* 0x5D */
    int32_t PID_TORQUE_FLUX_LIMITS            ; /* 0x5E */
    int32_t PID_ACCELERATION_LIMIT            ; /* 0x5F */
    int32_t PID_VELOCITY_LIMIT                ; /* 0x60 */
    int32_t PID_POSITION_LIMIT_LOW            ; /* 0x61 */
    int32_t PID_POSITION_LIMIT_HIGH           ; /* 0x62 */
    int32_t MODE_RAMP_MODE_MOTION             ; /* 0x63 */
    int32_t PID_TORQUE_FLUX_TARGET            ; /* 0x64 */
    int32_t PID_TORQUE_FLUX_OFFSET            ; /* 0x65 */
    int32_t PID_VELOCITY_TARGET               ; /* 0x66 */
    int32_t PID_VELOCITY_OFFSET               ; /* 0x67 */
    int32_t PID_POSITION_TARGET               ; /* 0x68 */
    int32_t PID_TORQUE_FLUX_ACTUAL            ; /* 0x69 */
    int32_t PID_VELOCITY_ACTUAL               ; /* 0x6A */
    int32_t PID_POSITION_ACTUAL               ; /* 0x6B */
    int32_t PID_ERROR_DATA                    ; /* 0x6C */
    int32_t PID_ERROR_ADDR                    ; /* 0x6D */
    int32_t INTERIM_DATA                      ; /* 0x6E */
    int32_t INTERIM_ADDR                      ; /* 0x6F */
    int32_t RESERVE_17                        ; /* 0x70 */
    int32_t RESERVE_18                        ; /* 0x71 */
    int32_t RESERVE_19                        ; /* 0x72 */
    int32_t RESERVE_20                        ; /* 0x73 */
    int32_t WATCHDOG_CFG                      ; /* 0x74 */
    int32_t ADC_VM_LIMITS                     ; /* 0x75 */
    int32_t INPUTS_RAW                        ; /* 0x76 */
    int32_t OUTPUTS_RAW                       ; /* 0x77 */
    int32_t STEP_WIDTH                        ; /* 0x78 */
    int32_t UART_BPS                          ; /* 0x79 */
    int32_t UART_ADDRS                        ; /* 0x7A */
    int32_t GPIO_dsADCI_CONFIG                ; /* 0x7B */
    int32_t STATUS_FLAGS                      ; /* 0x7C */
    int32_t STATUS_MASK                       ; /* 0x7D */
    int32_t RESERVE_21                        ; /* 0x7E */
    int32_t RESERVE_22                        ; /* 0x7F */
  } R; /* Actual Values */
} volatile TMC_TypeDef;

/* Typedef ***************************************************************************************************/
typedef struct {
#if defined (CORE_CM7)
  SPI_HandleTypeDef *hSPIx;
#elif defined (CORE_CM4)
  UART_HandleTypeDef *hUARTx;
#endif
  TMC_TypeDef  *Instance;
  uint32_t      Flags;
  osThreadId_t  TxRxThreadID;
  uint8_t      *TxData;
  uint8_t      *RxData;
} volatile MOT_HandleTypeDef;

/* Constants *************************************************************************************************/
#define TMC1 ((TMC_TypeDef *) 0x30040000U)
#define TMC2 ((TMC_TypeDef *) 0x30041000U)
#define TMC3 ((TMC_TypeDef *) 0x30042000U)
#define TMC4 ((TMC_TypeDef *) 0x30043000U)
#define TMC5 ((TMC_TypeDef *) 0x30044000U)
#define TMC6 ((TMC_TypeDef *) 0x30045000U)

/* Constants *************************************************************************************************/
#define UDP_QueueStruct ((FIFO_QueueTypeDef *) 0x30046800U)
#define UDP_QueueSpaces ((uint8_t           *) 0x30047000U)

/* Function **************************************************************************************************/
extern uint32_t osRtxErrorNotify(uint32_t code, void *object_id);

/* __cplusplus ***********************************************************************************************/
#ifdef __cplusplus
  }
#endif

/* __BSP_STM32H7_H__ *****************************************************************************************/
#endif
