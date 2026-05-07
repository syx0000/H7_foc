/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   DPT dual magnetic encoder driver (RS485, half-duplex)
  *          Based on DPT datasheet v0.5.
  *          Bus: USART2 @ 2.5 Mbps, direction pin: RS485_DIR (PA1).
  *          24-bit resolution per angle.
  ******************************************************************************
  */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ---- RS485 protocol commands (from datasheet chapter 7) ------------------ */
#define DPT_CMD_READ_INNER_ANGLE         0x29u  /* 3B: A0 A1 A2 (no CRC)      */
#define DPT_CMD_READ_OUTER_ANGLE         0x30u  /* 3B: B0 B1 B2 (no CRC)      */
#define DPT_CMD_READ_INNER_ANGLE_CRC     0x31u  /* 4B: A0 A1 A2 CRC           */
#define DPT_CMD_READ_OUTER_ANGLE_CRC     0x32u  /* 4B: B0 B1 B2 CRC           */
#define DPT_CMD_READ_DUAL_ANGLE          0x33u  /* 7B: A0..A2 B0..B2 CRC      */
#define DPT_CMD_READ_INNER_WITH_STATUS   0x41u  /* 5B: A0 A1 A2 S CRC         */
#define DPT_CMD_READ_OUTER_WITH_STATUS   0x42u  /* 5B: B0 B1 B2 S CRC         */
#define DPT_CMD_READ_DUAL_WITH_STATUS    0x43u  /* 8B: A0..A2 B0..B2 S CRC    */
#define DPT_CMD_READ_TEMPERATURE         0x74u  /* 3B: T0 T1 CRC (0.1 C/LSB)  */

/* ---- Status byte bit layout (datasheet chapter 11) ----------------------- */
#define DPT_STATUS_BISSC_ERR_BIT         0x2000u /* b13: BiSS-C warning        */
#define DPT_STATUS_BISSC_WARN_BIT        0x1000u /* b12: BiSS-C error          */
#define DPT_STATUS_RS485_ERR_BIT         0x80u   /* b7:  RS485 mirror warning  */
#define DPT_STATUS_RS485_WARN_BIT        0x40u   /* b6:  RS485 mirror error    */
#define DPT_STATUS_LED_WARN_BIT          0x08u   /* b3                         */
#define DPT_STATUS_LED_ERR_BIT           0x04u   /* b2                         */

typedef enum {
    DPT_OK             = 0,
    DPT_ERR_TIMEOUT    = 1,
    DPT_ERR_CRC        = 2,
    DPT_ERR_BUSY       = 3,
    DPT_ERR_HAL        = 4,
    DPT_ERR_PARAM      = 5,
} DPT_Status;

typedef enum {
    DPT_STATE_IDLE = 0,
    DPT_STATE_TX_BUSY,
    DPT_STATE_RX_BUSY,
} DPT_TransferState;

typedef struct {
    uint32_t inner_raw;        /* 24-bit inner angle count [0, 1<<24)       */
    uint32_t outer_raw;        /* 24-bit outer angle count [0, 1<<24)       */
    float    inner_deg;        /* inner angle converted to degrees          */
    float    outer_deg;        /* outer angle converted to degrees          */
    uint8_t  status;           /* valid only for *_WithStatus reads         */
    uint8_t  has_status;       /* 1 if status field populated               */
} DPT_Angles;

/* Initializes internal state. Must be called once after peripherals are up. */
void DPT_Encoder_Init(UART_HandleTypeDef *huart);

/* Non-blocking read of both angles (command 0x33). Returns DPT_ERR_BUSY if previous transfer not done. */
DPT_Status DPT_ReadDualAngle_DMA(DPT_Angles *out, uint32_t timeout_ms);

/* Non-blocking read of both angles plus status byte (command 0x43). */
DPT_Status DPT_ReadDualAngleWithStatus_DMA(DPT_Angles *out, uint32_t timeout_ms);

/* Non-blocking read of internal temperature in 0.1 degC units (command 0x74). */
DPT_Status DPT_ReadTemperature_DMA(int16_t *temp_tenths, uint32_t timeout_ms);

/* Check if DMA transfer is complete. Call this in main loop or use callbacks. */
DPT_TransferState DPT_GetTransferState(void);

/* Must be called in HAL_UART_TxCpltCallback to switch RS485 direction after TX. */
void DPT_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/* Must be called in HAL_UART_RxCpltCallback to process received data. */
void DPT_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* Blocking read of both angles (command 0x33). */
DPT_Status DPT_ReadDualAngle(DPT_Angles *out, uint32_t timeout_ms);

/* Blocking read of both angles plus status byte (command 0x43). */
DPT_Status DPT_ReadDualAngleWithStatus(DPT_Angles *out, uint32_t timeout_ms);

/* Blocking read of internal temperature in 0.1 degC units (command 0x74).   */
DPT_Status DPT_ReadTemperature(int16_t *temp_tenths, uint32_t timeout_ms);

/* ---- 异步/中断驱动接口 --------------------------------------------------- */
/* 在中断中调用（如TIM1 CC4），立即返回，不等传输完成。
   数据到达后会自动更新内部缓冲区，并置位新数据标志。 */
DPT_Status DPT_TriggerDualAngleRead_Async(void);
DPT_Status DPT_TriggerDualAngleWithStatusRead_Async(void);

/* 主循环中检查是否有新数据。返回1表示有新数据，调用后自动清零。 */
uint8_t DPT_HasNewData(void);

/* 获取最新的编码器数据（中断安全）。 */
void DPT_GetLatestAngles(DPT_Angles *out);

/* 查询上一次传输的状态（OK/CRC错误/超时等） */
DPT_Status DPT_GetLastAsyncStatus(void);

/* 获取并重置统计信息：触发次数、成功次数、BUSY跳过次数、耗时（us） */
void DPT_GetAndResetStats(uint32_t *trigger_count,
                          uint32_t *success_count,
                          uint32_t *busy_skip_count,
                          uint32_t *last_us,
                          uint32_t *min_us,
                          uint32_t *max_us);

/* CRC-8 over raw bytes, poly x^8+x^7+x^4+x^2+x+1 (datasheet chapter 12).
   Exposed for testing. Length is total bytes of the response including CRC; a
   valid frame returns 0. */
uint8_t DPT_CalcCRC8(const uint8_t *buffer, uint8_t length);

#ifdef __cplusplus
}
#endif
#endif /* __ENCODER_H__ */
