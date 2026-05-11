/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   DPT dual magnetic encoder driver implementation (RS485 half-duplex)
  *          Uses hardware RS485 DE mode - no software direction switching needed.
  *          Implements commands 0x33 (dual angle), 0x43 (dual+status), 0x74 (temp).
  *          CRC-8 table from datasheet chapter 12.
  ******************************************************************************
  */
#include "encoder.h"
#include "tim.h"
#include "usart.h"
#include <string.h>

/* ---- CRC-8 lookup table (poly x^8+x^7+x^4+x^2+x+1) from datasheet -------- */
static const uint8_t crc8_table[256] = {
    0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
    0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C,
    0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
    0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 0x58, 0xCF,
    0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7,
    0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
    0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24,
    0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E,
    0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
    0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E,
    0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7,
    0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
    0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65,
    0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF,
    0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
    0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C
};

uint8_t DPT_CalcCRC8(const uint8_t *buffer, uint8_t length) {
    uint8_t temp = *buffer++;
    while (--length) {
        temp = *buffer++ ^ crc8_table[temp];
    }
    return crc8_table[temp];
}

/* ---- Internal state ------------------------------------------------------ */
static UART_HandleTypeDef *g_huart = NULL;
static volatile DPT_TransferState g_state = DPT_STATE_IDLE;
static uint8_t g_tx_buf[1];
static uint8_t g_rx_buf[8];
static uint8_t g_rx_len = 0;
static DPT_Angles *g_result_ptr = NULL;
static int16_t *g_temp_ptr = NULL;
static uint8_t g_cmd_type = 0;

/* 异步模式专用 - 中断触发的周期读取 */
static volatile DPT_Angles g_latest_angles = {0};
static volatile uint8_t g_new_data_flag = 0;
static volatile DPT_Status g_last_async_status = DPT_OK;
static uint8_t g_async_mode = 0;  /* 1=当前传输是异步模式，需要更新内部缓冲区 */

/* 统计数据 */
static volatile uint32_t g_trigger_count = 0;      /* 触发次数（含跳过的） */
static volatile uint32_t g_success_count = 0;     /* 成功完成次数 */
static volatile uint32_t g_busy_skip_count = 0;   /* 因BUSY跳过次数 */
static volatile uint32_t g_trigger_start_cycles = 0;  /* 本次触发的DWT周期 */
static volatile uint32_t g_last_elapsed_us = 0;   /* 上次触发到完成的耗时 */
static volatile uint32_t g_max_elapsed_us = 0;    /* 最大耗时 */
static volatile uint32_t g_min_elapsed_us = 0xFFFFFFFF;  /* 最小耗时 */

void DPT_Encoder_Init(UART_HandleTypeDef *huart)
{
    g_huart = huart;
    g_state = DPT_STATE_IDLE;
}

/* ---- DMA-based API ------------------------------------------------------- */
DPT_TransferState DPT_GetTransferState(void) {
    return g_state;
}

/* TX完成回调 - 硬件DE模式下无需切换方向，直接启动RX DMA */
void DPT_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != g_huart) return;

    /* 清除可能的回环噪声 */
    __HAL_UART_SEND_REQ(g_huart, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_OREFLAG(g_huart);
    __HAL_UART_CLEAR_NEFLAG(g_huart);
    __HAL_UART_CLEAR_FEFLAG(g_huart);

    g_state = DPT_STATE_RX_BUSY;
    if (HAL_UART_Receive_DMA(g_huart, g_rx_buf, g_rx_len) != HAL_OK) {
        g_state = DPT_STATE_IDLE;
    }
}

void DPT_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != g_huart) return;

    if (DPT_CalcCRC8(g_rx_buf, g_rx_len) != 0) {
        g_last_async_status = DPT_ERR_CRC;
        g_state = DPT_STATE_IDLE;
        return;
    }

    /* 异步模式：数据写入内部缓冲区 */
    DPT_Angles *dst = g_async_mode ? (DPT_Angles*)&g_latest_angles : g_result_ptr;

    if (g_cmd_type == DPT_CMD_READ_DUAL_ANGLE && dst) {
        dst->inner_raw = ((uint32_t)g_rx_buf[2] << 16) | ((uint32_t)g_rx_buf[1] << 8) | g_rx_buf[0];
        dst->outer_raw = ((uint32_t)g_rx_buf[5] << 16) | ((uint32_t)g_rx_buf[4] << 8) | g_rx_buf[3];
        dst->inner_deg = (float)dst->inner_raw / (float)(1u << 24) * 360.0f;
        dst->outer_deg = (float)dst->outer_raw / (float)(1u << 24) * 360.0f;
        dst->has_status = 0;
    }
    else if (g_cmd_type == DPT_CMD_READ_DUAL_WITH_STATUS && dst) {
        dst->inner_raw = ((uint32_t)g_rx_buf[2] << 16) | ((uint32_t)g_rx_buf[1] << 8) | g_rx_buf[0];
        dst->outer_raw = ((uint32_t)g_rx_buf[5] << 16) | ((uint32_t)g_rx_buf[4] << 8) | g_rx_buf[3];
        dst->inner_deg = (float)dst->inner_raw / (float)(1u << 24) * 360.0f;
        dst->outer_deg = (float)dst->outer_raw / (float)(1u << 24) * 360.0f;
        dst->status = g_rx_buf[6];
        dst->has_status = 1;
    }
    else if (g_cmd_type == DPT_CMD_READ_TEMPERATURE && g_temp_ptr) {
        *g_temp_ptr = (int16_t)((uint16_t)g_rx_buf[1] << 8 | g_rx_buf[0]);
    }

    if (g_async_mode) {
        /* 记录编码器读取完成时的DWT周期 */
        g_tim1_enc_done_cycles = DWT_GetCycles();

        /* 计算本次触发到完成的耗时 */
        uint32_t elapsed_us = DWT_CyclesToUs(g_tim1_enc_done_cycles - g_trigger_start_cycles);
        g_last_elapsed_us = elapsed_us;
        if (elapsed_us > g_max_elapsed_us) g_max_elapsed_us = elapsed_us;
        if (elapsed_us < g_min_elapsed_us) g_min_elapsed_us = elapsed_us;

        g_success_count++;
        g_new_data_flag = 1;
        g_last_async_status = DPT_OK;
    }
    g_state = DPT_STATE_IDLE;
}

/* UART错误回调 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        HAL_UART_Abort(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        g_state = DPT_STATE_IDLE;
    } else if (huart->Instance == USART1) {
        /* 清错误标志并重启调试RX（启动时printf DMA与用户输入冲突易触发Overrun） */
        HAL_UART_AbortReceive(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        USART1_DebugRx_Start();
    }
}

static DPT_Status start_dma_transaction(uint8_t cmd, uint8_t rx_len, void *result_ptr) {
    if (!g_huart) return DPT_ERR_PARAM;

    /* 卡死恢复 */
    if (g_state != DPT_STATE_IDLE ||
        g_huart->gState != HAL_UART_STATE_READY ||
        g_huart->RxState != HAL_UART_STATE_READY) {
        HAL_UART_Abort(g_huart);
        __HAL_UART_CLEAR_OREFLAG(g_huart);
        __HAL_UART_CLEAR_NEFLAG(g_huart);
        __HAL_UART_CLEAR_FEFLAG(g_huart);
        __HAL_UART_CLEAR_PEFLAG(g_huart);
        g_state = DPT_STATE_IDLE;
    }

    g_tx_buf[0] = cmd;
    g_rx_len = rx_len;
    g_cmd_type = cmd;
    g_result_ptr = (cmd == DPT_CMD_READ_TEMPERATURE) ? NULL : (DPT_Angles*)result_ptr;
    g_temp_ptr = (cmd == DPT_CMD_READ_TEMPERATURE) ? (int16_t*)result_ptr : NULL;
    g_async_mode = 0;  /* 同步调用 */

    g_state = DPT_STATE_TX_BUSY;

    if (HAL_UART_Transmit_DMA(g_huart, g_tx_buf, 1) != HAL_OK) {
        g_state = DPT_STATE_IDLE;
        return DPT_ERR_HAL;
    }

    return DPT_OK;
}

DPT_Status DPT_ReadDualAngle_DMA(DPT_Angles *out, uint32_t timeout_ms) {
    DPT_Status st = start_dma_transaction(DPT_CMD_READ_DUAL_ANGLE, 7, out);
    if (st != DPT_OK) return st;

    uint32_t start = HAL_GetTick();
    while (g_state != DPT_STATE_IDLE) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            HAL_UART_DMAStop(g_huart);
            g_state = DPT_STATE_IDLE;
            return DPT_ERR_TIMEOUT;
        }
    }

    return (DPT_CalcCRC8(g_rx_buf, 7) == 0) ? DPT_OK : DPT_ERR_CRC;
}

DPT_Status DPT_ReadDualAngleWithStatus_DMA(DPT_Angles *out, uint32_t timeout_ms) {
    DPT_Status st = start_dma_transaction(DPT_CMD_READ_DUAL_WITH_STATUS, 8, out);
    if (st != DPT_OK) return st;

    uint32_t start = HAL_GetTick();
    while (g_state != DPT_STATE_IDLE) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            HAL_UART_DMAStop(g_huart);
            g_state = DPT_STATE_IDLE;
            return DPT_ERR_TIMEOUT;
        }
    }

    return (DPT_CalcCRC8(g_rx_buf, 8) == 0) ? DPT_OK : DPT_ERR_CRC;
}

DPT_Status DPT_ReadTemperature_DMA(int16_t *temp_tenths, uint32_t timeout_ms) {
    DPT_Status st = start_dma_transaction(DPT_CMD_READ_TEMPERATURE, 3, temp_tenths);
    if (st != DPT_OK) return st;

    uint32_t start = HAL_GetTick();
    while (g_state != DPT_STATE_IDLE) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            HAL_UART_DMAStop(g_huart);
            g_state = DPT_STATE_IDLE;
            return DPT_ERR_TIMEOUT;
        }
    }

    return (DPT_CalcCRC8(g_rx_buf, 3) == 0) ? DPT_OK : DPT_ERR_CRC;
}

/* ---- HAL Callbacks (automatically called by HAL) ------------------------- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        DPT_UART_TxCpltCallback(huart);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        DPT_UART_RxCpltCallback(huart);
    }
}

/* ---- Blocking API (fallback) --------------------------------------------- */
static DPT_Status transact_rs485(uint8_t cmd, uint8_t *rx_buf, uint8_t rx_len,
                                 uint32_t timeout_ms)
{
    if (!g_huart) return DPT_ERR_PARAM;

    if (g_huart->gState != HAL_UART_STATE_READY ||
        g_huart->RxState != HAL_UART_STATE_READY) {
        HAL_UART_Abort(g_huart);
        __HAL_UART_CLEAR_OREFLAG(g_huart);
        __HAL_UART_CLEAR_NEFLAG(g_huart);
        __HAL_UART_CLEAR_FEFLAG(g_huart);
        __HAL_UART_CLEAR_PEFLAG(g_huart);
    }

    HAL_StatusTypeDef st = HAL_UART_Transmit(g_huart, &cmd, 1, timeout_ms);
    if (st != HAL_OK) {
        HAL_UART_Abort(g_huart);
        return DPT_ERR_HAL;
    }

    /* 清除TX回环产生的噪声 */
    __HAL_UART_SEND_REQ(g_huart, UART_RXDATA_FLUSH_REQUEST);
    __HAL_UART_CLEAR_OREFLAG(g_huart);

    st = HAL_UART_Receive(g_huart, rx_buf, rx_len, timeout_ms);
    if (st != HAL_OK) {
        HAL_UART_Abort(g_huart);
        __HAL_UART_CLEAR_OREFLAG(g_huart);
        __HAL_UART_CLEAR_NEFLAG(g_huart);
        __HAL_UART_CLEAR_FEFLAG(g_huart);
        return (st == HAL_TIMEOUT) ? DPT_ERR_TIMEOUT : DPT_ERR_HAL;
    }

    if (DPT_CalcCRC8(rx_buf, rx_len) != 0) return DPT_ERR_CRC;
    return DPT_OK;
}

DPT_Status DPT_ReadDualAngle(DPT_Angles *out, uint32_t timeout_ms)
{
    uint8_t buf[7];
    DPT_Status st = transact_rs485(DPT_CMD_READ_DUAL_ANGLE, buf, 7, timeout_ms);
    if (st != DPT_OK) return st;

    out->inner_raw = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
    out->outer_raw = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];
    out->inner_deg = (float)out->inner_raw / (float)(1u << 24) * 360.0f;
    out->outer_deg = (float)out->outer_raw / (float)(1u << 24) * 360.0f;
    out->has_status = 0;
    return DPT_OK;
}

DPT_Status DPT_ReadDualAngleWithStatus(DPT_Angles *out, uint32_t timeout_ms)
{
    uint8_t buf[8];
    DPT_Status st = transact_rs485(DPT_CMD_READ_DUAL_WITH_STATUS, buf, 8, timeout_ms);
    if (st != DPT_OK) return st;

    out->inner_raw = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
    out->outer_raw = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];
    out->inner_deg = (float)out->inner_raw / (float)(1u << 24) * 360.0f;
    out->outer_deg = (float)out->outer_raw / (float)(1u << 24) * 360.0f;
    out->status = buf[6];
    out->has_status = 1;
    return DPT_OK;
}

DPT_Status DPT_ReadTemperature(int16_t *temp_tenths, uint32_t timeout_ms)
{
    uint8_t buf[3];
    DPT_Status st = transact_rs485(DPT_CMD_READ_TEMPERATURE, buf, 3, timeout_ms);
    if (st != DPT_OK) return st;

    *temp_tenths = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    return DPT_OK;
}

/* ---- 异步/中断驱动接口实现 ----------------------------------------------- */
static DPT_Status trigger_async(uint8_t cmd, uint8_t rx_len) {
    if (!g_huart) return DPT_ERR_PARAM;

    g_trigger_count++;

    /* 上次没完成就跳过本次触发，避免冲突 */
    if (g_state != DPT_STATE_IDLE ||
        g_huart->gState != HAL_UART_STATE_READY ||
        g_huart->RxState != HAL_UART_STATE_READY) {
        g_busy_skip_count++;
        g_last_async_status = DPT_ERR_BUSY;
        return DPT_ERR_BUSY;
    }

    /* 记录触发时刻，用于测量触发到完成的耗时 */
    g_trigger_start_cycles = DWT_GetCycles();

    g_tx_buf[0] = cmd;
    g_rx_len = rx_len;
    g_cmd_type = cmd;
    g_result_ptr = NULL;
    g_temp_ptr = NULL;
    g_async_mode = 1;
    g_state = DPT_STATE_TX_BUSY;

    if (HAL_UART_Transmit_DMA(g_huart, g_tx_buf, 1) != HAL_OK) {
        g_state = DPT_STATE_IDLE;
        g_last_async_status = DPT_ERR_HAL;
        return DPT_ERR_HAL;
    }
    return DPT_OK;
}

DPT_Status DPT_TriggerDualAngleRead_Async(void) {
    return trigger_async(DPT_CMD_READ_DUAL_ANGLE, 7);
}

DPT_Status DPT_TriggerDualAngleWithStatusRead_Async(void) {
    return trigger_async(DPT_CMD_READ_DUAL_WITH_STATUS, 8);
}

uint8_t DPT_HasNewData(void) {
    if (g_new_data_flag) {
        g_new_data_flag = 0;
        return 1;
    }
    return 0;
}

void DPT_GetLatestAngles(DPT_Angles *out) {
    /* 禁中断保证一致性 */
    __disable_irq();
    *out = *(const DPT_Angles*)&g_latest_angles;
    __enable_irq();
}

/* ISR 专用：不禁中断直接读（调用方需保证不会被更高优先级的 RxCplt 打断） */
void DPT_GetLatestAngles_ISR(DPT_Angles *out) {
    *out = *(const DPT_Angles*)&g_latest_angles;
}

DPT_Status DPT_GetLastAsyncStatus(void) {
    return g_last_async_status;
}

/* 获取并重置统计信息（原子操作） */
void DPT_GetAndResetStats(uint32_t *trigger_count,
                          uint32_t *success_count,
                          uint32_t *busy_skip_count,
                          uint32_t *last_us,
                          uint32_t *min_us,
                          uint32_t *max_us)
{
    __disable_irq();
    *trigger_count   = g_trigger_count;
    *success_count   = g_success_count;
    *busy_skip_count = g_busy_skip_count;
    *last_us         = g_last_elapsed_us;
    *min_us          = (g_min_elapsed_us == 0xFFFFFFFF) ? 0 : g_min_elapsed_us;
    *max_us          = g_max_elapsed_us;
    g_trigger_count   = 0;
    g_success_count   = 0;
    g_busy_skip_count = 0;
    g_min_elapsed_us  = 0xFFFFFFFF;
    g_max_elapsed_us  = 0;
    __enable_irq();
}
