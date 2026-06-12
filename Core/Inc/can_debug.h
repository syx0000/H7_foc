#ifndef __CAN_DEBUG_H__
#define __CAN_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* CAN ID */
#define CAN_DBG_ID_CMD      0x7E0
#define CAN_DBG_ID_RESP     0x7E1
#define CAN_DBG_ID_LOG      0x7E2
#define CAN_DBG_ID_EVENT    0x7E3
#define CAN_DBG_ID_OTA_DATA 0x7E4
#define CAN_DBG_ID_OTA_ACK  0x7E5
#define CAN_DBG_ID_TEXT     0x7E6

#define CAN_DBG_PROTO_VER   1
#define CAN_DBG_ERR_FLAG    0xFF

typedef enum {
    CAN_DBG_OK              = 0x00,
    CAN_DBG_ERR_UNKNOWN_CMD = 0x01,
    CAN_DBG_ERR_BAD_LEN     = 0x02,
    CAN_DBG_ERR_OUT_OF_RANGE= 0x03,
    CAN_DBG_ERR_BUSY        = 0x04,
    CAN_DBG_ERR_FAULT       = 0x05,
    CAN_DBG_ERR_BRAKE       = 0x06,
    CAN_DBG_ERR_INTERNAL    = 0x07,
} can_dbg_err_t;

/* CMD_ID */
#define CAN_DBG_CMD_PING            0x00
#define CAN_DBG_CMD_VERSION         0x01
#define CAN_DBG_CMD_RESET           0x02
#define CAN_DBG_CMD_GET_PARAMS      0x03
#define CAN_DBG_CMD_LOGID_SET       0x10
#define CAN_DBG_CMD_LOGFREQ_SET     0x11
#define CAN_DBG_CMD_CUR_PID_SET     0x20
#define CAN_DBG_CMD_SPD_PID_SET     0x21
#define CAN_DBG_CMD_POS_PID_SET     0x22
#define CAN_DBG_CMD_FLASH_WRITE     0x40
#define CAN_DBG_CMD_FLASH_ERASE     0x41
#define CAN_DBG_CMD_FLASH_COMPARE   0x42
#define CAN_DBG_CMD_FAULT_CLR       0x43
#define CAN_DBG_CMD_ENABLE          0x50
#define CAN_DBG_CMD_PHASE_COMP_SET  0x52
#define CAN_DBG_CMD_PHASE_COMP_SAVE 0x53
#define CAN_DBG_CMD_CALI            0x5F
#define CAN_DBG_CMD_BWTEST          0x60
#define CAN_DBG_CMD_CANRXDBG        0x61
#define CAN_DBG_CMD_OVERLOAD_SET    0x62
#define CAN_DBG_CMD_OTA_BEGIN       0x70
#define CAN_DBG_CMD_OTA_END         0x71
#define CAN_DBG_CMD_OTA_ABORT       0x72
#define CAN_DBG_CMD_OTA_SWAP        0x73

/* Event ID */
#define CAN_DBG_EVT_BWTEST_DONE     0x30
#define CAN_DBG_EVT_CALI_DONE       0x31

void can_debug_init(void);
void can_debug_rx_isr(uint32_t id, const uint8_t *data, uint32_t len);
void can_debug_poll(void);
void can_debug_send_log(void);
void can_debug_send_event(uint8_t evt_id, const uint8_t *payload, uint32_t len);

/* CAN OTA: 处理 0x7E4 数据帧 (从 fdcan_rx_user 调用) */
void can_debug_ota_data_rx(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_DEBUG_H__ */