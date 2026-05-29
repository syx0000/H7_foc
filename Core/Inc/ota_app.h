/**
 * @file    ota_app.h
 * @brief   OTA firmware upgrade state machine (application side, Stage 1)
 *
 * Receives firmware chunks into Bank2 Sector 0~5 (App-B slot, 768KB).
 * Does NOT implement bootloader or slot swapping — that's Stage 2.
 * This module only proves the upload path: PC → USART1 → Flash → CRC verify.
 */
#ifndef __OTA_APP_H__
#define __OTA_APP_H__

#include <stdint.h>
#include "main.h"

/* Stage 2 single-slot + staging design
 *
 *   0x08020000  App slot       (Bank1 Sector 1~6, 768KB, the only exec region)
 *   0x080E0000  App header     (Bank1 Sector 7, 128KB)
 *   0x08100000  Staging slot   (Bank2 Sector 0~5, 768KB, OTA receive area)
 *   0x081C0000  Staging header (Bank2 Sector 6, 128KB)
 *
 * App always executes from 0x08020000. OTA writes to staging; on next reboot
 * the bootloader copies staging → App if staging is valid and PENDING.
 */
#define OTA_APP_START_ADDR      0x08020000U
#define OTA_APP_HEADER_ADDR     0x080E0000U
#define OTA_APP_SIZE            (768 * 1024)

#define OTA_STAGING_START_ADDR  0x08100000U
#define OTA_STAGING_HEADER_ADDR 0x081C0000U
#define OTA_STAGING_SIZE        (768 * 1024)

/* Legacy alias kept for any code still referencing the old "App-A/B" naming. */
#define OTA_APP_A_START_ADDR    OTA_APP_START_ADDR
#define OTA_APP_A_HEADER_ADDR   OTA_APP_HEADER_ADDR
#define OTA_APP_B_START_ADDR    OTA_STAGING_START_ADDR
#define OTA_APP_B_HEADER_ADDR   OTA_STAGING_HEADER_ADDR
#define OTA_APP_B_SIZE          OTA_STAGING_SIZE

/* Header flag bits */
#define APP_FLAG_VALID          0x01U   /* CRC checked, ready to run */
#define APP_FLAG_PENDING        0x02U   /* Staging only: copy me on next boot */

/* OTA chunk size (must match upper computer protocol.py::_OTA_CHUNK_SIZE) */
#define OTA_CHUNK_SIZE          256

/* Ring buffer for OTA RX data (4KB, holds ~16 chunks) */
#define OTA_RX_RING_SIZE        4096

/* App header magic (ASCII 'FOCA' = 0x41434F46) */
#define APP_HEADER_MAGIC        0x41434F46U

/**
 * App slot header (128 bytes, stored at the start of each header sector).
 * Stage 1: only magic/size/crc32 are used; boot_count/flags/version are
 * reserved for Stage 2 bootloader.
 */
typedef struct {
    uint32_t magic;        // 'FOCA' = 0x41434F46, marks valid slot
    uint32_t version;      // Firmware version (major.minor.patch encoded)
    uint32_t app_size;     // Application byte count (not including header)
    uint32_t app_crc32;    // IEEE 802.3 CRC32 of the app region
    uint32_t boot_count;   // Boot attempts (Stage 2 rollback mechanism)
    uint32_t flags;        // bit0=valid, bit1=tested, bit2=rollback_pending
    uint32_t build_time;   // Unix timestamp
    uint32_t reserved[25]; // Pad to 128B
} app_header_t;

/**
 * OTA state machine states.
 */
typedef enum {
    OTA_IDLE = 0,       // No session active
    OTA_ERASING,        // Erasing App-B sectors (blocking, ~1.5s)
    OTA_RECEIVING,      // Receiving data chunks
    OTA_FINALIZING,     // Writing header + verifying CRC
    OTA_DONE,           // Success
    OTA_ERROR           // Failure (call ota_abort to reset)
} ota_state_t;

/**
 * Initialize OTA module (call once at startup).
 */
void ota_init(void);

/**
 * Begin OTA session: erase App-B sectors, prepare to receive.
 * Blocks for ~1.5s while erasing 6 sectors.
 *
 * @param size      Expected firmware size in bytes (must fit in 768KB)
 * @param crc32     Expected CRC32 of the entire firmware
 * @param version   Version number (free-form, for header only)
 * @return          0 on success, -1 on error
 */
int ota_begin(uint32_t size, uint32_t crc32, uint32_t version);

/**
 * Feed raw bytes from USART RX into the OTA ring buffer.
 * Called from HAL_UARTEx_RxEventCallback when in OTA mode.
 *
 * @param data      Pointer to received bytes
 * @param len       Number of bytes
 */
void ota_rx_feed(const uint8_t *data, uint16_t len);

/**
 * Process accumulated RX data: parse OTA_DATA frames, write to Flash.
 * Call periodically from main loop (non-blocking, processes one frame per call).
 *
 * @return          Number of bytes written to Flash this call (0 if no complete frame)
 */
uint32_t ota_process(void);

/**
 * Finalize OTA session: verify whole-image CRC, write header.
 *
 * @return          0 on success, -1 on CRC mismatch or Flash error
 */
int ota_end(void);

/**
 * Abort OTA session: clear App-B header to mark slot invalid.
 */
void ota_abort(void);

/**
 * Get current OTA state.
 */
ota_state_t ota_get_state(void);

/**
 * Get progress: bytes written / total expected.
 */
void ota_get_progress(uint32_t *written, uint32_t *total);

/**
 * Stage 2 rollback support: clear our own slot's boot_count to 0.
 *
 * Bootloader increments boot_count before jumping. If the application
 * starts up healthy, it should call this once after the system has been
 * stable for a few seconds. Three consecutive boots without this call
 * means the bootloader will rollback to the other slot.
 *
 * Looks at SCB->VTOR to figure out which slot we're running from
 * (0x08020000 = App-A, 0x08100000 = App-B). For unsigned/dev images
 * (e.g. when loaded via debugger at 0x08000000) it does nothing.
 *
 * Call no more than once per power cycle. Costs one full sector
 * erase+write (~250ms) so do not call from a fast loop.
 */
void ota_mark_self_stable(void);

#endif /* __OTA_APP_H__ */
