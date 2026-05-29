/**
 * @file    ota_app.c
 * @brief   OTA firmware upgrade state machine implementation
 */
#include "ota_app.h"
#include "flash_port.h"
#include <string.h>
#include <stdio.h>

/* OTA session state */
static struct {
    ota_state_t state;
    uint32_t expected_size;
    uint32_t expected_crc32;
    uint32_t version;
    uint32_t bytes_written;
    uint16_t next_seq;          // Expected next chunk sequence number
    uint8_t  write_buf[32];     // Accumulator for 32B Flash write granularity
    uint8_t  write_buf_len;
    /* Target slot (determined at ota_begin based on which slot we're running from) */
    uint32_t target_start;      // App region start address
    uint32_t target_header;     // Header address
    uint32_t target_bank;       // FLASH_BANK_x for erase
    uint32_t target_sector0;    // First sector number of app region
    uint32_t target_nsectors;   // Number of sectors to erase (app + header)
    uint32_t header_sector;     // Sector number of header
} g_ota;

/* Ring buffer for RX data (fed from USART ISR, consumed by main loop) */
static struct {
    uint8_t  buf[OTA_RX_RING_SIZE];
    uint16_t head;  // Write index (ISR)
    uint16_t tail;  // Read index (main loop)
} g_ota_rx;

/* CRC16-MODBUS (poly=0xA001 reflected, init=0xFFFF, no xorOut) */
static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void ota_init(void)
{
    memset(&g_ota, 0, sizeof(g_ota));
    memset(&g_ota_rx, 0, sizeof(g_ota_rx));
    g_ota.state = OTA_IDLE;
}

int ota_begin(uint32_t size, uint32_t crc32, uint32_t version)
{
    if (g_ota.state != OTA_IDLE) {
        printf("OTA_ERR already_active\r\n");
        return -1;
    }
    if (size > OTA_STAGING_SIZE) {
        printf("OTA_ERR size_too_large\r\n");
        return -1;
    }

    /* Stage 2: target is always the staging area (Bank2 Sector 0~6).
       App always runs from 0x08020000; bootloader copies staging→App. */
    g_ota.target_start    = OTA_STAGING_START_ADDR;
    g_ota.target_header   = OTA_STAGING_HEADER_ADDR;
    g_ota.target_bank     = FLASH_BANK_2;
    g_ota.target_sector0  = FLASH_SECTOR_0;
    g_ota.target_nsectors = 7;  /* Sector 0~6: 768KB app + 128KB header */
    g_ota.header_sector   = FLASH_SECTOR_6;

    g_ota.state = OTA_ERASING;
    g_ota.expected_size = size;
    g_ota.expected_crc32 = crc32;
    g_ota.version = version;
    g_ota.bytes_written = 0;
    g_ota.next_seq = 0;
    g_ota.write_buf_len = 0;

    /* Reset RX ring buffer so leftover bytes from a previous aborted OTA
       (or stray text-mode bytes) don't poison the first chunk. */
    g_ota_rx.head = 0;
    g_ota_rx.tail = 0;

    printf("OTA: Erasing staging (start=0x%08X, %u sectors)...\r\n",
           (unsigned)g_ota.target_start, (unsigned)g_ota.target_nsectors);

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = g_ota.target_bank;
    erase.Sector = g_ota.target_sector0;
    erase.NbSectors = g_ota.target_nsectors;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t err = 0;
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &err);
    HAL_FLASH_Lock();

    if (st != HAL_OK) {
        printf("OTA_ERR erase_failed sector=%u\r\n", (unsigned)err);
        g_ota.state = OTA_ERROR;
        return -1;
    }

    g_ota.state = OTA_RECEIVING;
    printf("OTA_READY chunk=%d\r\n", OTA_CHUNK_SIZE);
    return 0;
}

void ota_rx_feed(const uint8_t *data, uint16_t len)
{
    /* Called from USART ISR — keep it fast, just copy to ring buffer */
    for (uint16_t i = 0; i < len; i++) {
        g_ota_rx.buf[g_ota_rx.head] = data[i];
        g_ota_rx.head = (g_ota_rx.head + 1) % OTA_RX_RING_SIZE;

        /* Overflow check: if head catches tail, we're dropping data */
        if (g_ota_rx.head == g_ota_rx.tail) {
            /* Advance tail to discard oldest byte (lossy, but prevents deadlock) */
            g_ota_rx.tail = (g_ota_rx.tail + 1) % OTA_RX_RING_SIZE;
        }
    }
}

/* Helper: peek N bytes from ring buffer without consuming */
static uint16_t ring_peek(uint8_t *out, uint16_t len)
{
    uint16_t avail = (g_ota_rx.head >= g_ota_rx.tail)
                     ? (g_ota_rx.head - g_ota_rx.tail)
                     : (OTA_RX_RING_SIZE - g_ota_rx.tail + g_ota_rx.head);
    if (avail < len) return 0;

    uint16_t idx = g_ota_rx.tail;
    for (uint16_t i = 0; i < len; i++) {
        out[i] = g_ota_rx.buf[idx];
        idx = (idx + 1) % OTA_RX_RING_SIZE;
    }
    return len;
}

/* Helper: consume N bytes from ring buffer */
static void ring_consume(uint16_t len)
{
    g_ota_rx.tail = (g_ota_rx.tail + len) % OTA_RX_RING_SIZE;
}

uint32_t ota_process(void)
{
    if (g_ota.state != OTA_RECEIVING) return 0;

    /* OTA_DATA frame layout (little-endian):
         offset  size  field
         0       2     'OD' magic
         2       2     seq
         4       2     len
         6       len   payload
         6+len   2     crc16 over bytes [0 .. 6+len)
    */

    uint8_t header[6];
    if (ring_peek(header, 6) < 6) return 0;  // Not enough data yet

    /* Check magic */
    if (header[0] != 'O' || header[1] != 'D') {
        /* Sync error: discard 1 byte and retry */
        ring_consume(1);
        return 0;
    }

    uint16_t seq = header[2] | (header[3] << 8);
    uint16_t len = header[4] | (header[5] << 8);

    if (len > OTA_CHUNK_SIZE) {
        printf("OTA_NAK %u bad_len=%u\r\n", seq, len);
        ring_consume(6);  // Discard bad header
        return 0;
    }

    uint16_t frame_len = 6 + len + 2;  // header + payload + crc16
    uint8_t frame_buf[6 + OTA_CHUNK_SIZE + 2];

    if (ring_peek(frame_buf, frame_len) < frame_len) return 0;  // Incomplete frame

    /* Verify CRC16 */
    uint16_t expected_crc = crc16_modbus(frame_buf, 6 + len);
    uint16_t actual_crc = frame_buf[6 + len] | (frame_buf[6 + len + 1] << 8);

    if (actual_crc != expected_crc) {
        printf("OTA_NAK %u crc_mismatch exp=%04X got=%04X len=%u "
               "head=%02X%02X%02X%02X%02X%02X tail=%02X%02X "
               "ring_h=%u ring_t=%u\r\n",
               seq, expected_crc, actual_crc, len,
               frame_buf[0], frame_buf[1], frame_buf[2],
               frame_buf[3], frame_buf[4], frame_buf[5],
               frame_buf[6 + len], frame_buf[6 + len + 1],
               (unsigned)g_ota_rx.head, (unsigned)g_ota_rx.tail);
        /* dump payload first 32B to compare with what upper computer sent */
        printf("  payload[0..31]=");
        for (int k = 0; k < 32; k++) printf("%02X", frame_buf[6 + k]);
        printf("\r\n");
        printf("  payload[224..255]=");
        for (int k = 224; k < 256; k++) printf("%02X", frame_buf[6 + k]);
        printf("\r\n");
        ring_consume(frame_len);
        return 0;
    }

    /* Check sequence */
    if (seq != g_ota.next_seq) {
        printf("OTA_NAK %u seq_mismatch expected=%u\r\n", seq, g_ota.next_seq);
        ring_consume(frame_len);
        return 0;
    }

    /* Write payload to Flash (accumulate to 32B boundary) */
    const uint8_t *payload = &frame_buf[6];
    uint32_t written_this_call = 0;

    for (uint16_t i = 0; i < len; i++) {
        g_ota.write_buf[g_ota.write_buf_len++] = payload[i];

        if (g_ota.write_buf_len == 32) {
            uint32_t addr = g_ota.target_start + g_ota.bytes_written;
            HAL_StatusTypeDef st = Flash_WriteData(addr, g_ota.write_buf, 32);
            if (st != HAL_OK) {
                printf("OTA_NAK %u flash_write_failed\r\n", seq);
                g_ota.state = OTA_ERROR;
                ring_consume(frame_len);
                return 0;
            }
            g_ota.bytes_written += 32;
            written_this_call += 32;
            g_ota.write_buf_len = 0;
        }
    }

    /* ACK */
    printf("OTA_ACK %u\r\n", seq);
    g_ota.next_seq = (seq + 1) & 0xFFFF;
    ring_consume(frame_len);

    /* Auto-exit OTA RX mode once we've received all expected bytes.
       Subsequent commands ('otaend') need to come through the text path,
       which requires g_ota_rx_mode to be cleared. */
    uint32_t total_received = g_ota.bytes_written + g_ota.write_buf_len;
    if (total_received >= g_ota.expected_size) {
        extern volatile uint8_t g_ota_rx_mode;
        g_ota_rx_mode = 0;
        printf("OTA: all data received (%u bytes), waiting for otaend\r\n",
               (unsigned)total_received);
    }

    return written_this_call;
}

int ota_end(void)
{
    if (g_ota.state != OTA_RECEIVING) {
        printf("OTA_FAIL not_receiving\r\n");
        return -1;
    }

    /* Flush any partial write buffer (pad with 0xFF) */
    if (g_ota.write_buf_len > 0) {
        memset(&g_ota.write_buf[g_ota.write_buf_len], 0xFF, 32 - g_ota.write_buf_len);
        uint32_t addr = OTA_APP_B_START_ADDR + g_ota.bytes_written;
        HAL_StatusTypeDef st = Flash_WriteData(addr, g_ota.write_buf, 32);
        if (st != HAL_OK) {
            printf("OTA_FAIL flush_failed\r\n");
            g_ota.state = OTA_ERROR;
            return -1;
        }
        g_ota.bytes_written += g_ota.write_buf_len;
        g_ota.write_buf_len = 0;
    }

    /* Verify whole-image CRC32 */
    printf("OTA: Verifying CRC32 over %u bytes...\r\n", (unsigned)g_ota.expected_size);
    uint32_t actual_crc = Flash_Crc32((const void *)(uintptr_t)g_ota.target_start,
                                      g_ota.expected_size);

    if (actual_crc != g_ota.expected_crc32) {
        printf("OTA_FAIL crc32_mismatch expected=0x%08X actual=0x%08X\r\n",
               (unsigned)g_ota.expected_crc32, (unsigned)actual_crc);
        g_ota.state = OTA_ERROR;
        return -1;
    }

    /* Write header to staging header sector. PENDING flag tells bootloader
       to copy staging→App on next boot. */
    app_header_t header;
    memset(&header, 0xFF, sizeof(header));
    header.magic = APP_HEADER_MAGIC;
    header.version = g_ota.version;
    header.app_size = g_ota.expected_size;
    header.app_crc32 = g_ota.expected_crc32;
    header.boot_count = 0;
    header.flags = APP_FLAG_VALID | APP_FLAG_PENDING;
    header.build_time = 0;

    HAL_StatusTypeDef st = Flash_WriteData(g_ota.target_header, &header, sizeof(header));
    if (st != HAL_OK) {
        printf("OTA_FAIL header_write_failed\r\n");
        g_ota.state = OTA_ERROR;
        return -1;
    }

    g_ota.state = OTA_DONE;
    printf("OTA_DONE size=%u crc=0x%08X\r\n",
           (unsigned)g_ota.bytes_written, (unsigned)actual_crc);
    return 0;
}

void ota_abort(void)
{
    /* Clear target slot header to mark it invalid */
    uint8_t blank[128];
    memset(blank, 0xFF, sizeof(blank));
    Flash_WriteData(g_ota.target_header, blank, sizeof(blank));

    g_ota.state = OTA_IDLE;
    g_ota.bytes_written = 0;
    g_ota.next_seq = 0;
    g_ota.write_buf_len = 0;
    printf("OTA aborted, target header cleared\r\n");
}

ota_state_t ota_get_state(void)
{
    return g_ota.state;
}

void ota_get_progress(uint32_t *written, uint32_t *total)
{
    *written = g_ota.bytes_written;
    *total = g_ota.expected_size;
}

void ota_mark_self_stable(void)
{
    /* Stage 2: app always lives at 0x08020000 / Bank1 Sector 7 header. */
    const uint32_t header_addr = OTA_APP_HEADER_ADDR;
    const uint32_t bank   = FLASH_BANK_1;
    const uint32_t sector = FLASH_SECTOR_7;

    const app_header_t *cur = (const app_header_t *)(uintptr_t)header_addr;
    if (cur->magic != APP_HEADER_MAGIC) {
        return;  /* No valid header (debugger flashed without post-build) */
    }
    if (cur->boot_count == 0) {
        return;  /* Already cleared */
    }

    app_header_t fresh;
    memcpy(&fresh, cur, sizeof(fresh));
    fresh.boot_count = 0;

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Banks        = bank,
        .Sector       = sector,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t err = 0;
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &err);
    HAL_FLASH_Lock();
    if (st != HAL_OK) {
        printf("BOOT: header erase failed sector=%u\r\n", (unsigned)err);
        return;
    }

    st = Flash_WriteData(header_addr, &fresh, sizeof(fresh));
    if (st != HAL_OK) {
        printf("BOOT: header write failed\r\n");
        return;
    }
    printf("BOOT: marked self stable (ver=%u)\r\n", (unsigned)fresh.version);
}
