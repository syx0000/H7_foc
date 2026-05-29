/**
 * @file    boot_main.c
 * @brief   Stage 2 OTA bootloader (single-slot + staging copy).
 *
 * Layout:
 *   0x08000000  Bootloader      128KB  (this code)
 *   0x08020000  App             768KB  (the only execution slot)
 *   0x080E0000  App header      128KB  (Bank1 Sector 7)
 *   0x08100000  Staging         768KB  (Bank2 Sector 0~5, OTA receive area)
 *   0x081C0000  Staging header  128KB  (Bank2 Sector 6)
 *   0x081E0000  FOC params      128KB  (untouched)
 *
 * Boot flow:
 *   1. If staging header is valid + PENDING + CRC ok:
 *        erase App + copy staging → App + write App header + clear staging.
 *      If staging is PENDING but CRC fails: clear staging, fall through.
 *   2. If App header is valid + boot_count < 3: count++ then jump.
 *   3. Otherwise: dev fallback (sane vector table) or stuck loop.
 */
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define APP_HEADER_MAGIC        0x41434F46U   /* 'FOCA' */
#define APP_FLAG_VALID          0x01U
#define APP_FLAG_PENDING        0x02U
#define MAX_BOOT_COUNT          3U

#define APP_BASE                0x08020000U
#define APP_HEADER_BASE         0x080E0000U
#define APP_SIZE_MAX            (768 * 1024)

#define STAGING_BASE            0x08100000U
#define STAGING_HEADER_BASE     0x081C0000U

#define APP_BANK                FLASH_BANK_1
#define APP_FIRST_SECTOR        FLASH_SECTOR_1   /* App: Sector 1~6 */
#define APP_NUM_SECTORS         6U
#define APP_HEADER_SECTOR       FLASH_SECTOR_7

#define STAGING_BANK            FLASH_BANK_2
#define STAGING_FIRST_SECTOR    FLASH_SECTOR_0   /* Staging: Sector 0~5 */
#define STAGING_NUM_SECTORS     6U
#define STAGING_HEADER_SECTOR   FLASH_SECTOR_6

#define FLASH_WORD_BYTES        32U   /* H7 256-bit programming word */

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t app_size;
    uint32_t app_crc32;
    uint32_t boot_count;
    uint32_t flags;
    uint32_t build_time;
    uint32_t reserved[25];
} app_header_t;

extern void boot_uart_init(void);
extern void boot_clock_init(void);
extern uint32_t boot_crc32(const void *data, uint32_t len);

static int header_valid(const app_header_t *h, uint32_t base)
{
    if (h->magic != APP_HEADER_MAGIC)             return 0;
    if (!(h->flags & APP_FLAG_VALID))             return 0;
    if (h->app_size == 0 || h->app_size > APP_SIZE_MAX) return 0;
    uint32_t crc = boot_crc32((const void *)(uintptr_t)base, h->app_size);
    return (crc == h->app_crc32);
}

static HAL_StatusTypeDef erase_sectors(uint32_t bank, uint32_t first, uint32_t n)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef e = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Banks        = bank,
        .Sector       = first,
        .NbSectors    = n,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t err = 0;
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &err);
    HAL_FLASH_Lock();
    return st;
}

/* Program `len` bytes at `dst` from `src`. Both addresses must be 32-byte
   aligned; len is rounded up to FLASH_WORD_BYTES. The destination sector(s)
   must already be erased. */
static HAL_StatusTypeDef program_block(uint32_t dst, const void *src, uint32_t len)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_OK;
    const uint8_t *p = (const uint8_t *)src;
    uint32_t addr = dst;
    uint32_t remaining = len;
    uint8_t pad[FLASH_WORD_BYTES];

    while (remaining > 0) {
        const uint8_t *word_src;
        if (remaining >= FLASH_WORD_BYTES) {
            word_src = p;
        } else {
            memset(pad, 0xFF, FLASH_WORD_BYTES);
            memcpy(pad, p, remaining);
            word_src = pad;
        }
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr,
                               (uint32_t)(uintptr_t)word_src);
        if (st != HAL_OK) break;

        addr += FLASH_WORD_BYTES;
        if (remaining >= FLASH_WORD_BYTES) {
            p += FLASH_WORD_BYTES;
            remaining -= FLASH_WORD_BYTES;
        } else {
            remaining = 0;
        }
    }
    HAL_FLASH_Lock();
    return st;
}

static int try_apply_pending(void)
{
    const app_header_t *sh = (const app_header_t *)(uintptr_t)STAGING_HEADER_BASE;
    if (sh->magic != APP_HEADER_MAGIC)        return 0;
    if (!(sh->flags & APP_FLAG_PENDING))      return 0;

    /* Snapshot before we touch flash (header struct lives in flash). */
    app_header_t snap;
    memcpy(&snap, sh, sizeof(snap));

    printf("BOOT: pending update found ver=%u size=%u crc=0x%08X\r\n",
           (unsigned)snap.version, (unsigned)snap.app_size,
           (unsigned)snap.app_crc32);

    if (snap.app_size == 0 || snap.app_size > APP_SIZE_MAX) {
        printf("BOOT: staging size invalid, clearing\r\n");
        erase_sectors(STAGING_BANK, STAGING_HEADER_SECTOR, 1);
        return 0;
    }

    uint32_t crc = boot_crc32((const void *)(uintptr_t)STAGING_BASE, snap.app_size);
    if (crc != snap.app_crc32) {
        printf("BOOT: staging CRC mismatch (got 0x%08X), discarding\r\n",
               (unsigned)crc);
        erase_sectors(STAGING_BANK, STAGING_HEADER_SECTOR, 1);
        return 0;
    }

    /* Erase App slot (sectors 1~6 of Bank1) and the App header sector. */
    printf("BOOT: erasing App slot...\r\n");
    if (erase_sectors(APP_BANK, APP_FIRST_SECTOR, APP_NUM_SECTORS) != HAL_OK) {
        printf("BOOT: App erase failed\r\n");
        return 0;
    }
    if (erase_sectors(APP_BANK, APP_HEADER_SECTOR, 1) != HAL_OK) {
        printf("BOOT: App header erase failed\r\n");
        return 0;
    }

    /* Copy staging → App. We can read directly from staging Flash and
       program to App Flash because the two banks are independent. */
    printf("BOOT: copying %u bytes staging -> App...\r\n",
           (unsigned)snap.app_size);
    if (program_block(APP_BASE,
                      (const void *)(uintptr_t)STAGING_BASE,
                      snap.app_size) != HAL_OK) {
        printf("BOOT: copy failed\r\n");
        return 0;
    }

    /* Verify the freshly written App slot. */
    uint32_t app_crc = boot_crc32((const void *)(uintptr_t)APP_BASE, snap.app_size);
    if (app_crc != snap.app_crc32) {
        printf("BOOT: post-copy CRC mismatch 0x%08X vs 0x%08X\r\n",
               (unsigned)app_crc, (unsigned)snap.app_crc32);
        return 0;
    }

    /* Build App header (no PENDING flag, fresh boot_count). */
    app_header_t fresh;
    memset(&fresh, 0xFF, sizeof(fresh));
    fresh.magic     = APP_HEADER_MAGIC;
    fresh.version   = snap.version;
    fresh.app_size  = snap.app_size;
    fresh.app_crc32 = snap.app_crc32;
    fresh.boot_count = 0;
    fresh.flags     = APP_FLAG_VALID;
    fresh.build_time = snap.build_time;
    if (program_block(APP_HEADER_BASE, &fresh, sizeof(fresh)) != HAL_OK) {
        printf("BOOT: App header write failed\r\n");
        return 0;
    }

    /* Clear staging header so we don't apply it again next boot. */
    erase_sectors(STAGING_BANK, STAGING_HEADER_SECTOR, 1);

    printf("BOOT: update applied OK, ver=%u\r\n", (unsigned)snap.version);
    return 1;
}

static void (*s_app_entry)(void);

__attribute__((noreturn))
static void jump_to_app(uint32_t app_base)
{
    HAL_RCC_DeInit();
    HAL_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    for (int i = 0; i < 8; i++) NVIC->ICER[i] = 0xFFFFFFFFU;
    for (int i = 0; i < 8; i++) NVIC->ICPR[i] = 0xFFFFFFFFU;

    SCB->VTOR = app_base;
    __DSB(); __ISB();

    s_app_entry = (void (*)(void))(*(volatile uint32_t *)(app_base + 4U));
    __set_MSP(*(volatile uint32_t *)app_base);
    s_app_entry();
    while (1);
}

__attribute__((noreturn))
static void stuck_loop(const char *reason)
{
    while (1) {
        printf("BOOT: stuck (%s); flash a fresh image via DAP\r\n", reason);
        HAL_Delay(1000);
    }
}

int main(void)
{
    HAL_Init();
    boot_clock_init();
    boot_uart_init();

    printf("\r\n=== STM32H743 Stage 2 Bootloader ===\r\n");

    /* Step 1: apply pending update if present. */
    (void)try_apply_pending();

    /* Step 2: validate App slot and (maybe) jump. */
    const app_header_t *ah = (const app_header_t *)(uintptr_t)APP_HEADER_BASE;
    if (header_valid(ah, APP_BASE)) {
        if (ah->boot_count >= MAX_BOOT_COUNT) {
            printf("BOOT: App boot_count=%u >= %u (rollback needed but no other slot)\r\n",
                   (unsigned)ah->boot_count, MAX_BOOT_COUNT);
        }
        printf("BOOT: jumping to App ver=%u count=%u->%u\r\n",
               (unsigned)ah->version,
               (unsigned)ah->boot_count,
               (unsigned)(ah->boot_count + 1));

        app_header_t fresh;
        memcpy(&fresh, ah, sizeof(fresh));
        fresh.boot_count++;
        erase_sectors(APP_BANK, APP_HEADER_SECTOR, 1);
        program_block(APP_HEADER_BASE, &fresh, sizeof(fresh));

        HAL_Delay(20);
        jump_to_app(APP_BASE);
    }

    /* Step 3: dev fallback — App may have been flashed by debugger
       without a header. Check vector table looks sane. */
    uint32_t sp = *(volatile uint32_t *)(APP_BASE);
    uint32_t pc = *(volatile uint32_t *)(APP_BASE + 4U);
    if (sp >= 0x20000000U && sp <= 0x30000000U &&
        pc >= 0x08000000U && pc <  0x08200000U) {
        printf("BOOT: no App header, dev-mode jump (SP=0x%08X PC=0x%08X)\r\n",
               (unsigned)sp, (unsigned)pc);
        HAL_Delay(20);
        jump_to_app(APP_BASE);
    }

    printf("BOOT: App vector check failed SP=0x%08X PC=0x%08X\r\n",
           (unsigned)sp, (unsigned)pc);
    stuck_loop("no valid app");
}
