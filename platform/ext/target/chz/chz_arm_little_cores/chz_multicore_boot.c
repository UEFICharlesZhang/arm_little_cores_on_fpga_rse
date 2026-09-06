/*
 * CHZ Arm Little Cores SoC — RSE (core0) multicore firmware distributor
 *
 * After TF-M platform init, core0 reads one 16-byte image header per slave
 * core from the SPI NOR flash and copies the described firmware into the
 * core's ITCM through its TCM slave window, then releases the cores from
 * CPUWAIT one at a time.
 *
 * Flash header layout (same 4x u32 little-endian shape as the BL2 header at
 * 0xFFC0, see romcode/main.c), at 0xE000 + (n-1)*0x10 for core n = 1..4:
 *   { magic 'MCC1'..'MCC4', src_off, dst, len }
 *     src_off — flash offset of the image (slot n @ 0x100000 + (n-1)*0x20000)
 *     dst     — must equal the core's ITCM window base (validated here)
 *     len     — image bytes, 4-byte multiple, <= 128 KB (ITCM size)
 * An absent/invalid header skips that core: TF-M on core0 still boots.
 *
 * Console output goes straight to the PL011 (stdio/printf is not linked
 * into the SPM; polled UART writes are the proven pattern on this SoC —
 * romcode and multicore_loader do the same).  Cores are released staggered
 * (1 s) because all five consoles currently share UART0 on this board (only
 * one UART pin pair exists).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <string.h>
#include "chz_multicore_boot.h"
#include "Driver_Flash.h"

/* defined in cmsis_drivers/Driver_Flash.c (no exporting header in target) */
extern ARM_DRIVER_FLASH Driver_FLASH0;

#define CHZ_CORE_FW_FIRST      1
#define CHZ_CORE_FW_LAST       4

/* flash offsets (see partition/flash_layout.h).  Headers live in their own
 * 4 KB sector (0xE000-0xEFFF) so burning them never touches the romcode
 * image-header sector (0xF000-0xFFFF holds the TFMS header at 0xFFC0). */
#define CHZ_FW_HDR_OFF(n)      (0xE000u + ((uint32_t)(n) - 1u) * 0x10u)
#define CHZ_FW_SLOT_OFF(n)     (0x100000u + ((uint32_t)(n) - 1u) * 0x20000u)
#define CHZ_FW_SLOT_SIZE       0x20000u               /* 128 KB = ITCM */
#define FLASH_TOTAL_SIZE_      0x400000u              /* 4 MB W25Q32JV */

/* core n ITCM slave window (hardware/memmap.h) */
#define CHZ_ITCM_WIN(n)        (0x60000000u + (uint32_t)(n) * 0x80000u)

/* SYSCTRL 寄存器块 (doc/SYSCTRL.md):
 *   WAIT  +0x00 [3:0] 1=保持核 n (复位值 0xF; 与旧 GPIO 同址同语义)
 *   RESET +0x04 [3:0] 1=复位核 n
 *   KEY   +0x0C 写 0x5A5A55A5 解锁 WAIT/RESET 写 */
#define CHZ_SYSCTRL_WAIT       0x40030000u
#define CHZ_SYSCTRL_KEY        0x4003000Cu

/* header magic 'MCC1'..'MCC4', little-endian uint32 (ASCII in flash:
 * bytes 4D 43 43 3n) */
#define CHZ_FW_MAGIC(n)        (0x43434Du | (((uint32_t)('0') + (uint32_t)(n)) << 24))

/* chunked flash->TCM copy buffer. Static: the S-image MSP stack is only
 * 2 KB in early boot (same reason Driver_Flash.c keeps statics). */
static uint8_t  fw_chunk[256];
static uint32_t fw_vec0_save[4];

/* ------------------------------------------------------------------ */
/* Polled PL011 console (UART0, already baud-configured by stdio_init) */
/* ------------------------------------------------------------------ */
#define CHZ_UART0_DR   0x40000000u
#define CHZ_UART0_FR   0x40000018u
#define CHZ_UART_FR_TXFF (1u << 5)

static void mcb_putc(char c)
{
    while (*(volatile uint32_t *)CHZ_UART0_FR & CHZ_UART_FR_TXFF) {
    }
    *(volatile uint32_t *)CHZ_UART0_DR = (uint32_t)(uint8_t)c;
}

static void mcb_puts(const char *s)
{
    while (*s) {
        mcb_putc(*s++);
    }
}

static void mcb_puthex32(uint32_t v)
{
    static const char hex[] = "0123456789abcdef";
    int i;
    mcb_puts("0x");
    for (i = 28; i >= 0; i -= 4) {
        mcb_putc(hex[(v >> i) & 0xFu]);
    }
}

static void mcb_putdec32(uint32_t v)
{
    char tmp[10];
    int i = 0;
    if (v == 0) {
        mcb_putc('0');
        return;
    }
    while (v) {
        tmp[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (i) {
        mcb_putc(tmp[--i]);
    }
}

/* ~1 ms at 100 MHz (~5 cycles per iteration), same calibration as
 * software/multicore_loader */
static void mcb_delay_ms(uint32_t ms)
{
    volatile uint32_t t;
    while (ms--) {
        for (t = 0; t < 20000u; t++) {
        }
    }
}

/* Probe the TCM window path before streaming the whole image: a broken
 * window (address collapsed to one spot) echoes the last written value on
 * every read — write a canary at +0x40 and read +0x44: if the canary
 * comes back the window is an echo, not a memory. */
static int mcb_window_ok(volatile uint32_t *win)
{
    uint32_t save44 = win[0x44 / 4];
    win[0x40 / 4] = 0xCAFEF00Du;
    __asm volatile("dsb" ::: "memory");
    uint32_t got = win[0x44 / 4];
    win[0x40 / 4] = 0x0;
    win[0x44 / 4] = save44;
    return got != 0xCAFEF00Du;
}

/* Copy one core's firmware from flash to its ITCM window.
 * Returns 1 when the core was loaded and should be released. */
static int mcb_load_core(int n)
{
    uint32_t hdr[4];
    volatile uint32_t *win;
    uint32_t off, words;
    int i;

    if (Driver_FLASH0.ReadData(CHZ_FW_HDR_OFF(n), hdr, sizeof(hdr))
            != (int32_t)sizeof(hdr)) {
        mcb_puts("[RSE] core");
        mcb_putdec32((uint32_t)n);
        mcb_puts(": header read failed\r\n");
        return 0;
    }
    if (hdr[0] != CHZ_FW_MAGIC(n)) {
        /* absent firmware is not an error on this core */
        return 0;
    }
    if (hdr[2] != CHZ_ITCM_WIN(n)) {
        mcb_puts("[RSE] core");
        mcb_putdec32((uint32_t)n);
        mcb_puts(": bad dst ");
        mcb_puthex32(hdr[2]);
        mcb_puts("\r\n");
        return 0;
    }
    if (hdr[3] == 0 || hdr[3] > CHZ_FW_SLOT_SIZE ||
            (hdr[3] & 3u) || hdr[1] < CHZ_FW_SLOT_OFF(n) ||
            hdr[1] + hdr[3] > CHZ_FW_SLOT_OFF(n) + CHZ_FW_SLOT_SIZE) {
        mcb_puts("[RSE] core");
        mcb_putdec32((uint32_t)n);
        mcb_puts(": bad len/src (len=");
        mcb_puthex32(hdr[3]);
        mcb_puts(" src=");
        mcb_puthex32(hdr[1]);
        mcb_puts(")\r\n");
        return 0;
    }

    win = (volatile uint32_t *)hdr[2];

    if (!mcb_window_ok(win)) {
        /* Window path unusable (see BRINGUP 2026-09-05): the core's ITCM
         * holds a self-load ROM from the bitstream — releasing CPUWAIT is
         * enough, it pulls its own firmware from flash. */
        mcb_puts("[RSE] core");
        mcb_putdec32((uint32_t)n);
        mcb_puts(": window broken, releasing for self-load\r\n");
        return 1;
    }

    mcb_puts("[RSE] core");
    mcb_putdec32((uint32_t)n);
    mcb_puts(": loading ");
    mcb_putdec32(hdr[3]);
    mcb_puts(" B ");
    mcb_puthex32(hdr[1]);
    mcb_puts(" -> ");
    mcb_puthex32(hdr[2]);
    mcb_puts(" ...\r\n");
    for (off = 0; off < hdr[3]; off += sizeof(fw_chunk)) {
        uint32_t chunk = hdr[3] - off;
        if (chunk > sizeof(fw_chunk)) {
            chunk = sizeof(fw_chunk);
        }
        if (Driver_FLASH0.ReadData(hdr[1] + off, fw_chunk, chunk)
                != (int32_t)chunk) {
            mcb_puts("[RSE] core");
            mcb_putdec32((uint32_t)n);
            mcb_puts(": flash read failed at ");
            mcb_puthex32(hdr[1] + off);
            mcb_puts("\r\n");
            return 0;
        }
        words = chunk / 4u;
        for (i = 0; i < (int)words; i++) {
            win[(off >> 2) + i] =
                ((uint32_t)fw_chunk[i * 4]) |
                ((uint32_t)fw_chunk[i * 4 + 1] << 8) |
                ((uint32_t)fw_chunk[i * 4 + 2] << 16) |
                ((uint32_t)fw_chunk[i * 4 + 3] << 24);
        }
        if (off == 0) {
            /* remember the FLASH-side vector words: the readback check must
             * compare the window against what we INTENDED to write, not
             * against a previous window read (an echo-style broken window
             * would otherwise verify against itself) */
            for (i = 0; i < 4; i++) {
                fw_vec0_save[i] =
                    ((uint32_t)fw_chunk[i * 4]) |
                    ((uint32_t)fw_chunk[i * 4 + 1] << 8) |
                    ((uint32_t)fw_chunk[i * 4 + 2] << 16) |
                    ((uint32_t)fw_chunk[i * 4 + 3] << 24);
            }
        }
    }
    __asm volatile("dsb" ::: "memory");

    /* window sanity: vector words read back as written */
    for (i = 0; i < 4; i++) {
        if (win[i] != fw_vec0_save[i]) {
            mcb_puts("[RSE] core");
            mcb_putdec32((uint32_t)n);
            mcb_puts(": window verify FAIL @");
            mcb_putdec32((uint32_t)i);
            mcb_puts(" — releasing for self-load\r\n");
            return 1;
        }
    }
    mcb_puts("[RSE] core");
    mcb_putdec32((uint32_t)n);
    mcb_puts(": loaded, vec SP=");
    mcb_puthex32(fw_vec0_save[0]);
    mcb_puts(" PC=");
    mcb_puthex32(fw_vec0_save[1]);
    mcb_puts("\r\n");
    return 1;
}

void chz_multicore_boot(void)
{
    int n, released = 0, loaded[5] = {0, 0, 0, 0, 0};
    uint32_t mask;

    if (Driver_FLASH0.Initialize(NULL) != ARM_DRIVER_OK ||
            Driver_FLASH0.PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK) {
        mcb_puts("[RSE] multicore: flash driver init failed\r\n");
        return;
    }

    mcb_puts("\r\n[RSE] multicore boot: distributing core1-4 firmware\r\n");

    /* unlock SYSCTRL WAIT/RESET writes (KEY, before touching WAIT) */
    *(volatile uint32_t *)CHZ_SYSCTRL_KEY = 0x5A5A55A5u;

    /* make sure every slave core is held while we write their ITCM */
    *(volatile uint32_t *)CHZ_SYSCTRL_WAIT = 0xFu;
    mcb_delay_ms(10);

    for (n = CHZ_CORE_FW_FIRST; n <= CHZ_CORE_FW_LAST; n++) {
        loaded[n] = mcb_load_core(n);
    }

    /* release one by one: all consoles share UART0 during bring-up, the
     * stagger keeps the SCP boot banners legible */
    mask = 0xFu;
    for (n = CHZ_CORE_FW_FIRST; n <= CHZ_CORE_FW_LAST; n++) {
        if (!loaded[n]) {
            continue;
        }
        mask &= ~(1u << (n - 1));
        mcb_puts("[RSE] core");
        mcb_putdec32((uint32_t)n);
        mcb_puts(": CPUWAIT released (gpio=");
        mcb_puthex32(mask);
        mcb_puts("), banner next\r\n");
        *(volatile uint32_t *)CHZ_SYSCTRL_WAIT = mask;
        released++;
        mcb_delay_ms(1000);
    }

    mcb_puts("[RSE] multicore boot done (");
    mcb_putdec32((uint32_t)released);
    mcb_puts(" cores released)\r\n");
}
