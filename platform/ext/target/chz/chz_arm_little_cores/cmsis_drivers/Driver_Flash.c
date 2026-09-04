/*
 * CHZ Arm Little Cores SoC - SPI NOR flash driver (AXI Quad SPI)
 *
 * Replaces the an521 SRAM-emulated flash driver with a real driver for a SPI
 * NOR flash behind a Xilinx AXI Quad SPI controller.
 *
 * The XIP memory window (FLASH0_BASE_S = 0x1000_0000) was removed from the
 * hardware (2026-09-03): C_XIP_MODE=0, standard mode only.  All reads go
 * over the register interface with the 0x03 read command; erase/program
 * use the standard SPI NOR commands.
 *
 * Register map (v3.2, decoded from axi_quad_spi_v3_2_rfs.vhd — the old
 * PG153/DS558 bit assumptions were wrong for this IP version):
 *   SPICR 0x60 (rst 0x180): bit1 SPE, bit2 MASTER, bit5/6 TX/RX FIFO rst
 *       (self-clearing), bit7 Manual_SS, bit8 TR_INHIBIT
 *   SPISR 0x64 (rst 0xA5):  bit0 RX_EMPTY, bit2 TX_EMPTY
 *   SPISSR 0x70: bit0 = CS (active low, manual mode)
 *
 * Transfer model (verified on hardware via software/spi_test and
 * software/flash_prog):
 *   - CS asserted/deasserted by SPISSR (manual mode) around a transaction
 *   - every TX byte is followed by a ~25 us pause, then its RX echo is
 *     read.  Two quirks forced this: (a) a DTR write landing while a
 *     transfer is in flight is silently dropped, and (b) a tight per-byte
 *     interleave (write, poll echo immediately, ~1-2 us/byte) misaligns
 *     the RX echo by one byte on some code layouts (JEDEC comes back
 *     00 00 EF 40 instead of 00 EF 40 16); >= 25 us between bytes is
 *     reliably aligned.  The pause also keeps the 16-deep RX FIFO from
 *     filling up mid-transfer (a full RX FIFO wedges the IP: SCK never
 *     resumes).
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdint.h>
#include "Driver_Flash.h"
#include "platform_retarget.h"
#include "RTE_Device.h"

#ifndef ARG_UNUSED
#define ARG_UNUSED(arg)  ((void)arg)
#endif

/* Driver version */
#define ARM_FLASH_DRV_VERSION      ARM_DRIVER_VERSION_MAJOR_MINOR(1, 2)
#define ARM_FLASH_DRV_ERASE_VALUE  0xFF

/* ------------------------------------------------------------------ */
/* AXI Quad SPI (Xilinx PG153 / v3.2 RFS) register map                 */
/* ------------------------------------------------------------------ */
#define AXI_QSPI_BASE          0x40060000UL

#define AXI_QSPI_SRR           (AXI_QSPI_BASE + 0x40)  /* Software Reset    */
#define AXI_QSPI_SPICR         (AXI_QSPI_BASE + 0x60)  /* SPI Control       */
#define AXI_QSPI_SPISR         (AXI_QSPI_BASE + 0x64)  /* SPI Status        */
#define AXI_QSPI_SPI_DTR       (AXI_QSPI_BASE + 0x68)  /* Data Transmit FIFO*/
#define AXI_QSPI_SPI_DRR       (AXI_QSPI_BASE + 0x6C)  /* Data Receive FIFO */
#define AXI_QSPI_SPISSR        (AXI_QSPI_BASE + 0x70)  /* Slave Select      */

/* SPICR (v3.2): bit0 LOOP, bit1 SPE, bit2 MASTER, bit3 CPOL, bit4 CPHA,
 * bit5 TX_FIFO_RST / bit6 RX_FIFO_RST (self-clearing), bit7 Manual_SS,
 * bit8 TR_INHIBIT (MTI), bit9 LSB-first */
#define SPICR_SPE            (1u << 1)
#define SPICR_MASTER         (1u << 2)
#define SPICR_CPOL           (1u << 3)
#define SPICR_CPHA           (1u << 4)
#define SPICR_TX_FIFO_RST    (1u << 5)
#define SPICR_RX_FIFO_RST    (1u << 6)
#define SPICR_MANUAL_SS      (1u << 7)
#define SPICR_TR_INHIBIT     (1u << 8)

/* SPISR (v3.2): bit0 RX_EMPTY, bit1 RX_FULL, bit2 TX_EMPTY, bit3 TX_FULL */
#define SPISR_RX_EMPTY       (1u << 0)
#define SPISR_RX_FULL        (1u << 1)
#define SPISR_TX_EMPTY       (1u << 2)
#define SPISR_TX_FULL        (1u << 3)

/* SRR software reset value */
#define SRR_SW_RESET         0x0AU

/* Busy-wait bounds (loops of an MMIO read + branch).  RX echo of one
 * byte at 12.5 MHz SCK takes ~640 ns; these bounds are generous. */
#define QSPI_XFER_TIMEOUT    40000u
#define QSPI_IDLE_TIMEOUT    100000u

/* ------------------------------------------------------------------ */
/* SPI NOR flash commands (Winbond/ISSI/Micron compatible)            */
/* ------------------------------------------------------------------ */
#define CMD_WREN             0x06U
#define CMD_WRDI             0x04U
#define CMD_RDSR1            0x05U
#define CMD_PAGE_PROGRAM     0x02U
#define CMD_READ             0x03U
#define CMD_SECTOR_ERASE     0x20U  /* 4 KB sector */
#define CMD_BLOCK_ERASE      0xD8U  /* 64 KB block */
#define CMD_CHIP_ERASE       0xC7U

#define FLASH_WIP_MASK       0x01U  /* Write In Progress, RDSR1 bit 0 */

/* ------------------------------------------------------------------ */
/* Low-level AXI Quad SPI accessors                                    */
/* ------------------------------------------------------------------ */
static void qspi_delay(uint32_t n)
{
    volatile uint32_t i;
    for (i = 0; i < n; i++) {
    }
}

static void qspi_wr(uint32_t addr, uint32_t val)
{
    *(volatile uint32_t *)addr = val;
}

static uint32_t qspi_rd(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

/*
 * One SPI transaction: CS low, n bytes out, n bytes in (full duplex),
 * CS high.  Every TX byte is followed by a ~25 us pause before its RX
 * echo is collected (see header comment for the two quirks this paces
 * around).  Returns 0 on success, -1 on timeout.
 */
static int32_t qspi_xfer(const uint8_t *tx, uint8_t *rx, uint32_t n)
{
    uint32_t i;

    qspi_wr(AXI_QSPI_SPISSR, 0x0);       /* assert CS */
    qspi_delay(5000);
    for (i = 0; i < n; i++) {
        uint32_t t = 0;
        qspi_wr(AXI_QSPI_SPI_DTR, tx[i]);
        qspi_delay(5000);                /* ~25 us: see header comment */
        do {
        } while ((qspi_rd(AXI_QSPI_SPISR) & SPISR_RX_EMPTY) &&
                 ++t < QSPI_XFER_TIMEOUT);
        if (t >= QSPI_XFER_TIMEOUT) {
            qspi_wr(AXI_QSPI_SPISSR, 0x1);
            return -1;
        }
        if (rx != NULL) {
            rx[i] = (uint8_t)qspi_rd(AXI_QSPI_SPI_DRR);
        } else {
            (void)qspi_rd(AXI_QSPI_SPI_DRR);
        }
    }
    qspi_wr(AXI_QSPI_SPISSR, 0x1);       /* deassert CS */
    qspi_delay(500);
    return 0;
}

/* Read len bytes from flash offset addr (0x03 command, no dummy bytes). */
static int32_t flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t tx[4 + 256];
    uint8_t rx[4 + 256];
    uint32_t chunk;

    while (len > 0) {
        chunk = (len > 256) ? 256 : len;
        tx[0] = CMD_READ;
        tx[1] = (uint8_t)(addr >> 16);
        tx[2] = (uint8_t)(addr >> 8);
        tx[3] = (uint8_t)addr;
        memset(&tx[4], 0x00, chunk);     /* dummy clocks for the data */
        if (qspi_xfer(tx, rx, 4 + chunk) != 0) {
            return -1;
        }
        memcpy(buf, &rx[4], chunk);
        addr += chunk;
        buf += chunk;
        len -= chunk;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* SPI NOR flash operations                                            */
/* ------------------------------------------------------------------ */
static int32_t flash_write_enable(void)
{
    uint8_t cmd = CMD_WREN;

    return qspi_xfer(&cmd, NULL, 1);
}

static int32_t flash_read_status(uint8_t *sr)
{
    uint8_t tx[2] = { CMD_RDSR1, 0x00 };
    uint8_t rx[2];

    if (qspi_xfer(tx, rx, 2) != 0) {
        return -1;
    }
    *sr = rx[1];                         /* 1st byte after the command */
    return 0;
}

static int32_t flash_wait_wip_clear(void)
{
    uint32_t t;
    uint8_t sr;

    for (t = 0; t < QSPI_IDLE_TIMEOUT; t++) {
        if (flash_read_status(&sr) != 0) {
            return -1;
        }
        if (!(sr & FLASH_WIP_MASK)) {
            return 0;
        }
        qspi_delay(10000);               /* ~50 us */
    }
    return -1;
}

static int32_t flash_sector_erase(uint32_t addr)
{
    uint8_t cmd[4] = {
        CMD_SECTOR_ERASE,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)(addr),
    };

    if (flash_write_enable() != 0) {
        return -1;
    }
    if (qspi_xfer(cmd, NULL, sizeof(cmd)) != 0) {
        return -1;
    }
    return flash_wait_wip_clear();
}

static int32_t flash_chip_erase(void)
{
    uint8_t cmd = CMD_CHIP_ERASE;

    if (flash_write_enable() != 0) {
        return -1;
    }
    if (qspi_xfer(&cmd, NULL, 1) != 0) {
        return -1;
    }
    return flash_wait_wip_clear();
}

static int32_t flash_page_program(uint32_t addr, const uint8_t *data,
                                  uint32_t len)
{
    uint8_t cmd[4 + 256];
    uint32_t chunk;

    while (len > 0) {
        chunk = (len > 256) ? 256 : len;
        cmd[0] = CMD_PAGE_PROGRAM;
        cmd[1] = (uint8_t)(addr >> 16);
        cmd[2] = (uint8_t)(addr >> 8);
        cmd[3] = (uint8_t)addr;
        memcpy(&cmd[4], data, chunk);
        if (flash_write_enable() != 0) {
            return -1;
        }
        if (qspi_xfer(cmd, NULL, 4 + chunk) != 0) {
            return -1;
        }
        if (flash_wait_wip_clear() != 0) {
            return -1;
        }
        addr += chunk;
        data += chunk;
        len -= chunk;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* CMSIS ARM_DRIVER_FLASH                                             */
/* ------------------------------------------------------------------ */
enum {
    DATA_WIDTH_8BIT   = 0u,
    DATA_WIDTH_16BIT,
    DATA_WIDTH_32BIT,
    DATA_WIDTH_ENUM_SIZE
};

static const uint32_t data_width_byte[DATA_WIDTH_ENUM_SIZE] = {
    sizeof(uint8_t),
    sizeof(uint16_t),
    sizeof(uint32_t),
};

struct arm_flash_dev_t {
    const uint32_t memory_base;
    ARM_FLASH_INFO *data;
};

static ARM_FLASH_STATUS FlashStatus = { 0, 0, 0 };

static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_FLASH_API_VERSION,
    ARM_FLASH_DRV_VERSION
};

static const ARM_FLASH_CAPABILITIES DriverCapabilities = {
    0, /* event_ready */
    0, /* data_width = 8-bit */
    1  /* erase_chip */
};

static int32_t is_range_valid(struct arm_flash_dev_t *flash_dev, uint32_t offset)
{
    uint32_t flash_limit =
        (flash_dev->data->sector_count * flash_dev->data->sector_size) - 1;

    if (offset > flash_limit) {
        return -1;
    }
    return 0;
}

static int32_t is_write_aligned(struct arm_flash_dev_t *flash_dev,
                                uint32_t param)
{
    if ((param % flash_dev->data->program_unit) != 0) {
        return -1;
    }
    return 0;
}

static int32_t is_sector_aligned(struct arm_flash_dev_t *flash_dev,
                                 uint32_t offset)
{
    if ((offset % flash_dev->data->sector_size) != 0) {
        return -1;
    }
    return 0;
}

#if (RTE_FLASH0)
static ARM_FLASH_INFO ARM_FLASH0_DEV_DATA = {
    .sector_info  = NULL,
    .sector_count = FLASH0_SIZE / FLASH0_SECTOR_SIZE,
    .sector_size  = FLASH0_SECTOR_SIZE,
    .page_size    = FLASH0_PAGE_SIZE,
    .program_unit = FLASH0_PROGRAM_UNIT,
    .erased_value = ARM_FLASH_DRV_ERASE_VALUE,
};

static struct arm_flash_dev_t ARM_FLASH0_DEV = {
#if (__DOMAIN_NS == 1)
    .memory_base = FLASH0_BASE_NS,
#else
    .memory_base = FLASH0_BASE_S,
#endif
    .data = &(ARM_FLASH0_DEV_DATA),
};

struct arm_flash_dev_t *FLASH0_DEV = &ARM_FLASH0_DEV;

static ARM_DRIVER_VERSION ARM_Flash_GetVersion(void)
{
    return DriverVersion;
}

static ARM_FLASH_CAPABILITIES ARM_Flash_GetCapabilities(void)
{
    return DriverCapabilities;
}

static int32_t ARM_Flash_Initialize(ARM_Flash_SignalEvent_t cb_event)
{
    ARG_UNUSED(cb_event);

    if (FLASH0_PROGRAM_UNIT % data_width_byte[DriverCapabilities.data_width] ||
        DriverCapabilities.data_width >= DATA_WIDTH_ENUM_SIZE) {
        return ARM_DRIVER_ERROR;
    }

    /* Software reset the AXI Quad SPI core, then enable standard-mode
     * master with manual CS (SPE|MASTER|ManualSS, MTI=0). */
    qspi_wr(AXI_QSPI_SRR, SRR_SW_RESET);
    qspi_delay(1000);
    qspi_wr(AXI_QSPI_SPICR,
            SPICR_SPE | SPICR_MASTER | SPICR_MANUAL_SS);
    qspi_wr(AXI_QSPI_SPISSR, 0x1);       /* CS deasserted */
    qspi_delay(1000);

    return ARM_DRIVER_OK;
}

static int32_t ARM_Flash_Uninitialize(void)
{
    return ARM_DRIVER_OK;
}

static int32_t ARM_Flash_PowerControl(ARM_POWER_STATE state)
{
    switch (state) {
    case ARM_POWER_FULL:
        return ARM_DRIVER_OK;
    case ARM_POWER_OFF:
    case ARM_POWER_LOW:
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
}

static int32_t ARM_Flash_ReadData(uint32_t addr, void *data, uint32_t cnt)
{
    int32_t rc;

    if (addr % data_width_byte[DriverCapabilities.data_width] != 0) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    cnt *= data_width_byte[DriverCapabilities.data_width];

    rc = is_range_valid(FLASH0_DEV, addr + cnt);
    if (rc != 0) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (flash_read(addr, (uint8_t *)data, cnt) != 0) {
        return ARM_DRIVER_ERROR;
    }

    cnt /= data_width_byte[DriverCapabilities.data_width];

    return cnt;
}

static int32_t ARM_Flash_ProgramData(uint32_t addr, const void *data,
                                     uint32_t cnt)
{
    int32_t rc;

    cnt *= data_width_byte[DriverCapabilities.data_width];

    rc  = is_range_valid(FLASH0_DEV, addr + cnt);
    rc |= is_write_aligned(FLASH0_DEV, addr);
    rc |= is_write_aligned(FLASH0_DEV, cnt);
    if (rc != 0) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (flash_page_program(addr, (const uint8_t *)data, cnt) != 0) {
        return ARM_DRIVER_ERROR;
    }

    cnt /= data_width_byte[DriverCapabilities.data_width];

    return cnt;
}

static int32_t ARM_Flash_EraseSector(uint32_t addr)
{
    int32_t rc;

    rc  = is_range_valid(FLASH0_DEV, addr);
    rc |= is_sector_aligned(FLASH0_DEV, addr);
    if (rc != 0) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (flash_sector_erase(addr) != 0) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

static int32_t ARM_Flash_EraseChip(void)
{
    if (DriverCapabilities.erase_chip != 1) {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    if (flash_chip_erase() != 0) {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

static ARM_FLASH_STATUS ARM_Flash_GetStatus(void)
{
    return FlashStatus;
}

static ARM_FLASH_INFO * ARM_Flash_GetInfo(void)
{
    return FLASH0_DEV->data;
}

ARM_DRIVER_FLASH Driver_FLASH0 = {
    ARM_Flash_GetVersion,
    ARM_Flash_GetCapabilities,
    ARM_Flash_Initialize,
    ARM_Flash_Uninitialize,
    ARM_Flash_PowerControl,
    ARM_Flash_ReadData,
    ARM_Flash_ProgramData,
    ARM_Flash_EraseSector,
    ARM_Flash_EraseChip,
    ARM_Flash_GetStatus,
    ARM_Flash_GetInfo
};
#endif /* RTE_FLASH0 */
