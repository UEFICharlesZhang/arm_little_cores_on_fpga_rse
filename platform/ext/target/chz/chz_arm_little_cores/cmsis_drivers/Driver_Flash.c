/*
 * CHZ Arm Little Cores SoC - SPI NOR flash driver (AXI Quad SPI)
 *
 * Replaces the an521 SRAM-emulated flash driver with a real driver for a SPI
 * NOR flash behind a Xilinx AXI Quad SPI controller (PG153).
 *
 * Read:  memory-mapped XIP window (FLASH0_BASE_S = 0x1000_0000).
 * Erase/Program: AXI Quad SPI legacy register interface (SPI NOR commands).
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
#define ARM_FLASH_DRV_VERSION      ARM_DRIVER_VERSION_MAJOR_MINOR(1, 1)
#define ARM_FLASH_DRV_ERASE_VALUE  0xFF

/* ------------------------------------------------------------------ */
/* AXI Quad SPI (Xilinx PG153) register map                           */
/* ------------------------------------------------------------------ */
#define AXI_QSPI_BASE          0x40060000UL

#define AXI_QSPI_SRR           (AXI_QSPI_BASE + 0x40)  /* Software Reset       */
#define AXI_QSPI_SPICR         (AXI_QSPI_BASE + 0x60)  /* SPI Control          */
#define AXI_QSPI_SPISR         (AXI_QSPI_BASE + 0x64)  /* SPI Status           */
#define AXI_QSPI_SPI_DTR       (AXI_QSPI_BASE + 0x68)  /* Data Transmit (FIFO) */
#define AXI_QSPI_SPI_DRR       (AXI_QSPI_BASE + 0x6C)  /* Data Receive  (FIFO) */
#define AXI_QSPI_SPISSR        (AXI_QSPI_BASE + 0x70)  /* Slave Select         */
#define AXI_QSPI_TX_FIFO_OCY   (AXI_QSPI_BASE + 0x74)  /* TX FIFO Occupancy    */
#define AXI_QSPI_RX_FIFO_OCY   (AXI_QSPI_BASE + 0x78)  /* RX FIFO Occupancy    */

/* SPICR bit fields (PG153, [TODO: 核对] 具体位定义) */
#define SPICR_SPE            (1u << 0)   /* SPI Enable                    */
#define SPICR_MASTER         (1u << 1)   /* Master mode                   */
#define SPICR_CPOL           (1u << 2)
#define SPICR_CPHA           (1u << 3)
#define SPICR_TX_FIFO_RST    (1u << 4)
#define SPICR_RX_FIFO_RST    (1u << 5)
#define SPICR_MTI            (1u << 6)   /* Master Transaction Inhibit   */
/* [TODO: 核对] XIP 模式位 (PG153); 若 IP 支持 XIP 与 legacy 并存则无需切换 */
#define SPICR_XIP_MODE       (1u << 23)

/* SPISR bit fields (PG153, [TODO: 核对]) */
#define SPISR_RX_EMPTY       (1u << 0)
#define SPISR_RX_FULL        (1u << 1)
#define SPISR_TX_EMPTY       (1u << 2)
#define SPISR_TX_FULL        (1u << 3)

/* SRR software reset value */
#define SRR_SW_RESET         0x0AU

/* ------------------------------------------------------------------ */
/* SPI NOR flash commands (Winbond/ISSI/Micron compatible)            */
/* ------------------------------------------------------------------ */
#define CMD_WREN             0x06U
#define CMD_WRDI             0x04U
#define CMD_RDSR1            0x05U
#define CMD_PAGE_PROGRAM     0x02U
#define CMD_READ             0x03U
#define CMD_SECTOR_ERASE     0x20U  /* 4 KB sector */
#define CMD_CHIP_ERASE       0xC7U

#define FLASH_WIP_MASK       0x01U  /* Write In Progress, RDSR1 bit 0 */

/* ------------------------------------------------------------------ */
/* Low-level AXI Quad SPI accessors                                    */
/* ------------------------------------------------------------------ */
static volatile uint32_t *qspi_reg(uint32_t addr)
{
    return (volatile uint32_t *)addr;
}

static void qspi_wr(uint32_t addr, uint32_t val)
{
    *qspi_reg(addr) = val;
}

static uint32_t qspi_rd(uint32_t addr)
{
    return *qspi_reg(addr);
}

static void qspi_tx_byte(uint8_t b)
{
    qspi_wr(AXI_QSPI_SPI_DTR, b);
    while (!(qspi_rd(AXI_QSPI_SPISR) & SPISR_TX_EMPTY)) {
    }
}

static uint8_t qspi_xfer_byte(uint8_t b)
{
    qspi_tx_byte(b);
    while (qspi_rd(AXI_QSPI_SPISR) & SPISR_RX_EMPTY) {
    }
    return (uint8_t)qspi_rd(AXI_QSPI_SPI_DRR);
}

static void qspi_send(const uint8_t *buf, uint32_t len)
{
    while (len--) {
        qspi_tx_byte(*buf++);
    }
}

static void qspi_xip_enable(void)
{
    qspi_wr(AXI_QSPI_SPICR, qspi_rd(AXI_QSPI_SPICR) | SPICR_XIP_MODE);
}

static void qspi_xip_disable(void)
{
    qspi_wr(AXI_QSPI_SPICR, qspi_rd(AXI_QSPI_SPICR) & ~SPICR_XIP_MODE);
}

/* ------------------------------------------------------------------ */
/* SPI NOR flash operations                                            */
/* ------------------------------------------------------------------ */
static void flash_write_enable(void)
{
    uint8_t cmd = CMD_WREN;
    qspi_send(&cmd, 1);
}

static uint8_t flash_read_status(void)
{
    qspi_xfer_byte(CMD_RDSR1);
    return qspi_xfer_byte(0x00);
}

static void flash_wait_wip_clear(void)
{
    while (flash_read_status() & FLASH_WIP_MASK) {
    }
}

static void flash_sector_erase(uint32_t addr)
{
    uint8_t cmd[4] = {
        CMD_SECTOR_ERASE,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)(addr),
    };

    qspi_xip_disable();
    flash_write_enable();
    qspi_send(cmd, sizeof(cmd));
    flash_wait_wip_clear();
    qspi_xip_enable();
}

static void flash_page_program(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint8_t cmd[4] = {
        CMD_PAGE_PROGRAM,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)(addr),
    };

    qspi_xip_disable();
    flash_write_enable();
    qspi_send(cmd, sizeof(cmd));
    qspi_send(data, len);
    flash_wait_wip_clear();
    qspi_xip_enable();
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

    /* Software reset the AXI Quad SPI core */
    qspi_wr(AXI_QSPI_SRR, SRR_SW_RESET);

    /* Enable SPI in master mode; deassert all slave selects */
    qspi_wr(AXI_QSPI_SPICR,
            SPICR_SPE | SPICR_MASTER | SPICR_TX_FIFO_RST | SPICR_RX_FIFO_RST);
    qspi_wr(AXI_QSPI_SPISSR, 0xFFFFFFFFU);

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
    uint32_t start_addr = FLASH0_DEV->memory_base + addr;
    int32_t rc = 0;

    if (addr % data_width_byte[DriverCapabilities.data_width] != 0) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    cnt *= data_width_byte[DriverCapabilities.data_width];

    rc = is_range_valid(FLASH0_DEV, addr + cnt);
    if (rc != 0) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* XIP: read directly from the memory-mapped SPI flash window */
    memcpy(data, (void *)start_addr, cnt);

    cnt /= data_width_byte[DriverCapabilities.data_width];

    return cnt;
}

static int32_t ARM_Flash_ProgramData(uint32_t addr, const void *data,
                                     uint32_t cnt)
{
    int32_t rc = 0;

    cnt *= data_width_byte[DriverCapabilities.data_width];

    rc  = is_range_valid(FLASH0_DEV, addr + cnt);
    rc |= is_write_aligned(FLASH0_DEV, addr);
    rc |= is_write_aligned(FLASH0_DEV, cnt);
    if (rc != 0) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    flash_page_program(addr, (const uint8_t *)data, cnt);

    cnt /= data_width_byte[DriverCapabilities.data_width];

    return cnt;
}

static int32_t ARM_Flash_EraseSector(uint32_t addr)
{
    int32_t rc = 0;

    rc  = is_range_valid(FLASH0_DEV, addr);
    rc |= is_sector_aligned(FLASH0_DEV, addr);
    if (rc != 0) {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    flash_sector_erase(addr);

    return ARM_DRIVER_OK;
}

static int32_t ARM_Flash_EraseChip(void)
{
    uint8_t cmd = CMD_CHIP_ERASE;

    if (DriverCapabilities.erase_chip != 1) {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    qspi_xip_disable();
    flash_write_enable();
    qspi_send(&cmd, 1);
    flash_wait_wip_clear();
    qspi_xip_enable();

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
