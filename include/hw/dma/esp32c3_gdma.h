/*
 * ESP32-C3 GDMA emulation
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/misc/esp32c3_reg.h"

#define TYPE_ESP32C3_GDMA "esp32c3.gdma"
#define ESP32C3_GDMA(obj) OBJECT_CHECK(ESP32C3GdmaState, (obj), TYPE_ESP32C3_GDMA)

#define ESP32C3_GDMA_REGS_SIZE (A_DMA_OUT_PERI_SEL_CH2 + 4)

#define ESP32C3_GDMA_CHANNEL_COUNT 3

#define ESP32C3_GDMA_IN_IDX     0
#define ESP32C3_GDMA_OUT_IDX    1
#define ESP32C3_GDMA_CONF_COUNT (ESP32C3_GDMA_OUT_IDX + 1)

#define ESP32C3_GDMA_RAM_ADDR   0x3FC80000


/**
 * @brief Number for each peripheral that can access GDMA
 */
typedef enum {
    GDMA_SPI2  = 0,
    GDMA_RSVD1 = 1,
    GDMA_UHCI0 = 2,
    GDMA_I2S   = 3,
    GDMA_RSVD4 = 4,
    GDMA_RSVD5 = 5,
    GDMA_AES   = 6,
    GDMA_SHA   = 7,
    GDMA_ADC   = 8,
    GDMA_LAST  = GDMA_ADC,
} GdmaPeripheral;


/**
 * Size of the interrupt registers, in bytes, for a single channel
 */
#define ESP32C3_GDMA_INT_REGS_SIZE  0x10

typedef struct {
    uint32_t raw;
    uint32_t st;
    uint32_t ena;
    /* Not really necessary to have this variable here as it will never contain
    * any data, but will simplify the code (offset calculation) */
    uint32_t clr;
} DmaIntState;


typedef struct {
    uint32_t conf0;
    uint32_t conf1;
    uint32_t status;
    uint32_t push_pop;
    uint32_t link;
    /* Status registers */
    uint32_t state;
    uint32_t suc_eof_desc_addr; // Address of descriptor when EOF bit is 1
    uint32_t err_eof_desc_addr; // Address of descriptor when error occurs (UHCI0 only)
    uint32_t desc_addr; // Address of the next descriptor (n + 1)
    uint32_t bfr_desc_addr; // Address of the current descriptor (n)
    uint32_t bfr_bfr_desc_addr; // Address of the previous descriptor (n - 1)
    uint32_t priority;
    uint32_t peripheral;
} DmaConfigState;


typedef struct ESP32C3GdmaState {
    SysBusDevice parent_object;
    MemoryRegion iomem;
    qemu_irq irq[ESP32C3_GDMA_CHANNEL_COUNT];

    DmaIntState ch_int[ESP32C3_GDMA_CHANNEL_COUNT];
    DmaConfigState ch_conf[ESP32C3_GDMA_CHANNEL_COUNT][ESP32C3_GDMA_CONF_COUNT];

    /* Use this register mainly for enabling and disabling priorities */
    uint32_t misc_conf;

    /* Keep a pointer to the SoC DRAM */
    MemoryRegion* soc_mr;
    AddressSpace dma_as;

} ESP32C3GdmaState;


/**
 * @brief Get the channel configured for the given peripheral
 *
 * @param s GDMA state
 * @param periph Peripheral to search
 * @param dir Direction from the GDMA point of view: ESP32C3_GDMA_IN_IDX or ESP32C3_GDMA_OUT_IDX.
 *            For example, to find a channel that needs to be written to, use ESP32C3_GDMA_IN_IDX
 *            (because GDMA receives the data)
 * @param chan Returned channel index linked to the peripheral
 *
 * @returns index of the GDMA channel bound to the peripheral, -1 if not found
 */
bool esp32c3_gdma_get_channel_periph(ESP32C3GdmaState *s, GdmaPeripheral periph, int dir,
                                     uint32_t* chan);

bool esp32c3_gdma_read_channel(ESP32C3GdmaState *s, uint32_t chan, uint8_t* buffer, uint32_t size);
bool esp32c3_gdma_write_channel(ESP32C3GdmaState *s, uint32_t chan, uint8_t* buffer, uint32_t size);

REG32(DMA_INT_RAW_CH0, 0x000)
    FIELD(DMA_INT_RAW_CH0, OUTFIFO_UDF_CH0_INT_RAW, 12, 1)
    FIELD(DMA_INT_RAW_CH0, OUTFIFO_OVF_CH0_INT_RAW, 11, 1)
    FIELD(DMA_INT_RAW_CH0, INFIFO_UDF_CH0_INT_RAW, 10, 1)
    FIELD(DMA_INT_RAW_CH0, INFIFO_OVF_CH0_INT_RAW, 9, 1)
    FIELD(DMA_INT_RAW_CH0, OUT_TOTAL_EOF_CH0_INT_RAW, 8, 1)
    FIELD(DMA_INT_RAW_CH0, IN_DSCR_EMPTY_CH0_INT_RAW, 7, 1)
    FIELD(DMA_INT_RAW_CH0, OUT_DSCR_ERR_CH0_INT_RAW, 6, 1)
    FIELD(DMA_INT_RAW_CH0, IN_DSCR_ERR_CH0_INT_RAW, 5, 1)
    FIELD(DMA_INT_RAW_CH0, OUT_EOF_CH0_INT_RAW, 4, 1)
    FIELD(DMA_INT_RAW_CH0, OUT_DONE_CH0_INT_RAW, 3, 1)
    FIELD(DMA_INT_RAW_CH0, IN_ERR_EOF_CH0_INT_RAW, 2, 1)
    FIELD(DMA_INT_RAW_CH0, IN_SUC_EOF_CH0_INT_RAW, 1, 1)
    FIELD(DMA_INT_RAW_CH0, IN_DONE_CH0_INT_RAW, 0, 1)

REG32(DMA_INT_ST_CH0, 0x004)
    FIELD(DMA_INT_ST_CH0, OUTFIFO_UDF_CH0_INT_ST, 12, 1)
    FIELD(DMA_INT_ST_CH0, OUTFIFO_OVF_CH0_INT_ST, 11, 1)
    FIELD(DMA_INT_ST_CH0, INFIFO_UDF_CH0_INT_ST, 10, 1)
    FIELD(DMA_INT_ST_CH0, INFIFO_OVF_CH0_INT_ST, 9, 1)
    FIELD(DMA_INT_ST_CH0, OUT_TOTAL_EOF_CH0_INT_ST, 8, 1)
    FIELD(DMA_INT_ST_CH0, IN_DSCR_EMPTY_CH0_INT_ST, 7, 1)
    FIELD(DMA_INT_ST_CH0, OUT_DSCR_ERR_CH0_INT_ST, 6, 1)
    FIELD(DMA_INT_ST_CH0, IN_DSCR_ERR_CH0_INT_ST, 5, 1)
    FIELD(DMA_INT_ST_CH0, OUT_EOF_CH0_INT_ST, 4, 1)
    FIELD(DMA_INT_ST_CH0, OUT_DONE_CH0_INT_ST, 3, 1)
    FIELD(DMA_INT_ST_CH0, IN_ERR_EOF_CH0_INT_ST, 2, 1)
    FIELD(DMA_INT_ST_CH0, IN_SUC_EOF_CH0_INT_ST, 1, 1)
    FIELD(DMA_INT_ST_CH0, IN_DONE_CH0_INT_ST, 0, 1)

REG32(DMA_INT_ENA_CH0, 0x008)
    FIELD(DMA_INT_ENA_CH0, OUTFIFO_UDF_CH0_INT_ENA, 12, 1)
    FIELD(DMA_INT_ENA_CH0, OUTFIFO_OVF_CH0_INT_ENA, 11, 1)
    FIELD(DMA_INT_ENA_CH0, INFIFO_UDF_CH0_INT_ENA, 10, 1)
    FIELD(DMA_INT_ENA_CH0, INFIFO_OVF_CH0_INT_ENA, 9, 1)
    FIELD(DMA_INT_ENA_CH0, OUT_TOTAL_EOF_CH0_INT_ENA, 8, 1)
    FIELD(DMA_INT_ENA_CH0, IN_DSCR_EMPTY_CH0_INT_ENA, 7, 1)
    FIELD(DMA_INT_ENA_CH0, OUT_DSCR_ERR_CH0_INT_ENA, 6, 1)
    FIELD(DMA_INT_ENA_CH0, IN_DSCR_ERR_CH0_INT_ENA, 5, 1)
    FIELD(DMA_INT_ENA_CH0, OUT_EOF_CH0_INT_ENA, 4, 1)
    FIELD(DMA_INT_ENA_CH0, OUT_DONE_CH0_INT_ENA, 3, 1)
    FIELD(DMA_INT_ENA_CH0, IN_ERR_EOF_CH0_INT_ENA, 2, 1)
    FIELD(DMA_INT_ENA_CH0, IN_SUC_EOF_CH0_INT_ENA, 1, 1)
    FIELD(DMA_INT_ENA_CH0, IN_DONE_CH0_INT_ENA, 0, 1)

REG32(DMA_INT_CLR_CH0, 0x00C)
    FIELD(DMA_INT_CLR_CH0, OUTFIFO_UDF_CH0_INT_CLR, 12, 1)
    FIELD(DMA_INT_CLR_CH0, OUTFIFO_OVF_CH0_INT_CLR, 11, 1)
    FIELD(DMA_INT_CLR_CH0, INFIFO_UDF_CH0_INT_CLR, 10, 1)
    FIELD(DMA_INT_CLR_CH0, INFIFO_OVF_CH0_INT_CLR, 9, 1)
    FIELD(DMA_INT_CLR_CH0, OUT_TOTAL_EOF_CH0_INT_CLR, 8, 1)
    FIELD(DMA_INT_CLR_CH0, IN_DSCR_EMPTY_CH0_INT_CLR, 7, 1)
    FIELD(DMA_INT_CLR_CH0, OUT_DSCR_ERR_CH0_INT_CLR, 6, 1)
    FIELD(DMA_INT_CLR_CH0, IN_DSCR_ERR_CH0_INT_CLR, 5, 1)
    FIELD(DMA_INT_CLR_CH0, OUT_EOF_CH0_INT_CLR, 4, 1)
    FIELD(DMA_INT_CLR_CH0, OUT_DONE_CH0_INT_CLR, 3, 1)
    FIELD(DMA_INT_CLR_CH0, IN_ERR_EOF_CH0_INT_CLR, 2, 1)
    FIELD(DMA_INT_CLR_CH0, IN_SUC_EOF_CH0_INT_CLR, 1, 1)
    FIELD(DMA_INT_CLR_CH0, IN_DONE_CH0_INT_CLR, 0, 1)

REG32(DMA_INT_RAW_CH1, 0x010)
    FIELD(DMA_INT_RAW_CH1, OUTFIFO_UDF_CH1_INT_RAW, 12, 1)
    FIELD(DMA_INT_RAW_CH1, OUTFIFO_OVF_CH1_INT_RAW, 11, 1)
    FIELD(DMA_INT_RAW_CH1, INFIFO_UDF_CH1_INT_RAW, 10, 1)
    FIELD(DMA_INT_RAW_CH1, INFIFO_OVF_CH1_INT_RAW, 9, 1)
    FIELD(DMA_INT_RAW_CH1, OUT_TOTAL_EOF_CH1_INT_RAW, 8, 1)
    FIELD(DMA_INT_RAW_CH1, IN_DSCR_EMPTY_CH1_INT_RAW, 7, 1)
    FIELD(DMA_INT_RAW_CH1, OUT_DSCR_ERR_CH1_INT_RAW, 6, 1)
    FIELD(DMA_INT_RAW_CH1, IN_DSCR_ERR_CH1_INT_RAW, 5, 1)
    FIELD(DMA_INT_RAW_CH1, OUT_EOF_CH1_INT_RAW, 4, 1)
    FIELD(DMA_INT_RAW_CH1, OUT_DONE_CH1_INT_RAW, 3, 1)
    FIELD(DMA_INT_RAW_CH1, IN_ERR_EOF_CH1_INT_RAW, 2, 1)
    FIELD(DMA_INT_RAW_CH1, IN_SUC_EOF_CH1_INT_RAW, 1, 1)
    FIELD(DMA_INT_RAW_CH1, IN_DONE_CH1_INT_RAW, 0, 1)

REG32(DMA_INT_ST_CH1, 0x014)
    FIELD(DMA_INT_ST_CH1, OUTFIFO_UDF_CH1_INT_ST, 12, 1)
    FIELD(DMA_INT_ST_CH1, OUTFIFO_OVF_CH1_INT_ST, 11, 1)
    FIELD(DMA_INT_ST_CH1, INFIFO_UDF_CH1_INT_ST, 10, 1)
    FIELD(DMA_INT_ST_CH1, INFIFO_OVF_CH1_INT_ST, 9, 1)
    FIELD(DMA_INT_ST_CH1, OUT_TOTAL_EOF_CH1_INT_ST, 8, 1)
    FIELD(DMA_INT_ST_CH1, IN_DSCR_EMPTY_CH1_INT_ST, 7, 1)
    FIELD(DMA_INT_ST_CH1, OUT_DSCR_ERR_CH1_INT_ST, 6, 1)
    FIELD(DMA_INT_ST_CH1, IN_DSCR_ERR_CH1_INT_ST, 5, 1)
    FIELD(DMA_INT_ST_CH1, OUT_EOF_CH1_INT_ST, 4, 1)
    FIELD(DMA_INT_ST_CH1, OUT_DONE_CH1_INT_ST, 3, 1)
    FIELD(DMA_INT_ST_CH1, IN_ERR_EOF_CH1_INT_ST, 2, 1)
    FIELD(DMA_INT_ST_CH1, IN_SUC_EOF_CH1_INT_ST, 1, 1)
    FIELD(DMA_INT_ST_CH1, IN_DONE_CH1_INT_ST, 0, 1)

REG32(DMA_INT_ENA_CH1, 0x018)
    FIELD(DMA_INT_ENA_CH1, OUTFIFO_UDF_CH1_INT_ENA, 12, 1)
    FIELD(DMA_INT_ENA_CH1, OUTFIFO_OVF_CH1_INT_ENA, 11, 1)
    FIELD(DMA_INT_ENA_CH1, INFIFO_UDF_CH1_INT_ENA, 10, 1)
    FIELD(DMA_INT_ENA_CH1, INFIFO_OVF_CH1_INT_ENA, 9, 1)
    FIELD(DMA_INT_ENA_CH1, OUT_TOTAL_EOF_CH1_INT_ENA, 8, 1)
    FIELD(DMA_INT_ENA_CH1, IN_DSCR_EMPTY_CH1_INT_ENA, 7, 1)
    FIELD(DMA_INT_ENA_CH1, OUT_DSCR_ERR_CH1_INT_ENA, 6, 1)
    FIELD(DMA_INT_ENA_CH1, IN_DSCR_ERR_CH1_INT_ENA, 5, 1)
    FIELD(DMA_INT_ENA_CH1, OUT_EOF_CH1_INT_ENA, 4, 1)
    FIELD(DMA_INT_ENA_CH1, OUT_DONE_CH1_INT_ENA, 3, 1)
    FIELD(DMA_INT_ENA_CH1, IN_ERR_EOF_CH1_INT_ENA, 2, 1)
    FIELD(DMA_INT_ENA_CH1, IN_SUC_EOF_CH1_INT_ENA, 1, 1)
    FIELD(DMA_INT_ENA_CH1, IN_DONE_CH1_INT_ENA, 0, 1)

REG32(DMA_INT_CLR_CH1, 0x01C)
    FIELD(DMA_INT_CLR_CH1, OUTFIFO_UDF_CH1_INT_CLR, 12, 1)
    FIELD(DMA_INT_CLR_CH1, OUTFIFO_OVF_CH1_INT_CLR, 11, 1)
    FIELD(DMA_INT_CLR_CH1, INFIFO_UDF_CH1_INT_CLR, 10, 1)
    FIELD(DMA_INT_CLR_CH1, INFIFO_OVF_CH1_INT_CLR, 9, 1)
    FIELD(DMA_INT_CLR_CH1, OUT_TOTAL_EOF_CH1_INT_CLR, 8, 1)
    FIELD(DMA_INT_CLR_CH1, IN_DSCR_EMPTY_CH1_INT_CLR, 7, 1)
    FIELD(DMA_INT_CLR_CH1, OUT_DSCR_ERR_CH1_INT_CLR, 6, 1)
    FIELD(DMA_INT_CLR_CH1, IN_DSCR_ERR_CH1_INT_CLR, 5, 1)
    FIELD(DMA_INT_CLR_CH1, OUT_EOF_CH1_INT_CLR, 4, 1)
    FIELD(DMA_INT_CLR_CH1, OUT_DONE_CH1_INT_CLR, 3, 1)
    FIELD(DMA_INT_CLR_CH1, IN_ERR_EOF_CH1_INT_CLR, 2, 1)
    FIELD(DMA_INT_CLR_CH1, IN_SUC_EOF_CH1_INT_CLR, 1, 1)
    FIELD(DMA_INT_CLR_CH1, IN_DONE_CH1_INT_CLR, 0, 1)

REG32(DMA_INT_RAW_CH2, 0x020)
    FIELD(DMA_INT_RAW_CH2, OUTFIFO_UDF_CH2_INT_RAW, 12, 1)
    FIELD(DMA_INT_RAW_CH2, OUTFIFO_OVF_CH2_INT_RAW, 11, 1)
    FIELD(DMA_INT_RAW_CH2, INFIFO_UDF_CH2_INT_RAW, 10, 1)
    FIELD(DMA_INT_RAW_CH2, INFIFO_OVF_CH2_INT_RAW, 9, 1)
    FIELD(DMA_INT_RAW_CH2, OUT_TOTAL_EOF_CH2_INT_RAW, 8, 1)
    FIELD(DMA_INT_RAW_CH2, IN_DSCR_EMPTY_CH2_INT_RAW, 7, 1)
    FIELD(DMA_INT_RAW_CH2, OUT_DSCR_ERR_CH2_INT_RAW, 6, 1)
    FIELD(DMA_INT_RAW_CH2, IN_DSCR_ERR_CH2_INT_RAW, 5, 1)
    FIELD(DMA_INT_RAW_CH2, OUT_EOF_CH2_INT_RAW, 4, 1)
    FIELD(DMA_INT_RAW_CH2, OUT_DONE_CH2_INT_RAW, 3, 1)
    FIELD(DMA_INT_RAW_CH2, IN_ERR_EOF_CH2_INT_RAW, 2, 1)
    FIELD(DMA_INT_RAW_CH2, IN_SUC_EOF_CH2_INT_RAW, 1, 1)
    FIELD(DMA_INT_RAW_CH2, IN_DONE_CH2_INT_RAW, 0, 1)

REG32(DMA_INT_ST_CH2, 0x024)
    FIELD(DMA_INT_ST_CH2, OUTFIFO_UDF_CH2_INT_ST, 12, 1)
    FIELD(DMA_INT_ST_CH2, OUTFIFO_OVF_CH2_INT_ST, 11, 1)
    FIELD(DMA_INT_ST_CH2, INFIFO_UDF_CH2_INT_ST, 10, 1)
    FIELD(DMA_INT_ST_CH2, INFIFO_OVF_CH2_INT_ST, 9, 1)
    FIELD(DMA_INT_ST_CH2, OUT_TOTAL_EOF_CH2_INT_ST, 8, 1)
    FIELD(DMA_INT_ST_CH2, IN_DSCR_EMPTY_CH2_INT_ST, 7, 1)
    FIELD(DMA_INT_ST_CH2, OUT_DSCR_ERR_CH2_INT_ST, 6, 1)
    FIELD(DMA_INT_ST_CH2, IN_DSCR_ERR_CH2_INT_ST, 5, 1)
    FIELD(DMA_INT_ST_CH2, OUT_EOF_CH2_INT_ST, 4, 1)
    FIELD(DMA_INT_ST_CH2, OUT_DONE_CH2_INT_ST, 3, 1)
    FIELD(DMA_INT_ST_CH2, IN_ERR_EOF_CH2_INT_ST, 2, 1)
    FIELD(DMA_INT_ST_CH2, IN_SUC_EOF_CH2_INT_ST, 1, 1)
    FIELD(DMA_INT_ST_CH2, IN_DONE_CH2_INT_ST, 0, 1)

REG32(DMA_INT_ENA_CH2, 0x028)
    FIELD(DMA_INT_ENA_CH2, OUTFIFO_UDF_CH2_INT_ENA, 12, 1)
    FIELD(DMA_INT_ENA_CH2, OUTFIFO_OVF_CH2_INT_ENA, 11, 1)
    FIELD(DMA_INT_ENA_CH2, INFIFO_UDF_CH2_INT_ENA, 10, 1)
    FIELD(DMA_INT_ENA_CH2, INFIFO_OVF_CH2_INT_ENA, 9, 1)
    FIELD(DMA_INT_ENA_CH2, OUT_TOTAL_EOF_CH2_INT_ENA, 8, 1)
    FIELD(DMA_INT_ENA_CH2, IN_DSCR_EMPTY_CH2_INT_ENA, 7, 1)
    FIELD(DMA_INT_ENA_CH2, OUT_DSCR_ERR_CH2_INT_ENA, 6, 1)
    FIELD(DMA_INT_ENA_CH2, IN_DSCR_ERR_CH2_INT_ENA, 5, 1)
    FIELD(DMA_INT_ENA_CH2, OUT_EOF_CH2_INT_ENA, 4, 1)
    FIELD(DMA_INT_ENA_CH2, OUT_DONE_CH2_INT_ENA, 3, 1)
    FIELD(DMA_INT_ENA_CH2, IN_ERR_EOF_CH2_INT_ENA, 2, 1)
    FIELD(DMA_INT_ENA_CH2, IN_SUC_EOF_CH2_INT_ENA, 1, 1)
    FIELD(DMA_INT_ENA_CH2, IN_DONE_CH2_INT_ENA, 0, 1)

REG32(DMA_INT_CLR_CH2, 0x02C)
    FIELD(DMA_INT_CLR_CH2, OUTFIFO_UDF_CH2_INT_CLR, 12, 1)
    FIELD(DMA_INT_CLR_CH2, OUTFIFO_OVF_CH2_INT_CLR, 11, 1)
    FIELD(DMA_INT_CLR_CH2, INFIFO_UDF_CH2_INT_CLR, 10, 1)
    FIELD(DMA_INT_CLR_CH2, INFIFO_OVF_CH2_INT_CLR, 9, 1)
    FIELD(DMA_INT_CLR_CH2, OUT_TOTAL_EOF_CH2_INT_CLR, 8, 1)
    FIELD(DMA_INT_CLR_CH2, IN_DSCR_EMPTY_CH2_INT_CLR, 7, 1)
    FIELD(DMA_INT_CLR_CH2, OUT_DSCR_ERR_CH2_INT_CLR, 6, 1)
    FIELD(DMA_INT_CLR_CH2, IN_DSCR_ERR_CH2_INT_CLR, 5, 1)
    FIELD(DMA_INT_CLR_CH2, OUT_EOF_CH2_INT_CLR, 4, 1)
    FIELD(DMA_INT_CLR_CH2, OUT_DONE_CH2_INT_CLR, 3, 1)
    FIELD(DMA_INT_CLR_CH2, IN_ERR_EOF_CH2_INT_CLR, 2, 1)
    FIELD(DMA_INT_CLR_CH2, IN_SUC_EOF_CH2_INT_CLR, 1, 1)
    FIELD(DMA_INT_CLR_CH2, IN_DONE_CH2_INT_CLR, 0, 1)

REG32(DMA_AHB_TEST, 0x040)
    FIELD(DMA_AHB_TEST, AHB_TESTADDR, 4, 2)
    FIELD(DMA_AHB_TEST, AHB_TESTMODE, 0, 3)

REG32(DMA_MISC_CONF, 0x044)
    FIELD(DMA_MISC_CONF, CLK_EN, 3, 1)
    FIELD(DMA_MISC_CONF, ARB_PRI_DIS, 2, 1)
    FIELD(DMA_MISC_CONF, AHBM_RST_INTER, 0, 1)

REG32(DMA_DATE, 0x048)
    FIELD(DMA_DATE, DATE, 0, 32)

REG32(DMA_IN_CONF0_CH0, 0x070)
    FIELD(DMA_IN_CONF0_CH0, MEM_TRANS_EN_CH0, 4, 1)
    FIELD(DMA_IN_CONF0_CH0, IN_DATA_BURST_EN_CH0, 3, 1)
    FIELD(DMA_IN_CONF0_CH0, INDSCR_BURST_EN_CH0, 2, 1)
    FIELD(DMA_IN_CONF0_CH0, IN_LOOP_TEST_CH0, 1, 1)
    FIELD(DMA_IN_CONF0_CH0, IN_RST_CH0, 0, 1)

REG32(DMA_IN_CONF1_CH0, 0x074)
    FIELD(DMA_IN_CONF1_CH0, IN_CHECK_OWNER_CH0, 12, 1)

REG32(DMA_INFIFO_STATUS_CH0, 0x078)
    FIELD(DMA_INFIFO_STATUS_CH0, IN_BUF_HUNGRY_CH0, 27, 1)
    FIELD(DMA_INFIFO_STATUS_CH0, IN_REMAIN_UNDER_4B_CH0, 26, 1)
    FIELD(DMA_INFIFO_STATUS_CH0, IN_REMAIN_UNDER_3B_CH0, 25, 1)
    FIELD(DMA_INFIFO_STATUS_CH0, IN_REMAIN_UNDER_2B_CH0, 24, 1)
    FIELD(DMA_INFIFO_STATUS_CH0, IN_REMAIN_UNDER_1B_CH0, 23, 1)
    FIELD(DMA_INFIFO_STATUS_CH0, INFIFO_CNT_CH0, 2, 6)
    FIELD(DMA_INFIFO_STATUS_CH0, INFIFO_EMPTY_CH0, 1, 1)
    FIELD(DMA_INFIFO_STATUS_CH0, INFIFO_FULL_CH0, 0, 1)

REG32(DMA_IN_POP_CH0, 0x07C)
    FIELD(DMA_IN_POP_CH0, INFIFO_POP_CH0, 12, 1)
    FIELD(DMA_IN_POP_CH0, INFIFO_RDATA_CH0, 0, 12)

REG32(DMA_IN_LINK_CH0, 0x080)
    FIELD(DMA_IN_LINK_CH0, INLINK_PARK_CH0, 24, 1)
    FIELD(DMA_IN_LINK_CH0, INLINK_RESTART_CH0, 23, 1)
    FIELD(DMA_IN_LINK_CH0, INLINK_START_CH0, 22, 1)
    FIELD(DMA_IN_LINK_CH0, INLINK_STOP_CH0, 21, 1)
    FIELD(DMA_IN_LINK_CH0, INLINK_AUTO_RET_CH0, 20, 1)
    FIELD(DMA_IN_LINK_CH0, INLINK_ADDR_CH0, 0, 20)

REG32(DMA_IN_STATE_CH0, 0x084)
    FIELD(DMA_IN_STATE_CH0, IN_STATE_CH0, 20, 3)
    FIELD(DMA_IN_STATE_CH0, IN_DSCR_STATE_CH0, 18, 2)
    FIELD(DMA_IN_STATE_CH0, INLINK_DSCR_ADDR_CH0, 0, 18)

REG32(DMA_IN_SUC_EOF_DES_ADDR_CH0, 0x088)
    FIELD(DMA_IN_SUC_EOF_DES_ADDR_CH0, IN_SUC_EOF_DES_ADDR_CH0, 0, 32)

REG32(DMA_IN_ERR_EOF_DES_ADDR_CH0, 0x08C)
    FIELD(DMA_IN_ERR_EOF_DES_ADDR_CH0, IN_ERR_EOF_DES_ADDR_CH0, 0, 32)

REG32(DMA_IN_DSCR_CH0, 0x090)
    FIELD(DMA_IN_DSCR_CH0, INLINK_DSCR_CH0, 0, 32)

REG32(DMA_IN_DSCR_BF0_CH0, 0x094)
    FIELD(DMA_IN_DSCR_BF0_CH0, INLINK_DSCR_BF0_CH0, 0, 32)

REG32(DMA_IN_DSCR_BF1_CH0, 0x098)
    FIELD(DMA_IN_DSCR_BF1_CH0, INLINK_DSCR_BF1_CH0, 0, 32)

REG32(DMA_IN_PRI_CH0, 0x09C)
    FIELD(DMA_IN_PRI_CH0, RX_PRI_CH0, 0, 4)

REG32(DMA_IN_PERI_SEL_CH0, 0x0A0)
    FIELD(DMA_IN_PERI_SEL_CH0, PERI_IN_SEL_CH0, 0, 6)

REG32(DMA_OUT_CONF0_CH0, 0x0D0)
    FIELD(DMA_OUT_CONF0_CH0, OUT_DATA_BURST_EN_CH0, 5, 1)
    FIELD(DMA_OUT_CONF0_CH0, OUTDSCR_BURST_EN_CH0, 4, 1)
    FIELD(DMA_OUT_CONF0_CH0, OUT_EOF_MODE_CH0, 3, 1)
    FIELD(DMA_OUT_CONF0_CH0, OUT_AUTO_WRBACK_CH0, 2, 1)
    FIELD(DMA_OUT_CONF0_CH0, OUT_LOOP_TEST_CH0, 1, 1)
    FIELD(DMA_OUT_CONF0_CH0, OUT_RST_CH0, 0, 1)

REG32(DMA_OUT_CONF1_CH0, 0x0D4)
    FIELD(DMA_OUT_CONF1_CH0, OUT_CHECK_OWNER_CH0, 12, 1)

REG32(DMA_OUTFIFO_STATUS_CH0, 0x0D8)
    FIELD(DMA_OUTFIFO_STATUS_CH0, OUT_REMAIN_UNDER_4B_CH0, 26, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH0, OUT_REMAIN_UNDER_3B_CH0, 25, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH0, OUT_REMAIN_UNDER_2B_CH0, 24, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH0, OUT_REMAIN_UNDER_1B_CH0, 23, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH0, OUTFIFO_CNT_CH0, 2, 6)
    FIELD(DMA_OUTFIFO_STATUS_CH0, OUTFIFO_EMPTY_CH0, 1, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH0, OUTFIFO_FULL_CH0, 0, 1)

REG32(DMA_OUT_PUSH_CH0, 0x0DC)
    FIELD(DMA_OUT_PUSH_CH0, OUTFIFO_PUSH_CH0, 9, 1)
    FIELD(DMA_OUT_PUSH_CH0, OUTFIFO_WDATA_CH0, 0, 9)

REG32(DMA_OUT_LINK_CH0, 0x0E0)
    FIELD(DMA_OUT_LINK_CH0, OUTLINK_PARK_CH0, 23, 1)
    FIELD(DMA_OUT_LINK_CH0, OUTLINK_RESTART_CH0, 22, 1)
    FIELD(DMA_OUT_LINK_CH0, OUTLINK_START_CH0, 21, 1)
    FIELD(DMA_OUT_LINK_CH0, OUTLINK_STOP_CH0, 20, 1)
    FIELD(DMA_OUT_LINK_CH0, OUTLINK_ADDR_CH0, 0, 20)

REG32(DMA_OUT_STATE_CH0, 0x0E4)
    FIELD(DMA_OUT_STATE_CH0, OUT_STATE_CH0, 20, 3)
    FIELD(DMA_OUT_STATE_CH0, OUT_DSCR_STATE_CH0, 18, 2)
    FIELD(DMA_OUT_STATE_CH0, OUTLINK_DSCR_ADDR_CH0, 0, 18)

REG32(DMA_OUT_EOF_DES_ADDR_CH0, 0x0E8)
    FIELD(DMA_OUT_EOF_DES_ADDR_CH0, OUT_EOF_DES_ADDR_CH0, 0, 32)

REG32(DMA_OUT_EOF_BFR_DES_ADDR_CH0, 0x0EC)
    FIELD(DMA_OUT_EOF_BFR_DES_ADDR_CH0, OUT_EOF_BFR_DES_ADDR_CH0, 0, 32)

REG32(DMA_OUT_DSCR_CH0, 0x0F0)
    FIELD(DMA_OUT_DSCR_CH0, OUTLINK_DSCR_CH0, 0, 32)

REG32(DMA_OUT_DSCR_BF0_CH0, 0x0F4)
    FIELD(DMA_OUT_DSCR_BF0_CH0, OUTLINK_DSCR_BF0_CH0, 0, 32)

REG32(DMA_OUT_DSCR_BF1_CH0, 0x0F8)
    FIELD(DMA_OUT_DSCR_BF1_CH0, OUTLINK_DSCR_BF1_CH0, 0, 32)

REG32(DMA_OUT_PRI_CH0, 0x0FC)
    FIELD(DMA_OUT_PRI_CH0, TX_PRI_CH0, 0, 4)

REG32(DMA_OUT_PERI_SEL_CH0, 0x100)
    FIELD(DMA_OUT_PERI_SEL_CH0, PERI_OUT_SEL_CH0, 0, 6)

REG32(DMA_IN_CONF0_CH1, 0x130)
    FIELD(DMA_IN_CONF0_CH1, MEM_TRANS_EN_CH1, 4, 1)
    FIELD(DMA_IN_CONF0_CH1, IN_DATA_BURST_EN_CH1, 3, 1)
    FIELD(DMA_IN_CONF0_CH1, INDSCR_BURST_EN_CH1, 2, 1)
    FIELD(DMA_IN_CONF0_CH1, IN_LOOP_TEST_CH1, 1, 1)
    FIELD(DMA_IN_CONF0_CH1, IN_RST_CH1, 0, 1)

REG32(DMA_IN_CONF1_CH1, 0x134)
    FIELD(DMA_IN_CONF1_CH1, IN_CHECK_OWNER_CH1, 12, 1)

REG32(DMA_INFIFO_STATUS_CH1, 0x138)
    FIELD(DMA_INFIFO_STATUS_CH1, IN_BUF_HUNGRY_CH1, 27, 1)
    FIELD(DMA_INFIFO_STATUS_CH1, IN_REMAIN_UNDER_4B_CH1, 26, 1)
    FIELD(DMA_INFIFO_STATUS_CH1, IN_REMAIN_UNDER_3B_CH1, 25, 1)
    FIELD(DMA_INFIFO_STATUS_CH1, IN_REMAIN_UNDER_2B_CH1, 24, 1)
    FIELD(DMA_INFIFO_STATUS_CH1, IN_REMAIN_UNDER_1B_CH1, 23, 1)
    FIELD(DMA_INFIFO_STATUS_CH1, INFIFO_CNT_CH1, 2, 6)
    FIELD(DMA_INFIFO_STATUS_CH1, INFIFO_EMPTY_CH1, 1, 1)
    FIELD(DMA_INFIFO_STATUS_CH1, INFIFO_FULL_CH1, 0, 1)

REG32(DMA_IN_POP_CH1, 0x13C)
    FIELD(DMA_IN_POP_CH1, INFIFO_POP_CH1, 12, 1)
    FIELD(DMA_IN_POP_CH1, INFIFO_RDATA_CH1, 0, 12)

REG32(DMA_IN_LINK_CH1, 0x140)
    FIELD(DMA_IN_LINK_CH1, INLINK_PARK_CH1, 24, 1)
    FIELD(DMA_IN_LINK_CH1, INLINK_RESTART_CH1, 23, 1)
    FIELD(DMA_IN_LINK_CH1, INLINK_START_CH1, 22, 1)
    FIELD(DMA_IN_LINK_CH1, INLINK_STOP_CH1, 21, 1)
    FIELD(DMA_IN_LINK_CH1, INLINK_AUTO_RET_CH1, 20, 1)
    FIELD(DMA_IN_LINK_CH1, INLINK_ADDR_CH1, 0, 20)

REG32(DMA_IN_STATE_CH1, 0x144)
    FIELD(DMA_IN_STATE_CH1, IN_STATE_CH1, 20, 3)
    FIELD(DMA_IN_STATE_CH1, IN_DSCR_STATE_CH1, 18, 2)
    FIELD(DMA_IN_STATE_CH1, INLINK_DSCR_ADDR_CH1, 0, 18)

REG32(DMA_IN_SUC_EOF_DES_ADDR_CH1, 0x148)
    FIELD(DMA_IN_SUC_EOF_DES_ADDR_CH1, IN_SUC_EOF_DES_ADDR_CH1, 0, 32)

REG32(DMA_IN_ERR_EOF_DES_ADDR_CH1, 0x14C)
    FIELD(DMA_IN_ERR_EOF_DES_ADDR_CH1, IN_ERR_EOF_DES_ADDR_CH1, 0, 32)

REG32(DMA_IN_DSCR_CH1, 0x150)
    FIELD(DMA_IN_DSCR_CH1, INLINK_DSCR_CH1, 0, 32)

REG32(DMA_IN_DSCR_BF0_CH1, 0x154)
    FIELD(DMA_IN_DSCR_BF0_CH1, INLINK_DSCR_BF0_CH1, 0, 32)

REG32(DMA_IN_DSCR_BF1_CH1, 0x158)
    FIELD(DMA_IN_DSCR_BF1_CH1, INLINK_DSCR_BF1_CH1, 0, 32)

REG32(DMA_IN_PRI_CH1, 0x15C)
    FIELD(DMA_IN_PRI_CH1, RX_PRI_CH1, 0, 4)

REG32(DMA_IN_PERI_SEL_CH1, 0x160)
    FIELD(DMA_IN_PERI_SEL_CH1, PERI_IN_SEL_CH1, 0, 6)

REG32(DMA_OUT_CONF0_CH1, 0x190)
    FIELD(DMA_OUT_CONF0_CH1, OUT_DATA_BURST_EN_CH1, 5, 1)
    FIELD(DMA_OUT_CONF0_CH1, OUTDSCR_BURST_EN_CH1, 4, 1)
    FIELD(DMA_OUT_CONF0_CH1, OUT_EOF_MODE_CH1, 3, 1)
    FIELD(DMA_OUT_CONF0_CH1, OUT_AUTO_WRBACK_CH1, 2, 1)
    FIELD(DMA_OUT_CONF0_CH1, OUT_LOOP_TEST_CH1, 1, 1)
    FIELD(DMA_OUT_CONF0_CH1, OUT_RST_CH1, 0, 1)

REG32(DMA_OUT_CONF1_CH1, 0x194)
    FIELD(DMA_OUT_CONF1_CH1, OUT_CHECK_OWNER_CH1, 12, 1)

REG32(DMA_OUTFIFO_STATUS_CH1, 0x198)
    FIELD(DMA_OUTFIFO_STATUS_CH1, OUT_REMAIN_UNDER_4B_CH1, 26, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH1, OUT_REMAIN_UNDER_3B_CH1, 25, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH1, OUT_REMAIN_UNDER_2B_CH1, 24, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH1, OUT_REMAIN_UNDER_1B_CH1, 23, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH1, OUTFIFO_CNT_CH1, 2, 6)
    FIELD(DMA_OUTFIFO_STATUS_CH1, OUTFIFO_EMPTY_CH1, 1, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH1, OUTFIFO_FULL_CH1, 0, 1)

REG32(DMA_OUT_PUSH_CH1, 0x19C)
    FIELD(DMA_OUT_PUSH_CH1, OUTFIFO_PUSH_CH1, 9, 1)
    FIELD(DMA_OUT_PUSH_CH1, OUTFIFO_WDATA_CH1, 0, 9)

REG32(DMA_OUT_LINK_CH1, 0x1A0)
    FIELD(DMA_OUT_LINK_CH1, OUTLINK_PARK_CH1, 23, 1)
    FIELD(DMA_OUT_LINK_CH1, OUTLINK_RESTART_CH1, 22, 1)
    FIELD(DMA_OUT_LINK_CH1, OUTLINK_START_CH1, 21, 1)
    FIELD(DMA_OUT_LINK_CH1, OUTLINK_STOP_CH1, 20, 1)
    FIELD(DMA_OUT_LINK_CH1, OUTLINK_ADDR_CH1, 0, 20)

REG32(DMA_OUT_STATE_CH1, 0x1A4)
    FIELD(DMA_OUT_STATE_CH1, OUT_STATE_CH1, 20, 3)
    FIELD(DMA_OUT_STATE_CH1, OUT_DSCR_STATE_CH1, 18, 2)
    FIELD(DMA_OUT_STATE_CH1, OUTLINK_DSCR_ADDR_CH1, 0, 18)

REG32(DMA_OUT_EOF_DES_ADDR_CH1, 0x1A8)
    FIELD(DMA_OUT_EOF_DES_ADDR_CH1, OUT_EOF_DES_ADDR_CH1, 0, 32)

REG32(DMA_OUT_EOF_BFR_DES_ADDR_CH1, 0x1AC)
    FIELD(DMA_OUT_EOF_BFR_DES_ADDR_CH1, OUT_EOF_BFR_DES_ADDR_CH1, 0, 32)

REG32(DMA_OUT_DSCR_CH1, 0x1B0)
    FIELD(DMA_OUT_DSCR_CH1, OUTLINK_DSCR_CH1, 0, 32)

REG32(DMA_OUT_DSCR_BF0_CH1, 0x1B4)
    FIELD(DMA_OUT_DSCR_BF0_CH1, OUTLINK_DSCR_BF0_CH1, 0, 32)

REG32(DMA_OUT_DSCR_BF1_CH1, 0x1B8)
    FIELD(DMA_OUT_DSCR_BF1_CH1, OUTLINK_DSCR_BF1_CH1, 0, 32)

REG32(DMA_OUT_PRI_CH1, 0x1BC)
    FIELD(DMA_OUT_PRI_CH1, TX_PRI_CH1, 0, 4)

REG32(DMA_OUT_PERI_SEL_CH1, 0x1C0)
    FIELD(DMA_OUT_PERI_SEL_CH1, PERI_OUT_SEL_CH1, 0, 6)

REG32(DMA_IN_CONF0_CH2, 0x1F0)
    FIELD(DMA_IN_CONF0_CH2, MEM_TRANS_EN_CH2, 4, 1)
    FIELD(DMA_IN_CONF0_CH2, IN_DATA_BURST_EN_CH2, 3, 1)
    FIELD(DMA_IN_CONF0_CH2, INDSCR_BURST_EN_CH2, 2, 1)
    FIELD(DMA_IN_CONF0_CH2, IN_LOOP_TEST_CH2, 1, 1)
    FIELD(DMA_IN_CONF0_CH2, IN_RST_CH2, 0, 1)

REG32(DMA_IN_CONF1_CH2, 0x1F4)
    FIELD(DMA_IN_CONF1_CH2, IN_CHECK_OWNER_CH2, 12, 1)

REG32(DMA_INFIFO_STATUS_CH2, 0x1F8)
    FIELD(DMA_INFIFO_STATUS_CH2, IN_BUF_HUNGRY_CH2, 27, 1)
    FIELD(DMA_INFIFO_STATUS_CH2, IN_REMAIN_UNDER_4B_CH2, 26, 1)
    FIELD(DMA_INFIFO_STATUS_CH2, IN_REMAIN_UNDER_3B_CH2, 25, 1)
    FIELD(DMA_INFIFO_STATUS_CH2, IN_REMAIN_UNDER_2B_CH2, 24, 1)
    FIELD(DMA_INFIFO_STATUS_CH2, IN_REMAIN_UNDER_1B_CH2, 23, 1)
    FIELD(DMA_INFIFO_STATUS_CH2, INFIFO_CNT_CH2, 2, 6)
    FIELD(DMA_INFIFO_STATUS_CH2, INFIFO_EMPTY_CH2, 1, 1)
    FIELD(DMA_INFIFO_STATUS_CH2, INFIFO_FULL_CH2, 0, 1)

REG32(DMA_IN_POP_CH2, 0x1FC)
    FIELD(DMA_IN_POP_CH2, INFIFO_POP_CH2, 12, 1)
    FIELD(DMA_IN_POP_CH2, INFIFO_RDATA_CH2, 0, 12)

REG32(DMA_IN_LINK_CH2, 0x200)
    FIELD(DMA_IN_LINK_CH2, INLINK_PARK_CH2, 24, 1)
    FIELD(DMA_IN_LINK_CH2, INLINK_RESTART_CH2, 23, 1)
    FIELD(DMA_IN_LINK_CH2, INLINK_START_CH2, 22, 1)
    FIELD(DMA_IN_LINK_CH2, INLINK_STOP_CH2, 21, 1)
    FIELD(DMA_IN_LINK_CH2, INLINK_AUTO_RET_CH2, 20, 1)
    FIELD(DMA_IN_LINK_CH2, INLINK_ADDR_CH2, 0, 20)

REG32(DMA_IN_STATE_CH2, 0x204)
    FIELD(DMA_IN_STATE_CH2, IN_STATE_CH2, 20, 3)
    FIELD(DMA_IN_STATE_CH2, IN_DSCR_STATE_CH2, 18, 2)
    FIELD(DMA_IN_STATE_CH2, INLINK_DSCR_ADDR_CH2, 0, 18)

REG32(DMA_IN_SUC_EOF_DES_ADDR_CH2, 0x208)
    FIELD(DMA_IN_SUC_EOF_DES_ADDR_CH2, IN_SUC_EOF_DES_ADDR_CH2, 0, 32)

REG32(DMA_IN_ERR_EOF_DES_ADDR_CH2, 0x20C)
    FIELD(DMA_IN_ERR_EOF_DES_ADDR_CH2, IN_ERR_EOF_DES_ADDR_CH2, 0, 32)

REG32(DMA_IN_DSCR_CH2, 0x210)
    FIELD(DMA_IN_DSCR_CH2, INLINK_DSCR_CH2, 0, 32)

REG32(DMA_IN_DSCR_BF0_CH2, 0x214)
    FIELD(DMA_IN_DSCR_BF0_CH2, INLINK_DSCR_BF0_CH2, 0, 32)

REG32(DMA_IN_DSCR_BF1_CH2, 0x218)
    FIELD(DMA_IN_DSCR_BF1_CH2, INLINK_DSCR_BF1_CH2, 0, 32)

REG32(DMA_IN_PRI_CH2, 0x21C)
    FIELD(DMA_IN_PRI_CH2, RX_PRI_CH2, 0, 4)

REG32(DMA_IN_PERI_SEL_CH2, 0x220)
    FIELD(DMA_IN_PERI_SEL_CH2, PERI_IN_SEL_CH2, 0, 6)

REG32(DMA_OUT_CONF0_CH2, 0x250)
    FIELD(DMA_OUT_CONF0_CH2, OUT_DATA_BURST_EN_CH2, 5, 1)
    FIELD(DMA_OUT_CONF0_CH2, OUTDSCR_BURST_EN_CH2, 4, 1)
    FIELD(DMA_OUT_CONF0_CH2, OUT_EOF_MODE_CH2, 3, 1)
    FIELD(DMA_OUT_CONF0_CH2, OUT_AUTO_WRBACK_CH2, 2, 1)
    FIELD(DMA_OUT_CONF0_CH2, OUT_LOOP_TEST_CH2, 1, 1)
    FIELD(DMA_OUT_CONF0_CH2, OUT_RST_CH2, 0, 1)

REG32(DMA_OUT_CONF1_CH2, 0x254)
    FIELD(DMA_OUT_CONF1_CH2, OUT_CHECK_OWNER_CH2, 12, 1)

REG32(DMA_OUTFIFO_STATUS_CH2, 0x258)
    FIELD(DMA_OUTFIFO_STATUS_CH2, OUT_REMAIN_UNDER_4B_CH2, 26, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH2, OUT_REMAIN_UNDER_3B_CH2, 25, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH2, OUT_REMAIN_UNDER_2B_CH2, 24, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH2, OUT_REMAIN_UNDER_1B_CH2, 23, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH2, OUTFIFO_CNT_CH2, 2, 6)
    FIELD(DMA_OUTFIFO_STATUS_CH2, OUTFIFO_EMPTY_CH2, 1, 1)
    FIELD(DMA_OUTFIFO_STATUS_CH2, OUTFIFO_FULL_CH2, 0, 1)

REG32(DMA_OUT_PUSH_CH2, 0x25C)
    FIELD(DMA_OUT_PUSH_CH2, OUTFIFO_PUSH_CH2, 9, 1)
    FIELD(DMA_OUT_PUSH_CH2, OUTFIFO_WDATA_CH2, 0, 9)

REG32(DMA_OUT_LINK_CH2, 0x260)
    FIELD(DMA_OUT_LINK_CH2, OUTLINK_PARK_CH2, 23, 1)
    FIELD(DMA_OUT_LINK_CH2, OUTLINK_RESTART_CH2, 22, 1)
    FIELD(DMA_OUT_LINK_CH2, OUTLINK_START_CH2, 21, 1)
    FIELD(DMA_OUT_LINK_CH2, OUTLINK_STOP_CH2, 20, 1)
    FIELD(DMA_OUT_LINK_CH2, OUTLINK_ADDR_CH2, 0, 20)

REG32(DMA_OUT_STATE_CH2, 0x264)
    FIELD(DMA_OUT_STATE_CH2, OUT_STATE_CH2, 20, 3)
    FIELD(DMA_OUT_STATE_CH2, OUT_DSCR_STATE_CH2, 18, 2)
    FIELD(DMA_OUT_STATE_CH2, OUTLINK_DSCR_ADDR_CH2, 0, 18)

REG32(DMA_OUT_EOF_DES_ADDR_CH2, 0x268)
    FIELD(DMA_OUT_EOF_DES_ADDR_CH2, OUT_EOF_DES_ADDR_CH2, 0, 32)

REG32(DMA_OUT_EOF_BFR_DES_ADDR_CH2, 0x26C)
    FIELD(DMA_OUT_EOF_BFR_DES_ADDR_CH2, OUT_EOF_BFR_DES_ADDR_CH2, 0, 32)

REG32(DMA_OUT_DSCR_CH2, 0x270)
    FIELD(DMA_OUT_DSCR_CH2, OUTLINK_DSCR_CH2, 0, 32)

REG32(DMA_OUT_DSCR_BF0_CH2, 0x274)
    FIELD(DMA_OUT_DSCR_BF0_CH2, OUTLINK_DSCR_BF0_CH2, 0, 32)

REG32(DMA_OUT_DSCR_BF1_CH2, 0x278)
    FIELD(DMA_OUT_DSCR_BF1_CH2, OUTLINK_DSCR_BF1_CH2, 0, 32)

REG32(DMA_OUT_PRI_CH2, 0x27C)
    FIELD(DMA_OUT_PRI_CH2, TX_PRI_CH2, 0, 4)

REG32(DMA_OUT_PERI_SEL_CH2, 0x280)
    FIELD(DMA_OUT_PERI_SEL_CH2, PERI_OUT_SEL_CH2, 0, 6)
