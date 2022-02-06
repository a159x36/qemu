#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/ssi/ssi.h"

#define TYPE_ESP32_RMT "ssi.esp32.rmt"
#define ESP32_RMT(obj) OBJECT_CHECK(Esp32RmtState, (obj), TYPE_ESP32_RMT)

#define ESP32_RMT_CS_COUNT      3
#define ESP32_RMT_BUF_WORDS     512

typedef struct Esp32RmtState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    int num_cs;
    SSIBus *rmt;
    uint32_t conf0[8];
    uint32_t conf1[8];
    uint32_t int_raw;
    uint32_t int_en;
    uint32_t txlim[8];
    uint32_t apb_conf;
    int sent;
    bool unsent_data;
    uint32_t data[ESP32_RMT_BUF_WORDS];
    QEMUTimer rmt_timer;
} Esp32RmtState;


REG32(RMT_CH0CONF0, 0x20)
REG32(RMT_CH0CONF1, 0x24)
REG32(RMT_INT_RAW, 0xa0)
REG32(RMT_INT_ST, 0xa4)
REG32(RMT_INT_ENA, 0xa8)
REG32(RMT_INT_CLR, 0xac)
REG32(RMT_DATA, 0x800)
REG32(RMT_TX_LIM,0xd0)
REG32(RMT_APB_CONF,0xf0)




