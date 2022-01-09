#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_reg.h"


#define TYPE_ESP32_RAMDEV "misc.esp32.ramdev"
#define ESP32_RAMDEV(obj) OBJECT_CHECK(Esp32RamdevState, (obj), TYPE_ESP32_RAMDEV)

typedef struct Esp32RamdevState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mem[1024];
} Esp32RamdevState;


