/*
 * ESP32-S3 PMS Dummy
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"


#define TYPE_ESP32S3_PMS "misc.esp32s3.pms"
#define ESP32S3_PMS(obj) OBJECT_CHECK(ESP32S3PmsState, (obj), TYPE_ESP32S3_PMS)

#define ESP32S3_PMS_REG_COUNT (0x1000 / sizeof(uint32_t))
#define ESP32S3_PMS_REG_IDX(addr) ((addr) / sizeof(uint32_t))

typedef struct ESP32S3PmsState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    uint32_t regs[ESP32S3_PMS_REG_COUNT];
} ESP32S3PmsState;
