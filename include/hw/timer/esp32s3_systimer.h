/*
 * ESP32-S3 System Timer emulation
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/timer/esp_systimer.h"

#define TYPE_ESP32S3_SYSTIMER           "esp32s3.systimer"
#define ESP32S3_SYSTIMER(obj)           OBJECT_CHECK(ESP32S3SysTimerState, (obj), TYPE_ESP32S3_SYSTIMER)
#define ESP32S3_SYSTIMER_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3SysTimerClass, obj, TYPE_ESP32S3_SYSTIMER)
#define ESP32S3_SYSTIMER_CLASS(klass)   OBJECT_CLASS_CHECK(ESP32S3SysTimerClass, (klass), TYPE_ESP32S3_SYSTIMER)


typedef struct ESP32S3SysTimerState {
    ESPSysTimerState parent;
} ESP32S3SysTimerState;


typedef struct ESP32S3SysTimerClass {
    ESPSysTimerClass parent_class;

    /* Virtual attributes/methods overriden */
    void (*parent_systimer_write)(void *opaque, hwaddr addr, uint64_t value, unsigned int size);
} ESP32S3SysTimerClass;