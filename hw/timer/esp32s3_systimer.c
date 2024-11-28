/*
 * ESP32-S3 System Timer
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/boards.h"
#include "hw/timer/esp32s3_systimer.h"


static void esp32c3_systimer_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    ESP32S3SysTimerClass *class = ESP32S3_SYSTIMER_GET_CLASS(opaque);
    ESP32S3SysTimerState *s = ESP32S3_SYSTIMER(opaque);

    class->parent_systimer_write(opaque, addr, value, size);

    if (addr == A_SYSTIMER_INT_ENA) {
        /* If the timer virtual interrupt is enabled too late, we may miss the interrupt in QEMU
         * and not reprogram the next iteration. */
        class->parent_class.comparators_reprogram(&s->parent);
    }
}



static void esp32s3_systimer_class_init(ObjectClass *klass, void *data)
{
    ESP32S3SysTimerClass* esp32s3 = ESP32S3_SYSTIMER_CLASS(klass);
    ESPSysTimerClass* esp = ESP_SYSTIMER_CLASS(klass);

    /* Override the register read method */
    esp32s3->parent_systimer_write = esp->systimer_ops.write;
    esp->systimer_ops.write = esp32c3_systimer_write;
}


static const TypeInfo esp32s3_systimer_info = {
    .name = TYPE_ESP32S3_SYSTIMER,
    .parent = TYPE_ESP_SYSTIMER,
    .instance_size = sizeof(ESP32S3SysTimerState),
    .class_init = esp32s3_systimer_class_init,
    .class_size = sizeof(ESP32S3SysTimerClass),
};


static void esp32s3_systimer_register_types(void)
{
    type_register_static(&esp32s3_systimer_info);
}


type_init(esp32s3_systimer_register_types)
