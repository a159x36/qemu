/*
 * ESP32 Random Number Generator peripheral
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_sens.h"


static uint64_t esp32_sens_read(void *opaque, hwaddr addr, unsigned int size)
{
//Esp32SensState *s = ESP32_SENS(opaque);
    uint32_t r = 0;
//    printf("esp32_sens_read %ld\n",addr);
    switch(addr) {
    case 0x54:
        return 0x10000+2800+rand()%4;
    }
//    qemu_guest_getrandom_nofail(&r, sizeof(r));
    return r;
}

static void esp32_sens_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size) {
  //  Esp32SensState *s = ESP32_SENS(opaque);
//    printf("esp32_sens_write %ld %ld\n",addr, value);
}

static const MemoryRegionOps esp32_sens_ops = {
    .read =  esp32_sens_read,
    .write = esp32_sens_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_sens_init(Object *obj)
{
    Esp32SensState *s = ESP32_SENS(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_sens_ops, s,
                          TYPE_ESP32_SENS, 0x400);
    sysbus_init_mmio(sbd, &s->iomem);
}


static const TypeInfo esp32_sens_info = {
    .name = TYPE_ESP32_SENS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SensState),
    .instance_init = esp32_sens_init,
};

static void esp32_sens_register_types(void)
{
    type_register_static(&esp32_sens_info);
}

type_init(esp32_sens_register_types)
