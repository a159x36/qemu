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
#include "hw/misc/esp32_ana.h"

static uint64_t esp32_ana_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32AnaState *s = ESP32_ANA(opaque);
    uint32_t r = s->mem[addr/4];
    switch(addr) {
        case 4: r=4261412863;
        break;
        case 68: 
        case 76:
        case 196: r=4294967295;
        break;
        default: r=s->mem[addr/4]; 
    }
    printf("esp32_ana_read %ld=%d\n",addr,r);
    return r;
}

static void esp32_ana_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size) {
    Esp32AnaState *s = ESP32_ANA(opaque);
    printf("esp32_ana_write %ld %ld\n",addr, value);
    if(addr==196) {
        s->wifi_channel=value&255;
        printf("wifi channel=%d\n",(int)(value&255));
    }
    s->mem[addr/4]=value;
}

static const MemoryRegionOps esp32_ana_ops = {
    .read =  esp32_ana_read,
    .write = esp32_ana_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_ana_init(Object *obj)
{
    Esp32AnaState *s = ESP32_ANA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_ana_ops, s,
                          TYPE_ESP32_ANA, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    memset(s->mem,0,sizeof(s->mem));
}


static const TypeInfo esp32_ana_info = {
    .name = TYPE_ESP32_ANA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32AnaState),
    .instance_init = esp32_ana_init,
};

static void esp32_ana_register_types(void)
{
    type_register_static(&esp32_ana_info);
}

type_init(esp32_ana_register_types)
