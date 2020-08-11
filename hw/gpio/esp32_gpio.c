/*
 * ESP32 GPIO emulation
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
#include "qapi/error.h"
#include "hw/hw.h"
#include "ui/input.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"

static unsigned gpio_in_low=0x1;
static unsigned gpio_in_high=0x8;

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;
    switch (addr) {
    case 4:
       r= s->gpio_out;
       break;
    case A_GPIO_STRAP:
        r = s->strap_mode;
        break;
    case 0x3C: // in_low
        r = gpio_in_low;
        break;
    case 0x40: // in_high
        r = gpio_in_high;
        break;
    default:
        break;
    }
    return r;
}

static void esp32_gpio_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
  Esp32GpioState *s = ESP32_GPIO(opaque);
  switch (addr) {
     case 4:
     s->gpio_out = value;
     break;
     case 8:
     s->gpio_out |= value;
     break;
     case 12:
     s->gpio_out &= ~value;
     break;
  }
}

static const MemoryRegionOps uart_ops = {
    .read =  esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset(DeviceState *dev)
{
}

static void gpio_keyboard_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{

    int qcode=qemu_input_key_value_to_qcode(evt->u.key.data->key);
    int down=1-evt->u.key.data->down;

   // printf("keyboard_event:%d %d\n",qcode, evt->u.key.data->down);
    if(qcode==Q_KEY_CODE_1) {
        gpio_in_low=down;
    }
    if(qcode==Q_KEY_CODE_2) {
        gpio_in_high=down<<3;
    }

}

static QemuInputHandler gpio_keyboard_handler = {
    .name  = "GPIO Keys",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = gpio_keyboard_event,
};

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
    qemu_input_handler_register(dev, &gpio_keyboard_handler);
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static Property esp32_gpio_properties[] = {
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, ESP32_STRAP_MODE_FLASH_BOOT),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_gpio_reset;
    dc->realize = esp32_gpio_realize;
    device_class_set_props(dc, esp32_gpio_properties);
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
