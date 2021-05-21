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
#include "ui/console.h"
#include "hw/hw.h"
#include "ui/input.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"
#include "sysemu/runstate.h"

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;
    switch (addr) {
        case 4:
            r = s->gpio_out;
            break;
        case A_GPIO_STRAP:
            r = s->strap_mode;
            break;
        case 0x3C:  // in_low
            r = s->gpio_in;
            break;
        case 0x40:  // in_high
            r = s->gpio_in1;
            break;
        case 0x44:
            r = s->gpio_status;
            break;
        case 0x50:
            r = s->gpio_status1;
            break;
        case 0x60:
            r = s->gpio_acpu_int;
            break;
        case 0x68:
            r = s->gpio_pcpu_int;
            break;
        case 0x74:
            r = s->gpio_acpu_int1;
            break;
        case 0x7c:
            r = s->gpio_pcpu_int1;
            break;
        default:
            break;
    }
    if (addr >= 0x88 && addr < 0x130) {
        int n = (addr - 0x88) / 4;
        r = s->gpio_pin[n];
    }
    return r;
}

static int get_triggering(int int_type, int oldval, int val) {
    switch (int_type) {
        case 1:
            return (val > oldval);
        case 2:
            return (val < oldval);
        case 3:
            return (val != oldval);
        case 4:
            return (val == 0);
        case 5:
            return (val == 1);
    }
    return 0;
}
static void set_gpio(void *opaque, int n, int val) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    if (n < 32) {
        int oldval = (s->gpio_in >> n) & 1;
        int int_type = (s->gpio_pin[n] >> 7) & 7;
        s->gpio_in &= ~(1 << n);
        s->gpio_in |= (val << n);
        int irq = get_triggering(int_type, oldval, val);
        // says bit 16 in the ref manual, is that wrong?
        if (irq && (s->gpio_pin[n] & (1 << 15))) {  // pro cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_pcpu_int |= (1 << n);
        }
        if (irq && (s->gpio_pin[n] & (1 << 13))) {  // app cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_acpu_int |= (1 << n);
        }
    } else {
        int n1 = n - 32;
        int oldval = (s->gpio_in1 >> n1) & 1;
        int int_type = (s->gpio_pin[n] >> 7) & 7;
        s->gpio_in1 &= ~(1 << n1);
        s->gpio_in1 |= (val << n1);
        int irq = get_triggering(int_type, oldval, val);
        // says bit 16 in the ref manual, is that wrong?
        if (irq && (s->gpio_pin[n] & (1 << 15))) {  // pro cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_pcpu_int1 |= (1 << n1);
        }
        if (irq && (s->gpio_pin[n] & (1 << 13))) {  // app cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_acpu_int1 |= (1 << n1);
        }
    }
}
extern const struct {
    int width;
    int height;
} ttgo_board_skin;

static void esp32_gpio_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    int clearirq;
    uint32_t oldvalue;
    oldvalue = s->gpio_out;
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
        case A_GPIO_STRAP:
            s->strap_mode = value;
            break;
        case 0x44:
            s->gpio_status = value;
            break;
        case 0x48:
            s->gpio_status |= value;
            break;
        case 0x4c:
            clearirq = 1;
            for (int i = 0; i < 32; i++) {
                if ((1 << i) & value) {
                    int int_type = (s->gpio_pin[i] >> 7) & 7;
                    if ((int_type == 4 && !(s->gpio_in & (1 << i))) ||
                        (int_type == 5 && (s->gpio_in & (1 << i))))
                        clearirq = 0;
                }
            }
            if (clearirq) {
                s->gpio_status &= ~value;
                s->gpio_pcpu_int &= ~value;
                s->gpio_acpu_int &= ~value;
                qemu_set_irq(s->irq, 0);
            }
            break;
        case 0x50:
            s->gpio_status1 = value;
            break;
        case 0x54:
            s->gpio_status1 |= value;
            break;
        case 0x58:
            clearirq = 1;
            for (int i = 0; i < 32; i++) {
                if ((1 << i) & value) {
                    int int_type = (s->gpio_pin[i + 32] >> 7) & 7;
                    if ((int_type == 4 && !(s->gpio_in1 & (1 << i))) ||
                        (int_type == 5 && (s->gpio_in1 & (1 << i))))
                        clearirq = 0;
                }
            }
            if (clearirq) {
                s->gpio_status1 &= ~value;
                s->gpio_pcpu_int1 &= ~value;
                s->gpio_acpu_int1 &= ~value;
                qemu_set_irq(s->irq, 0);
            }
            break;
    }
    if (addr >= 0x88 && addr < 0x130) {
        int n = (addr - 0x88) / 4;
        s->gpio_pin[n] = value;
    }

    if (addr > 0x530 && addr < 0x5d0) {
        //    printf("IOMUX %lx %lx\n",(addr-0x530)/4,value);
        //    if(value==10)
        //        printf("Connect GPIO %ld to HSPI_DATA\n",(addr-0x530)/4);
    }

    if (s->gpio_out != oldvalue) {
        uint32_t diff = (s->gpio_out ^ oldvalue);
        for (int i = 0; i < 32; i++) {
            if ((1 << i) & diff) {
                qemu_set_irq(s->gpios[i], (s->gpio_out & (1 << i)) ? 1 : 0);
            }
        }
    }
}

static const MemoryRegionOps uart_ops = {
    .read = esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset(DeviceState *dev) {}

static void esp32_gpio_realize(DeviceState *dev, Error **errp) {}

static void esp32_gpio_init(Object *obj) {
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s, TYPE_ESP32_GPIO,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(s), &s->irq, SYSBUS_DEVICE_GPIO_IRQ, 1);
    qdev_init_gpio_out_named(DEVICE(s), s->gpios, ESP32_GPIOS, 32);
    qdev_init_gpio_in_named(DEVICE(s), set_gpio, ESP32_GPIOS_IN, 40);
    s->gpio_in = 0x1;
    s->gpio_in1 = 0x8;
}

static Property esp32_gpio_properties[] = {
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode,
                       ESP32_STRAP_MODE_FLASH_BOOT),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data) {
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
    .class_init = esp32_gpio_class_init};

static void esp32_gpio_register_types(void) {
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
