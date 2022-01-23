#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_ramdev.h"

static uint64_t esp32_ramdev_read(void *opaque, hwaddr addr, unsigned int size)
{
    uint32_t r = 0;
    Esp32RamdevState *s = ESP32_RAMDEV(opaque);
    r=s->mem[addr/4];
    return r;
}

static void esp32_ramdev_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size) {
  Esp32RamdevState *s = ESP32_RAMDEV(opaque);
  s->mem[addr/4]=(uint32_t)value;
}

static const MemoryRegionOps esp32_ramdev_ops = {
    .read =  esp32_ramdev_read,
    .write = esp32_ramdev_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_ramdev_init(Object *obj)
{
    Esp32RamdevState *s = ESP32_RAMDEV(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_ramdev_ops, s,
                          TYPE_ESP32_RAMDEV, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    memset(s->mem,0,sizeof(s->mem));
}


static const TypeInfo esp32_ramdev_info = {
    .name = TYPE_ESP32_RAMDEV,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32RamdevState),
    .instance_init = esp32_ramdev_init,
};

static void esp32_ramdev_register_types(void)
{
    type_register_static(&esp32_ramdev_info);
}

type_init(esp32_ramdev_register_types)
