#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj)             OBJECT_CHECK(Esp32GpioState, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32GpioClass, obj, TYPE_ESP32_GPIO)
#define ESP32_GPIO_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32GpioClass, klass, TYPE_ESP32_GPIO)

REG32(GPIO_OUT, 0x04)
REG32(GPIO_OUT_W1TS, 0x08)
REG32(GPIO_OUT_W1TC, 0x0c)
REG32(GPIO_ENABLE, 0x020)
REG32(GPIO_ENABLE_W1TS, 0x024)
REG32(GPIO_ENABLE_W1TC, 0x028)
REG32(GPIO_STRAP, 0x0038)
REG32(GPIO_IN, 0x003c)
REG32(GPIO_IN1, 0x0040)
REG32(GPIO_STATUS, 0x0044)
REG32(GPIO_STATUS_W1TS, 0x0048)
REG32(GPIO_STATUS_W1TC, 0x004c)
REG32(GPIO_STATUS1, 0x0050)
REG32(GPIO_STATUS1_W1TS, 0x0054)
REG32(GPIO_STATUS1_W1TC, 0x0058)
REG32(GPIO_ACPU_INT, 0x0060)
REG32(GPIO_PCPU_INT, 0x0068)
REG32(GPIO_ACPU_INT1, 0x0074)
REG32(GPIO_PCPU_INT1, 0x007c)
REG32(GPIO_PIN_BASE,0x88)
REG32(GPIO_FUNC_IN_SEL_CFG_BASE,0x130)
REG32(GPIO_FUNC_OUT_SEL_CFG_BASE,0x530)

#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT  0x0f

typedef struct Esp32GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t gpio_out;
    uint32_t gpio_out1;
    uint32_t strap_mode;
    uint32_t gpio_in;
    uint32_t gpio_in1;
    uint32_t gpio_status;
    uint32_t gpio_status1;
    uint32_t gpio_pcpu_int;
    uint32_t gpio_pcpu_int1;
    uint32_t gpio_acpu_int;
    uint32_t gpio_acpu_int1;
    uint32_t gpio_enable;
    uint32_t gpio_pin[40];
    uint32_t gpio_in_sel[256];
    uint32_t gpio_out_sel[40];
    qemu_irq gpios[32];
} Esp32GpioState;

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;
} Esp32GpioClass;

#define ESP32_GPIOS "esp32_gpios"
#define ESP32_GPIOS_IN "esp32_gpios_in"
#define ESP32_GPIOS_FUNC "esp32_gpios_func"
