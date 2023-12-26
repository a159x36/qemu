#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/misc/led.h"
#include "qemu/timer.h"

#define TYPE_ESP32_LEDC "misc.esp32.ledc"
#define ESP32_LEDC(obj) OBJECT_CHECK(Esp32LEDCState, (obj), TYPE_ESP32_LEDC)
#define ESP32_LEDC_TIMER_CNT 8
#define ESP32_LEDC_CHANNEL_CNT 16

typedef struct Esp32LEDCState {
    SysBusDevice parent_object;
    MemoryRegion iomem;
    uint32_t duty_res[ESP32_LEDC_TIMER_CNT];
    uint32_t timer_conf_reg[ESP32_LEDC_TIMER_CNT];
    uint32_t channel_conf0_reg[ESP32_LEDC_CHANNEL_CNT];
    uint32_t channel_conf1_reg[ESP32_LEDC_CHANNEL_CNT];
    int32_t duty_reg[ESP32_LEDC_CHANNEL_CNT];
    uint32_t cycle[ESP32_LEDC_CHANNEL_CNT];
    uint32_t duty_init_reg[ESP32_LEDC_CHANNEL_CNT];
    uint32_t ledc_conf;
    LEDState led[ESP32_LEDC_CHANNEL_CNT];
    QEMUTimer led_timer[ESP32_LEDC_CHANNEL_CNT];
    uint32_t op_val[ESP32_LEDC_CHANNEL_CNT];
    qemu_irq func_irq;
    qemu_irq irq;
    uint32_t int_raw;
    uint32_t int_en;
} Esp32LEDCState;

REG32(LEDC_CONF_REG, 0x190)

#define LEDC_REG_GROUP(name, base) \
    REG32(name ## _CONF0_REG, (base)) \
    REG32(name ## _CONF1_REG, ((base) + 0x00C)) \
    REG32(name ## _DUTY_REG, ((base) + 0x008)) \
    REG32(name ## _DUTY_R_REG, ((base) + 0x010))

#define LEDC_TIMER_REG_GROUP(name, base) \
    REG32(name ## _CONF_REG, (base)) \
    REG32(name ## _VALUE_REG, ((base) + 0x004))

LEDC_REG_GROUP(LEDC_HSCH0, 0x000)
LEDC_REG_GROUP(LEDC_HSCH1, 0x014)
LEDC_REG_GROUP(LEDC_HSCH2, 0x028)
LEDC_REG_GROUP(LEDC_HSCH3, 0x03C)
LEDC_REG_GROUP(LEDC_HSCH4, 0x050)
LEDC_REG_GROUP(LEDC_HSCH5, 0x064)
LEDC_REG_GROUP(LEDC_HSCH6, 0x078)
LEDC_REG_GROUP(LEDC_HSCH7, 0x08C)

LEDC_REG_GROUP(LEDC_LSCH0, 0x0A0)
LEDC_REG_GROUP(LEDC_LSCH1, 0x0B4)
LEDC_REG_GROUP(LEDC_LSCH2, 0x0C8)
LEDC_REG_GROUP(LEDC_LSCH3, 0x0DC)
LEDC_REG_GROUP(LEDC_LSCH4, 0x0F0)
LEDC_REG_GROUP(LEDC_LSCH5, 0x104)
LEDC_REG_GROUP(LEDC_LSCH6, 0x118)
LEDC_REG_GROUP(LEDC_LSCH7, 0x12C)

LEDC_TIMER_REG_GROUP(LEDC_HSTIMER0, 0x140)
LEDC_TIMER_REG_GROUP(LEDC_HSTIMER1, 0x148)
LEDC_TIMER_REG_GROUP(LEDC_HSTIMER2, 0x150)
LEDC_TIMER_REG_GROUP(LEDC_HSTIMER3, 0x158)

LEDC_TIMER_REG_GROUP(LEDC_LSTIMER0, 0x160)
LEDC_TIMER_REG_GROUP(LEDC_LSTIMER1, 0x168)
LEDC_TIMER_REG_GROUP(LEDC_LSTIMER2, 0x170)
LEDC_TIMER_REG_GROUP(LEDC_LSTIMER3, 0x178)

REG32(LEDC_INT_RAW, 0x180)
REG32(LEDC_INT_ST, 0x184)
REG32(LEDC_INT_ENA, 0x188)
REG32(LEDC_INT_CLR, 0x18C)
REG32(LEDC_CONF, 0x190)
