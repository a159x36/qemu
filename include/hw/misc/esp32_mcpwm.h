#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/misc/esp32_reg.h"
#include "qemu/timer.h"

#define TYPE_ESP32_MCPWM "misc.esp32.mcpwm"
#define ESP32_MCPWM(obj) OBJECT_CHECK(Esp32McpwmState, (obj), TYPE_ESP32_MCPWM)
#define ESP32_MCPWM_TIMER_CNT 3
#define ESP32_MCPWM_OPERATOR_CNT 3

struct Esp32McpwmState;

typedef struct PwmTimer {
    struct Esp32McpwmState *s;
    QEMUTimer timer;
    uint64_t on_time;
    uint64_t off_time;
    uint32_t op_val;
    int index;
} PwmTimer;

typedef struct Esp32McpwmState {
    SysBusDevice parent_object;
    MemoryRegion iomem;
    uint32_t func_sig_start;
    PwmTimer mcpwm_timer[ESP32_MCPWM_TIMER_CNT*2];
    int timersel[ESP32_MCPWM_OPERATOR_CNT];
    uint32_t prescaler;
    uint32_t op_timersel;
    uint32_t timer_cfg0[ESP32_MCPWM_TIMER_CNT];
    uint32_t timer_cfg1[ESP32_MCPWM_TIMER_CNT];
    uint32_t op_gen_tstmp_a[ESP32_MCPWM_OPERATOR_CNT];
    uint32_t op_gen_tstmp_b[ESP32_MCPWM_OPERATOR_CNT];
    uint32_t op_gen_a[ESP32_MCPWM_OPERATOR_CNT];
    uint32_t op_gen_b[ESP32_MCPWM_OPERATOR_CNT];
    qemu_irq func_irq;
    qemu_irq irq;
    uint32_t int_raw;
    uint32_t int_en;
} Esp32McpwmState;

REG32(PWM_CLK_CFG_REG, 0x000)

#define MCPWM_OP_REG_GROUP(name, base) \
    REG32(name ## _GEN_STMP_CFG_REG, (base)) \
    REG32(name ## _GEN_TSTMP_A_REG, ((base) + 0x004)) \
    REG32(name ## _GEN_TSTMP_B_REG, ((base) + 0x008)) \
    REG32(name ## _GEN_CFG0_REG, ((base) + 0x00c)) \
    REG32(name ## _GEN_FORCE_REG, ((base) + 0x010)) \
    REG32(name ## _GEN_A_REG, ((base) + 0x014)) \
    REG32(name ## _GEN_B_REG, ((base) + 0x018)) \
    REG32(name ## _DT_CFG_REG, ((base) + 0x01c)) \
    REG32(name ## _DT_FED_CFG_REG, ((base) + 0x020)) \
    REG32(name ## _DT_RED_CFG_REG, ((base) + 0x024)) \
    REG32(name ## _CARRIER_CFG_REG, ((base) + 0x028)) \
    REG32(name ## _FH_CFG0_REG, ((base) + 0x02c)) \
    REG32(name ## _FH_CFG1_REG, ((base) + 0x030)) \
    REG32(name ## _FH_STATUS_REG, ((base) + 0x034)) 

#define MCPWM_TIMER_REG_GROUP(name, base) \
    REG32(name ## _CFG0_REG, (base)) \
    REG32(name ## _CFG1_REG, (base) + 0x004) \
    REG32(name ## _SYNC_REG, ((base) + 0x008)) \
    REG32(name ## _STATUS_REG, ((base) + 0x00C))

MCPWM_TIMER_REG_GROUP(PWM_TIMER0, 0x004)
MCPWM_TIMER_REG_GROUP(PWM_TIMER1, 0x014)
MCPWM_TIMER_REG_GROUP(PWM_TIMER2, 0x024)

REG32(PWM_TIMER_SYNCI_CFG_REG, 0x034)
REG32(PWM_OPERATOR_TIMERSEL_REG, 0x038)

MCPWM_OP_REG_GROUP(PWM_OP0, 0x03C)
MCPWM_OP_REG_GROUP(PWM_OP1, 0x074)
MCPWM_OP_REG_GROUP(PWM_OP2, 0x0AC)

REG32(PWM_FAULT_DETECT_REG, 0x0E4)
REG32(PWM_CAP_TIMER_CFG_REG, 0x0E8)
REG32(PWM_CAP_TIMER_PHASE_REG, 0x0EC)
REG32(PWM_CAP_CH0_CFG_REG, 0x0F0)
REG32(PWM_CAP_CH1_CFG_REG, 0x0F4)
REG32(PWM_CAP_CH2_CFG_REG, 0x0F8)
REG32(PWM_CAP_CH0_REG, 0x0FC)
REG32(PWM_CAP_CH1_REG, 0x100)
REG32(PWM_CAP_CH2_REG, 0x104)
REG32(PWM_CAP_STATUS_REG, 0x108)
REG32(PWM_UPDATE_CFG_REG, 0x10C)

REG32(PWM_INT_ENA_REG, 0x110)
REG32(PWM_INT_RAW_REG, 0x114)
REG32(PWM_INT_ST_REG, 0x118)
REG32(PWM_INT_CLR_REG, 0x11C)
