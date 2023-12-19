#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "hw/misc/esp32_mcpwm.h"
#include "hw/irq.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"

#define ESP32_MCPWM_REGS_SIZE 0x120

#define DEBUG 0

// get on and off time for pwm signal
// there are 3 timers and 3 operators, each operator controls 2 signals, A and B which can control any gpio
// there are 2 compare registers for each operator (also called A and B for added confusion)
// the 6 output signals are pwm0a, pwm0b, pwm1a, pwm1b, pwm2a, pwm2b
// index is the timer index, returns the signal index
static int get_duty_time(Esp32McpwmState *s, int index, uint32_t *on_time, uint32_t *off_time) {
    uint64_t clock=160000000/(s->prescaler+1);
    int op=index/2;
    int timer=s->timersel[op];
    uint64_t t0=0;
    clock=clock/((s->timer_cfg0[timer] & 0xff) +1);
    uint64_t period=(s->timer_cfg0[timer] >> 8) & 0xfffff;
    period=(period*1000000000)/clock;
    if(((s->op_gen_a[op]>>4)&3) == 1 ) {// PWM_UTEA for PWMA
                t0=s->op_gen_tstmp_a[op];
                index&=0xfe;
    }
    if(((s->op_gen_a[op]>>6)&3) == 1 ) {// PWM_UTEB for PWMA
                t0=s->op_gen_tstmp_b[op];
                index&=0xfe;
    }
    if(((s->op_gen_b[op]>>4)&3) == 1 ) { // PWM_UTEA for PWMB
                t0=s->op_gen_tstmp_a[op];
                index|=1;
    }
    if(((s->op_gen_b[op]>>6)&3) == 1 ) {// PWM_UTEB for PWMB
                t0=s->op_gen_tstmp_b[op];
                index|=1;
    }
    *on_time=(t0*1000000000)/clock;
    *off_time=period-*on_time;
    if(DEBUG) printf("get_duty_time %d %d %d\n", op, *on_time,*off_time);
    return index;
}

// timer callback
static void pwm_timer_cb(void *v) {
    
    PwmTimer *ps = (PwmTimer *)v;
    int index=ps->index;    
    Esp32McpwmState *s=ps->s;
    uint32_t on_time=0, off_time=0;
    int op_index=get_duty_time(s, index, &on_time, &off_time);
    uint64_t now=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (ps->op_val == 0) {
        ps->op_val = 1;
        // this is sent to the gpio matrix to turn a gpio on for a certain time
        qemu_set_irq(s->func_irq, (s->func_sig_start + op_index) * 2 + 1 + ((on_time / 1000) << 10));
        timer_mod_ns(&ps->timer, now + on_time);
    } else {
        ps->op_val = 0;
        qemu_set_irq(s->func_irq, (s->func_sig_start + op_index) * 2 + ((off_time / 1000) << 10));
        timer_mod_ns(&ps->timer, now + off_time);
    }
}

static uint64_t esp32_mcpwm_read(void *opaque, hwaddr addr, unsigned int size) {
    Esp32McpwmState *s = ESP32_MCPWM(opaque);
    uint64_t r = 0;
    int timer_no=(addr-A_PWM_TIMER0_CFG0_REG)/0x10;
    int op_no=(addr-A_PWM_OP0_GEN_STMP_CFG_REG)/0x38;
    switch (addr) {
        case A_PWM_OPERATOR_TIMERSEL_REG:
            r=s->op_timersel;
            break;
        case A_PWM_CLK_CFG_REG:
            r=s->prescaler;
            break;
        case A_PWM_TIMER0_CFG0_REG:
        case A_PWM_TIMER1_CFG0_REG:
        case A_PWM_TIMER2_CFG0_REG:
            r=s->timer_cfg0[timer_no];
            break;
        case A_PWM_TIMER0_CFG1_REG:
        case A_PWM_TIMER1_CFG1_REG:
        case A_PWM_TIMER2_CFG1_REG:
            r=s->timer_cfg1[timer_no];
            break;
        case A_PWM_OP0_GEN_A_REG:
        case A_PWM_OP1_GEN_A_REG:
        case A_PWM_OP2_GEN_A_REG:
            r=s->op_gen_a[op_no];
            break;
        case A_PWM_OP0_GEN_B_REG:
        case A_PWM_OP1_GEN_B_REG:
        case A_PWM_OP2_GEN_B_REG:
            r=s->op_gen_b[op_no];
            break;
        case A_PWM_OP0_GEN_TSTMP_A_REG:
        case A_PWM_OP1_GEN_TSTMP_A_REG:
        case A_PWM_OP2_GEN_TSTMP_A_REG:
            r=s->op_gen_tstmp_a[op_no];
            break;
        case A_PWM_OP0_GEN_TSTMP_B_REG:
        case A_PWM_OP1_GEN_TSTMP_B_REG:
        case A_PWM_OP2_GEN_TSTMP_B_REG:
            r=s->op_gen_tstmp_b[op_no];
            break;
    }
    if(DEBUG)
        printf("esp32_mcpwm_read %lx,%lx\n",addr,r);
    return r;
}


static void esp32_mcpwm_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    Esp32McpwmState *s = ESP32_MCPWM(opaque);
    if(DEBUG) 
        printf("esp32_mcpwm_write %lx,%lx\n",addr,value);
    int timer_no=(addr-A_PWM_TIMER0_CFG0_REG)/0x10;
    int op_no=(addr-A_PWM_OP0_GEN_STMP_CFG_REG)/0x38;
    uint32_t old;
    switch (addr) {
        case A_PWM_OPERATOR_TIMERSEL_REG:
            s->op_timersel=value;
            s->timersel[0]=value&3;
            s->timersel[1]=(value>>2)&3;
            s->timersel[2]=(value>>4)&3;
            break;
        case A_PWM_CLK_CFG_REG:
            s->prescaler=value;
            break;
        case A_PWM_TIMER0_CFG0_REG:
        case A_PWM_TIMER1_CFG0_REG:
        case A_PWM_TIMER2_CFG0_REG:
            s->timer_cfg0[timer_no]=value;
            break;
        case A_PWM_TIMER0_CFG1_REG:
        case A_PWM_TIMER1_CFG1_REG:
        case A_PWM_TIMER2_CFG1_REG:
            s->timer_cfg1[timer_no]=value;
            break;
        case A_PWM_OP0_GEN_A_REG:
        case A_PWM_OP1_GEN_A_REG:
        case A_PWM_OP2_GEN_A_REG:
            s->op_gen_a[op_no]=value;
            break;
        case A_PWM_OP0_GEN_B_REG:
        case A_PWM_OP1_GEN_B_REG:
        case A_PWM_OP2_GEN_B_REG:
            s->op_gen_b[op_no]=value;
            break;
        case A_PWM_OP0_GEN_TSTMP_A_REG:
        case A_PWM_OP1_GEN_TSTMP_A_REG:
        case A_PWM_OP2_GEN_TSTMP_A_REG:
            old=s->op_gen_tstmp_a[op_no];
            s->op_gen_tstmp_a[op_no]=value;
            if(value!=0 && value!=old) {
                pwm_timer_cb(&s->mcpwm_timer[op_no*2]);
            }
            break;
        case A_PWM_OP0_GEN_TSTMP_B_REG:
        case A_PWM_OP1_GEN_TSTMP_B_REG:
        case A_PWM_OP2_GEN_TSTMP_B_REG:
            old=s->op_gen_tstmp_b[op_no];
            s->op_gen_tstmp_b[op_no]=value;
            if(value!=0 && value!=old) {
                pwm_timer_cb(&s->mcpwm_timer[op_no*2+1]);
            }            
            break;
    }
}

static const MemoryRegionOps esp32_mcpwm_ops = {
    .read = esp32_mcpwm_read,
    .write = esp32_mcpwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};


static void esp32_mcpwm_init(Object *obj) {
    Esp32McpwmState *s = ESP32_MCPWM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    memory_region_init_io(&s->iomem, obj, &esp32_mcpwm_ops, s, TYPE_ESP32_MCPWM,
                          ESP32_MCPWM_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    for (int i = 0; i < ESP32_MCPWM_TIMER_CNT*2; i++) {
            s->mcpwm_timer[i].index=i;
            s->mcpwm_timer[i].s=s;
            timer_init_ns(&s->mcpwm_timer[i].timer, QEMU_CLOCK_VIRTUAL, pwm_timer_cb,
                      (void *)&s->mcpwm_timer[i]);
    }
    qdev_init_gpio_out_named(DEVICE(s), &s->func_irq, "func_irq", 1);
}

static Property esp32_mcpwm_properties[] = {
    DEFINE_PROP_UINT32("func_sig_start",Esp32McpwmState,func_sig_start,0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_mcpwm_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    device_class_set_props(dc, esp32_mcpwm_properties);
}

static const TypeInfo esp32_mcpwm_info = {
    .name = TYPE_ESP32_MCPWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32McpwmState),
    .instance_init = esp32_mcpwm_init,
    .class_init = esp32_mcpwm_class_init};

static void esp32_mcpwm_register_types(void) {
    type_register_static(&esp32_mcpwm_info);
}

type_init(esp32_mcpwm_register_types)
