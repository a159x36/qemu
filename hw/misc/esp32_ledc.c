#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "hw/misc/esp32_ledc.h"
#include "hw/irq.h"
#include "qapi/error.h"

#define ESP32_LEDC_REGS_SIZE (A_LEDC_CONF_REG + 4)

static Esp32LEDCState *current_state;

static uint64_t esp32_ledc_read(void *opaque, hwaddr addr, unsigned int size) {
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    uint64_t r = 0;
    switch (addr) {
        case A_LEDC_HSTIMER0_CONF_REG ... A_LEDC_LSTIMER3_CONF_REG:
            r = s->timer_conf_reg[(addr - A_LEDC_HSTIMER0_CONF_REG) / 0x8];
            break;
        case A_LEDC_HSCH0_CONF0_REG:
        case A_LEDC_HSCH1_CONF0_REG:
        case A_LEDC_HSCH2_CONF0_REG:
        case A_LEDC_HSCH3_CONF0_REG:
        case A_LEDC_HSCH4_CONF0_REG:
        case A_LEDC_HSCH5_CONF0_REG:
        case A_LEDC_HSCH6_CONF0_REG:
        case A_LEDC_HSCH7_CONF0_REG:
        case A_LEDC_LSCH0_CONF0_REG:
        case A_LEDC_LSCH1_CONF0_REG:
        case A_LEDC_LSCH2_CONF0_REG:
        case A_LEDC_LSCH3_CONF0_REG:
        case A_LEDC_LSCH4_CONF0_REG:
        case A_LEDC_LSCH5_CONF0_REG:
        case A_LEDC_LSCH6_CONF0_REG:
        case A_LEDC_LSCH7_CONF0_REG:
            r = s->channel_conf0_reg[(addr - A_LEDC_HSCH0_CONF0_REG) / 0x14];
            break;
        case A_LEDC_HSCH0_CONF1_REG:
        case A_LEDC_HSCH1_CONF1_REG:
        case A_LEDC_HSCH2_CONF1_REG:
        case A_LEDC_HSCH3_CONF1_REG:
        case A_LEDC_HSCH4_CONF1_REG:
        case A_LEDC_HSCH5_CONF1_REG:
        case A_LEDC_HSCH6_CONF1_REG:
        case A_LEDC_HSCH7_CONF1_REG:
        case A_LEDC_LSCH0_CONF1_REG:
        case A_LEDC_LSCH1_CONF1_REG:
        case A_LEDC_LSCH2_CONF1_REG:
        case A_LEDC_LSCH3_CONF1_REG:
        case A_LEDC_LSCH4_CONF1_REG:
        case A_LEDC_LSCH5_CONF1_REG:
        case A_LEDC_LSCH6_CONF1_REG:
        case A_LEDC_LSCH7_CONF1_REG:
            r = s->channel_conf1_reg[(addr - A_LEDC_HSCH0_CONF1_REG) / 0x14];
            break;
        case A_LEDC_HSCH0_DUTY_REG:
        case A_LEDC_HSCH1_DUTY_REG:
        case A_LEDC_HSCH2_DUTY_REG:
        case A_LEDC_HSCH3_DUTY_REG:
        case A_LEDC_HSCH4_DUTY_REG:
        case A_LEDC_HSCH5_DUTY_REG:
        case A_LEDC_HSCH6_DUTY_REG:
        case A_LEDC_HSCH7_DUTY_REG:
        case A_LEDC_LSCH0_DUTY_REG:
        case A_LEDC_LSCH1_DUTY_REG:
        case A_LEDC_LSCH2_DUTY_REG:
        case A_LEDC_LSCH3_DUTY_REG:
        case A_LEDC_LSCH4_DUTY_REG:
        case A_LEDC_LSCH5_DUTY_REG:
        case A_LEDC_LSCH6_DUTY_REG:
        case A_LEDC_LSCH7_DUTY_REG:
            r = s->duty_init_reg[(addr - A_LEDC_HSCH0_DUTY_REG) / 0x14];
            break;
        case A_LEDC_HSCH0_DUTY_R_REG:
        case A_LEDC_HSCH1_DUTY_R_REG:
        case A_LEDC_HSCH2_DUTY_R_REG:
        case A_LEDC_HSCH3_DUTY_R_REG:
        case A_LEDC_HSCH4_DUTY_R_REG:
        case A_LEDC_HSCH5_DUTY_R_REG:
        case A_LEDC_HSCH6_DUTY_R_REG:
        case A_LEDC_HSCH7_DUTY_R_REG:
        case A_LEDC_LSCH0_DUTY_R_REG:
        case A_LEDC_LSCH1_DUTY_R_REG:
        case A_LEDC_LSCH2_DUTY_R_REG:
        case A_LEDC_LSCH3_DUTY_R_REG:
        case A_LEDC_LSCH4_DUTY_R_REG:
        case A_LEDC_LSCH5_DUTY_R_REG:
        case A_LEDC_LSCH6_DUTY_R_REG:
        case A_LEDC_LSCH7_DUTY_R_REG:
            r = s->duty_reg[(addr - A_LEDC_HSCH0_DUTY_R_REG) / 0x14];
            break;
        case A_LEDC_INT_RAW:
            r = s->int_raw;
            break;
        case A_LEDC_INT_ST:
            r = s->int_raw & s->int_en;
            break;
        case A_LEDC_INT_ENA:
            r = s->int_en;
            break;
        case A_LEDC_CONF:
            r = s->ledc_conf;
            break;
    }
    // printf("esp32_ledc_read %lx,%lx\n",addr,r);
    return r;
}

static uint32_t esp32_ledc_get_percent(Esp32LEDCState *s, uint32_t value,
                                       hwaddr addr) {
    uint32_t duty_val = (value >> 4) & ((1 << 20) - 1);
    uint32_t duty_res;
    if (((addr - A_LEDC_HSCH0_DUTY_REG) / 0x14) < 8) {
        /* get duty res for the high speed channel from high speed timer */
        duty_res = s->duty_res[(
            s->channel_conf0_reg[(addr - A_LEDC_HSCH0_DUTY_REG) / 0x14] &
            ((1 << 2) - 1))];
    } else {
        /* get duty res for the low speed channel from low speed timer */
        duty_res =
            s->duty_res[((s->channel_conf0_reg[(addr - A_LEDC_HSCH0_DUTY_REG) /
                                               0x14]) &
                         ((1 << 2) - 1)) +
                        4];
    }
    return duty_res ? (100 * duty_val / ((2 << (duty_res - 1)) - 1)) : 0;
}

// calculate on and off time in microseconds
// index is the channel index 0-7 = HS, 8-15 = LS
static void get_duty_time(Esp32LEDCState *s, int index, uint32_t *on_time,
                          uint32_t *off_time) {
    int timer = s->channel_conf0_reg[index] & 3;
    if (index > 7) timer += 4;
    int expire = (s->duty_reg[index] >> 4) & ((1 << 20) - 1);
    uint32_t duty_res;
    if (index < 8) {
        duty_res = s->duty_res[(s->channel_conf0_reg[index] & ((1 << 2) - 1))];
    } else {
        duty_res =
            s->duty_res[(s->channel_conf0_reg[index] & ((1 << 2) - 1)) + 4];
    }
    int duty_max = 2 << (duty_res - 1);
    int divider = (s->timer_conf_reg[timer] >> 5) & ((1 << 17) - 1);
    int clk = 1000000;
    if (s->timer_conf_reg[timer] & (1 << 25)) {
        clk = 80000000;
        if (timer > 4) {  // low speed only can use 8MHz clock
            if (s->ledc_conf & 1) {
                clk = 8000000;
            }
        }
    }
    if ((divider * duty_max / 256) == 0) return;
    // int f=clk/(divider*duty_max/256);
    // if(f==0) return;
    // int64_t timeus=1000000000l/f;
    int64_t timeus = (divider * duty_max / 256) * (1000000000l / clk);
    *on_time = expire * timeus / duty_max;
    if (*on_time == 0) {
        *on_time = 1000;
    }
    if (*on_time > timeus - 1000) {
        *on_time = timeus - 1000;
    }
    *off_time = timeus - *on_time;
    // printf("duty time %d %d\n",*on_time,*off_time);
}

static void esp32_ledc_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    // printf("esp32_ledc_write %lx,%lx\n",addr,value);
    int index;
    switch (addr) {
        case A_LEDC_HSTIMER0_CONF_REG ... A_LEDC_LSTIMER3_CONF_REG:
            /* get duty resolution from timer config */
            if (((uint32_t)value & ((1 << 4) - 1)) != 0) {
                s->duty_res[(addr - A_LEDC_HSTIMER0_CONF_REG) / 0x8] =
                    (uint32_t)value & ((1 << 4) - 1);
            }
            s->timer_conf_reg[(addr - A_LEDC_HSTIMER0_CONF_REG) / 0x8] = value;
            break;
        case A_LEDC_HSCH0_CONF0_REG:
        case A_LEDC_HSCH1_CONF0_REG:
        case A_LEDC_HSCH2_CONF0_REG:
        case A_LEDC_HSCH3_CONF0_REG:
        case A_LEDC_HSCH4_CONF0_REG:
        case A_LEDC_HSCH5_CONF0_REG:
        case A_LEDC_HSCH6_CONF0_REG:
        case A_LEDC_HSCH7_CONF0_REG:
        case A_LEDC_LSCH0_CONF0_REG:
        case A_LEDC_LSCH1_CONF0_REG:
        case A_LEDC_LSCH2_CONF0_REG:
        case A_LEDC_LSCH3_CONF0_REG:
        case A_LEDC_LSCH4_CONF0_REG:
        case A_LEDC_LSCH5_CONF0_REG:
        case A_LEDC_LSCH6_CONF0_REG:
        case A_LEDC_LSCH7_CONF0_REG:
            s->channel_conf0_reg[(addr - A_LEDC_HSCH0_CONF0_REG) / 0x14] =
                value;
            break;
        case A_LEDC_HSCH0_CONF1_REG:
        case A_LEDC_HSCH1_CONF1_REG:
        case A_LEDC_HSCH2_CONF1_REG:
        case A_LEDC_HSCH3_CONF1_REG:
        case A_LEDC_HSCH4_CONF1_REG:
        case A_LEDC_HSCH5_CONF1_REG:
        case A_LEDC_HSCH6_CONF1_REG:
        case A_LEDC_HSCH7_CONF1_REG:
        case A_LEDC_LSCH0_CONF1_REG:
        case A_LEDC_LSCH1_CONF1_REG:
        case A_LEDC_LSCH2_CONF1_REG:
        case A_LEDC_LSCH3_CONF1_REG:
        case A_LEDC_LSCH4_CONF1_REG:
        case A_LEDC_LSCH5_CONF1_REG:
        case A_LEDC_LSCH6_CONF1_REG:
        case A_LEDC_LSCH7_CONF1_REG:
            s->channel_conf1_reg[(addr - A_LEDC_HSCH0_CONF1_REG) / 0x14] =
                value;
            break;
        case A_LEDC_HSCH0_DUTY_REG:
        case A_LEDC_HSCH1_DUTY_REG:
        case A_LEDC_HSCH2_DUTY_REG:
        case A_LEDC_HSCH3_DUTY_REG:
        case A_LEDC_HSCH4_DUTY_REG:
        case A_LEDC_HSCH5_DUTY_REG:
        case A_LEDC_HSCH6_DUTY_REG:
        case A_LEDC_HSCH7_DUTY_REG:
        case A_LEDC_LSCH0_DUTY_REG:
        case A_LEDC_LSCH1_DUTY_REG:
        case A_LEDC_LSCH2_DUTY_REG:
        case A_LEDC_LSCH3_DUTY_REG:
        case A_LEDC_LSCH4_DUTY_REG:
        case A_LEDC_LSCH5_DUTY_REG:
        case A_LEDC_LSCH6_DUTY_REG:
        case A_LEDC_LSCH7_DUTY_REG:
            index = (addr - A_LEDC_HSCH0_DUTY_REG) / 0x14;
            s->duty_reg[index] = value;
            s->duty_init_reg[index] = value;
            uint32_t on_time, off_time;
            get_duty_time(s, index, &on_time, &off_time);
            led_set_intensity(&s->led[index],
                              esp32_ledc_get_percent(s, value, addr));
            s->op_val[index] = 1;
            qemu_set_irq(s->func_irq,
                         (71 + index) * 2 + 1 + ((on_time / 1000) << 10));
            s->cycle[index] = 0;
            timer_mod_anticipate_ns(
                &s->led_timer[index],
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + on_time);
            break;
        case A_LEDC_INT_ENA:
            s->int_en = value;
            if (s->int_en & s->int_raw)
                qemu_irq_raise(s->irq);
            else
                qemu_irq_lower(s->irq);
            break;
        case A_LEDC_INT_CLR:
            s->int_raw &= (~value);
            if ((s->int_raw & s->int_en) == 0) {
                qemu_irq_lower(s->irq);
            }
            break;
        case A_LEDC_CONF:
            s->ledc_conf = value;
            break;
    }
}

static const MemoryRegionOps esp32_ledc_ops = {
    .read = esp32_ledc_read,
    .write = esp32_ledc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_ledc_realize(DeviceState *dev, Error **errp) {
    Esp32LEDCState *s = ESP32_LEDC(dev);
    for (int i = 0; i < ESP32_LEDC_CHANNEL_CNT; i++) {
        qdev_realize(DEVICE(&s->led[i]), NULL, &error_fatal);
    }
}

static void ledc_timer_cb(void *v) {
    long long int index = (long long int)v;
    Esp32LEDCState *s = current_state;
    uint32_t on_time, off_time;
    get_duty_time(s, index, &on_time, &off_time);
    if (s->op_val[index] == 0) {
        s->op_val[index] = 1;
        qemu_set_irq(s->func_irq,
                     (71 + index) * 2 + 1 + ((on_time / 1000) << 10));
        timer_mod_anticipate_ns(
            &s->led_timer[index],
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + on_time);
    } else {
        s->op_val[index] = 0;
        qemu_set_irq(s->func_irq, (71 + index) * 2 + ((off_time / 1000) << 10));
        timer_mod_anticipate_ns(
            &s->led_timer[index],
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + off_time);
        uint32_t c1 = s->channel_conf1_reg[index];
        int duty_num = (c1 >> 20) & 0x3ff;
        int duty_cycle = (c1 >> 10) & 0x3ff;
        int duty_scale = (c1 & 0x3ff) * 16;
        int duty_inc = c1 & (1 << 30);
        if (duty_num != 0) {  // fade control
            s->cycle[index]++;
            // printf("fade %d %d %d
            // %d\n",duty_num,duty_cycle,duty_scale/16,s->duty_reg[index]/16);
            if (s->cycle[index] >= duty_cycle) {
                s->cycle[index] = 0;
                if (duty_inc) {
                    s->duty_reg[index] += duty_scale;
                } else {
                    s->duty_reg[index] -= duty_scale;
                }
                duty_num--;
                if (duty_num < 0) duty_num = 0;
                if (duty_num == 0 && (s->int_en & (1 << (index + 8)))) {
                    s->int_raw |= (1 << (index + 8));
                    qemu_irq_raise(s->irq);
                }
                s->channel_conf1_reg[index] =
                    (s->channel_conf1_reg[index] & 0xc00fffff) | duty_num << 20;
            }
        }
    }
}

static void esp32_ledc_init(Object *obj) {
    Esp32LEDCState *s = ESP32_LEDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    memory_region_init_io(&s->iomem, obj, &esp32_ledc_ops, s, TYPE_ESP32_LEDC,
                          ESP32_LEDC_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    for (int i = 0; i < ESP32_LEDC_CHANNEL_CNT; i++) {
        object_initialize_child(obj, g_strdup_printf("led%d", i + 1),
                                &s->led[i], TYPE_LED);
        s->led[i].color = (char *)"blue";
    }
    current_state = s;
    for (int i = 0; i < ESP32_LEDC_CHANNEL_CNT; i++) {
        timer_init_ns(&s->led_timer[i], QEMU_CLOCK_VIRTUAL, ledc_timer_cb,
                      (void *)(long long)i);
    }
    qdev_init_gpio_out_named(DEVICE(s), &s->func_irq, "func_irq", 1);
}

static void esp32_ledc_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = esp32_ledc_realize;
}

static const TypeInfo esp32_ledc_info = {
    .name = TYPE_ESP32_LEDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32LEDCState),
    .instance_init = esp32_ledc_init,
    .class_init = esp32_ledc_class_init};

static void esp32_ledc_register_types(void) {
    type_register_static(&esp32_ledc_info);
}

type_init(esp32_ledc_register_types)
