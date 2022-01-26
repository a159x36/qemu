/*
 * ESP32 RMT controller
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32_rmt.h"

/*
static void update_irq(Esp32RmtState *s) {
    if (s->int_en & R_RMT_SLAVE_TRANS_INTEN_MASK) {
        if (s->slave_reg & R_RMT_SLAVE_TRANS_DONE_MASK)
            qemu_irq_raise(s->irq);
        else
            qemu_irq_lower(s->irq);
    }
}
*/
#define ESP32_RMT_REG_SIZE    0x1000

//static void esp32_rmt_cs_set(Esp32RmtState *s, int value);

static void esp32_rmt_timer_cb(void *opaque) {
    Esp32RmtState *s = ESP32_RMT(opaque);
    int channel=0;
       BusState *b = BUS(s->rmt);
                BusChild *ch = QTAILQ_FIRST(&b->children);
                SSISlave *slave = SSI_SLAVE(ch->child);
                SSISlaveClass *ssc = SSI_SLAVE_GET_CLASS(slave);
printf("sedn\n");
                for (int i = 0; i < s->txlim ; i++) {    
                    int v=s->data[(i+s->sent)%64+channel*64];
//                     printf("send %d %d\n",v,i+channel*64);

                    if(v==0) {
//                        printf("done\n");
                        break;
                    }
                    ssc->transfer(slave,v);
                }
    s->sent+=s->txlim;
        //if((s->sent%32)==0)
            s->int_raw|=(1<<(channel*3));
   // if((s->sent%32)==0)
            s->int_raw|=(1<<(channel+24));
   // if(s->int_en & s->int_raw) {
        qemu_irq_raise(s->irq);
        uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t ns_to_timeout = 10000000 * 25;  // about 75fps, same a real hw
                timer_mod_ns(&s->rmt_timer,
                        ns_now + ns_to_timeout);
   // }
    
}

//static void esp32_rmt_do_command(Esp32RmtState* state, uint32_t cmd_reg);


static uint64_t esp32_rmt_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32RmtState *s = ESP32_RMT(opaque);
    uint64_t r = 0;
    int channel=(addr-32)/16;
    switch (addr) {
    case A_RMT_CH0CONF0 ... A_RMT_CH0CONF0+8*16:
        if(addr & 0x4) 
            r=s->conf1[channel];
        else 
            r=s->conf0[channel];
        break;
    case A_RMT_INT_CLR:
    case A_RMT_INT_ST:
        r = s->int_raw;
        break;
    case A_RMT_INT_ENA:
        r = s->int_en;
        break;
    case A_RMT_TX_LIM:
        r = s->txlim;
        break;
    case A_RMT_DATA ... A_RMT_DATA+(ESP32_RMT_BUF_WORDS-1)* sizeof(uint32_t):
        r = s->data[(addr-A_RMT_DATA)/sizeof(uint32_t)];
        break;
    case A_RMT_APB_CONF:
        r = s->apb_conf;
        break;
    }
    printf("rmt read %ld %ld\n",addr,r);
    return r;
}

static void esp32_rmt_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32RmtState *s = ESP32_RMT(opaque);
    printf("rmt write %ld %ld\n",addr,value);
    int channel;
    switch (addr) {
    case A_RMT_CH0CONF0 ...  A_RMT_CH0CONF0+8*16:
        channel=(addr-32)/16;
        if((addr & 0x4)==0) {
            s->conf0[channel]=value;
        } else {
         //   int old=s->conf1[channel];
            s->conf1[channel]=value;
            if((value & 0x8)) {
         //       s->sent=0;
            }
            if((value & 0x1)) {//} && !(old & 1)) {
                 BusState *b = BUS(s->rmt);
                BusChild *ch = QTAILQ_FIRST(&b->children);
                SSISlave *slave = SSI_SLAVE(ch->child);
                SSISlaveClass *ssc = SSI_SLAVE_GET_CLASS(slave);
                printf("sedn %d\n",s->sent);
                for (int i = 0; i < s->txlim ; i++) {    
                    int v=s->data[(i+s->sent)%64+channel*64];
//                     printf("send %d %d\n",v,i+channel*64);

                    if(v==0) {
//                        printf("done\n");
                        break;
                    }
                    ssc->transfer(slave,v);
                }
                s->sent+=s->txlim;
 //               printf("Start Transfer");
                // start transfer
           //     uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            
             
               
/*
                uint64_t ns_to_timeout = 10000000 * 25;  // about 75fps, same a real hw
                if(timer_expired(&s->rmt_timer,ns_now))
                timer_mod_ns(&s->rmt_timer,
                                            ns_now + ns_to_timeout);
                                            */
           //     s->int_raw|=(1<<(channel+24));
                s->int_raw|=(1<<(channel*3));
                qemu_irq_raise(s->irq);
            }
        }

        break;
    case A_RMT_INT_ST:
        s->int_raw=value;
        break;
    case A_RMT_TX_LIM:
        s->txlim=value;
        break;
    case A_RMT_INT_ENA:
        s->int_en=value;
        break;
    case A_RMT_INT_CLR:
        s->int_raw&=(~value);
        if(s->int_raw==0)
            qemu_irq_lower(s->irq);
        break;
    case A_RMT_DATA ... A_RMT_DATA+(ESP32_RMT_BUF_WORDS-1)* sizeof(uint32_t):
        s->data[(addr-A_RMT_DATA)/sizeof(uint32_t)]=value;
 //       printf("store %ld %ld\n",(addr-A_RMT_DATA)/sizeof(uint32_t),value);
 /*
  BusState *b = BUS(s->rmt);
                BusChild *ch = QTAILQ_FIRST(&b->children);
                SSISlave *slave = SSI_SLAVE(ch->child);
                SSISlaveClass *ssc = SSI_SLAVE_GET_CLASS(slave);
 ssc->transfer(slave,value);
 */
 
        break;
    case A_RMT_APB_CONF:
        s->apb_conf=value;
        break;
    }
}


static const MemoryRegionOps esp32_rmt_ops = {
    .read =  esp32_rmt_read,
    .write = esp32_rmt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_rmt_reset(DeviceState *dev)
{
  //  Esp32RmtState *s = ESP32_RMT(dev);
    /*
    s->pin_reg = 0x6;
    s->user1_reg = FIELD_DP32(0, RMT_USER1, ADDR_BITLEN, 23);
    s->user1_reg = FIELD_DP32(s->user1_reg, RMT_USER1, DUMMY_CYCLELEN, 7);
    s->user2_reg = 0x70000000;
    s->status_reg = 0;
    */
}

static void esp32_rmt_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_rmt_init(Object *obj)
{
    Esp32RmtState *s = ESP32_RMT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_rmt_ops, s,
                          TYPE_ESP32_RMT, ESP32_RMT_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    timer_init_ns(&s->rmt_timer, QEMU_CLOCK_VIRTUAL, esp32_rmt_timer_cb, s);
    s->rmt = ssi_create_bus(DEVICE(s), "rmt");
}

static Property esp32_rmt_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_rmt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_rmt_reset;
    dc->realize = esp32_rmt_realize;
    device_class_set_props(dc, esp32_rmt_properties);
}

static const TypeInfo esp32_rmt_info = {
    .name = TYPE_ESP32_RMT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32RmtState),
    .instance_init = esp32_rmt_init,
    .class_init = esp32_rmt_class_init
};

static void esp32_rmt_register_types(void)
{
    type_register_static(&esp32_rmt_info);
}

type_init(esp32_rmt_register_types)
