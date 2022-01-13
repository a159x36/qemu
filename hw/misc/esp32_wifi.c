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
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_wifi.h"
#include "exec/address-spaces.h"
#include "esp32_wlan_packet.h"


static uint64_t esp32_wifi_read(void *opaque, hwaddr addr, unsigned int size)
{
    
  //  printf("esp32_wifi_read %ld %p\n",addr,opaque);
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t r = s->mem[addr/4];
    
    switch(addr) {
        case 132:
            r=0;
            break;
       // case 3072:
       //     r=-1;
       //     break;
        case 3144:
            r=s->event;
            break;
        case 3272:
            r=31;
            break;
        case 3364:
            r=1;
            break;
    }
    printf("esp32_wifi_read %ld=%d\n",addr,r);
    return r;
}
static void setEvent(Esp32WifiState *s,int e) {
    s->event |= e;
   // qemu_set_irq(s->irq, 0);
    qemu_irq_raise(s->irq);
//    qemu_irq_pulse(s->irq);
}
void Esp32_WLAN_insert_frame(Esp32WifiState *s, struct mac80211_frame *frame);
extern int wifi_channel;
static void esp32_wifi_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size) {
    Esp32WifiState *s = ESP32_WIFI(opaque);
    printf("esp32_wifi_write %ld=%ld\n",addr, value);
    switch (addr) {
                case 36:
                    if(65536 & value) s->rxInterface = 0;
                    break;
                case 44:
                    if(65536 & value) s->rxInterface = 1;
                    break;
                case 136:
                    s->rxBuffer = value;
                    break;
           //     case 3144:
           //          qemu_set_irq(s->irq,0);
           //         break;
                case 3148:
                    s->event &= ~value;
                    printf("event=%x\n",s->event);
              //      if(s->event==0) qemu_set_irq(s->irq,1);
                    if(s->event==0)
                        qemu_irq_lower(s->irq);
               //     else qemu_set_irq(s->irq,1);
                   // qemu_irq_pulse(s->irq);
                    break;
                case 3360:
                case 3352:
                case 3344:
                case 3336:
                case 3328:
                    if (3221225472 & value) {
                //        int channel= (3360 - addr) / 8;
                        
                        // a DMA transfer
                        
                        int data = 0;
                        int len;
                        uint8_t buffer[sizeof(mac80211_frame)];
                        int v[3];
                        unsigned addr = (0x3ff00000 | (value & 0xfffff));
                        address_space_read(&address_space_memory, addr,
                                   MEMTXATTRS_UNSPECIFIED, v, 12);
                        len = (v[0]>>12) & 4095;
                        data = v[1];
                        addr = v[2];
                        buffer[0]=0;
                        address_space_read(&address_space_memory, data,
                                   MEMTXATTRS_UNSPECIFIED, buffer, len);
                        struct mac80211_frame *frame=(struct mac80211_frame *)buffer;
                        printf("SendTxFrame %x %d %d %x\n",data, len,buffer[0],addr);
                        for(int i=0;i<len;i++) {
                            printf("%d: %d\n",i,buffer[i]);
                        }
                        // frame from esp32 to ap
                        
                        frame->frame_length=len-4;
                        
                     //   if(wifi_channel==6 || wifi_channel==8)
                         if(wifi_channel==AP_WIFI_CHANNEL) {
                            Esp32_WLAN_handle_frame(s, frame);
                            /*
                        mac80211_frame *framecopy=(mac80211_frame *)malloc(sizeof(mac80211_frame));
                        uint8_t *cp=(uint8_t *)framecopy;
                        for(int i=0;i<len;i++) *cp++=buffer[i];
                        framecopy->frame_length=len-4;
                        printf("framecopy=%p\n",framecopy);
                        */
                     //   if(channel==wifi_channel)
        //                Esp32_WLAN_insert_frame(s,framecopy);
                    //    s->event=128;
                    //    qemu_set_irq(s->irq,1);
                    //    qemu_set_irq(s->irq,0);
                        uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                        timer_mod_ns(&s->wifi_timer,ns_now + 10000000);
                         }
                   //     setEvent(s,128);
//                        this.onTX(o, this, t),
//                        this.cpu.schedule(this.txComplete, 1e3)
                    }
                }
    s->mem[addr/4]=value;
}
// frame from ap to esp32
void Esp32_sendFrame(Esp32WifiState *s, uint8_t *frame,int length) {
    printf("SendRxFrame %d %d %d\n",s->rxBuffer,length,frame[0]);
    
    if(s->rxBuffer==0) {
    //    setEvent(s,16777252);
        return;
    }
    uint8_t header[28+length+3];
    for(int i=0;i<sizeof(header);i++) header[i]=0;
    header[0]=(-60+96) & 255;
    header[1]=11;
    header[2]=177;
    header[3]=s->rxInterface ? 32 : 16;
    header[24]=(length + 4)&0xff;
    header[25]=((length + 4)>>8)&15;
    for(int i=0;i<length;i++) {
        header[28+i]=*frame++;
    }
    length+=28;
//    header[length-3]=3;
//    header[length-2]=1;
//    header[length-1]=6;
    int v[3];
    int data;
    int addr=s->rxBuffer;
    address_space_read(&address_space_memory, addr,
                                   MEMTXATTRS_UNSPECIFIED, v, 12);
    //int len = v[0] & 4095;
    data = v[1];
    //int next=v[2];
    printf("v: %x %x %x\n",v[0],v[1],v[2]);
    for(int i=0;i<length;i++) {
        printf("%d: %d\n",i,header[i]);
    }
    address_space_write(&address_space_memory, data, MEMTXATTRS_UNSPECIFIED, header, length);
    v[0]=(v[0]&0xFF000FFF)|(length<<12)|0x40000000;
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,v,4);
    s->rxBuffer=v[2];
  //  qemu_set_irq(s->irq,0);
    setEvent(s,16777252);
    
    
}

static void esp32_wifi_timer_cb(void *opaque) {
    Esp32WifiState *s = ESP32_WIFI(opaque);
    
    setEvent(s,128);
}

static const MemoryRegionOps esp32_wifi_ops = {
    .read =  esp32_wifi_read,
    .write = esp32_wifi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_wifi_init(Object *obj)
{
    Esp32WifiState *s = ESP32_WIFI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    s->rxBuffer=0;

    memory_region_init_io(&s->iomem, obj, &esp32_wifi_ops, s,
                          TYPE_ESP32_WIFI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    timer_init_ns(&s->wifi_timer, QEMU_CLOCK_VIRTUAL, esp32_wifi_timer_cb, s);
    memset(s->mem,0,sizeof(s->mem));
//    uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
//    timer_mod_ns(&s->wifi_timer,ns_now + 10000000);
    Esp32_WLAN_setup_ap(s);
    
}


static const TypeInfo esp32_wifi_info = {
    .name = TYPE_ESP32_WIFI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32WifiState),
    .instance_init = esp32_wifi_init,
};

static void esp32_wifi_register_types(void)
{
    type_register_static(&esp32_wifi_info);
}

type_init(esp32_wifi_register_types)
