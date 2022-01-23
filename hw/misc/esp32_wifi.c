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
#include "hw/qdev-properties.h"

static uint64_t esp32_wifi_read(void *opaque, hwaddr addr, unsigned int size)
{
    
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t r = s->mem[addr/4];
    
    switch(addr) {
        case A_WIFI_DMA_IN_STATUS:
            r=0;
            break;
        case A_WIFI_DMA_INT_STATUS:
        case A_WIFI_DMA_INT_CLR:
            r=s->event;
            break;
        case A_WIFI_STATUS:
        case A_WIFI_DMA_OUT_STATUS:
            r=1;
            break;
    }
//    printf("esp32_wifi_read %ld=%d\n",addr,r);

    return r;
}
static void setEvent(Esp32WifiState *s,int e) {
    s->event |= e;
    qemu_set_irq(s->irq, 1);
}
void Esp32_WLAN_insert_frame(Esp32WifiState *s, struct mac80211_frame *frame);

static void esp32_wifi_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size) {
    Esp32WifiState *s = ESP32_WIFI(opaque);
//    printf("esp32_wifi_write %ld=%ld\n",addr, value);

    switch (addr) {
        case A_WIFI_DMA_INLINK:
            s->rxBuffer = value;
            break;
        case A_WIFI_DMA_INT_CLR:
            s->event &= ~value;
            if(s->event==0)
                qemu_set_irq(s->irq, 0);
            break;
        case A_WIFI_DMA_OUTLINK:
            if (value & 0xc0000000) {                        
                // a DMA transfer
                int data = 0;
                int len;
                uint8_t *buffer=malloc(sizeof(struct mac80211_frame));
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
                // frame from esp32 to ap
                frame->frame_length=len-4;
                frame->next_frame=0;
                if(wifi_channel==AP_WIFI_CHANNEL || (wifi_channel==AP_WIFI_CHANNEL1) || (wifi_channel==AP_WIFI_CHANNEL2))
                    Esp32_WLAN_handle_frame(s, frame);
                free(buffer);
                setEvent(s,128);
            }
    }
    s->mem[addr/4]=value;
}
// frame from ap to esp32
void Esp32_sendFrame(Esp32WifiState *s, uint8_t *frame,int length) {    
    if(s->rxBuffer==0) {
        return;
    }
    uint8_t header[28+length];
    for(int i=0;i<sizeof(header);i++) header[i]=0;
    header[0]=(-66+(rand()%10)+96) & 255;
    header[1]=11;
    header[2]=177;
    header[3]=s->rxInterface ? 32 : 16;
    header[24]=(length + 4)&0xff;
    header[25]=((length + 4)>>8)&15;
    for(int i=0;i<length;i++) {
        header[28+i]=*frame++;
    }
    length+=28;
/*
    for(int i=0;i<length;i++) {
        printf("%d: %d\n",i,header[i]);
    }
    */
    int v[3];
    int data;
    int addr=s->rxBuffer;
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED, v, 12);
    data = v[1];
    address_space_write(&address_space_memory, data, MEMTXATTRS_UNSPECIFIED, header, length);
    v[0]=(v[0]&0xFF000FFF)|(length<<12)|0x40000000;
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,v,4);
    s->rxBuffer=v[2];
    setEvent(s,0x1000024);
}

static const MemoryRegionOps esp32_wifi_ops = {
    .read =  esp32_wifi_read,
    .write = esp32_wifi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_wifi_realize(DeviceState *dev, Error **errp)
{
    Esp32WifiState *s = ESP32_WIFI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    s->rxBuffer=0;

    memory_region_init_io(&s->iomem, OBJECT(dev), &esp32_wifi_ops, s,
                          TYPE_ESP32_WIFI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    memset(s->mem,0,sizeof(s->mem));
    Esp32_WLAN_setup_ap(dev, s);
    
}
static Property esp32_wifi_properties[] = {
    DEFINE_NIC_PROPERTIES(Esp32WifiState, conf),
    DEFINE_PROP_END_OF_LIST(),
};
static void esp32_wifi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32_wifi_realize;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "Esp32 WiFi";
    device_class_set_props(dc, esp32_wifi_properties);
}


static const TypeInfo esp32_wifi_info = {
    .name = TYPE_ESP32_WIFI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32WifiState),
    .class_init    = esp32_wifi_class_init,
};

static void esp32_wifi_register_types(void)
{
    type_register_static(&esp32_wifi_info);
}

type_init(esp32_wifi_register_types)
