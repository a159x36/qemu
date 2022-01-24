#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_reg.h"
#include "sysemu/sysemu.h"
#include "net/net.h"

#define TYPE_ESP32_WIFI "esp32_wifi"
#define ESP32_WIFI(obj) OBJECT_CHECK(Esp32WifiState, (obj), TYPE_ESP32_WIFI)
//OBJECT_CHECK(Esp32WifiState, (obj), TYPE_ESP32_WIFI)
//(Esp32WifiState *)(obj)

typedef struct Esp32WifiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    int raw_interrupt;
    qemu_irq irq;
    uint32_t mem[1024];
    int rxBuffer;
    uint32_t ap_state;
    int inject_queue_size;
    struct mac80211_frame *inject_queue;
    int inject_timer_running;
    unsigned int inject_sequence_number;

    hwaddr receive_queue_address;
    uint32_t receive_queue_count;
    NICConf conf;
    NICState *nic;
    // various timers
    QEMUTimer *beacon_timer;
    QEMUTimer *inject_timer;
    uint8_t ipaddr[4];              
    uint8_t macaddr[6];             

    uint8_t ap_ipaddr[4];               
    uint8_t ap_macaddr[6];              

} Esp32WifiState;


void Esp32_WLAN_handle_frame(Esp32WifiState *s, struct mac80211_frame *frame);
void Esp32_WLAN_setup_ap(DeviceState *dev,Esp32WifiState *s);
void Esp32_sendFrame(Esp32WifiState *s, uint8_t *frame,int length, int signal_strength);

REG32(WIFI_DMA_IN_STATUS, 0x84);
REG32(WIFI_DMA_INLINK, 0x88);
REG32(WIFI_DMA_INT_STATUS, 0xc48);
REG32(WIFI_DMA_INT_CLR, 0xc4c);
REG32(WIFI_STATUS, 0xcc8);
REG32(WIFI_DMA_OUTLINK, 0xd20);
REG32(WIFI_DMA_OUT_STATUS, 0xd24);


