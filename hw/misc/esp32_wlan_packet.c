/**
 * QEMU WLAN access point emulation
 *
 * Copyright (c) 2008 Clemens Kolbitsch
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Modifications:
 *  2008-February-24  Clemens Kolbitsch :
 *                                  New implementation based on ne2000.c
 *  18/1/22 Martin JOhnson : Modified for esp32 wifi emilation
 */

#include "qemu/osdep.h"

#include "net/net.h"
/*
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/mman.h>
#include <netinet/in.h>
#include <netdb.h>
*/


#include "esp32_wlan.h"
#include "esp32_wlan_crc32.h"
#include "esp32_wlan_packet.h"

#define FRAME_INSERT(_8bit_data)    buf[i++] = _8bit_data

const char *ssid_table[]={"ERR","My Wifi",0,0,0,0,"Public Wifi",0,"Massey_Wifi"};
/*
static int insertCRC(mac80211_frame *frame, uint32_t frame_length)
{
    unsigned long crc;
    unsigned char *fcs = (unsigned char *)frame;

    crc = crc32_ccitt(fcs, frame_length);
    memcpy(&fcs[frame_length], &crc, 4);

    return frame_length + 4;
}
*/

static int add_ssid(unsigned char *buf,int i,int channel) {
    FRAME_INSERT(IEEE80211_BEACON_PARAM_SSID);
    const char *ssid=ssid_table[channel];
    int len=strlen(ssid);
    FRAME_INSERT(len);    /* length */
    memcpy((char *)buf+i,ssid,len);
    return len+i;
}

void Esp32_WLAN_init_frame(Esp32WifiState *s, mac80211_frame *frame)
{
    if (!frame) {
        return;
    }

    frame->sequence_control.sequence_number = s->inject_sequence_number++;
    memcpy(frame->source_address, s->ap_macaddr, 6);
    memcpy(frame->bssid_address, s->ap_macaddr, 6);

    //frame->frame_length = insertCRC(frame, frame->frame_length)-4;
}


mac80211_frame *Esp32_WLAN_create_beacon_frame(int channel)
{
    unsigned int i;
    unsigned char *buf;
    mac80211_frame *frame;

    frame = (mac80211_frame *)malloc(sizeof(mac80211_frame));
    if (!frame) {
        return NULL;
    }

    frame->next_frame = NULL;
    frame->frame_control.protocol_version = 0;
    frame->frame_control.type = IEEE80211_TYPE_MGT;
    frame->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_BEACON;
    frame->frame_control.flags = 0;
    frame->duration_id = 0;
    frame->sequence_control.fragment_number = 0;

    for (i = 0; i < 6; frame->destination_address[i] = 0xff, i++) {
    }

    i = 0;
    buf = (unsigned char *)frame->data_and_fcs;

    /*
     * Fixed params... typical AP params (12 byte)
     *
     * They include
     *  - Timestamp
     *  - Beacon Interval
     *  - Capability Information
     */
    frame->beacon_info.timestamp=qemu_clock_get_ns(QEMU_CLOCK_REALTIME)/1000;
    frame->beacon_info.interval=1000;
    frame->beacon_info.capability=1;
    /*
    buf[i++] = 0x8d;
    buf[i++] = 0x61;
    buf[i++] = 0xa5;
    buf[i++] = 0x18;
    buf[i++] = 0x00;
    buf[i++] = 0x00;

    FRAME_INSERT(0x00);
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x64);
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x01);
    FRAME_INSERT(0x00);
    */
    i=12;

    i=add_ssid(buf,i,channel);

    FRAME_INSERT(IEEE80211_BEACON_PARAM_RATES);
    FRAME_INSERT(8);    /* length */
    FRAME_INSERT(0x82);
    FRAME_INSERT(0x84);
    FRAME_INSERT(0x8b);
    FRAME_INSERT(0x96);
    FRAME_INSERT(0x24);
    FRAME_INSERT(0x30);
    FRAME_INSERT(0x48);
    FRAME_INSERT(0x6c);

    FRAME_INSERT(IEEE80211_BEACON_PARAM_CHANNEL);
    FRAME_INSERT(1);    /* length */
    FRAME_INSERT(channel);

    FRAME_INSERT(IEEE80211_BEACON_PARAM_TIM);
    FRAME_INSERT(4);
    FRAME_INSERT(1);
    FRAME_INSERT(3);
    FRAME_INSERT(0);
    FRAME_INSERT(0);
    frame->frame_length = IEEE80211_HEADER_SIZE + i;
    return frame;
}

mac80211_frame *Esp32_WLAN_create_probe_response(void)
{
    unsigned int i;
    unsigned char *buf;
    mac80211_frame *frame;

    frame = (mac80211_frame *)malloc(sizeof(mac80211_frame));
    if (!frame) {
        return NULL;
    }

    frame->next_frame = NULL;
    frame->frame_control.protocol_version = 0;
    frame->frame_control.type = IEEE80211_TYPE_MGT;
    frame->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_PROBE_RESP;
    frame->frame_control.flags = 0;
    frame->duration_id = 314;
    frame->sequence_control.fragment_number = 0;
    
    
    buf = (unsigned char *)frame->data_and_fcs;

    frame->beacon_info.timestamp=qemu_clock_get_ns(QEMU_CLOCK_REALTIME)/1000;
    frame->beacon_info.interval=1000;
    frame->beacon_info.capability=1;
    
    
    i=12;

    i=add_ssid(buf,i,wifi_channel);
    
    FRAME_INSERT(IEEE80211_BEACON_PARAM_RATES);
    FRAME_INSERT(8);    /* length */
    /*
    FRAME_INSERT(0x82);
    FRAME_INSERT(0x84);
    FRAME_INSERT(0x8b);
    FRAME_INSERT(0x96);
    FRAME_INSERT(0x24);
    FRAME_INSERT(0x30);
    FRAME_INSERT(0x48);
    FRAME_INSERT(0x6c);
    */

    FRAME_INSERT(140);
    FRAME_INSERT(18);
    FRAME_INSERT(152);
    FRAME_INSERT(36);
    FRAME_INSERT(176);
    FRAME_INSERT(72);
    FRAME_INSERT(96);
    FRAME_INSERT(108);

    FRAME_INSERT(IEEE80211_BEACON_PARAM_CHANNEL);
    FRAME_INSERT(1);    /* length */
//    FRAME_INSERT(AP_WIFI_CHANNEL);
    FRAME_INSERT(wifi_channel);

    frame->frame_length = IEEE80211_HEADER_SIZE + i;
    return frame;
}

mac80211_frame *Esp32_WLAN_create_authentication(void)
{
    unsigned int i;
    unsigned char *buf;
    mac80211_frame *frame;

    frame = (mac80211_frame *)malloc(sizeof(mac80211_frame));
    if (!frame) {
        return NULL;
    }

    frame->next_frame = NULL;
    frame->frame_control.protocol_version = 0;
    frame->frame_control.type = IEEE80211_TYPE_MGT;
    frame->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION;
    frame->frame_control.flags = 0;
    frame->duration_id = 314;
    frame->sequence_control.fragment_number = 0;

    i = 0;
    buf = (unsigned char *)frame->data_and_fcs;

    /*
     * Fixed params... typical AP params (6 byte)
     *
     * They include
     *  - Authentication Algorithm (here: Open System)
     *  - Authentication SEQ
     *  - Status code (successful 0x0)
     */
    
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x02);
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x00);
    

    i=add_ssid(buf,i,wifi_channel);

    frame->frame_length = IEEE80211_HEADER_SIZE + i;
    return frame;
}

mac80211_frame *Esp32_WLAN_create_deauthentication(void)
{
    unsigned int i;
    unsigned char *buf;
    mac80211_frame *frame;

    frame = (mac80211_frame *)malloc(sizeof(mac80211_frame));
    if (!frame) {
        return NULL;
    }

    frame->next_frame = NULL;
    frame->frame_control.protocol_version = 0;
    frame->frame_control.type = IEEE80211_TYPE_MGT;
    frame->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_DEAUTHENTICATION;
    frame->frame_control.flags = 0;
    frame->duration_id = 314;
    frame->sequence_control.fragment_number = 0;

    i = 0;
    buf = (unsigned char *)frame->data_and_fcs;

    /*
     * Insert reason code:
     *  "Deauthentication because sending STA is leaving"
     */
    FRAME_INSERT(0x03);
    FRAME_INSERT(0x00);

    frame->frame_length = IEEE80211_HEADER_SIZE + i;
    return frame;
}

mac80211_frame *Esp32_WLAN_create_association_response(void)
{
    unsigned int i;
    unsigned char *buf;
    mac80211_frame *frame;

    frame = (mac80211_frame *)malloc(sizeof(mac80211_frame));
    if (!frame) {
        return NULL;
    }

    frame->next_frame = NULL;
    frame->frame_control.protocol_version = 0;
    frame->frame_control.type = IEEE80211_TYPE_MGT;
    frame->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP;
    frame->frame_control.flags = 0;
#if 0
    frame->duration_id = 314;
#endif
    frame->sequence_control.fragment_number = 0;

    i = 0;
    buf = (unsigned char *)frame->data_and_fcs;

    /*
     * Fixed params... typical AP params (6 byte)
     *
     * They include
     *  - Capability Information
     *  - Status code (successful 0x0)
     *  - Association ID
     
    FRAME_INSERT(0x01);
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x01);
    FRAME_INSERT(0xc0);
    */

    FRAME_INSERT(33);
    FRAME_INSERT(4);
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x00);
    FRAME_INSERT(0x01);
    FRAME_INSERT(0xc0);

    i=add_ssid(buf,i,wifi_channel);


    FRAME_INSERT(IEEE80211_BEACON_PARAM_RATES);
    FRAME_INSERT(8);    /* length */
    

    FRAME_INSERT(140);
    FRAME_INSERT(18);
    FRAME_INSERT(152);
    FRAME_INSERT(36);
    FRAME_INSERT(176);
    FRAME_INSERT(72);
    FRAME_INSERT(96);
    FRAME_INSERT(108);
    /*
    FRAME_INSERT(IEEE80211_BEACON_PARAM_RATES);
    FRAME_INSERT(8);    
    FRAME_INSERT(0x82);
    FRAME_INSERT(0x84);
    FRAME_INSERT(0x8b);
    FRAME_INSERT(0x96);
    FRAME_INSERT(0x24);
    FRAME_INSERT(0x30);
    FRAME_INSERT(0x48);
    FRAME_INSERT(0x6c);

    FRAME_INSERT(IEEE80211_BEACON_PARAM_EXTENDED_RATES);
    FRAME_INSERT(4);   
    FRAME_INSERT(0x0c);
    FRAME_INSERT(0x12);
    FRAME_INSERT(0x18);
    FRAME_INSERT(0x60);
    */

    frame->frame_length = IEEE80211_HEADER_SIZE + i;
    return frame;
}

mac80211_frame *Esp32_WLAN_create_disassociation(void)
{
    unsigned int i;
    unsigned char *buf;
    mac80211_frame *frame;

    frame = (mac80211_frame *)malloc(sizeof(mac80211_frame));
    if (!frame) {
        return NULL;
    }

    frame->next_frame = NULL;
    frame->frame_control.protocol_version = 0;
    frame->frame_control.type = IEEE80211_TYPE_MGT;
    frame->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_DISASSOCIATION;
    frame->frame_control.flags = 0;
    frame->duration_id = 314;
    frame->sequence_control.fragment_number = 0;

    i = 0;
    buf = (unsigned char *)frame->data_and_fcs;

    /*
     * Insert reason code:
     *  "Disassociation because sending STA is leaving"
     */
    FRAME_INSERT(0x03);
    FRAME_INSERT(0x00);

    frame->frame_length = IEEE80211_HEADER_SIZE + i;
    return frame;
}

mac80211_frame *Esp32_WLAN_create_data_packet(Esp32WifiState *s,
        const uint8_t *buf, int size)
{
    mac80211_frame *frame;

    frame = (mac80211_frame *)malloc(sizeof(mac80211_frame));
    if (!frame) {
        return NULL;
    }
 //   printf("create_data_packet\n");
    frame->next_frame = NULL;
    frame->frame_control.protocol_version = 0;
    frame->frame_control.type = IEEE80211_TYPE_DATA;
    frame->frame_control.sub_type = IEEE80211_TYPE_DATA_SUBTYPE_DATA;
    frame->frame_control.flags = 0x2; /* from station back to station via AP */
    frame->duration_id = 44;
    frame->sequence_control.fragment_number = 0;

    /* send message to wlan-device */
    memcpy(frame->destination_address, s->macaddr, 6);

    size -= 12; /* remove old 803.2 header */
    size += 6; /* add new 803.11 header */
    if (size > sizeof(frame->data_and_fcs)) {
        /* sanitize memcpy */
        size = sizeof(frame->data_and_fcs);
    }

    /* LLC */
    frame->data_and_fcs[0] = 0xaa;
    frame->data_and_fcs[1] = 0xaa;
    frame->data_and_fcs[2] = 0x03;
    frame->data_and_fcs[3] = 0x00;
    frame->data_and_fcs[4] = 0x00;
    frame->data_and_fcs[5] = 0x00;

    memcpy(&frame->data_and_fcs[6], &buf[12], size);
    frame->frame_length = IEEE80211_HEADER_SIZE + size;

    return frame;
}
