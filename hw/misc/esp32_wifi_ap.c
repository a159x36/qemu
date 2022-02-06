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
 *  18/1/22 Martin Johnson : Modified for esp32 wifi emulation
 */

#include "qemu/osdep.h"
#include "net/net.h"
#include "qemu/timer.h"

#include "hw/misc/esp32_wifi.h"
#include "esp32_wlan.h"
#include "esp32_wlan_packet.h"

// 10ms between beacons
#define BEACON_TIME 10000000
#define INTER_FRAME_TIME 5000000
#define DEBUG 0
#define DEBUG_DUMPFRAMES 0

access_point_info access_points[]={
    {"Open Wifi",4,-40,{0x10,0x01,0x00,0xc4,0x0a,0x51}},
    {"MasseyWifi",6,-30,{0x10,0x01,0x00,0xc4,0x0a,0x52}},
    {"Home Wifi",7,-70,{0x10,0x01,0x00,0xc4,0x0a,0x53}},
    {"My Wifi",8,-75,{0x10,0x01,0x00,0xc4,0x0a,0x54}},
    {"New Wifi",10,-90,{0x10,0x01,0x00,0xc4,0x0a,0x55}},
    {"MartinsWifi",12,-25,{0x10,0x01,0x00,0xc4,0x0a,0x56}}
};

int nb_aps=sizeof(access_points)/sizeof(access_point_info);

static void Esp32_WLAN_beacon_timer(void *opaque)
{
    struct mac80211_frame *frame;
    Esp32WifiState *s = (Esp32WifiState *)opaque;

    // only send a beacon if we are an access point on the same channel
    if(s->ap_state!=Esp32_WLAN__STATE_STA_ASSOCIATED) {
        if (access_points[s->beacon_ap].channel==esp32_wifi_channel) {
            frame = Esp32_WLAN_create_beacon_frame(&access_points[s->beacon_ap]);
            Esp32_WLAN_init_ap_frame(s, frame);
            memcpy(frame->source_address, access_points[s->beacon_ap].mac_address, 6);
            memcpy(frame->bssid_address, access_points[s->beacon_ap].mac_address, 6);
            Esp32_WLAN_insert_frame(s, frame);
        }
        s->beacon_ap=(s->beacon_ap+1)%nb_aps;
    }
    timer_mod(s->beacon_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + BEACON_TIME);
}

static void Esp32_WLAN_inject_timer(void *opaque)
{
    Esp32WifiState *s = (Esp32WifiState *)opaque;
    struct mac80211_frame *frame;

    frame = s->inject_queue;
    if (frame) {
        // remove from queue
        s->inject_queue_size--;
        s->inject_queue = frame->next_frame;
        Esp32_sendFrame(s, (void *)frame, frame->frame_length,frame->signal_strength);
        free(frame);
    }
    if (s->inject_queue_size > 0) {
        // there are more packets... schedule
        // the timer for sending them as well
        timer_mod(s->inject_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + INTER_FRAME_TIME);
    } else {
        // we wait until a new packet schedules
        // us again
        s->inject_timer_running = 0;
    }

}

static void macprint(uint8_t *p, const char * name) {
    printf("%s: %2x:%2x:%2x:%2x:%2x:%2x\n",name, p[0],p[1],p[2],p[3],p[4],p[5]);
}

static void infoprint(struct mac80211_frame *frame) {
    if(DEBUG_DUMPFRAMES) {
        printf("Frame Info %d %d %d %d %d\n",frame->frame_control.type,frame->frame_control.sub_type,frame->frame_control.flags,frame->duration_id, frame->frame_length);
        macprint(frame->destination_address,"destination");
        macprint(frame->source_address,"source");
        macprint(frame->bssid_address,"bssid");
        uint8_t *b=(uint8_t *)frame;
        for(int i=0;i<frame->frame_length;i++) {
            if((i%16)==0) printf("\n%04x: ",i);
            printf("%02x ",b[i]);
        }
        printf("\n");
    }
}

void Esp32_WLAN_insert_frame(Esp32WifiState *s, struct mac80211_frame *frame)
{
    struct mac80211_frame *i_frame;

    insertCRC(frame);
    if(DEBUG) printf("Send Frame %d %d\n",frame->frame_control.type,frame->frame_control.sub_type);
    infoprint(frame);
    s->inject_queue_size++;
    i_frame = s->inject_queue;
    if (!i_frame) {
        s->inject_queue = frame;
    } else {
        while (i_frame->next_frame) {
            i_frame = i_frame->next_frame;
        }
        i_frame->next_frame = frame;
    }

    if (!s->inject_timer_running) {
        // if the injection timer is not
        // running currently, let's schedule
        // one run...
        s->inject_timer_running = 1;
        timer_mod(s->inject_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + INTER_FRAME_TIME);
    }

}

static _Bool Esp32_WLAN_can_receive(NetClientState *ncs)
{
    Esp32WifiState *s = qemu_get_nic_opaque(ncs);

    if (s->ap_state != Esp32_WLAN__STATE_ASSOCIATED  && s->ap_state != Esp32_WLAN__STATE_STA_ASSOCIATED) {
        // we are currently not connected
        // to the access point
        return 0;
    }
    if (s->inject_queue_size > Esp32_WLAN__MAX_INJECT_QUEUE_SIZE) {
        // overload, please give me some time...
        return 0;
    }

    return 1;
}

static ssize_t Esp32_WLAN_receive(NetClientState *ncs,
                                    const uint8_t *buf, size_t size)
{
    Esp32WifiState *s = qemu_get_nic_opaque(ncs);
    struct mac80211_frame *frame;
    if (!Esp32_WLAN_can_receive(ncs)) {
        // this should not happen, but in
        // case it does, let's simply drop
        // the packet
        return -1;
    }

    if (!s) {
        return -1;
    }
    /*
     * A 802.3 packet comes from the qemu network. The
     * access points turns it into a 802.11 frame and
     * forwards it to the wireless device
     */
    frame = Esp32_WLAN_create_data_packet(s, buf, size);
    if (frame) {
        memcpy(s->ap_macaddr,s->associated_ap_macaddr,6);
        if(s->ap_state==Esp32_WLAN__STATE_STA_ASSOCIATED) {
            frame->frame_control.flags=1;
            // if it's an arp request put the correct reply mac address in the packet 
            if( frame->data_and_fcs[6]==8 && frame->data_and_fcs[7]==6) {
               memcpy(frame->data_and_fcs+16,s->macaddr,6);
            }
        }
        Esp32_WLAN_init_ap_frame(s, frame);
        Esp32_WLAN_insert_frame(s, frame);
    }
    return size;
}
static void Esp32_WLAN_cleanup(NetClientState *ncs) { }

static NetClientInfo net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = Esp32_WLAN_can_receive,
    .receive = Esp32_WLAN_receive,
    .cleanup = Esp32_WLAN_cleanup,
};

void Esp32_WLAN_setup_ap(DeviceState *dev,Esp32WifiState *s) {

    s->ap_state = Esp32_WLAN__STATE_NOT_AUTHENTICATED;
    s->beacon_ap=0;
    memcpy(s->ap_macaddr,(uint8_t[]){0x01,0x13,0x46,0xbf,0x31,0x50},sizeof(s->ap_macaddr));
    memcpy(s->macaddr,(uint8_t[]){0x10,0x01,0x00,0xc4,0x0a,0x24},sizeof(s->macaddr));

    s->inject_timer_running = 0;
    s->inject_sequence_number = 0;

    s->inject_queue = NULL;
    s->inject_queue_size = 0;

    s->beacon_timer = timer_new_ns(QEMU_CLOCK_REALTIME, Esp32_WLAN_beacon_timer, s);
    timer_mod(s->beacon_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME)+100000000);

    // setup the timer but only schedule
    // it when necessary...
    s->inject_timer = timer_new_ns(QEMU_CLOCK_REALTIME, Esp32_WLAN_inject_timer, s);

    s->nic = qemu_new_nic(&net_info, &s->conf, object_get_typename(OBJECT(s)), dev->id, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->macaddr);
}

static void send_single_frame(Esp32WifiState *s, struct mac80211_frame *frame, struct mac80211_frame *reply) {
    reply->sequence_control.sequence_number = s->inject_sequence_number++ +0x730;
    reply->signal_strength=-10;

    if(frame) {
        memcpy(reply->destination_address, frame->source_address, 6);
        memcpy(reply->source_address, s->macaddr, 6);
        memcpy(reply->bssid_address, frame->source_address, 6);
    }
    
    Esp32_WLAN_insert_frame(s, reply);
}
void Esp32_WLAN_handle_frame(Esp32WifiState *s, struct mac80211_frame *frame)
{
    struct mac80211_frame *reply = NULL;
    static access_point_info dummy_ap={0};
    char ssid[64];
    unsigned long ethernet_frame_size;
    unsigned char ethernet_frame[1518];
    if(DEBUG) 
        printf("-------------------------\nHandle Frame %d %d %d %d\n",frame->frame_control.type,frame->frame_control.sub_type,esp32_wifi_channel,s->ap_state);
    infoprint(frame);
    access_point_info *ap_info=0;
    for(int i=0;i<nb_aps;i++)
        if(access_points[i].channel==esp32_wifi_channel)
            ap_info=&access_points[i];
   
    if(frame->frame_control.type == IEEE80211_TYPE_MGT) {        
        switch(frame->frame_control.sub_type) {
            case IEEE80211_TYPE_MGT_SUBTYPE_BEACON:
                if(s->ap_state==Esp32_WLAN__STATE_NOT_AUTHENTICATED || s->ap_state==Esp32_WLAN__STATE_AUTHENTICATED) {
                    strncpy(ssid,(char *)frame->data_and_fcs+14,frame->data_and_fcs[13]);
                    if(DEBUG) printf("beacon from %s\n",ssid);
                    dummy_ap.ssid=ssid;
                    s->ap_state=Esp32_WLAN__STATE_STA_NOT_AUTHENTICATED;
                    send_single_frame(s,frame,Esp32_WLAN_create_probe_request(&dummy_ap));
                }
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_PROBE_RESP:
                ap_info=&dummy_ap;
                strncpy(ssid,(char *)frame->data_and_fcs+14,frame->data_and_fcs[13]);
                if(DEBUG) printf("probe resp from %s\n",ssid);
                dummy_ap.ssid=ssid;
                s->ap_state=Esp32_WLAN__STATE_STA_NOT_AUTHENTICATED;
                send_single_frame(s,frame,Esp32_WLAN_create_deauthentication());
                send_single_frame(s,frame,Esp32_WLAN_create_authentication_request());
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP:
                if(DEBUG) printf("assoc resp\n");
                mac80211_frame *frame1=Esp32_WLAN_create_dhcp_discover();
                memcpy(frame1->bssid_address,BROADCAST,6);
                memcpy(frame1->source_address,frame->destination_address,6);
                memcpy(frame1->destination_address,frame->source_address,6);
                send_single_frame(s,0,frame1);
                s->ap_state=Esp32_WLAN__STATE_STA_DHCP;
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_DISASSOCIATION:
                DEBUG_PRINT_AP(("Received disassociation!\n"));
                send_single_frame(s,frame,Esp32_WLAN_create_disassociation());
                if (s->ap_state == Esp32_WLAN__STATE_ASSOCIATED || s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED) {
                    s->ap_state = Esp32_WLAN__STATE_AUTHENTICATED;
                }
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_DEAUTHENTICATION:
                DEBUG_PRINT_AP(("Received deauthentication!\n"));
                //reply = Esp32_WLAN_create_authentication_response(ap_info);
                if (s->ap_state == Esp32_WLAN__STATE_AUTHENTICATED) {
                    s->ap_state = Esp32_WLAN__STATE_NOT_AUTHENTICATED;
                }
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION:
                DEBUG_PRINT_AP(("Received authentication!\n"));
                if(frame->data_and_fcs[2]==2) { // response
                    send_single_frame(s,frame,Esp32_WLAN_create_association_request(&dummy_ap));
                }
                break;
        }
        if(ap_info) {
            memcpy(s->ap_macaddr,ap_info->mac_address,6);
            switch(frame->frame_control.sub_type) {
                case IEEE80211_TYPE_MGT_SUBTYPE_PROBE_REQ:
                    DEBUG_PRINT_AP(("Received probe request!\n"));
                    reply = Esp32_WLAN_create_probe_response(ap_info);
                    break;
                case IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION:
                    DEBUG_PRINT_AP(("Received authentication req %d!\n",s->ap_state));
                    if(frame->data_and_fcs[2]==1) { // request
                        reply = Esp32_WLAN_create_authentication_response(ap_info);
                        if (s->ap_state == Esp32_WLAN__STATE_NOT_AUTHENTICATED) {
                            s->ap_state = Esp32_WLAN__STATE_AUTHENTICATED;
                        }
                    } 
                break;
                case IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ:
                    DEBUG_PRINT_AP(("Received association request!\n"));
                    reply = Esp32_WLAN_create_association_response(ap_info);
                    if (s->ap_state == Esp32_WLAN__STATE_AUTHENTICATED) {
                        s->ap_state = Esp32_WLAN__STATE_ASSOCIATED;
                        memcpy(s->associated_ap_macaddr,s->ap_macaddr,6);
                    }
                    break;
            }
            if (reply) {
                reply->signal_strength=ap_info->sigstrength;
                memcpy(reply->destination_address, frame->source_address, 6);
                Esp32_WLAN_init_ap_frame(s, reply);
                Esp32_WLAN_insert_frame(s, reply);
            }
        }
    }
    if ((frame->frame_control.type == IEEE80211_TYPE_DATA) &&
        (frame->frame_control.sub_type == IEEE80211_TYPE_DATA_SUBTYPE_DATA)) {
        if(s->ap_state == Esp32_WLAN__STATE_STA_DHCP) {
            dhcp_request_t *req=(dhcp_request_t *)&frame->data_and_fcs[8];
            // check for a dhcp offer
            if(req->dhcp.bp_options[0]==0x35 && req->dhcp.bp_options[2]==0x2) {
                mac80211_frame *frame1=Esp32_WLAN_create_dhcp_request(req->dhcp.yiaddr);
                memcpy(frame1->bssid_address,BROADCAST,6);
                memcpy(frame1->source_address,s->macaddr,6);
                memcpy(frame1->destination_address,frame->source_address,6);
                send_single_frame(s,0,frame1);
                memcpy(s->ap_macaddr,(uint8_t[]){0x10,0x01,0x00,0xc4,0x0a,0x25},sizeof(s->ap_macaddr));
                memcpy(s->macaddr,(uint8_t[]){0x10,0x01,0x00,0xc4,0x0a,0x24},sizeof(s->macaddr));
                memcpy(s->associated_ap_macaddr,s->ap_macaddr,sizeof(s->ap_macaddr));
                s->ap_state=Esp32_WLAN__STATE_STA_ASSOCIATED; 
            }
        } else if (s->ap_state == Esp32_WLAN__STATE_ASSOCIATED || s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED) {
            /*
            * The access point uses the 802.11 frame
            * and sends a 802.3 frame into the network...
            * This packet is then understandable by
            * qemu-slirp
            *
            * If we ever want the access point to offer
            * some services, it can be added here!!
            */
            // ethernet header type
            ethernet_frame[12] = frame->data_and_fcs[6];
            ethernet_frame[13] = frame->data_and_fcs[7];

            // the new originator of the packet is
            // the access point
            if(s->ap_state == Esp32_WLAN__STATE_ASSOCIATED)
                memcpy(&ethernet_frame[6], s->ap_macaddr, 6);
            else
                memcpy(&ethernet_frame[6], s->macaddr, 6);

            if (ethernet_frame[12] == 0x08 && ethernet_frame[13] == 0x06) {
                // for arp request, we use a broadcast
                memset(&ethernet_frame[0], 0xff, 6);
            } else {
                // otherwise we forward the packet to
                // where it really belongs
                memcpy(&ethernet_frame[0], frame->destination_address, 6);
            }

            // add packet content
            ethernet_frame_size = frame->frame_length - 24 - 4 - 8;

            // for some reason, the packet is 22 bytes too small (??)
            ethernet_frame_size += 22;
            if (ethernet_frame_size > sizeof(ethernet_frame)) {
                ethernet_frame_size = sizeof(ethernet_frame);
            }
            memcpy(&ethernet_frame[14], &frame->data_and_fcs[8], ethernet_frame_size);
            // add size of ethernet header
            ethernet_frame_size += 14;
            /*
            * Send 802.3 frame
            */
            qemu_send_packet(qemu_get_queue(s->nic), ethernet_frame, ethernet_frame_size);
        }
    }
}

