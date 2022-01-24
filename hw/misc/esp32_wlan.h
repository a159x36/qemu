/**
 * QEMU WLAN device emulation
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
 *
 */

#ifndef esp32_wlan_h
#define esp32_wlan_h 1


//#define DEBUG_Esp32_WLAN

#ifdef DEBUG_Esp32_WLAN
#define DEBUG_PRINT_AP(x) \
    do { \
        struct timeval __tt; \
        gettimeofday(&__tt, NULL); \
        printf("%u:%u  ", (unsigned)__tt.tv_sec, (unsigned)__tt.tv_usec); \
        printf x ;\
    } while (0)
#else
#define DEBUG_PRINT_AP(x)
#endif


#define IEEE80211_IDLE                  0xff

#define IEEE80211_TYPE_MGT              0x00
#define IEEE80211_TYPE_CTL              0x01
#define IEEE80211_TYPE_DATA             0x02

#define IEEE80211_TYPE_MGT_SUBTYPE_BEACON       0x08
#define IEEE80211_TYPE_MGT_SUBTYPE_ACTION       0x0d
#define IEEE80211_TYPE_MGT_SUBTYPE_PROBE_REQ        0x04
#define IEEE80211_TYPE_MGT_SUBTYPE_PROBE_RESP       0x05
#define IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION   0x0b
#define IEEE80211_TYPE_MGT_SUBTYPE_DEAUTHENTICATION 0x0c
#define IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ  0x00
#define IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP 0x01
#define IEEE80211_TYPE_MGT_SUBTYPE_DISASSOCIATION   0x09

#define IEEE80211_TYPE_CTL_SUBTYPE_ACK          0x0d

#define IEEE80211_TYPE_DATA_SUBTYPE_DATA        0x00


#define IEEE80211_BEACON_PARAM_SSID         0x00
#define IEEE80211_BEACON_PARAM_RATES            0x01
#define IEEE80211_BEACON_PARAM_CHANNEL          0x03
#define IEEE80211_BEACON_PARAM_EXTENDED_RATES       0x32
#define IEEE80211_BEACON_PARAM_TIM          0x05


#define IEEE80211_HEADER_SIZE               24


#define PACK __attribute__((gcc_struct,packed,aligned(1)))

typedef struct beacon_info_t {
    uint64_t timestamp;
    uint16_t interval;
    uint16_t capability;
}  PACK beacon_info_t;



typedef struct mac80211_frame {
    struct mac80211_frame_control {
        unsigned    protocol_version    : 2;
        unsigned    type            : 2;
        unsigned    sub_type        : 4;
        unsigned  flags:8;
    } PACK frame_control;
    uint16_t  duration_id;

    union {
        uint8_t     address_1[6];
        uint8_t     destination_address[6];
    };

    union {
        uint8_t     address_2[6];
        uint8_t     source_address[6];
    };

    union {
        uint8_t     address_3[6];
        uint8_t     bssid_address[6];
    };

    struct mac80211_sequence_control {
        unsigned    fragment_number     : 4;
        unsigned    sequence_number     : 12;
    } PACK sequence_control;

    // variable length, 2312 byte plus 4 byte frame-checksum
    union {
        uint8_t     data_and_fcs[2316];
        beacon_info_t beacon_info;
    };

    unsigned int frame_length;
    int signal_strength;
    struct mac80211_frame *next_frame;

}  PACK mac80211_frame;

typedef struct access_point_info {
    const char *ssid;
    int channel;
    int sigstrength;
} access_point_info;

#define Esp32_WLAN__STATE_NOT_AUTHENTICATED   0
#define Esp32_WLAN__STATE_AUTHENTICATED   1
#define Esp32_WLAN__STATE_ASSOCIATED      2


#define Esp32_WLAN__MAX_INJECT_QUEUE_SIZE 20

extern int wifi_channel;
#endif // esp32_wlan_h
