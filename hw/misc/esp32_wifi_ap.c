
#include "qemu/osdep.h"


//#include "hw.h"
//#include "pci/pci.h"
//#include "pc.h"
#include "net/net.h"
#include "qemu/timer.h"

#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/mman.h>
#include <netinet/in.h>
#include <netdb.h>
#include "hw/misc/esp32_wifi.h"
#include "esp32_wlan.h"
#include "esp32_wlan_packet.h"

void Esp32_WLAN_insert_frame(Esp32WifiState *s, struct mac80211_frame *frame);

static void Esp32_WLAN_beacon_timer(void *opaque)
{
    struct mac80211_frame *frame;
    Esp32WifiState *s = (Esp32WifiState *)opaque;

    frame = Esp32_WLAN_create_beacon_frame();
    if (frame) {
        Esp32_WLAN_init_frame(s, frame);
        Esp32_WLAN_insert_frame(s, frame);
    }

    timer_mod(s->beacon_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 250000000);
}

static void Esp32_WLAN_inject_timer(void *opaque)
{
    Esp32WifiState *s = (Esp32WifiState *)opaque;
    struct mac80211_frame *frame;
    printf("Inject Timer %d\n",s->inject_queue_size);


    frame = s->inject_queue;
    if (frame) {
        // remove from queue
        s->inject_queue_size--;
        s->inject_queue = frame->next_frame;
    }

    if (!frame) {
        goto timer_done;
    }

  //  if (s->receive_queue_address == 0) {
        // we drop the packet
  //  } else {
     //   Esp32_WLAN_handleRxBuffer(s, frame, frame->frame_length);
        Esp32_sendFrame(s, (void *)frame, frame->frame_length);
   // }

    free(frame);

timer_done:

    if (s->inject_queue_size > 0) {
        // there are more packets... schedule
        // the timer for sending them as well
        timer_mod(s->inject_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 25000000);
    } else {
        // we wait until a new packet schedules
        // us again
        s->inject_timer_running = 0;
    }

}


/*
 * PCI and EEPROM definitions
 */

void Esp32_WLAN_insert_frame(Esp32WifiState *s, struct mac80211_frame *frame)
{
    struct mac80211_frame *i_frame;


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
        timer_mod(s->inject_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 250000000);
    }

}

static _Bool Esp32_WLAN_can_receive(NetClientState *ncs)
{
    Esp32WifiState *s = qemu_get_nic_opaque(ncs);

    if (s->ap_state != Esp32_WLAN__STATE_ASSOCIATED) {
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
        Esp32_WLAN_init_frame(s, frame);
        Esp32_WLAN_insert_frame(s, frame);
    }
    return size;
}

static void Esp32_WLAN_cleanup(NetClientState *ncs)
{

}

static NetClientInfo net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = Esp32_WLAN_can_receive,
    .receive = Esp32_WLAN_receive,
    .cleanup = Esp32_WLAN_cleanup,
};

void Esp32_WLAN_setup_ap(Esp32WifiState *s)
{

    s->ap_state = Esp32_WLAN__STATE_NOT_AUTHENTICATED;
    s->ap_macaddr[0] = 0x00;
    s->ap_macaddr[1] = 0x13;
    s->ap_macaddr[2] = 0x46;
    s->ap_macaddr[3] = 0xbf;
    s->ap_macaddr[4] = 0x31;
    s->ap_macaddr[5] = 0x59;

    s->inject_timer_running = 0;
    s->inject_sequence_number = 0;

    s->inject_queue = NULL;
    s->inject_queue_size = 0;

   // s->access_semaphore = semget(ESP32_WLAN_ACCESS_SEM_KEY, 1, 0666 | IPC_CREAT);
   // semctl(s->access_semaphore, 0, SETVAL, 1);

    s->beacon_timer = timer_new_ns(QEMU_CLOCK_REALTIME, Esp32_WLAN_beacon_timer, s);
    timer_mod(s->beacon_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME)+250000000);

    // setup the timer but only schedule
    // it when necessary...
    s->inject_timer = timer_new_ns(QEMU_CLOCK_REALTIME, Esp32_WLAN_inject_timer, s);

    s->nic = qemu_new_nic(&net_info, &s->conf, "esp32", "wifi", s);

    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->macaddr);
}
/*
void Esp32_WLAN_disable_irq(void *arg)
{
    Esp32WifiState *s = (Esp32WifiState *)arg;
    SET_MEM_L(s->mem, ATH_HW_IRQ_PENDING, ATH_HW_IRQ_PENDING_FALSE);
    qemu_set_irq(s->irq, 0);
    DEBUG_PRINT((">> Disabling irq\n"));
}

void Esp32_WLAN_enable_irq(void *arg)
{
    Esp32WifiState *s = (Esp32WifiState *)arg;

    if (!s->interrupt_enabled) {
        DEBUG_PRINT((">> Wanted to enable irq, but they are disabled\n"));
        Esp32_WLAN_disable_irq(s);
        return;
    }

    DEBUG_PRINT((">> Enabling irq\n"));
    SET_MEM_L(s->mem, ATH_HW_IRQ_PENDING, ATH_HW_IRQ_PENDING_TRUE);
    qemu_set_irq(s->irq, 1);
}
*/
#if 0
void Esp32_WLAN_update_irq(void *arg)
{
    Esp32WifiState *s = (Esp32WifiState *)arg;
    DEBUG_PRINT((">> Updating... irq-enabled is %u\n", s->interrupt_enabled));
    /*
     * NOTE: Since we use shared interrupts
     * the device driver will check if the
     * interrupt really comes from this hardware
     *
     * This is done by checking the
     * ATH_HW_IRQ_PENDING memory...
     */
    if (/*(!s->interrupt_enabled) ||*/
        (s->pending_interrupts == NULL)) {
        SET_MEM_L(s->mem, AR5K_RAC_PISR, 0);
        goto disable_further_interrupts;
    }

    /*
     * Make sure this is done atomically!!
     */
    wait_semaphore(s->access_semaphore, 0);
    uint32_t status = 0x0;
    struct pending_interrupt *i = s->pending_interrupts;
    struct pending_interrupt *next;

    s->pending_interrupts = NULL;
    while (i != NULL) {
        next = i->next;
        if (1) { //(s->interrupt_p_mask & i->status)
            status |= i->status;
        }
        free(i);

        i = next;
    }

    SET_MEM_L(s->mem, AR5K_RAC_PISR, status);
    DEBUG_PRINT((">> Status set to %u\n", status));
    /*
     * Atomic part done...
     */
    signal_semaphore(s->access_semaphore, 0);


disable_further_interrupts:
    /*
     * NOTE: At last, it will check if any
     * more interrupts are pending. The call
     * to check what type of interrupt was
     * pending already put down the interrupt_pending
     * bit for us (check the readl function for RAC)
     *
     * if_ath.c: 921
     */
    Esp32_WLAN_disable_irq(s);
}


void Esp32_WLAN_append_irq(Esp32WifiState *s, struct pending_interrupt intr)
{
    struct pending_interrupt *new_intr;
    new_intr = (struct pending_interrupt *)malloc(sizeof(struct pending_interrupt));
    memcpy(new_intr, &intr, sizeof(intr));

    /*
     * Make sure this is done atomically!!
     */
    wait_semaphore(s->access_semaphore, 0);

    if (s->pending_interrupts == NULL) {
        s->pending_interrupts = new_intr;
    } else {
        /*
         * Insert at the end of the
         * list to assure correct order
         * of interrupts!
         */
        struct pending_interrupt *i = s->pending_interrupts;
        while (i->next != NULL) {
            i = i->next;
        }

        new_intr->next = NULL;
        i->next = new_intr;
    }

    /*
     * Atomic part done...
     */
    signal_semaphore(s->access_semaphore, 0);
}








void Esp32_WLAN_handleRxBuffer(Esp32WifiState *s, struct mac80211_frame *frame, uint32_t frame_length)
{
    struct ath_desc desc;
    struct ath5k_ar5212_rx_status *rx_status;
    rx_status = (struct ath5k_ar5212_rx_status *)&desc.ds_hw[0];

    if (s->receive_queue_address == 0) {
        return;
    }

    cpu_physical_memory_read(s->receive_queue_address, (uint8_t *)&desc, sizeof(desc));

    /*
     * Put some good base-data into
     * the descriptor. Length & co
     * will be modified below...
     *
     * NOTE: Better set everything correctly
     *
     * Look at ath5k_hw.c: proc_tx_desc
     */
    desc.ds_ctl0 = 0x0;
    desc.ds_ctl1 = 0x9c0;
    desc.ds_hw[0] = 0x126d806a;
    desc.ds_hw[1] = 0x49860003;
    desc.ds_hw[2] = 0x0;
    desc.ds_hw[3] = 0x0;


    /*
     * Filter out old length and put in correct value...
     */
    rx_status->rx_status_0 &= ~AR5K_AR5212_DESC_RX_STATUS0_DATA_LEN;
    rx_status->rx_status_0 |= frame_length;
    rx_status->rx_status_0 &= ~AR5K_AR5211_DESC_RX_STATUS0_MORE;

    /*
     * Write descriptor and packet back to DMA memory...
     */
    cpu_physical_memory_write(s->receive_queue_address, (uint8_t *)&desc, sizeof(desc));
    cpu_physical_memory_write((hwaddr)desc.ds_data, (uint8_t *)frame, sizeof(struct mac80211_frame));

    /*
     * Set address to next position
     * in single-linked list
     *
     * The receive list's last element
     * points to itself to avoid overruns.
     * This way, at some point no more
     * packets will be received, but (I
     * ASSUME) that it is the drivers
     * responsibility to reset the address
     * list!
     *
     *
     * NOTE: It seems the real madwifi cannot
     * handle multiple packets at once. so we
     * set the buffer to NULL to make the injection
     * fail next time until an interrupt was
     * received by the driver and a new buffer
     * is registered!!
     */
    s->receive_queue_address =
        ((++s->receive_queue_count) > MAX_CONCURRENT_RX_FRAMES)
        ? 0
        : (hwaddr)desc.ds_link;


    DEBUG_PRINT((">> Enabling rx\n"));
    /*
     * Notify the driver about the new packet
     */
    struct pending_interrupt intr;
    intr.status = AR5K_INT_RX;
    Esp32_WLAN_append_irq(s, intr);
    Esp32_WLAN_enable_irq(s);
}



void Esp32_WLAN_handleTxBuffer(Esp32WifiState *s, uint32_t queue)
{
    struct ath_desc desc;
    struct mac80211_frame frame;

    if (s->transmit_queue_address[queue] == 0) {
        return;
    }

    cpu_physical_memory_read(s->transmit_queue_address[queue], (uint8_t *)&desc, sizeof(desc));

    if (s->transmit_queue_processed[queue]) {
        /*
         * Maybe we already processed the frame
         * and have not gotten the address of the
         * next frame buffer but still got a call
         * to send the next frame
         *
         * this way we have to process the next
         * frame in the single linked list!!
         */
        s->transmit_queue_address[queue] = (hwaddr)desc.ds_link;

        /*
         * And now get the frame we really have to process...
         */
        cpu_physical_memory_read(s->transmit_queue_address[queue], (uint8_t *)&desc, sizeof(desc));
    }

    uint32_t segment_len, frame_length = 0, more;
    uint8_t *frame_pos = (uint8_t *)&frame;
    struct ath5k_ar5212_tx_desc *tx_desc;
    tx_desc = (struct ath5k_ar5212_tx_desc *)&desc.ds_ctl0;
    do {
        more = tx_desc->tx_control_1 & AR5K_AR5211_DESC_TX_CTL1_MORE;
        segment_len = tx_desc->tx_control_1 & AR5K_AR5212_DESC_TX_CTL1_BUF_LEN;

        cpu_physical_memory_read((hwaddr)desc.ds_data, frame_pos, segment_len);
        frame_pos += segment_len;
        frame_length += segment_len;


        /*
         * Notify successful transmission
         *
         * NOTE: It'd be better to leave the
         * descriptor as it is and only modify
         * the transmit-ok-bits --> this way
         * the timestamp and co. would stay
         * valid...
         *
         * Look at ath5k_hw.c: proc_tx_desc
         *
         * NOTE: Not sure if this acknowledgement
         * must be copied back for every single
         * descriptor in a multi-segment frame,
         * but better safe than sorry!!
         */
        desc.ds_ctl0 = 0x213f002f;
        desc.ds_ctl1 = 0x2b;
        desc.ds_hw[0] = 0xf0000;
        desc.ds_hw[1] = 0x1b;
        desc.ds_hw[2] = 0xab640001;
        desc.ds_hw[3] = 0x4a019;

        /*
         *
         * struct ath5k_tx_status *tx_status = (struct ath5k_tx_status*)&desc.ds_hw[2];
         * tx_status->tx_status_1 |= AR5K_DESC_TX_STATUS1_DONE;
         * tx_status->tx_status_0 |= AR5K_DESC_TX_STATUS0_FRAME_XMIT_OK;
         *
         *
         * Write descriptor back to DMA memory...
         */
        cpu_physical_memory_write(s->transmit_queue_address[queue], (uint8_t *)&desc, sizeof(desc));

        if (more && frame_length < sizeof(frame)) {
            /*
             * This is done at the end of the loop
             * since sometimes the next-link is not
             * yet set (assuming frame is a 1-segment
             * frame)!!
             *
             * This is very strange (and maybe obsolete
             * by this version) but let's do it the safe
             * way and not mess it up :-)
             */
            s->transmit_queue_address[queue] = (hwaddr)desc.ds_link;
            cpu_physical_memory_read(s->transmit_queue_address[queue], (uint8_t *)&desc, sizeof(desc));
        }
    } while (more && frame_length < sizeof(frame));


    struct pending_interrupt intr;
    intr.status = AR5K_INT_TX;
    Esp32_WLAN_append_irq(s, intr);
    Esp32_WLAN_enable_irq(s);

    /*
     * Set address to next position
     * in single-linked list
     *
     * The transmit list's last element
     * points to itself to avoid overruns.
     * This way, at some point no more
     * packets will be received, but (I
     * ASSUME) that it is the drivers
     * responsibility to reset the address
     * list!
     */
    s->transmit_queue_processed[queue] = 1;

    frame.frame_length = frame_length + 4;
    Esp32_WLAN_handle_frame(s, &frame);
}

#endif
void Esp32_WLAN_handle_frame(Esp32WifiState *s, struct mac80211_frame *frame)
{
    struct mac80211_frame *reply = NULL;
    unsigned long ethernet_frame_size;
    unsigned char ethernet_frame[1518];

    printf("Handle Frame %d %d\n",frame->frame_control.type,frame->frame_control.sub_type);

    if ((frame->frame_control.type == IEEE80211_TYPE_MGT) &&
            (frame->frame_control.sub_type == IEEE80211_TYPE_MGT_SUBTYPE_PROBE_REQ)) {
        reply = Esp32_WLAN_create_probe_response();
    } else if ((frame->frame_control.type == IEEE80211_TYPE_MGT) &&
               (frame->frame_control.sub_type == IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION)) {
        DEBUG_PRINT_AP(("Received authentication!\n"));
        reply = Esp32_WLAN_create_authentication();

        if (s->ap_state == Esp32_WLAN__STATE_NOT_AUTHENTICATED) {
            // if everything is going according to
            // the state machine, let's jump into the
            // next state
            s->ap_state = Esp32_WLAN__STATE_AUTHENTICATED;
        }
    } else if ((frame->frame_control.type == IEEE80211_TYPE_MGT) &&
               (frame->frame_control.sub_type == IEEE80211_TYPE_MGT_SUBTYPE_DEAUTHENTICATION)) {
        DEBUG_PRINT_AP(("Received deauthentication!\n"));
        reply = Esp32_WLAN_create_deauthentication();

        // some systems (e.g. WinXP) won't send a
        // disassociation. just believe that the
        // deauthentication is ok... nothing bad
        // can happen anyways ;-)
        s->ap_state = Esp32_WLAN__STATE_NOT_AUTHENTICATED;
    } else if ((frame->frame_control.type == IEEE80211_TYPE_MGT) &&
               (frame->frame_control.sub_type == IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ)) {
        DEBUG_PRINT_AP(("Received association request!\n"));
        reply = Esp32_WLAN_create_association_response();

        if (s->ap_state == Esp32_WLAN__STATE_AUTHENTICATED) {
            // if everything is going according to
            // the state machine, let's jump into the
            // next state
            s->ap_state = Esp32_WLAN__STATE_ASSOCIATED;
        }
    } else if ((frame->frame_control.type == IEEE80211_TYPE_MGT) &&
               (frame->frame_control.sub_type == IEEE80211_TYPE_MGT_SUBTYPE_DISASSOCIATION)) {
        DEBUG_PRINT_AP(("Received disassociation!\n"));
        reply = Esp32_WLAN_create_disassociation();

        if (s->ap_state == Esp32_WLAN__STATE_ASSOCIATED) {
            // if everything is going according to
            // the state machine, let's jump into the
            // next state
            s->ap_state = Esp32_WLAN__STATE_AUTHENTICATED;
        }
    } else if ((frame->frame_control.type == IEEE80211_TYPE_DATA) &&
               (s->ap_state == Esp32_WLAN__STATE_ASSOCIATED)) {
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
        memcpy(&ethernet_frame[6], s->ap_macaddr, 6);

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

    if (reply) {
        printf("Handle Frame Reply\n");
        memcpy(reply->destination_address, frame->source_address, 6);
        Esp32_WLAN_init_frame(s, reply);
        Esp32_WLAN_insert_frame(s, reply);
    }
}

//#endif /* CONFIG_WIN32 */
