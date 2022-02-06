/*
 * ESP32 SPI controller
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
#include "hw/ssi/esp32_spi.h"
#include "hw/misc/esp32_flash_enc.h"
#include "exec/address-spaces.h"


enum {
    CMD_RES = 0xab,
    CMD_DP = 0xb9,
    CMD_CE = 0x60,
    CMD_BE = 0xD8,
    CMD_SE = 0x20,
    CMD_PP = 0x02,
    CMD_WRSR = 0x1,
    CMD_RDSR = 0x5,
    CMD_RDID = 0x9f,
    CMD_WRDI = 0x4,
    CMD_WREN = 0x6,
    CMD_READ = 0x03,
};

static void update_irq(Esp32SpiState *s) {
    if (s->peripheral_reg & R_SPI_PERIPHERAL_TRANS_INTEN_MASK) {
        if (s->peripheral_reg & R_SPI_PERIPHERAL_TRANS_DONE_MASK)
            qemu_irq_raise(s->irq);
        else
            qemu_irq_lower(s->irq);
    }
}
#define ESP32_SPI_REG_SIZE    0x1000

static void esp32_spi_cs_set(Esp32SpiState *s, int value);

static void esp32_spi_timer_cb(void *opaque) {
    Esp32SpiState *s = ESP32_SPI(opaque);
    s->peripheral_reg |= R_SPI_PERIPHERAL_TRANS_DONE_MASK;
    esp32_spi_cs_set(s,1);
    update_irq(s);
}

static void esp32_spi_do_command(Esp32SpiState* state, uint32_t cmd_reg);

static uint64_t esp32_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_SPI_ADDR:
        r = s->addr_reg;
        break;
    case A_SPI_CTRL:
        r = s->ctrl_reg;
        break;
    case A_SPI_STATUS:
        r = s->status_reg;
        break;
    case A_SPI_CTRL1:
        r = s->ctrl1_reg;
        break;
    case A_SPI_CTRL2:
        r = s->ctrl2_reg;
        break;
    case A_SPI_USER:
        r = s->user_reg;
        break;
    case A_SPI_USER1:
        r = s->user1_reg;
        break;
    case A_SPI_USER2:
        r = s->user2_reg;
        break;
    case A_SPI_MOSI_DLEN:
        r = s->mosi_dlen_reg;
        break;
    case A_SPI_MISO_DLEN:
        r = s->miso_dlen_reg;
        break;
    case A_SPI_PIN:
        r = s->pin_reg;
        break;
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        r = s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)];
        break;
    case A_SPI_EXT2:
        r = 0;
        break;
//    case A_SPI_SLAVE:
//        r = BIT(R_SPI_SLAVE_TRANS_DONE_SHIFT) | BIT(R_SPI_SLAVE_TRANS_INTEN_SHIFT);
//        break;
    case A_SPI_PERIPHERAL:
        r = s->peripheral_reg;  // transaction done
        break;
    case A_SPI_DMA_OUT_LINK:
        r = s->outlink_reg;
        break;
    case A_SPI_DMA_CONF:
        r = s->dmaconfig_reg;
        break;
    }
    return r;
}

static void esp32_spi_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    switch (addr) {
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)] = value;
        break;
    case A_SPI_ADDR:
        s->addr_reg = value;
        break;
    case A_SPI_CTRL:
        s->ctrl_reg = value;
        break;
    case A_SPI_STATUS:
        s->status_reg = value;
        break;
    case A_SPI_CTRL1:
        s->ctrl1_reg = value;
        break;
    case A_SPI_CTRL2:
        s->ctrl2_reg = value;
        break;
    case A_SPI_USER:
        s->user_reg = value;
        break;
    case A_SPI_USER1:
        s->user1_reg = value;
        break;
    case A_SPI_USER2:
        s->user2_reg = value;
        break;
    case A_SPI_MOSI_DLEN:
        s->mosi_dlen_reg = value;
        break;
    case A_SPI_MISO_DLEN:
        s->miso_dlen_reg = value;
        break;
    case A_SPI_PIN:
        s->pin_reg = value;
        break;
    case A_SPI_CMD:
        esp32_spi_do_command(s, value);
        break;
    case A_SPI_PERIPHERAL:
        s->peripheral_reg = value;  // transaction done
        update_irq(s);
        break;

    case A_SPI_DMA_OUT_LINK:
        s->outlink_reg = value;
        break;

    case A_SPI_DMA_CONF:
        s->dmaconfig_reg = value;
        break;
    }
}

typedef struct Esp32SpiTransaction {
    int cmd_bytes;
    uint32_t cmd;
    int addr_bytes;
    uint32_t addr;
    int data_tx_bytes;
    int data_rx_bytes;
    uint32_t* data;
} Esp32SpiTransaction;

static void esp32_spi_txrx_buffer(Esp32SpiState *s, void *buf, int tx_bytes, int rx_bytes)
{
    int bytes = MAX(tx_bytes, rx_bytes);
    uint8_t *c_buf = (uint8_t*) buf;
    for (int i = 0; i < bytes; ++i) {
        uint8_t byte = 0;
        if (i < tx_bytes) {
            memcpy(&byte, c_buf + i, 1);
        }
        uint32_t res = ssi_transfer(s->spi, byte);
        if (i < rx_bytes) {
            memcpy(c_buf + i, &res, 1);
        }
    }
}

static void esp32_spi_cs_set(Esp32SpiState *s, int value)
{
    for (int i = 0; i < ESP32_SPI_CS_COUNT; ++i) {
        qemu_set_irq(s->cs_gpio[i], ((s->pin_reg & (1 << i)) == 0) ? value : 1);
    }
}

static void esp32_spi_transaction(Esp32SpiState *s, Esp32SpiTransaction *t)
{
    if(s->xfer_32_bits) {
        uint32_t *data=(uint32_t *)(t->data);
        for (int i = 0; i < (t->data_tx_bytes+3)/4; i++) {
            ssi_transfer(s->spi, *data++);
        }
        return;
    }
    esp32_spi_cs_set(s, 0);
    esp32_spi_txrx_buffer(s, &t->cmd, t->cmd_bytes, 0);
    esp32_spi_txrx_buffer(s, &t->addr, t->addr_bytes, 0);
    esp32_spi_txrx_buffer(s, t->data, t->data_tx_bytes, t->data_rx_bytes);
    esp32_spi_cs_set(s, 1);
}

/* Convert one of the hardware "bitlen" registers to a byte count */
static inline int bitlen_to_bytes(uint32_t val)
{
    return (val + 1 + 7) / 8; /* bitlen registers hold number of bits, minus one */
}

static void maybe_encrypt_data(Esp32SpiState *s)
{
    Esp32FlashEncryptionState* flash_enc = esp32_flash_encryption_find();
    if (esp32_flash_encryption_enabled(flash_enc)) {
        esp32_flash_encryption_get_result(flash_enc, &s->data_reg[0], 8);
    }
}

static void esp32_spi_do_command(Esp32SpiState* s, uint32_t cmd_reg)
{
    Esp32SpiTransaction t = {
        .cmd_bytes = 1
    };
    switch (cmd_reg) {
    case R_SPI_CMD_READ_MASK:
        t.cmd = CMD_READ;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        t.data = &s->data_reg[0];
        t.data_rx_bytes = bitlen_to_bytes(s->miso_dlen_reg);
        break;

    case R_SPI_CMD_WREN_MASK:
        t.cmd = CMD_WREN;
        break;

    case R_SPI_CMD_WRDI_MASK:
        t.cmd = CMD_WRDI;
        break;

    case R_SPI_CMD_RDID_MASK:
        t.cmd = CMD_RDID;
        t.data = &s->data_reg[0];
        t.data_rx_bytes = 3;
        break;

    case R_SPI_CMD_RDSR_MASK:
        t.cmd = CMD_RDSR;
        t.data = &s->status_reg;
        t.data_rx_bytes = 1;
        break;

    case R_SPI_CMD_WRSR_MASK:
        t.cmd = CMD_WRSR;
        t.data = &s->status_reg;
        t.data_tx_bytes = 1;
        break;

    case R_SPI_CMD_PP_MASK:
        maybe_encrypt_data(s);
        t.cmd = CMD_PP;
        t.data = &s->data_reg[0];
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> 8;
        t.data = &s->data_reg[0];
        t.data_tx_bytes = s->addr_reg >> 24;
        break;

    case R_SPI_CMD_SE_MASK:
        t.cmd = CMD_SE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_BE_MASK:
        t.cmd = CMD_BE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_CE_MASK:
        t.cmd = CMD_CE;
        break;

    case R_SPI_CMD_DP_MASK:
        t.cmd = CMD_DP;
        break;

    case R_SPI_CMD_RES_MASK:
        t.cmd = CMD_RES;
        t.data = &s->data_reg[0];
        t.data_rx_bytes = 3;
        break;

    case R_SPI_CMD_USR_MASK:
        if (s->outlink_reg & R_SPI_DMA_OUT_LINK_START_MASK) {
            // a DMA transfer
            int data = 0;
            int len;
            uint32_t buffer[1024];
            // outlink holds the bottom bits of the address of
            // the DMA command list 
            unsigned addr = (0x3ff00000 | (s->outlink_reg & R_SPI_DMA_OUT_LINK_ADDR_MASK));
            int v[3];
            int total_len=0;
            uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            
            BusState *b = BUS(s->spi);
            BusChild *ch = QTAILQ_FIRST(&b->children);
            SSIPeripheral *peripheral = SSI_PERIPHERAL(ch->child);
            SSIPeripheralClass *ssc = SSI_PERIPHERAL_GET_CLASS(peripheral);

            do {
                // read the next dma command from the list
                address_space_read(&address_space_memory, addr,
                                   MEMTXATTRS_UNSPECIFIED, v, 12);
                len = v[0] & 4095;
                data = v[1];
                addr = v[2];
                buffer[0]=0;
                // copy the data into a buffer (max 4092 bytes)
                address_space_read(&address_space_memory, data,
                                   MEMTXATTRS_UNSPECIFIED, buffer, len);
                if(s->xfer_32_bits) {
                    for (int i = 0; i < (len+3)/4; i++) {    
                        ssc->transfer(peripheral,buffer[i]);
                    }
                } else {
                    uint8_t *chb=(uint8_t *)buffer;
                    for (int i = 0; i < len; i++) {    
                        ssc->transfer(peripheral,chb[i]);
                    }
                }
                total_len+=len;
            } while (addr != 0);            
            uint64_t ns_to_timeout = s->mosi_dlen_reg * 25;  // about 75fps, same a real hw
            timer_mod_ns(&s->spi_timer,
                                            ns_now + ns_to_timeout);
            return;
        }
        maybe_encrypt_data(s);
        if (FIELD_EX32(s->user_reg, SPI_USER, COMMAND) || FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN)) {
            t.cmd = FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_VALUE);
            t.cmd_bytes = bitlen_to_bytes(FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN));
        } else {
            t.cmd_bytes = 0;
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, ADDR)) {
            t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
            t.addr = bswap32(s->addr_reg);
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, MOSI)) {
            t.data = &s->data_reg[0];
            t.data_tx_bytes = bitlen_to_bytes(s->mosi_dlen_reg);
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, MISO)) {
            t.data = &s->data_reg[0];
            t.data_rx_bytes = bitlen_to_bytes(s->miso_dlen_reg);
        }
        break;


    default:
        return;
    }
    esp32_spi_transaction(s, &t);
}


static const MemoryRegionOps esp32_spi_ops = {
    .read =  esp32_spi_read,
    .write = esp32_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_spi_reset(DeviceState *dev)
{
    Esp32SpiState *s = ESP32_SPI(dev);
    s->pin_reg = 0x6;
    s->user1_reg = FIELD_DP32(0, SPI_USER1, ADDR_BITLEN, 23);
    s->user1_reg = FIELD_DP32(s->user1_reg, SPI_USER1, DUMMY_CYCLELEN, 7);
    s->user2_reg = FIELD_DP32(0, SPI_USER2, COMMAND_BITLEN, 4);
    s->user2_reg = FIELD_DP32(s->user2_reg, SPI_USER2, COMMAND_VALUE, 0);
    s->status_reg = 0;
}

static void esp32_spi_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_spi_init(Object *obj)
{
    Esp32SpiState *s = ESP32_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_spi_ops, s,
                          TYPE_ESP32_SPI, ESP32_SPI_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    timer_init_ns(&s->spi_timer, QEMU_CLOCK_VIRTUAL, esp32_spi_timer_cb, s);

    s->spi = ssi_create_bus(DEVICE(s), "spi");
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS, ESP32_SPI_CS_COUNT);
}

static Property esp32_spi_properties[] = {
    DEFINE_PROP_BOOL("xfer_32_bits",Esp32SpiState,xfer_32_bits,false),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_spi_reset;
    dc->realize = esp32_spi_realize;
    device_class_set_props(dc, esp32_spi_properties);
}

static const TypeInfo esp32_spi_info = {
    .name = TYPE_ESP32_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SpiState),
    .instance_init = esp32_spi_init,
    .class_init = esp32_spi_class_init
};

static void esp32_spi_register_types(void)
{
    type_register_static(&esp32_spi_info);
}

type_init(esp32_spi_register_types)
