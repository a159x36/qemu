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
#include "hw/ssi/esp32_spi_st7789v.h"
#include "ui/console.h"

#define DEBUG 1

enum {
    CMD_CASET = 0x2a,
    CMD_RASET = 0x2b,
    CMD_RAMWR = 0x2c,
};


#define ESP32_SPI_REG_SIZE    0x1000

static void esp32_spi_do_command(Esp32Spi2State* state, uint32_t cmd_reg);
void update_irq(Esp32Spi2State *s);

void update_irq(Esp32Spi2State *s) {
    if(s->slave_reg & 0x200)
          qemu_irq_raise(s->irq);

}
static uint64_t esp32_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32Spi2State *s = ESP32_SPI_ST7789V(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_SPI2_ADDR:
        r = s->addr_reg;
        break;
    case A_SPI2_CTRL:
        r = s->ctrl_reg;
        break;
    case A_SPI2_STATUS:
        r = s->status_reg;
        break;
    case A_SPI2_CTRL1:
        r = s->ctrl1_reg;
        break;
    case A_SPI2_CTRL2:
        r = s->ctrl2_reg;
        break;
    case A_SPI2_USER:
        r = s->user_reg;
        break;
    case A_SPI2_USER1:
        r = s->user1_reg;
        break;
    case A_SPI2_USER2:
        r = s->user2_reg;
        break;
    case A_SPI2_MOSI_DLEN:
        r = s->mosi_dlen_reg;
        break;
    case A_SPI2_MISO_DLEN:
        r = s->miso_dlen_reg;
        break;
    case A_SPI2_PIN:
        r = s->pin_reg;
        break;
    case A_SPI2_W0 ... A_SPI2_W0 + (ESP32_SPI2_BUF_WORDS - 1) * sizeof(uint32_t):
        r = s->data_reg[(addr - A_SPI2_W0) / sizeof(uint32_t)];
        break;
    case A_SPI2_EXT2:
        r = 0;
        break;
    case A_SPI2_SLAVE:
        r= s->slave_reg;// transaction done
        break;
    }
    if(DEBUG)
      qemu_log("spi_read %lx, %lx\n",addr, r);
    update_irq(s);
    return r;
}

static void esp32_spi_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32Spi2State *s = ESP32_SPI_ST7789V(opaque);
    if(DEBUG)
      qemu_log("spi_write %lx, %lx\n",addr, value);
    switch (addr) {
    case A_SPI2_W0 ... A_SPI2_W0 + (ESP32_SPI2_BUF_WORDS - 1) * sizeof(uint32_t):
        s->data_reg[(addr - A_SPI2_W0) / sizeof(uint32_t)] = value;
        break;
    case A_SPI2_ADDR:
        s->addr_reg = value;
        break;
    case A_SPI2_CTRL:
        s->ctrl_reg = value;
        break;
    case A_SPI2_STATUS:
        s->status_reg = value;
        break;
    case A_SPI2_CTRL1:
        s->ctrl1_reg = value;
        break;
    case A_SPI2_CTRL2:
        s->ctrl2_reg = value;
        break;
    case A_SPI2_USER:
        s->user_reg = value;
        break;
    case A_SPI2_USER1:
        s->user1_reg = value;
        break;
    case A_SPI2_USER2:
        s->user2_reg = value;
        break;
    case A_SPI2_MOSI_DLEN:
        s->mosi_dlen_reg = value;
        break;
    case A_SPI2_MISO_DLEN:
        s->miso_dlen_reg = value;
        break;
    case A_SPI2_PIN:
        s->pin_reg = value;
        break;
    case A_SPI2_CMD:
        esp32_spi_do_command(s, value);
        if(value & 0x40000) {
            s->slave_reg |= 0x10;

        }
        break;
    case A_SPI2_SLAVE:
        s->slave_reg = value;// transaction done
       // s->slave_reg &= ~0x10;
       break;

    }
    update_irq(s);
}

typedef struct Esp32Spi2Transaction {
    int cmd_bytes;
    uint32_t cmd;
    int addr_bytes;
    uint32_t addr;
    int data_tx_bytes;
    int data_rx_bytes;
    uint32_t* data;
} Esp32Spi2Transaction;
/*
static void esp32_spi_txrx_buffer(Esp32SpiState *s, void *buf, int tx_bytes, int rx_bytes)
{
    int bytes = MAX(tx_bytes, rx_bytes);
    uint8_t *c_buf = (uint8_t*) buf;
    for (int i = 0; i < bytes; ++i) {
        uint8_t byte = 0;
        if (byte < tx_bytes) {
            memcpy(&byte, c_buf + i, 1);
        }
        uint32_t res = ssi_transfer(s->spi, byte);
        if (byte < rx_bytes) {
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
    esp32_spi_cs_set(s, 0);
    esp32_spi_txrx_buffer(s, &t->cmd, t->cmd_bytes, 0);
    esp32_spi_txrx_buffer(s, &t->addr, t->addr_bytes, 0);
    esp32_spi_txrx_buffer(s, t->data, t->data_tx_bytes, t->data_rx_bytes);
    esp32_spi_cs_set(s, 1);
}
*/

/* Convert one of the hardware "bitlen" registers to a byte count */
static inline int bitlen_to_bytes(uint32_t val)
{
    return (val + 1 + 7) / 8; /* bitlen registers hold number of bits, minus one */
}

static void esp32_spi_do_command(Esp32Spi2State* s, uint32_t cmd_reg)
{
    /*
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
    */
}


static const MemoryRegionOps esp32_spi_ops = {
    .read =  esp32_spi_read,
    .write = esp32_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_spi_reset(DeviceState *dev)
{
    Esp32Spi2State *s = ESP32_SPI_ST7789V(dev);
    s->pin_reg = 0x6;
    s->user1_reg = FIELD_DP32(0, SPI2_USER1, ADDR_BITLEN, 23);
    s->user1_reg = FIELD_DP32(s->user1_reg, SPI2_USER1, DUMMY_CYCLELEN, 7);
    s->status_reg = 0;
}

static void esp32_spi_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_spi_init(Object *obj)
{
    Esp32Spi2State *s = ESP32_SPI_ST7789V(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_spi_ops, s,
                          TYPE_ESP32_ST7789V, ESP32_SPI_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
//    sysbus_init_irq(sbd, &s->irq);
//    sysbus_init_irq(sbd, &s->irq_dma);

//    qemu_irq dma_irq=qdev_get_gpio_in(DEVICE(&s->dport.intmatrix), ETS_SPI2_DMA_INTR_SOURCE);
//    sysbus_init_irq(sbd, &s->dma_irq);
    
    s->spi = ssi_create_bus(DEVICE(s), "spi");
//    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS, ESP32_SPI2_CS_COUNT);
 qdev_init_gpio_out_named(DEVICE(s),&s->irq,SYSBUS_DEVICE_GPIO_IRQ, 1);
   printf("spi irq=%x\n",(s->irq).n); 

}

static Property esp32_spi_properties[] = {
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
    .name = TYPE_ESP32_ST7789V,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32Spi2State),
    .instance_init = esp32_spi_init,
    .class_init = esp32_spi_class_init
};

static void esp32_spi_register_types(void)
{
    type_register_static(&esp32_spi_info);
}

type_init(esp32_spi_register_types)
