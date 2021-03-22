#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/ssi/ssi.h"

#define TYPE_ESP32_ST7789V "ssi.esp32.st7789v"
#define ESP32_SPI_ST7789V(obj) OBJECT_CHECK(Esp32Spi2State, (obj), TYPE_ESP32_ST7789V)

#define ESP32_SPI2_CS_COUNT      3
#define ESP32_SPI2_BUF_WORDS     16

typedef struct Esp32Spi2State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
//    qemu_irq irq_dma;
    qemu_irq cs_gpio[ESP32_SPI2_CS_COUNT];
    int num_cs;
    SSIBus *spi;
    qemu_irq dma_irq;
    QemuConsole *con;
    uint32_t addr_reg;
    uint32_t ctrl_reg;
    uint32_t status_reg;
    uint32_t ctrl1_reg;
    uint32_t ctrl2_reg;
    uint32_t user_reg;
    uint32_t user1_reg;
    uint32_t user2_reg;
    uint32_t mosi_dlen_reg;
    uint32_t miso_dlen_reg;
    uint32_t pin_reg;
    uint32_t slave_reg;
    uint32_t outlink_reg;
    uint32_t dmaconfig_reg;
    uint32_t current_command;
    uint32_t redraw;
    uint32_t width;
    uint32_t height;
    int32_t x_start;
    int32_t x_end;
    int32_t x_offset;
    int32_t y_start;
    int32_t y_end;
    int32_t y_offset;
    int32_t x;
    int32_t y;
    int32_t little_endian;
    QEMUTimer spi_timer;
    uint32_t data_reg[ESP32_SPI2_BUF_WORDS];
} Esp32Spi2State;


REG32(SPI2_CMD, 0x00)
    FIELD(SPI2_CMD, USR, 18, 1)

REG32(SPI2_ADDR, 0x04)
REG32(SPI2_CTRL, 0x08)
REG32(SPI2_STATUS, 0x10)
    FIELD(SPI2_STATUS, STATUS, 0, 16)

REG32(SPI2_CTRL1, 0x0c)
REG32(SPI2_CTRL2, 0x14)
REG32(SPI2_CLOCK, 0x18)
REG32(SPI2_USER, 0x1C)
    FIELD(SPI2_USER, COMMAND, 31, 1)
    FIELD(SPI2_USER, ADDR, 30, 1)
    FIELD(SPI2_USER, DUMMY, 29, 1)
    FIELD(SPI2_USER, MISO, 28, 1)
    FIELD(SPI2_USER, MOSI, 27, 1)
    FIELD(SPI2_USER, SIO, 16, 1)
    FIELD(SPI2_USER, DOUTDIN, 0, 1)

REG32(SPI2_USER1, 0x20)
    FIELD(SPI2_USER1, ADDR_BITLEN, 26, 6)
    FIELD(SPI2_USER1, DUMMY_CYCLELEN, 0, 8)

REG32(SPI2_USER2, 0x24)
    FIELD(SPI2_USER2, COMMAND_BITLEN, 28, 4)
    FIELD(SPI2_USER2, COMMAND_VALUE, 0, 16)

REG32(SPI2_MOSI_DLEN, 0x28)
REG32(SPI2_MISO_DLEN, 0x2c)
REG32(SPI2_PIN, 0x34)
REG32(SPI2_SLAVE, 0x38)
    FIELD(SPI2_SLAVE, TRANS_DONE, 4, 1)
REG32(SPI2_W0, 0x80)
REG32(SPI2_EXT0, 0xF0)
REG32(SPI2_EXT1, 0xF4)
REG32(SPI2_EXT2, 0xF8)
REG32(SPI2_EXT3, 0xFC)
REG32(SPI2_DMA_CONF, 0x100)
REG32(SPI2_DMA_OUT_LINK, 0x104)
REG32(SPI2_DMA_IN_LINK, 0x108)

