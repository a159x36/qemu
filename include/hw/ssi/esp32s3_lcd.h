#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/ssi/ssi.h"
#include "hw/dma/esp32s3_gdma.h"

#define TYPE_ESP32S3_LCD "ssi.esp32s3.lcd"
#define ESP32S3_LCD(obj) OBJECT_CHECK(ESP32S3LcdState, (obj), TYPE_ESP32S3_LCD)

/**
 * Size of the LCD I/O registers area
 */
#define ESP32S3_LCD_IO_SIZE (256)


typedef struct ESP32S3LcdState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *lcd;
    uint32_t int_en;
    uint32_t int_raw;
    uint32_t cmd_val;
    uint32_t user;
        /* Public: must be set before realizing instance*/
    ESPGdmaState *gdma;
    qemu_irq cmd_gpio;
    qemu_irq irq;
    QEMUTimer lcd_timer;

} ESP32S3LcdState;


REG32(LCD_CAM_CLOCK, 0x000)
REG32(LCD_CAM_LCD_USER, 0x014)
    FIELD(LCD_CAM_LCD_USER, DOUT, 24, 1)
    FIELD(LCD_CAM_LCD_USER, DUMMY, 25, 1)
    FIELD(LCD_CAM_LCD_USER, CMD, 26, 1)
    FIELD(LCD_CAM_LCD_USER, START, 27, 1)
    FIELD(LCD_CAM_LCD_USER, RESET, 28, 1)
REG32(LCD_CAM_CMD_VAL, 0x028)
REG32(LCD_CAM_LC_DMA_INT_ENA, 0x064)
REG32(LCD_CAM_LC_DMA_INT_RAW, 0x068)
    FIELD(LCD_CAM_LC_DMA_INT_RAW,TRANS_DONE_INT_RAW,1,1)
REG32(LCD_CAM_LC_DMA_INT_ST, 0x06c)
REG32(LCD_CAM_LC_DMA_INT_CLR, 0x070)

