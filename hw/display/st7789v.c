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
#include "hw/ssi/esp32_spi_st7789v.h"
#include "hw/ssi/ssi.h"
#include "exec/address-spaces.h"

#include "ui/console.h"

#define DEBUG 0

enum {
    CMD_CASET = 0x2a,
    CMD_RASET = 0x2b,
    CMD_RAMWR = 0x2c,
};
#define MAGNIFY 1
#define REDUCE 2
#define ESP32_SPI_REG_SIZE 0x1000

//static void esp32_spi_do_command(Esp32Spi2State *state, uint32_t cmd_reg);
void update_irq(Esp32Spi2State *s);

unsigned short frame_buffer[240 * 135];

static int redraw=0;

void update_irq(Esp32Spi2State *s) {
    if (s->slave_reg & 0x200) {
        if (s->slave_reg & 0x10)
            qemu_irq_raise(s->irq);
        else
            qemu_irq_lower(s->irq);

        // s->slave_reg &= ~0x10;
    }
}

extern const struct {
  guint          width;
  guint          height;
  guint          bytes_per_pixel; // 2:RGB16, 3:RGB, 4:RGBA 
  guint8         pixel_data[416 * 948 * 4 + 1];
} ttgo_board_skin;

static int width,height,x_offset,y_offset;;

void draw_skin(Esp32Spi2State *s);

void draw_skin(Esp32Spi2State *s) {
    DisplaySurface *surface = qemu_console_surface(s->con);
    volatile unsigned *dest = (unsigned *)surface_data(surface);
    for (int i = 0; i < ttgo_board_skin.height/REDUCE; i++)
        for (int j = 0; j < ttgo_board_skin.width/REDUCE; j++) {
	    int red=0,green=0,blue=0,trans=0;
            for(int ii=0;ii<REDUCE;ii++)
		for(int jj=0;jj<REDUCE;jj++) {
            int index=((i*REDUCE+ii)*ttgo_board_skin.width+j*REDUCE+jj)*4;
            	red+=ttgo_board_skin.pixel_data[index];
            	green+=ttgo_board_skin.pixel_data[index+1];
            	blue+=ttgo_board_skin.pixel_data[index+2];
            	trans+=ttgo_board_skin.pixel_data[index+3];
		}
		red=red/(REDUCE*REDUCE);green=green/(REDUCE*REDUCE);
		blue=blue/(REDUCE*REDUCE);trans=trans/(REDUCE*REDUCE);
            if(trans<200) {red=green=blue=0;trans=255;}
            if(width<height) // portrait
                dest[i*ttgo_board_skin.width/REDUCE+j]=(trans<<24) | (red<<16) | (green<<8) | blue; 
            else
                dest[(ttgo_board_skin.width/REDUCE-j-1)*ttgo_board_skin.height/REDUCE+i]=(trans<<24) | (red<<16) | (green<<8) | blue; 

        }

}

static void esp32_spi_cs_set(Esp32Spi2State *s, int value)
{
    for (int i = 0; i < ESP32_SPI2_CS_COUNT; ++i) {
        qemu_set_irq(s->cs_gpio[i], ((s->pin_reg & (1 << i)) == 0) ? value : 1);
    }
}
static uint64_t esp32_spi_read(void *opaque, hwaddr addr, unsigned int size) {
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
        case A_SPI2_W0 ... A_SPI2_W0 +
            (ESP32_SPI2_BUF_WORDS - 1) * sizeof(uint32_t):
            r = s->data_reg[(addr - A_SPI2_W0) / sizeof(uint32_t)];
            break;
        case A_SPI2_EXT2:
            r = 0;
            break;
        case A_SPI2_SLAVE:
            r = s->slave_reg;  // transaction done
            break;
        case A_SPI2_DMA_OUT_LINK:
            r = s->outlink_reg;
            break;
        case A_SPI2_DMA_CONF:
            r = s->dmaconfig_reg;
            break;
    }
    #if DEBUG
    qemu_log("spi_read %lx, %lx\n", addr, r);
    #endif
    // update_irq(s);
    return r;
}
static void esp32_spi_timer_cb(void *opaque) {
    Esp32Spi2State *s = ESP32_SPI_ST7789V(opaque);
    // timer_del(&ts->alarm_timer);
    s->slave_reg |= 0x10;
    esp32_spi_cs_set(s,1);
    update_irq(s);
}

static void esp32_spi_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned int size) {
    Esp32Spi2State *s = ESP32_SPI_ST7789V(opaque);
    #if DEBUG
    qemu_log("spi_write %lx, %lx\n", addr, value);
    #endif
   // int doirq;
    switch (addr) {
        case A_SPI2_W0 ... A_SPI2_W0 +
            (ESP32_SPI2_BUF_WORDS - 1) * sizeof(uint32_t):
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
           // esp32_spi_do_command(s, value);
            if (value & 0x40000) {
                value=s->outlink_reg;
		int cmd=0;
		int data=0 ;
		char *dr=(char *)(s->data_reg);

                if ((value & 0x20000000)) {
                    unsigned addr = (0x3ff00000 + (value & 0xfffff));
                    int v[3];
                    address_space_read(&address_space_memory, addr,
                            MEMTXATTRS_UNSPECIFIED, v, 12);
                   // int size = v[0] & 0xfff;
                    data = v[1];
                    address_space_read(&address_space_memory, data,
                            MEMTXATTRS_UNSPECIFIED, &cmd, 1);
		} else cmd= s->data_reg[0] & 0xff;
                    int gpios;
                    address_space_read(&address_space_memory, 0x3FF44004,
                            MEMTXATTRS_UNSPECIFIED, &gpios, 4);
                    if(!(gpios & (1<<16))) {
                            s->current_command=cmd;
                    } else {
                    if(s->current_command==0x36) {
                        if(cmd==0 || cmd==8) { // portrait
                            qemu_console_resize(s->con, ttgo_board_skin.width/REDUCE, ttgo_board_skin.height/REDUCE);
                            width=135;
                            height=240;
                            x_offset=52;
                            y_offset=40;
                        } else {
                            qemu_console_resize(s->con, ttgo_board_skin.height/REDUCE,  ttgo_board_skin.width/REDUCE);
                            width=240;
                            height=135;
                            x_offset=40;
                            y_offset=53;
                        }
                        draw_skin(s);
                    }
                    if(s->current_command==0x2a) { //CAS_SET
                        unsigned char xx[4];
			if ((value & 0x20000000))
                        address_space_read(&address_space_memory, data,
                            MEMTXATTRS_UNSPECIFIED, xx, 4);
			else { for(int i=0;i<4;i++) xx[i]=dr[i];}
                        s->x_start=xx[1]+xx[0]*256;
                   //     if(s->x_start-s->x_offset <0)
                            
                        s->x_end=xx[3]+xx[2]*256;
			s->x=s->x_start;
                    }
                    if(s->current_command==0x2b) { //RAS_SET
                        unsigned char xx[4];
			if ((value & 0x20000000))
                        address_space_read(&address_space_memory, data,
                            MEMTXATTRS_UNSPECIFIED, xx, 4);
			else { for(int i=0;i<4;i++) xx[i]=dr[i];}
                        s->y_start=xx[1]+xx[0]*256;
                        s->y_end=xx[3]+xx[2]*256;
			s->y=s->y_start;
                    }
                    if(s->current_command==0xb0) { // RAM_CTRL
			s->little_endian=1;
                    }
                
                    if (s->current_command==0x2c) {
                        //printf("draw(%d,%d,%d,%d)\n",s->x_start-s->x_offset,
                        //s->y_start-y_offset,s->x_end-s->x_start+1,s->y_end-y_offset);
			if ((value & 0x20000000))
                        for(int y=s->y_start-y_offset;y<=s->y_end-y_offset;y++) {
                            if(y>=0 && y<height) {
                                address_space_read(&address_space_memory, data,
                                        MEMTXATTRS_UNSPECIFIED, frame_buffer+y*width+s->x_start-x_offset,
                                        (s->x_end-s->x_start+1)*2);
				if(!s->little_endian)
					for(int i=y*width+s->x_start-x_offset; i<(s->x_end-s->x_start+1); i++) {
						frame_buffer[i]=(frame_buffer[i]<<8)|(frame_buffer[i]>>8);
					} 
                            }
                            data+=(s->x_end-s->x_start+1)*2;
                        }
			else {
				for(int i=0;i<(s->mosi_dlen_reg+1)/16;i++) {
					uint16_t *udr=(uint16_t *)(s->data_reg);
					uint16_t offset=(s->y-y_offset)*width+s->x-x_offset;
					if(!s->little_endian) {udr[i]=(udr[i]<<8)|(udr[i]>>8);}
					if(offset<(135*240))
						frame_buffer[offset]=udr[i];
					s->x++;
					if(s->x > s->x_end) {
						s->x=s->x_start;
						s->y++;
					}
				}
			}
                        redraw = 1;
                    }
                }
		if((value & 0x20000000) || (s->y > s->y_end)) {
		  s->y=s->y_start;
		  s->x=s->x_start;
                  uint64_t ns_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                  uint64_t ns_to_timeout = s->mosi_dlen_reg * 35;//25;
                  timer_mod_anticipate_ns(&s->spi_timer, ns_now + ns_to_timeout);
                }
            }
            break;
        case A_SPI2_SLAVE:
          //  doirq=0;
          //  if((s->slave_reg & 0x10) && !(value & 0x10))
          //      doirq=1;
            s->slave_reg = value;  // transaction done
           // if(doirq)
                update_irq(s);
            break;

        case A_SPI2_DMA_OUT_LINK:
            s->outlink_reg = value;
            break;

        case A_SPI2_DMA_CONF:
            s->dmaconfig_reg = value;
            break;
    }

}
/*
typedef struct Esp32Spi2Transaction {
    int cmd_bytes;
    uint32_t cmd;
    int addr_bytes;
    uint32_t addr;
    int data_tx_bytes;
    int data_rx_bytes;
    uint32_t *data;
} Esp32Spi2Transaction;

static void esp32_spi_txrx_buffer(Esp32SpiState *s, void *buf, int tx_bytes, int
rx_bytes)
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
static inline int bitlen_to_bytes(uint32_t val) {
    return (val + 1 + 7) /
           8; /* bitlen registers hold number of bits, minus one */
}
/*
static void esp32_spi_do_command(Esp32Spi2State *s, uint32_t cmd_reg) {
    
    Esp32SpiTransaction t = {
        .cmd_bytes = 1
    };
    switch (cmd_reg) {
    case R_SPI_CMD_READ_MASK:
        t.cmd = CMD_READ;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1,
    ADDR_BITLEN)); t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
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
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1,
    ADDR_BITLEN)); t.addr = bswap32(s->addr_reg) >> 8; t.data = &s->data_reg[0];
        t.data_tx_bytes = s->addr_reg >> 24;
        break;

    case R_SPI_CMD_SE_MASK:
        t.cmd = CMD_SE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1,
    ADDR_BITLEN)); t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_BE_MASK:
        t.cmd = CMD_BE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1,
    ADDR_BITLEN)); t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
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
        if (FIELD_EX32(s->user_reg, SPI_USER, COMMAND) ||
    FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN)) { t.cmd =
    FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_VALUE); t.cmd_bytes =
    bitlen_to_bytes(FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN)); } else
    { t.cmd_bytes = 0;
        }
        if (FIELD_EX32(s->user_reg, SPI_USER, ADDR)) {
            t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1,
    ADDR_BITLEN)); t.addr = bswap32(s->addr_reg);
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
*/
static const MemoryRegionOps esp32_spi_ops = {
    .read = esp32_spi_read,
    .write = esp32_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_spi_reset(DeviceState *dev) {
    Esp32Spi2State *s = ESP32_SPI_ST7789V(dev);
    s->pin_reg = 0x6;
    s->user1_reg = FIELD_DP32(0, SPI2_USER1, ADDR_BITLEN, 23);
    s->user1_reg = FIELD_DP32(s->user1_reg, SPI2_USER1, DUMMY_CYCLELEN, 7);
    s->status_reg = 0;
    qemu_irq_lower(s->irq);
}

int pp = 0;
static void st7789_update_display(void *opaque) {
    Esp32Spi2State *s = (Esp32Spi2State *)opaque;
    if (!redraw) return;
    int gpios;
    address_space_read(&address_space_memory, 0x3FF44004,
            MEMTXATTRS_UNSPECIFIED, &gpios, 4);
    //  printf("update disp\n");
    DisplaySurface *surface = qemu_console_surface(s->con);
    volatile unsigned *dest = (unsigned *)surface_data(surface);
     //int bpp = surface_bits_per_pixel(surface);
      //printf("bpp = %d %d %d\n",bpp,frame_buffer[0],pp);

    for (int i = 0; i < width; i++)
        for (int j = 0; j < height; j++)
            for (int ii = 0; ii < MAGNIFY; ii++)
                for (int jj = 0; jj < MAGNIFY; jj++) {
                    unsigned fbv = frame_buffer[j * width + i];
                    int red = (fbv & 0xf800) >> 8;
                    int green = (fbv & 0x7e0) >> 3;
                    int blue = (fbv & 0x1f) << 3;
                    if(!(gpios & (1<<4))) {
                        red=red>>3;
                        green=green>>3;
                        blue=blue>>3;
                    }
                    if(width>height) { // landscape
                        int x=i*MAGNIFY+ii+126/REDUCE;
                        int y=j*MAGNIFY+jj+82/REDUCE;
                        *(dest + y*ttgo_board_skin.height/REDUCE+x) = (red << 16) | (green << 8) | blue;
                    } else {
                        int x=i*MAGNIFY+ii+62/REDUCE;
                        int y=j*MAGNIFY+jj+126/REDUCE;
                        *(dest + y*ttgo_board_skin.width/REDUCE+x) = (red << 16) | (green << 8) | blue;
                    }
                }
    redraw = 0;
//printf("update %d %d %d %d\n",126/REDUCE, 82/REDUCE, s->width * MAGNIFY, s->height * MAGNIFY);

    if(width>height)
        dpy_gfx_update(s->con, 126/REDUCE, 82/REDUCE, width * MAGNIFY, height * MAGNIFY);
    else
        dpy_gfx_update(s->con, 62/REDUCE, 126/REDUCE, width * MAGNIFY, height * MAGNIFY);
    pp += 10;
}
static void st7789_invalidate_display(void *opaque) {
//    Esp32Spi2State *s = (Esp32Spi2State *)opaque;
    redraw = 1;
}

static const GraphicHwOps st7789_ops = {
    .invalidate = st7789_invalidate_display,
    .gfx_update = st7789_update_display,
};



static void esp32_spi_realize(DeviceState *dev, Error **errp) {
    Esp32Spi2State *s = ESP32_SPI_ST7789V(dev);
    static QemuConsole *console=0;
    //printf("realise\n");
    if(console==0) {
      console = graphic_console_init(dev, 0, &st7789_ops, s);
      s->con = console;
      width=240;
      height=135;
      x_offset=40;
      y_offset=53;
      qemu_console_resize(s->con, ttgo_board_skin.height/REDUCE, ttgo_board_skin.width/REDUCE);
      draw_skin(s);
    } else {
      s->con = console;
    }
}

static void esp32_spi_init(Object *obj) {
    Esp32Spi2State *s = ESP32_SPI_ST7789V(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_spi_ops, s, TYPE_ESP32_ST7789V,
                          ESP32_SPI_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    timer_init_ns(&s->spi_timer, QEMU_CLOCK_VIRTUAL, esp32_spi_timer_cb, s);

    //    sysbus_init_irq(sbd, &s->irq_dma);

    //    qemu_irq dma_irq=qdev_get_gpio_in(DEVICE(&s->dport.intmatrix),
    //    ETS_SPI2_DMA_INTR_SOURCE); sysbus_init_irq(sbd, &s->dma_irq);

    s->spi = ssi_create_bus(DEVICE(s), "spi");
        qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS,
        ESP32_SPI2_CS_COUNT);
    qdev_init_gpio_out_named(DEVICE(s), &s->irq, SYSBUS_DEVICE_GPIO_IRQ, 1);
    // printf("spi irq=%x\n",(s->irq).n);
}

static Property esp32_spi_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_spi_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_spi_reset;
    dc->realize = esp32_spi_realize;
    device_class_set_props(dc, esp32_spi_properties);
}

static const TypeInfo esp32_spi2_info = {
    .name = TYPE_ESP32_ST7789V,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32Spi2State),
    .instance_init = esp32_spi_init,
    .class_init = esp32_spi_class_init};

static void esp32_spi2_register_types(void) {
    type_register_static(&esp32_spi2_info);
}

type_init(esp32_spi2_register_types)
