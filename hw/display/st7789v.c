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
#include "hw/ssi/ssi.h"
#include "ui/console.h"
#include "st7789v.h"

#define DEBUG 0

enum {
    CMD_CASET = 0x2a,
    CMD_RASET = 0x2b,
    CMD_RAMWR = 0x2c,
};
struct  St7789vState {
    SSISlave ssidev;
    QemuConsole *con;

    uint32_t cmd_len;
    int32_t cmd;
    int32_t cmd_data[8];
    uint32_t current_command;
    int data_index;
    uint32_t redraw;
    int cmd_mode;
    int little_endian;
    int32_t x_start;
    int32_t x_end;
    int32_t y_start;
    int32_t y_end;
    int32_t x;
    int32_t y;
    int backlight;
};

#define TYPE_ST7789V "st7789v"
OBJECT_DECLARE_SIMPLE_TYPE(St7789vState, ST7789V)

#define MAGNIFY 1
#define REDUCE 2
#define st7789v_REG_SIZE 0x1000

void update_irq(St7789vState *s);

unsigned short frame_buffer[240 * 135];

static int redraw=0;

extern const struct {
  guint          width;
  guint          height;
  guint          bytes_per_pixel; 
  guint8         pixel_data[416 * 948 * 4 + 1];
} ttgo_board_skin;

static int width,height,x_offset,y_offset;;

void draw_skin(St7789vState *s);

void draw_skin(St7789vState *s) {
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

static uint32_t st7789v_transfer(SSISlave *dev, uint32_t data)
{
    St7789vState *s = ST7789V(dev);

    if(s->cmd_mode) {
        s->current_command=data;
        s->data_index=0;
    } else {
        switch (s->current_command) {
            case ST7789_MADCTL:
                if (data == 0 || data == 8) {  // portrait
                    qemu_console_resize(s->con, ttgo_board_skin.width / REDUCE,
                                        ttgo_board_skin.height / REDUCE);
                    width = 135;
                    height = 240;
                    x_offset = 52;
                    y_offset = 40;
                } else {  // landscape
                    qemu_console_resize(s->con, ttgo_board_skin.height / REDUCE,
                                        ttgo_board_skin.width / REDUCE);
                    width = 240;
                    height = 135;
                    x_offset = 40;
                    y_offset = 53;
                }
                draw_skin(s);
                break;
            case ST7789_CASET:
                s->cmd_data[s->data_index++]=data;
                if(s->data_index==4) {
                    s->x_start = s->cmd_data[1] + s->cmd_data[0] * 256;
                    s->x_end = s->cmd_data[3] + s->cmd_data[2] * 256;
                    s->x = s->x_start;
                }
                break;
            case ST7789_RASET:
                s->cmd_data[s->data_index++]=data;
                if(s->data_index==4) {
                    s->y_start = s->cmd_data[1] + s->cmd_data[0] * 256;
                    s->y_end = s->cmd_data[3] + s->cmd_data[2] * 256;
                    s->y = s->y_start;
                }
                break;
            case ST7789_RAMCTRL:
                s->little_endian = 1;
                break;
            case ST7789_RAMWR:
                s->cmd_data[(s->data_index++)%2]=data;
                if(((s->data_index)%2)==1) break;

                if (!s->little_endian) {
                    data = (s->cmd_data[0] << 8) | s->cmd_data[1];
                } else {
                    data = (s->cmd_data[1] << 8) | s->cmd_data[0];
                }
                uint16_t offset = (s->y - y_offset) * width + s->x - x_offset;
                if (offset < (135 * 240)) frame_buffer[offset] = data;
                s->x++;
                if (s->x > s->x_end) {
                    s->x = s->x_start;
                    s->y++;
                }
                if ((s->y > s->y_end)) {
                    s->y = s->y_start;
                    s->x = s->x_start;
                    redraw=1;
                }
                break;
            }
    }
    return 0;
}


/* Command/data input.  */
static void st7789v_cd(void *opaque, int n, int level)
{
    St7789vState *s = (St7789vState *)opaque;
    s->cmd_mode = !level;
}

static void st7789v_backlight(void *opaque, int n, int level)
{
    St7789vState *s = (St7789vState *)opaque;
    s->backlight = level;
}

int pp = 0;
static void st7789_update_display(void *opaque) {
    St7789vState *s = (St7789vState *)opaque;
    if (!redraw) return;
    
    DisplaySurface *surface = qemu_console_surface(s->con);
    volatile unsigned *dest = (unsigned *)surface_data(surface);


    for (int i = 0; i < width; i++)
        for (int j = 0; j < height; j++)
            for (int ii = 0; ii < MAGNIFY; ii++)
                for (int jj = 0; jj < MAGNIFY; jj++) {
                    unsigned fbv = frame_buffer[j * width + i];
                    int red = (fbv & 0xf800) >> 8;
                    int green = (fbv & 0x7e0) >> 3;
                    int blue = (fbv & 0x1f) << 3;
                    if(!(s->backlight)) {
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

    if(width>height)
        dpy_gfx_update(s->con, 126/REDUCE, 82/REDUCE, width * MAGNIFY, height * MAGNIFY);
    else
        dpy_gfx_update(s->con, 62/REDUCE, 126/REDUCE, width * MAGNIFY, height * MAGNIFY);
    pp += 10;
}
static void st7789_invalidate_display(void *opaque) {
    redraw = 1;
}

static const GraphicHwOps st7789_ops = {
    .invalidate = st7789_invalidate_display,
    .gfx_update = st7789_update_display,
};



static void st7789v_realize(SSISlave *d, Error **errp) {
    DeviceState *dev = DEVICE(d);
    St7789vState *s = ST7789V(d);
    static QemuConsole *console=0;
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
    qdev_init_gpio_in_named(dev, st7789v_cd, "cmd",1);
    qdev_init_gpio_in_named(dev, st7789v_backlight, "backlight", 1);
}

static void st7789v_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);

    k->realize = st7789v_realize;
    k->transfer = st7789v_transfer;
    k->cs_polarity = SSI_CS_LOW;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo st7789v_info = {
    .name = TYPE_ST7789V,
    .parent = TYPE_SSI_SLAVE,
    .instance_size = sizeof(St7789vState),
    .class_init = st7789v_class_init};

static void st7789v_register_types(void) {
    type_register_static(&st7789v_info);
}

type_init(st7789v_register_types)
