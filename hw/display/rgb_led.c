/*
 * RGBLED  Emulation
 * 
 * 
 * Martin Johnson 2022 M.J.Johnson@massey.ac.nz
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
#include "ui/input.h"
#include "hw/irq.h"

#define PANEL_WIDTH 256
#define PANEL_HEIGHT 256

/*
typedef struct ConsoleState {
    QemuConsole *con;
    uint32_t redraw;
    uint32_t *data; // surface data
} ConsoleState;
*/
// only one console


struct  RgbledState {
    SSISlave ssidev;
    int width,height;
    QemuConsole *con;
    uint32_t redraw;
    uint32_t *data; 
    qemu_irq button[2];
    int current_led;
    int current_bit;
    int current_value;
};

#define TYPE_RGBLED "rgbled"
OBJECT_DECLARE_SIMPLE_TYPE(RgbledState, RGBLED)

typedef struct { uint8_t r; uint8_t g; uint8_t b; uint8_t a;} pixel;


// transfer 32 bits at a time to speed things up.
// this needs the spi controller to do the same thing.
static uint32_t rgbled_transfer(SSISlave *dev, uint32_t data)
{
    RgbledState *s = (RgbledState *)(dev);//RGBLED(dev);
   // ConsoleState *c=s->con;
  //  printf("data %x\n",data);
   // int v;
    int t1=data & 0x7fff;
    int t0=(data & 0x7fff0000)>>16;
    s->current_value <<= 1;
    if(t1>t0) s->current_value |= 1;
    s->current_bit++;
   // if((data&0x8000)==0 && (data&0x80000000)==0) {
       if(t1>17 || t0>17) {
        printf("end\n");
        s->current_led=0;
    }

    if(s->current_bit==24) {
        s->current_bit=0;
        int x=s->current_led%16;
        int y=s->current_led/16;
//        if(y&1) x=15-x;
        int pixel=(s->current_value) | 0xff000000;
        for(int i=0;i<16;i++)
            for(int j=0;j<16;j++)
                s->data[x*16+j+(y*16+i)*16*16]=pixel;
        s->current_led++;
        if(s->current_led==100) {
            s->current_led=0;
            s->redraw=1;
            printf("redraw\n");
        }
    }
    return 0;
}

static void rgbled_update_display(void *opaque) {
    RgbledState *s = RGBLED(opaque);
    //printf("update %d\n",s->redraw);
    if (!s->redraw) return;
    s->redraw = 0;
    dpy_gfx_update(s->con, 0, 0, PANEL_WIDTH, PANEL_HEIGHT);
}

static void rgbled_invalidate_display(void *opaque) {
    RgbledState *s = RGBLED(opaque);
    s->redraw = 1;
}

static const GraphicHwOps rgbled_ops = {
    .invalidate = rgbled_invalidate_display,
    .gfx_update = rgbled_update_display,
};


static void rgbled_realize(SSISlave *d, Error **errp) {
    
    RgbledState *s = RGBLED(d);
    DeviceState *dev = DEVICE(s);

    s->con=graphic_console_init(dev, 0, &rgbled_ops, s);
    qemu_console_resize(s->con, PANEL_HEIGHT,
                        PANEL_HEIGHT);
    s->data=surface_data(qemu_console_surface(s->con));
}

static void rgbled_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);

    k->realize = rgbled_realize;
    k->transfer = rgbled_transfer;
    k->cs_polarity = SSI_CS_NONE;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo rgbled_info = {
    .name = TYPE_RGBLED,
    .parent = TYPE_SSI_SLAVE,
    .instance_size = sizeof(RgbledState),
    .class_init = rgbled_class_init};

static void rgbled_register_types(void) {
    type_register_static(&rgbled_info);
}

type_init(rgbled_register_types)
