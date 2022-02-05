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
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "ui/console.h"
#include "ui/input.h"
#include "hw/irq.h"
#include <math.h>

#include "ws2812b.c"

#define PANEL_WIDTH (s->width*16)
#define PANEL_HEIGHT (s->height*16)

struct  RgbledState {
    SSISlave ssidev;
    int width,height;
    QemuConsole *con;
    uint32_t redraw;
    uint32_t *data; 
    int current_led;
    int current_bit;
    int current_value;
};

#define TYPE_RGBLED "rgbled"
OBJECT_DECLARE_SIMPLE_TYPE(RgbledState, RGBLED)

static int min(uint32_t a,uint32_t b) {
    return a<b?a:b;
}

static void draw_led(RgbledState *s, int x, int y, int r,int g, int b) {
    float m=sqrtf(r*r+g*g+b*b);
    if(m==0) m=1;
    b=min(255*b/m,255);
    r=min(255*r/m,255);
    g=min(255*g/m,255);  
    float radius=1.8f+(m/100.0f);

    for(int i=0;i<16;i++)
        for(int j=0;j<16;j++) {
            int rr=ws2812b.pixel_data[(i+j*16)*3];
            int gg=ws2812b.pixel_data[(i+j*16)*3+1];
            int bb=ws2812b.pixel_data[(i+j*16)*3+2];
            float d=(i-8)*(i-8)+(j-8)*(j-8);
            d=1.0f-sqrt(sqrt(d))/radius;
            if(d<0) d=0;
            rr=min(rr+(d*r),255);
            gg=min(gg+(d*g),255);
            bb=min(bb+(d*b),255);
            int pixel=(rr<<16) | (gg<<8) | bb | 0xff000000;
            s->data[x*16+j+(y*16+i)*16*s->width]=pixel;
        }
}

static uint32_t rgbled_transfer(SSISlave *dev, uint32_t data)
{
    RgbledState *s = (RgbledState *)(dev);
    int t1=data & 0x7fff;
    int t0=(data & 0x7fff0000)>>16;
    s->current_value <<= 1;
    if(t1>t0) s->current_value |= 1;
    s->current_bit++;
    if(s->current_bit==24) {
        s->current_bit=0;
        if(s->current_led<s->width*s->height) {
            int x=s->current_led%s->width;
            int y=s->current_led/s->width;
            if(y&1) x=15-x;
            int b=s->current_value & 0xff;
            int r=(s->current_value>>8) & 0xff;
            int g=(s->current_value>>16) & 0xff;
            draw_led(s,x,y,r,g,b);
        }
        s->current_led++;
    }
    if(t0>17) {
        s->current_bit=0;
        s->current_led=0;
        s->redraw=1;
    }
    return 0;
}

static void rgbled_update_display(void *opaque) {
    RgbledState *s = RGBLED(opaque);
    if (!s->redraw) return;
    s->redraw = 0;
    dpy_gfx_update(s->con, 0, 0, s->width*16, s->height*16);
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
    qemu_console_resize(s->con,s->width*16, s->height*16);
    s->data=surface_data(qemu_console_surface(s->con));

}
static void rgbled_reset(DeviceState *obj)
{
    RgbledState *s = RGBLED(obj);
    s->current_led=0;
    s->current_bit=0;
    for(int i=0;i<s->width;i++)
        for(int j=0;j<s->height;j++)
            draw_led(s,i,j,0,0,0);
}


static Property rgb_led_properties[] = {
    DEFINE_PROP_INT32("width", RgbledState, width, 16),
    DEFINE_PROP_INT32("height", RgbledState, height, 16),
    DEFINE_PROP_END_OF_LIST(),
};

static void rgbled_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);
    dc->reset = rgbled_reset;
    k->realize = rgbled_realize;
    k->transfer = rgbled_transfer;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    device_class_set_props(dc, rgb_led_properties);
}

static const TypeInfo rgbled_info = {
    .name = TYPE_RGBLED,
    .parent = TYPE_SSI_SLAVE,
    .instance_size = sizeof(RgbledState),
    .class_init = rgbled_class_init
};

static void rgbled_register_types(void) {
    type_register_static(&rgbled_info);
}

type_init(rgbled_register_types)
