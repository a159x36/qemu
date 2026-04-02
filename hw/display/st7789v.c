/*
 * ST7789V LCD Emulation
 * This emulates the display on a TTGO-TDisplay Board
 * It draws the LCD in a skin and handles mouse clicks on the buttons  
 * To speed up emulation, this uses 32 bit spi transfers from the controller.
 * It wouldn't be hard to make it do 8 bit transfers instead.
 * 
 * Martin Johnson 2020 M.J.Johnson@massey.ac.nz
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
#include "ui/input.h"
#include "hw/irq.h"
//#include "hw/display/st7789v.h"
#include "sysemu/runstate.h"
#include "qemu/timer.h"
#include "hw/qdev-properties.h"
#include <pixman.h>

#define PANEL_WIDTH 240
#define PANEL_HEIGHT 135

typedef struct ConsoleState {
    QemuConsole *con;
    uint32_t redraw;
    int width,height; // lcd size in panel memory
    int x_offset,y_offset; // offset in the panel memory
    int skin_x_offset, skin_y_offset; // offset on the skin
    int skin_width;
    int32_t x_start; // area to draw into
    int32_t x_end;
    int32_t y_start;
    int32_t y_end;
    int32_t x; // current draw position
    int32_t y;
    int little_endian;
    int backlight;
    uint32_t current_command;
    int cmd_mode;
    int64_t lasttime;
    int lastlevel;
    uint16_t *fb_data;
    uint16_t *data; // surface data
    int64_t time_off;
    int64_t time_on;
    QEMUTimer backlight_timer;

} ConsoleState;

// only one console
static ConsoleState console_state;

struct  St7789vState {
    SSIPeripheral ssidev;
    ConsoleState *con;
    bool iss3;
    qemu_irq button[2];
    qemu_irq reset;
    qemu_irq touch_sensor[4];
};

#define TYPE_ST7789V "st7789v"
OBJECT_DECLARE_SIMPLE_TYPE(St7789vState, ST7789V)

#define PORTRAIT_X_OFFSET (52)
#define PORTRAIT_Y_OFFSET (40)
#define LANDSCAPE_X_OFFSET (40)
#define LANDSCAPE_Y_OFFSET (53)

#define PORTRAIT_X_OFFSET_S3 (35)
#define PORTRAIT_Y_OFFSET_S3 (0)
#define LANDSCAPE_X_OFFSET_S3 (0)
#define LANDSCAPE_Y_OFFSET_S3 (35)

#define SKIN_PORTRAIT_X_OFFSET (62/2)
#define SKIN_PORTRAIT_Y_OFFSET (126/2)
#define SKIN_LANDSCAPE_X_OFFSET (126/2)
#define SKIN_LANDSCAPE_Y_OFFSET (82/2)

#define SKIN_PORTRAIT_X_OFFSET_S3 (38/2)
#define SKIN_PORTRAIT_Y_OFFSET_S3 (126/2)
#define SKIN_LANDSCAPE_X_OFFSET_S3 (126/2)
#define SKIN_LANDSCAPE_Y_OFFSET_S3 (35/2)

#define DEBUG 0

typedef struct { uint8_t r; uint8_t g; uint8_t b; uint8_t a;} pixel;

typedef struct {  
  guint          width;
  guint          height;
  guint          bytes_per_pixel; 
  pixel          pixel_data[];
} image_header;


extern image_header ttgo_board_skin;
extern image_header ttgos3_board_skin;

image_header *board_skin=&ttgos3_board_skin;


static uint16_t argbto565(uint32_t argb)
{
    uint8_t r = (argb >> 16) & 0xFF;
    uint8_t g = (argb >> 8) & 0xFF;
    uint8_t b = (argb )  & 0xFF;

    return (uint16_t)(
        ((r >> 3) << 11) |  // 5 bits red
        ((g >> 2) << 5)  |  // 6 bits green
        ( b >> 3)           // 5 bits blue
    );
}

static void draw_skin(ConsoleState *c) {
    volatile uint16_t *dest = c->data;
    for (int i = 0; i < board_skin->height; i++)
        for (int j = 0; j < board_skin->width; j++) {
            pixel p = board_skin->pixel_data[i * board_skin->width + j];
            p.r=(p.r*p.a)/255;
            p.g=(p.g*p.a)/255;
            p.b=(p.b*p.a)/255;
            uint32_t rgba= (p.a<<24) | (p.r<<16) | (p.g<<8) | p.b;
        //    if (p.a < 200)
        //        rgba=0xff000000;
            uint16_t rgb565=argbto565(rgba);
            if (c->width < c->height)  // portrait
                dest[i * board_skin->width + j] = rgb565;
            else
                dest[(board_skin->width - j - 1) * board_skin->height +i] = rgb565;
        }
    dpy_gfx_update_full(c->con);
}
static void bl_timer_cb(void *v) {
    uint64_t now=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ConsoleState *c=&console_state;
    if(c->time_on==0 || c->time_off==0) {
        c->backlight=256*c->lastlevel;
    } else {
        c->backlight=(c->time_on*256)/(c->time_on+c->time_off);
    }
    c->redraw=1;
    if(DEBUG)
        printf("bl:%d %d %d\n",(int)c->time_on,(int)c->time_off, c->backlight);
    c->time_on=0;
    c->time_off=0;
    timer_mod_ns(&console_state.backlight_timer, now + 100000000);
}
static void set_portrait(St7789vState *s) {
    ConsoleState *c=&console_state;
    if(s->iss3)
         board_skin=&ttgos3_board_skin;
    else
        board_skin=&ttgo_board_skin;
    qemu_console_resize(c->con, board_skin->width,
                        board_skin->height);
    DisplaySurface *surface=qemu_create_displaysurface_from(board_skin->width, board_skin->height,
                                                PIXMAN_r5g6b5,
                                                board_skin->width*2, NULL);
    dpy_gfx_replace_surface(c->con,surface);

    c->data=surface_data(qemu_console_surface(c->con));
    c->width = PANEL_HEIGHT;
    c->height = PANEL_WIDTH;
    c->x_offset = PORTRAIT_X_OFFSET;
    c->y_offset = PORTRAIT_Y_OFFSET;
    c->skin_x_offset = SKIN_PORTRAIT_X_OFFSET;
    c->skin_y_offset = SKIN_PORTRAIT_Y_OFFSET;
    if(s->iss3) {
        c->width = 170;
        c->height = 320;
        c->x_offset = PORTRAIT_X_OFFSET_S3;
        c->y_offset = PORTRAIT_Y_OFFSET_S3;
        c->skin_x_offset = SKIN_PORTRAIT_X_OFFSET_S3;
        c->skin_y_offset = SKIN_PORTRAIT_Y_OFFSET_S3;
    }
    c->skin_width = board_skin->width;
    draw_skin(c);
}

static void set_landscape(St7789vState *s) {
    ConsoleState *c=&console_state;
    if(s->iss3)
         board_skin=&ttgos3_board_skin;
    else
        board_skin=&ttgo_board_skin;
    qemu_console_resize(c->con, board_skin->height,
                    board_skin->width);
    DisplaySurface *surface=qemu_create_displaysurface_from(board_skin->height, board_skin->width,
                                                PIXMAN_r5g6b5,
                                                board_skin->height*2, NULL);
    dpy_gfx_replace_surface(c->con,surface);
    c->data=surface_data(qemu_console_surface(c->con));
    c->width = PANEL_WIDTH;
    c->height = PANEL_HEIGHT;
    c->x_offset = LANDSCAPE_X_OFFSET;
    c->y_offset = LANDSCAPE_Y_OFFSET;
    c->skin_x_offset = SKIN_LANDSCAPE_X_OFFSET;
    c->skin_y_offset = SKIN_LANDSCAPE_Y_OFFSET;
    if(s->iss3) {
        c->width = 320;
        c->height = 170;
        c->x_offset = LANDSCAPE_X_OFFSET_S3;
        c->y_offset = LANDSCAPE_Y_OFFSET_S3;
        c->skin_x_offset = SKIN_LANDSCAPE_X_OFFSET_S3;
        c->skin_y_offset = SKIN_LANDSCAPE_Y_OFFSET_S3;
    }
    c->skin_width = board_skin->height;
    draw_skin(c);
}

// transfer 32 bits at a time to speed things up.
// this needs the spi controller to do the same thing.
static uint32_t st7789v_transfer(SSIPeripheral *dev, uint32_t data)
{
    ConsoleState *c=&console_state;
    St7789vState *s = ST7789V(dev);
    
    uint8_t *bytes;
    if(c->cmd_mode) {
        c->current_command=data;
    } else {
        if(DEBUG && c->current_command!=0x2c)
            printf(" st7789 %x %x\n", c->current_command, data);
        switch (c->current_command) {
            case ST7789_MADCTL:
                if (data == 0 || data == 8) {  // portrait
                    set_portrait(s);
                } else {  // landscape
                    set_landscape(s);
                }
                break;
            case ST7789_CASET:
                bytes=(uint8_t *)&data;
                c->x_start = bytes[1]+bytes[0]*256;
                c->x_end = bytes[3]+bytes[2]*256;
                c->x = c->x_start;
                break;
            case ST7789_RASET:
                bytes=(uint8_t *)&data;
                c->y_start = bytes[1]+bytes[0]*256;
                c->y_end = bytes[3]+bytes[2]*256;
                c->y = c->y_start;
                break;
            case ST7789_RAMCTRL:
                if(data & 0x800)
                    c->little_endian = 1;
                else
                    c->little_endian = 0;
                if(data==0xe000) {
			c->little_endian = 1;
//			c->iss3=1;
//			set_landscape(c);
		}
		//printf("ST7789_RAMCTRL %x\n",data);
                break;
            case ST7789_RAMWR:
                for(int i=0;i<2;i++) {
                    uint16_t d16=data;
                    if(!c->little_endian) {
                        d16=(d16>>8) | (d16<<8);
                    }
                    uint32_t offset = c->y * 320 + c->x;
                    if(offset<320*320)
                        c->fb_data[offset] = d16;
                    c->x++;
                    if (c->x > c->x_end) {
                        c->x = c->x_start;
                        c->y++;
                    }
                    if ((c->y > c->y_end)) {
                        c->y = c->y_start;
                        c->x = c->x_start;
                        c->redraw=1;
                        break;
                    }
                    data=data>>16;
                }
                break;
            }
    }
    return 0;
}

/* Command/data input.  */
static void st7789v_cd(void *opaque, int n, int level)
{
    if(DEBUG)
        printf("st7789v_cd %d\n",level);
    ConsoleState *c=&console_state;
    c->cmd_mode = !level;
}

static void st7789v_backlight(void *opaque, int n, int level)
{
    int64_t now=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if(DEBUG)
        printf("st7789v_backlight %d\n",level);
    ConsoleState *c=&console_state;
    int v=level&1;
    int t=c->lastlevel>>1;
    c->lastlevel=level;
    if(t!=0) {
        now=c->lasttime+t;
    }
    if(now-c->lasttime<100000000) {
        if(v==0) {
            c->time_on+=now-c->lasttime;
        } else {
            c->time_off+=now-c->lasttime;
        }
    } else {
        c->backlight=256*v;
        c->redraw=1;
    }
    c->lasttime=now;
}

static void st7789_update_display(void *opaque) {
    ConsoleState *c = &console_state;
    if (!c->redraw) return;
    c->redraw = 0;

    for(int y=0;y<c->height;y++) {
        for(int x=0;x<c->width;x++) {
            uint16_t rgb565=c->fb_data[(y+c->y_offset)*320+x+c->x_offset];
            if(c->backlight<255) {
                // Extract native-bit counts
                uint32_t r = (rgb565 >> 11) & 0x1F;
                uint32_t g = (rgb565 >> 5) & 0x3F;
                uint32_t b = rgb565 & 0x1F;
                // Scale with rounding-to-nearest (add half LSB before shift)
                r = (r * c->backlight + 127) >> 8;
                g = (g * c->backlight + 127) >> 8;
                b = (b * c->backlight + 127) >> 8;
                rgb565=(uint16_t)((r << 11) | (g << 5) | b);
            }
            uint32_t index=(y+c->skin_y_offset)*c->skin_width+x+c->skin_x_offset;
            if(index<320*320)
              c->data[index]=rgb565;
        }
    }
    dpy_gfx_update(c->con, c->skin_x_offset, c->skin_y_offset, c->width, c->height);
}

static void st7789_invalidate_display(void *opaque) {
    ConsoleState *c = &console_state;
    c->redraw = 1;
}

static const GraphicHwOps st7789_ops = {
    .invalidate = st7789_invalidate_display,
    .gfx_update = st7789_update_display,
};

#define PW 1200
static void keyboard_event(DeviceState *dev, QemuConsole *src,
                                InputEvent *evt) {
    St7789vState *s = ST7789V(dev);
    ConsoleState *c = s->con;
    int qcode, up;
    InputMoveEvent *move;
    InputBtnEvent *btn;
    static int xpos = 0, ypos = 0;
    switch (evt->type) {
        case INPUT_EVENT_KIND_KEY:
            qcode = qemu_input_key_value_to_qcode(evt->u.key.data->key);
            up = 1 - evt->u.key.data->down;
            if (qcode == Q_KEY_CODE_1) {
                qemu_set_irq(s->button[0], up);
            }
            if (qcode == Q_KEY_CODE_2) {
                qemu_set_irq(s->button[1], up);
            }
            if (qcode == Q_KEY_CODE_R) {
                qemu_set_irq(s->reset, up);
            }
            int touch_codes[] = {Q_KEY_CODE_7, Q_KEY_CODE_8, Q_KEY_CODE_9,
                                 Q_KEY_CODE_0};
        //    int tsens[4] = {2, 3, 8, 9};
            if(s->iss3) {
         //   	tsens[0]=0;
         //   	tsens[1]=1;
         //   	tsens[2]=11;
         //   	tsens[3]=12;
            }
            for (int i = 0; i < 4; i++)
                if (qcode == touch_codes[i])
                    qemu_set_irq(s->touch_sensor[i],1000 * (1 - up));
//                    touch_sensor[tsens[i]] = 1000 * (1 - up);
            break;

        case INPUT_EVENT_KIND_ABS:
            move = evt->u.abs.data;
            if (move->axis == 0) xpos = move->value;
            if (move->axis == 1) ypos = move->value;
            break;
        case INPUT_EVENT_KIND_BTN:
            btn = evt->u.btn.data;
            int portrait = c->height > c->width;
            up = (1 - btn->down);
            if (up) {
                qemu_set_irq(s->button[0], up);
                qemu_set_irq(s->button[1], up);
                for (int i = 0; i < 4; i++) 
                    qemu_set_irq(s->touch_sensor[i],0);
                break;
            }
            if (portrait) {
                if(s->iss3) {
                    if (xpos > 24575 && xpos < 30561 && ypos > 30063 &&
                        ypos < 31382) {
                        qemu_set_irq(s->button[1], up);
                    }
                    if (xpos > 2205 && xpos < 8979 && ypos > 29932 &&
                        ypos < 31448) {
                        qemu_set_irq(s->button[0], up);
                    }
                    if (xpos > 157 && xpos < 2835 && ypos > 25580 &&
                        ypos < 27294 && up == 0)
                        qemu_set_irq(s->reset, 1);
                } else {
                    if (xpos > 24996 && xpos < 27962 && ypos > 28481 &&
                        ypos < 30347) {
                        qemu_set_irq(s->button[1], up);
                    }
                    if (xpos > 3071 && xpos < 6616 && ypos > 28481 &&
                        ypos < 30347) {
                        qemu_set_irq(s->button[0], up);
                    }
                    if (xpos > 30876 && xpos < 32530 && ypos > 23503 &&
                        ypos < 24713 && up == 0)
                        qemu_set_irq(s->reset, 1);
                }
                if(!s->iss3) {
                    int xs[] = {1417,  1417,  30010, 30010};
                    int ys[] = {12132, 13791, 12201, 13860};
                    for (int i = 0; i < 4; i++) {
                        if (xpos > (xs[i] - PW) && xpos < (xs[i] + PW) &&
                            ypos > (ys[i] - PW) && ypos < (ys[i] + PW))
                                qemu_set_irq(s->touch_sensor[i],1000);

                    }
                } else {
                    /*
                        int xs[] = {0,    0, 1417,  1417,  1417,
                                1417, 0, 30010, 30010, 30010};
                    int ys[] = {0,     0, 12132, 13791, 15312,
                                16694, 0, 18388, 12201, 13860};
                    for (int i = 2; i < 10; i++) {
                        if (i != 6) {
                           // if (xpos > (xs[i] - PW) && xpos < (xs[i] + PW) &&
                           //     ypos > (ys[i] - PW) && ypos < (ys[i] + PW))
                            //  touch_sensor[i] = 1000;
                        }
                    }
                        */
                }
                
            } else {
                if(s->iss3) {
                    if (xpos > 29932 && xpos < 31382 && ypos > 2047 &&
                        ypos < 7089) {
                        qemu_set_irq(s->button[1], up);
                    }
                    if (xpos > 29866 && xpos < 31580 && ypos > 24575 &&
                        ypos < 29931) {
                        qemu_set_irq(s->button[0], up);
                    }
                    if (xpos > 25382 && xpos < 30561 && ypos > 30063 &&
                        ypos < 31382 && up == 0)
                        qemu_set_irq(s->reset, 1);
                } else {
                    if (xpos > 28308 && xpos < 30451 && ypos > 5199 &&
                        ypos < 8428) {
                        qemu_set_irq(s->button[1], up);
                    }
                    if (xpos > 28308 && xpos < 30451 && ypos > 26386 &&
                        ypos < 29852) {
                        qemu_set_irq(s->button[0], up);
                    }
                    if (xpos > 23607 && xpos < 24540 && ypos > 551 && ypos < 1732 &&
                        up == 0)
                        qemu_set_irq(s->reset, 1);
                }
                
                int xs[] = {12166, 13618, 12166, 13791};
                int ys[] = {31743, 31743, 2993,  2993};
                for (int i = 0; i < 4; i++) {
                    if (xpos > (xs[i] - PW) && xpos < (xs[i] + PW) &&
                        ypos > (ys[i] - PW) && ypos < (ys[i] + PW))
                        qemu_set_irq(s->touch_sensor[i], 1000);
                }
                    
            }
            break;
        default:
            break;
    }
}

static QemuInputHandler keyboard_handler = {
    .name  = "TDisplay Keys",
    .mask  = INPUT_EVENT_MASK_KEY | INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = keyboard_event,
};

static Property st7789v_properties[] = {
    DEFINE_PROP_BOOL("s3_skin",St7789vState,iss3,false),
    DEFINE_PROP_END_OF_LIST(),
};


static void st7789v_realize(SSIPeripheral *d, Error **errp) {
    
    St7789vState *s = ST7789V(d);
    DeviceState *dev = DEVICE(s);

    qemu_input_handler_register(dev, &keyboard_handler);
    qdev_init_gpio_in_named(dev, st7789v_cd, "cmd",1);
    qdev_init_gpio_in_named(dev, st7789v_backlight, "backlight", 1);
    qdev_init_gpio_out_named(dev,s->button,"buttons",2);
    qdev_init_gpio_out_named(dev,&s->reset,"reset",1);
    qdev_init_gpio_out_named(dev,s->touch_sensor,"touch_sensor",4);

    if (console_state.con == 0) {
        ConsoleState *c=&console_state;
        s->con=c;
        c->con = graphic_console_init(dev, 0, &st7789_ops, s);

        int64_t now=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        c->lastlevel=0;
        c->lasttime=now;
        c->cmd_mode=1;
        c->fb_data=malloc(320*320*2);
        timer_init_ns(&c->backlight_timer,QEMU_CLOCK_VIRTUAL, bl_timer_cb,0);
        timer_mod_ns(&c->backlight_timer, now + 100000000);
        set_landscape(s);
    } else {
        s->con = &console_state;
    }
}

static void st7789v_reset(DeviceState *dev) {
    if (console_state.con != 0) {
        int64_t now=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        console_state.lasttime=now;
    }
}

static void st7789v_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    device_class_set_legacy_reset(dc,st7789v_reset);
    k->realize = st7789v_realize;
    k->transfer = st7789v_transfer;
    k->cs_polarity = SSI_CS_NONE;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    device_class_set_props(dc, st7789v_properties);
}

static const TypeInfo st7789v_info = {
    .name = TYPE_ST7789V,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(St7789vState),
    .class_init = st7789v_class_init};

static void st7789v_register_types(void) {
    type_register_static(&st7789v_info);
}

type_init(st7789v_register_types)
