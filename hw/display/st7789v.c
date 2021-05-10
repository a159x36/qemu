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
#include "ui/input.h"
#include "hw/irq.h"
#include "hw/display/st7789v.h"
#include "sysemu/runstate.h"

#define PANEL_WIDTH 240
#define PANEL_HEIGHT 135

typedef struct ConsoleState {
    QemuConsole *con;
    uint32_t redraw;
    int width,height,x_offset,y_offset;
    int32_t x_start;
    int32_t x_end;
    int32_t y_start;
    int32_t y_end;
    int32_t x;
    int32_t y;
    unsigned short frame_buffer[PANEL_WIDTH*PANEL_HEIGHT];
} ConsoleState;

static ConsoleState console_state;

struct  St7789vState {
    SSISlave ssidev;
    struct ConsoleState *con;
    uint32_t current_command;
    uint32_t redraw;
    int cmd_mode;
    int little_endian;
    int backlight;
    qemu_irq button[2];
};

#define TYPE_ST7789V "st7789v"
OBJECT_DECLARE_SIMPLE_TYPE(St7789vState, ST7789V)

#define PORTRAIT_X_OFFSET 52
#define PORTRAIT_Y_OFFSET 40
#define LANDSCAPE_X_OFFSET 40
#define LANDSCAPE_Y_OFFSET 53

#define SKIN_PORTRAIT_X_OFFSET (62/2)
#define SKIN_PORTRAIT_Y_OFFSET (126/2)
#define SKIN_LANDSCAPE_X_OFFSET (126/2)
#define SKIN_LANDSCAPE_Y_OFFSET (82/2)

//#define MAGNIFY 1
//#define REDUCE 1
//#define OREDUCE 2

#define st7789v_REG_SIZE 0x1000

extern const struct {
  guint          width;
  guint          height;
  guint          bytes_per_pixel; 
  guint8         pixel_data[];
} ttgo_board_skin;

static void draw_skin(St7789vState *s) {
    ConsoleState *c = s->con;
    DisplaySurface *surface = qemu_console_surface(c->con);
    volatile unsigned *dest = (unsigned *)surface_data(surface);
    for (int i = 0; i < ttgo_board_skin.height; i++)
        for (int j = 0; j < ttgo_board_skin.width; j++) {
            int red = 0, green = 0, blue = 0, trans = 0;
            int index = (i * ttgo_board_skin.width + j) * 4;
            red = ttgo_board_skin.pixel_data[index];
            green = ttgo_board_skin.pixel_data[index + 1];
            blue = ttgo_board_skin.pixel_data[index + 2];
            trans = ttgo_board_skin.pixel_data[index + 3];
            if (trans < 200) {
                red = green = blue = 0;
                trans = 255;
            }
            if (c->width < c->height)  // portrait
                dest[i * ttgo_board_skin.width + j] =
                    (trans << 24) | (red << 16) | (green << 8) | blue;
            else
                dest[(ttgo_board_skin.width - j - 1) * ttgo_board_skin.height +i] = 
                    (trans << 24) | (red << 16) | (green << 8) | blue;
        }
}

// trnsfer 32 bits at a time to speed things up.
// this needs the spi controller to 
static uint32_t st7789v_transfer(SSISlave *dev, uint32_t data)
{
    St7789vState *s = ST7789V(dev);
    ConsoleState *c=s->con;
    if(s->cmd_mode) {
        s->current_command=data;
    } else {
        uint8_t *bytes=(uint8_t *)&data;
        switch (s->current_command) {
            case ST7789_MADCTL:
                if (data == 0 || data == 8) {  // portrait
                    qemu_console_resize(c->con, ttgo_board_skin.width,
                                        ttgo_board_skin.height);
                    c->width = PANEL_HEIGHT;
                    c->height = PANEL_WIDTH;
                    c->x_offset = PORTRAIT_X_OFFSET;
                    c->y_offset = PORTRAIT_Y_OFFSET;
                } else {  // landscape
                    qemu_console_resize(c->con, ttgo_board_skin.height,
                                        ttgo_board_skin.width);
                    c->width = PANEL_WIDTH;
                    c->height = PANEL_HEIGHT;
                    c->x_offset = LANDSCAPE_X_OFFSET;
                    c->y_offset = LANDSCAPE_Y_OFFSET;
                }
                draw_skin(s);
                break;
            case ST7789_CASET:
                c->x_start = bytes[1]+bytes[0]*256;
                c->x_end = bytes[3]+bytes[2]*256;
                c->x = c->x_start;
                break;
            case ST7789_RASET:
                c->y_start = bytes[1]+bytes[0]*256;
                c->y_end = bytes[3]+bytes[2]*256;
                c->y = c->y_start;
                break;
            case ST7789_RAMCTRL:
                s->little_endian = 1;
                break;
            case ST7789_RAMWR:
                for(int i=0;i<2;i++) {
                    uint16_t d16=data;
                    if(!s->little_endian) {
                        d16=(d16>>8) | (d16<<8);
                    }
                    uint16_t offset = (c->y - c->y_offset) * c->width + 
                        c->x - c->x_offset;
                    if (offset < (PANEL_WIDTH*PANEL_HEIGHT)) c->frame_buffer[offset] = d16;
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
    St7789vState *s = (St7789vState *)opaque;
    s->cmd_mode = !level;
}

static void st7789v_backlight(void *opaque, int n, int level)
{
    St7789vState *s = (St7789vState *)opaque;
    ConsoleState *c=s->con;
    if(s->backlight != level) {
        DisplaySurface *surface=qemu_console_surface(c->con);
        int portrait=surface_height(surface)>surface_width(surface);
        volatile unsigned *dest = (unsigned *)surface_data(surface);
        uint32_t px=level?(64<<16)|(64<<8)|(64):0;
        if(portrait) {
            for(int y=0;y<PANEL_WIDTH;y++)
                for(int x=0;x<PANEL_HEIGHT;x++)
                    dest[(y+SKIN_PORTRAIT_Y_OFFSET)*ttgo_board_skin.width+x+SKIN_PORTRAIT_X_OFFSET]=px^(rand()&0x0f0f0f);
        } else {
            for(int y=0;y<PANEL_HEIGHT;y++)
                for(int x=0;x<PANEL_WIDTH;x++)
                    dest[(y+SKIN_LANDSCAPE_Y_OFFSET)*ttgo_board_skin.height+x+SKIN_LANDSCAPE_X_OFFSET]=px^(rand()&0x0f0f0f);
        }
        dpy_gfx_update(c->con, 0, 0, surface_width(surface), surface_height(surface));
    }
    s->backlight = level;
}

static void st7789_update_display(void *opaque) {
    St7789vState *s = (St7789vState *)opaque;
    ConsoleState *c = s->con;
    if (!c->redraw) return;
    DisplaySurface *surface = qemu_console_surface(c->con);
    volatile unsigned *dest = (unsigned *)surface_data(surface);

    for (int i = 0; i < c->width; i++) {
        for (int j = 0; j < c->height; j++) {
            unsigned fbv = c->frame_buffer[j * c->width + i];
            int red = (fbv & 0xf800) >> 8;
            int green = (fbv & 0x7e0) >> 3;
            int blue = (fbv & 0x1f) << 3;
            if (!(s->backlight)) {
                red = red >> 3;
                green = green >> 3;
                blue = blue >> 3;
            }
            if (c->width > c->height) {  // landscape
                int x = i + SKIN_LANDSCAPE_X_OFFSET;
                int y = j + SKIN_LANDSCAPE_Y_OFFSET;
                *(dest + y * ttgo_board_skin.height + x) =
                    (red << 16) | (green << 8) | blue;
            } else {
                int x = i + SKIN_PORTRAIT_X_OFFSET;
                int y = j + SKIN_PORTRAIT_Y_OFFSET;
                *(dest + y * ttgo_board_skin.width + x) =
                    (red << 16) | (green << 8) | blue;
            }
        }
    }
    c->redraw = 0;

    if (c->width > c->height) // landscape
        dpy_gfx_update(c->con, SKIN_LANDSCAPE_X_OFFSET, SKIN_LANDSCAPE_Y_OFFSET, c->width, c->height);
    else
        dpy_gfx_update(c->con, SKIN_PORTRAIT_X_OFFSET, SKIN_PORTRAIT_Y_OFFSET, c->width, c->height);
}

static void st7789_invalidate_display(void *opaque) {
    St7789vState *s = (St7789vState *)opaque;
    s->con->redraw = 1;
}

static const GraphicHwOps st7789_ops = {
    .invalidate = st7789_invalidate_display,
    .gfx_update = st7789_update_display,
};

extern int touch_sensor[10];
#define PW 1200
static void gpio_keyboard_event(DeviceState *dev, QemuConsole *src,
                                InputEvent *evt) {
    St7789vState *s = ST7789V(dev);
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
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            }
            int touch_codes[] = {Q_KEY_CODE_7, Q_KEY_CODE_8, Q_KEY_CODE_9,
                                 Q_KEY_CODE_0};
            int tsens[] = {2, 3, 8, 9};
            for (int i = 0; i < 4; i++)
                if (qcode == touch_codes[i])
                    touch_sensor[tsens[i]] = 1000 * (1 - up);
            break;

        case INPUT_EVENT_KIND_ABS:
            move = evt->u.abs.data;
            if (move->axis == 0) xpos = move->value;
            if (move->axis == 1) ypos = move->value;
            break;
        case INPUT_EVENT_KIND_BTN:
            btn = evt->u.btn.data;
            DisplaySurface *surface = qemu_console_surface(s->con->con);
            int portrait = surface_height(surface) > surface_width(surface);
            up = (1 - btn->down);
            if (up) {
                qemu_set_irq(s->button[0], up);
                qemu_set_irq(s->button[1], up);
                for (int i = 2; i < 10; i++) touch_sensor[i] = 0;
                break;
            }
            if (portrait) {
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
                    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
                int xs[] = {0,    0, 1417,  1417,  1417,
                            1417, 0, 30010, 30010, 30010};
                int ys[] = {0,     0, 12132, 13791, 15312,
                            16694, 0, 18388, 12201, 13860};
                for (int i = 2; i < 10; i++) {
                    if (i != 6) {
                        if (xpos > (xs[i] - PW) && xpos < (xs[i] + PW) &&
                            ypos > (ys[i] - PW) && ypos < (ys[i] + PW))
                            touch_sensor[i] = 1000;
                    }
                }
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
                    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
                int xs[] = {0,     0, 12166, 13618, 15277,
                            16798, 0, 18388, 12166, 13791};
                int ys[] = {0,     0, 31743, 31743, 31743,
                            31743, 0, 2993,  2993,  2993};
                for (int i = 2; i < 10; i++) {
                    if (i != 6) {
                        if (xpos > (xs[i] - PW) && xpos < (xs[i] + PW) &&
                            ypos > (ys[i] - PW) && ypos < (ys[i] + PW))
                            touch_sensor[i] = 1000;
                    }
                }
            }
            break;
        default:
            break;
    }
}

static QemuInputHandler gpio_keyboard_handler = {
    .name  = "GPIO Keys",
    .mask  = INPUT_EVENT_MASK_KEY | INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = gpio_keyboard_event,
};

static void st7789v_realize(SSISlave *d, Error **errp) {
    
    St7789vState *s = ST7789V(d);
    DeviceState *dev = DEVICE(s);

    qemu_input_handler_register(dev, &gpio_keyboard_handler);
    qdev_init_gpio_in_named(dev, st7789v_cd, "cmd",1);
    qdev_init_gpio_in_named(dev, st7789v_backlight, "backlight", 1);
    qdev_init_gpio_out_named(dev,s->button,"buttons",2);

    if (console_state.con == 0) {
        s->con = &console_state;
        console_state.con = graphic_console_init(dev, 0, &st7789_ops, s);
        console_state.width = PANEL_WIDTH;
        console_state.height = PANEL_HEIGHT;
        console_state.x_offset = LANDSCAPE_X_OFFSET;
        console_state.y_offset = LANDSCAPE_Y_OFFSET;
        qemu_console_resize(console_state.con, ttgo_board_skin.height , ttgo_board_skin.width);
        draw_skin(s);
    } else {
        s->con = &console_state;
    }
}

static void st7789v_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);

    k->realize = st7789v_realize;
    k->transfer = st7789v_transfer;
    k->cs_polarity = SSI_CS_NONE;
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
