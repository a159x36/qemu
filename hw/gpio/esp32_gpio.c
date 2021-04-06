/*
 * ESP32 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "ui/console.h" 
#include "hw/hw.h"
#include "ui/input.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"
#include "sysemu/runstate.h"


static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;
    switch (addr) {
    case 4:
       r= s->gpio_out;
       break;
    case A_GPIO_STRAP:
        r = s->strap_mode;
        break;
    case 0x3C: // in_low
        r = s->gpio_in;
        break;
    case 0x40: // in_high
        r = s->gpio_in1;
        break;
    case 0x44:
        r = s->gpio_status;
        break;
    case 0x50:
        r = s->gpio_status1;
        break;
    case 0x60:
        r = s->gpio_acpu_int;
        break;
    case 0x68:
        r = s->gpio_pcpu_int;
        break;
    case 0x74:
        r = s->gpio_acpu_int1;
        break; 
    case 0x7c:
        r = s->gpio_pcpu_int1;
        break; 
    default:
        break;
    }
    if(addr>=0x88 && addr<0x130) {
        int n=(addr-0x88)/4;
        r=s->gpio_pin[n];
    }
//   printf("read gpio %lx = %lx\n",addr,r);
    return r;
}

static int get_triggering(int int_type, int oldval, int val) {
        switch(int_type) {
            case 1:
                return (val>oldval);
            case 2:
                return (val<oldval);
            case 3:
                return (val!=oldval);
            case 4:
                return (val==0);
            case 5:
                return (val==1);
        }
        return 0;
}
static void set_gpio(void *opaque, int n, int val) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    if(n<32) {
        int oldval=(s->gpio_in>>n) & 1;
        int int_type=(s->gpio_pin[n]>>7) & 7;
      //  printf("set_gpio %d %d %x %x %x \n",n,val,s->gpio_status,(s->gpio_pin[n]), int_type);
        s->gpio_in &= ~(1<<n);
        s->gpio_in |= (val<<n);
        int irq=get_triggering(int_type, oldval, val);
        // says bit 16 in the ref manual, is that wrong?
        if(irq && (s->gpio_pin[n] & (1<<15))) { // pro cpu int enable
            qemu_set_irq(s->irq,1);
            s->gpio_pcpu_int |= (1 << n);
        }
        if(irq && (s->gpio_pin[n] & (1<<13))) { // app cpu int enable
            qemu_set_irq(s->irq,1);
            s->gpio_acpu_int |= (1 << n);
        }
    } else {
        int n1=n-32;
        int oldval=(s->gpio_in1>>n1) & 1;
        int int_type=(s->gpio_pin[n]>>7) & 7;
     //   printf("set_gpio1 %d %d %x %x %x %x %x\n",n,val,s->gpio_status1,(s->gpio_pin[n]), int_type,s->gpio_in1,s->gpio_in );
        s->gpio_in1 &= ~(1<<n1);
        s->gpio_in1 |= (val<<n1);
        int irq=get_triggering(int_type, oldval, val);
        // says bit 16 in the ref manual, is that wrong?
        if(irq && (s->gpio_pin[n] & (1<<15))) {
            qemu_set_irq(s->irq,1);
            s->gpio_pcpu_int1 |= (1 << n1);
        }
        if(irq && (s->gpio_pin[n] & (1<<13))) { // app cpu int enable
            qemu_set_irq(s->irq,1);
            s->gpio_acpu_int1 |= (1 << n1);
        }
    }
}
extern const struct {int width;int height;} ttgo_board_skin;

static void esp32_gpio_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
  Esp32GpioState *s = ESP32_GPIO(opaque);
  int clearirq;
  uint32_t oldvalue;
  oldvalue=s->gpio_out;
  //printf("gpio_write %lx %lx\n",addr,value);
  switch (addr) {
    case 4:
    s->gpio_out = value;
    break;
    case 8:
    s->gpio_out |= value;
    break;
    case 12:
    s->gpio_out &= ~value;
    break;
    case 0x44:
    s->gpio_status = value;
    break;
    case 0x48:
    s->gpio_status |= value;
    break;
    case 0x4c:
        clearirq=1;
        for(int i=0;i<32;i++) {
            if((1<<i) & value) {
                int int_type=(s->gpio_pin[i]>>7) & 7;
                if((int_type==4 && !(s->gpio_in & (1<<i))) ||
                (int_type==5 && (s->gpio_in & (1<<i))))
                    clearirq=0;
            }
        }
        if(clearirq) {
            s->gpio_status &= ~value;
            s->gpio_pcpu_int &= ~value;
            s->gpio_acpu_int &= ~value;
            qemu_set_irq(s->irq,0);
        }
    break;
    case 0x50:
    s->gpio_status1 = value;
    break;
    case 0x54:
    s->gpio_status1 |= value;
    break;
    case 0x58:
        clearirq=1;
        for(int i=0;i<32;i++) {
            if((1<<i) & value) {
                int int_type=(s->gpio_pin[i+32]>>7) & 7;
                if((int_type==4 && !(s->gpio_in1 & (1<<i))) ||
                (int_type==5 && (s->gpio_in1 & (1<<i))))
                    clearirq=0;
            }
        }
        if(clearirq) {
            s->gpio_status1 &= ~value;
            s->gpio_pcpu_int1 &= ~value;
            s->gpio_acpu_int1 &= ~value;
            qemu_set_irq(s->irq,0);
        }
    break;


  }
  if(addr>=0x88 && addr<0x130) {
      int n=(addr-0x88)/4;
      s->gpio_pin[n]=value;
    }
#define MAG 1
#define RED 2
    if((s->gpio_out & (1<<4)) != (oldvalue & 1<<4)) {
        int bl=(s->gpio_out>>4)&1;
        QemuConsole *con = qemu_console_lookup_by_index(0);
        DisplaySurface *surface=qemu_console_surface(con);
        int portrait=surface_height(surface)>surface_width(surface);
        volatile unsigned *dest = (unsigned *)surface_data(surface);
        uint32_t px=bl?(64<<16)|(64<<8)|(64):0;
        if(portrait) {
            for(int y=0;y<240*MAG;y++)
                for(int x=0;x<135*MAG;x++)
                    dest[(y+126/RED)*ttgo_board_skin.width/RED+x+62/RED]=px^(rand()&0x0f0f0f);
        } else {
            for(int y=0;y<135*MAG;y++)
                for(int x=0;x<240*MAG;x++)
                    dest[(y+82/RED)*ttgo_board_skin.height/RED+x+126/RED]=px^(rand()&0x0f0f0f);
        }
        dpy_gfx_update(con, 0, 0, surface_width(surface), surface_height(surface));
    }
}

static const MemoryRegionOps uart_ops = {
    .read =  esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset(DeviceState *dev)
{
}
extern int touch_sensor[10];
#define PW 1200
static void gpio_keyboard_event(DeviceState *dev, QemuConsole *src,
                               InputEvent *evt)
{
    Esp32GpioState *s = ESP32_GPIO(dev);
    int qcode, up;
    InputMoveEvent *move;
    InputBtnEvent *btn;
    static int xpos=0,ypos=0;
    //printf("event type %d\n",evt->type);
    switch (evt->type) {
        case INPUT_EVENT_KIND_KEY:
            qcode=qemu_input_key_value_to_qcode(evt->u.key.data->key);
            up=1-evt->u.key.data->down;

 //           printf("keyboard_event:%d %d\n",qcode, evt->u.key.data->down);
            if(qcode==Q_KEY_CODE_1) {
                set_gpio(s,0,up);
            }
            if(qcode==Q_KEY_CODE_2) {
                set_gpio(s,35,up);
            }
            if(qcode==Q_KEY_CODE_R) {
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            }
            int touch_codes[]={Q_KEY_CODE_7,Q_KEY_CODE_8,Q_KEY_CODE_9,Q_KEY_CODE_0};
            int tsens[]={2,3,8,9};
            for(int i=0;i<4;i++)
	            if(qcode==touch_codes[i])
			touch_sensor[tsens[i]]=1000*(1-up);
        break;
        
        case INPUT_EVENT_KIND_ABS:
            move=evt->u.abs.data;

        //  printf("move %ld %d\n", move->value, move->axis);
            if(move->axis==0) xpos=move->value;
            if(move->axis==1) ypos=move->value;
            break;
        case INPUT_EVENT_KIND_BTN:
            btn = evt->u.btn.data;
            
//            printf("btn %d %d %d %d\n",xpos, ypos,  btn->button, btn->down);
            QemuConsole *con = qemu_console_lookup_by_index(0);
            DisplaySurface *surface=qemu_console_surface(con);
            int portrait=surface_height(surface)>surface_width(surface);
            up=(1-btn->down);
            if(up) {
                if(!(s->gpio_in&1))
                    set_gpio(s,0,up);
                if(!(s->gpio_in1&8))
                    set_gpio(s,35,up);
		for(int i=2;i<10;i++)
		    touch_sensor[i]=0;
                break;
            }
            if(portrait) {
                if(xpos>24996 && xpos<27962 && ypos>28481 && ypos<30347) {
                    set_gpio(s,35,up);
                } 
                if(xpos>3071 && xpos<6616 && ypos>28481 && ypos<30347) {
                    set_gpio(s,0,up);
                }
                if(xpos>30876 && xpos<32530 && ypos>23503 && ypos<24713 && up==0)
                    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
		int xs[]={0,0,1417,1417,1417,1417,0,30010,30010,30010};
		int ys[]={0,0,12132,13791,15312,16694,0,18388,13860,12201};
		for(int i=2;i<10;i++) {
			if(i!=6) {
				if(xpos>(xs[i]-PW) && xpos<(xs[i]+PW) &&
				ypos>(ys[i]-PW) && ypos<(ys[i]+PW))
					touch_sensor[i]=1000;
			}
		}
            } else {
                if(xpos>28308 && xpos<30451 && ypos>5199 && ypos<8428) {
                    set_gpio(s,35,up);
                } 
                if(xpos>28308 && xpos<30451 && ypos>26386 && ypos<29852) {
                    set_gpio(s,0,up);
                }
                if(xpos>23607 && xpos<24540 && ypos>551 && ypos<1732 && up==0)
                    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
		int xs[]={0,0,12166,13618,15277,16798,0,18388,13791,12166};
                int ys[]={0,0,31743,31743,31743,31743,0,2993,2993,2993};
                for(int i=2;i<10;i++) {
                        if(i!=6) { 
                                if(xpos>(xs[i]-PW) && xpos<(xs[i]+PW) &&
                                ypos>(ys[i]-PW) && ypos<(ys[i]+PW))
                                        touch_sensor[i]=1000;
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

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
    qemu_input_handler_register(dev, &gpio_keyboard_handler);
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(s), &s->irq, SYSBUS_DEVICE_GPIO_IRQ, 1);
    s->gpio_in=0x1;
    s->gpio_in1=0x8;

}

static Property esp32_gpio_properties[] = {
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, ESP32_STRAP_MODE_FLASH_BOOT),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_gpio_reset;
    dc->realize = esp32_gpio_realize;
    device_class_set_props(dc, esp32_gpio_properties);
    
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
