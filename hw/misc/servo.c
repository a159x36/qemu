/*
 * QEMU single Servo device
 *
 * Copyright (C) 2023 Martin Johnson <M.J.Johnson@massey.ac.nz>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"

#include "migration/vmstate.h"
#include "qemu/timer.h"
#include "ui/console.h"
#include "hw/qdev-properties.h"
#include "hw/misc/servo.h"
#include "trace.h"
#include <math.h>

#include "servo_skin.c"
#include "servoarm_skin.c"

#define SERVO_ANGLE_MAX   180
#define SERVO_ANGLE_MIN   0

#define SERVO_STEP 5

static void servo_set_state_gpio_handler(void *opaque, int line, int new_state)
{
    ServoState *s = SERVO(opaque);
    assert(line == 0);
	int v=new_state&1;
//	printf("servo %x %d\n",new_state,v);
    if(new_state!=s->last_state) {
        int64_t now=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if(v==0) {
            int angle=(((now-s->last_time)-1000000)*90)/1000000;
            int t=(s->last_state)>>1;
            if(t!=0) angle=((t-1000)*90)/1000;
//            printf("angle %d\n",angle);
            if (angle > SERVO_ANGLE_MAX) {
                angle = SERVO_ANGLE_MAX;
            }
            if (angle < SERVO_ANGLE_MIN) {
                angle = SERVO_ANGLE_MIN;
            }
            if(angle!=s->angle) {
                if(s->angle<angle-SERVO_STEP) {
    		        s->angle+=SERVO_STEP;
                } else {
    		        if(s->angle>angle+SERVO_STEP) {
                        s->angle-=SERVO_STEP;
                    } else {
                        s->angle=angle;
                    }
    	        }
//    		    printf("servo %ld %d %d\n",now-s->last_time,angle,s->angle);
    		    s->redraw=1;
    		}
    	}
    	s->last_state=new_state;
    	s->last_time=now;
    }
}

static void servo_reset(DeviceState *dev)
{
    ServoState *s = SERVO(dev);
    s->angle=0;
    s->redraw=1;
    s->last_state=0;
    s->last_time=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void servo_update_display(void *opaque) {
    ServoState *s = SERVO(opaque);
    if (!s->redraw) return;
    s->redraw = 0;
    float theta=(s->angle*3.1415926f)/180.0f;
    float ct=cos(theta);
    float st=sin(theta);
    const float ci=125-64;
    const float cj=104.5;
    for(int i=0;i<128;i++) {
        for(int j=0;j<256;j++) {
        	int jj=((j-cj)*ct-(i-ci)*st)+cj+0.5;
        	int ii=((j-cj)*st+(i-ci)*ct)+ci+0.5;
        	uint8_t *skin=(uint8_t *)servo_skin.pixel_data+(j+i*256)*4;
        	uint8_t *arm_skin=(uint8_t *)servo_arm_skin.pixel_data+(jj+ii*256)*4;
        	if(jj>0 && ii>0 && jj<256 && ii<128 && arm_skin[3]!=0) {
        		float p=arm_skin[3]/255.0;
        		int rr=p*arm_skin[0]+(1-p)*skin[0];
            	int gg=p*arm_skin[1]+(1-p)*skin[1];
            	int bb=p*arm_skin[2]+(1-p)*skin[2];
            	int pixel=(rr<<16) | (gg<<8) | bb | 0xff000000;
	            s->data[j+i*256]=pixel;
        	} else
        		s->data[j+i*256]=(skin[0]<<16) | (skin[1]<<8) | skin[2] | 0xff000000;
        }
    }
    dpy_gfx_update(s->con, 0, 0, 256, 128);
}


static void servo_invalidate_display(void *opaque) {
    ServoState *s = SERVO(opaque);
    s->redraw = 1;
}


static const GraphicHwOps servo_ops = {
    .invalidate = servo_invalidate_display,
    .gfx_update = servo_update_display,
};

static void servo_realize(DeviceState *dev, Error **errp)
{
    ServoState *s = SERVO(dev);
    if (s->description == NULL) {
        s->description = g_strdup("n/a");
    }
    qdev_init_gpio_in(DEVICE(s), servo_set_state_gpio_handler, 1);
    s->con=graphic_console_init(dev, 0, &servo_ops, s);
    qemu_console_resize(s->con,256, 128);
    s->data=surface_data(qemu_console_surface(s->con));
    s->redraw=1;
}

static Property servo_properties[] = {
    DEFINE_PROP_STRING("description", ServoState, description),
    DEFINE_PROP_END_OF_LIST(),
};

static void servo_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Servo Motor";
    dc->reset = servo_reset;
    dc->realize = servo_realize;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    device_class_set_props(dc, servo_properties);
}

static const TypeInfo Servo_info = {
    .name = TYPE_SERVO,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(ServoState),
    .class_init = servo_class_init
};

static void Servo_register_types(void)
{
    type_register_static(&Servo_info);
}

type_init(Servo_register_types)

ServoState *servo_create_simple(Object *parentobj,
                            const char *description)
{
    g_autofree char *name = NULL;
    DeviceState *dev;

    dev = qdev_new(TYPE_SERVO);
    if (!description) {
        static unsigned undescribed_Servo_id;
        name = g_strdup_printf("undescribed-Servo-#%u", undescribed_Servo_id++);
    } else {
        qdev_prop_set_string(dev, "description", description);
        name = g_ascii_strdown(description, -1);
        name = g_strdelimit(name, " #", '-');
    }
    object_property_add_child(parentobj, name, OBJECT(dev));
    qdev_realize_and_unref(dev, NULL, &error_fatal);

    return SERVO(dev);
}
