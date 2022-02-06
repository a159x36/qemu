/*
 * QEMU Servo device
 *
 * Copyright (C) 2023 Martin Johnson <M.J.Johnson@massey.ac.nz>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_SERVO_H
#define HW_MISC_SERVO_H

#include "qom/object.h"
#include "hw/qdev-core.h"

#define TYPE_SERVO "servo"


struct ServoState {
    /* Private */
    DeviceState parent_obj;
    /* Public */
    int64_t last_time;
    int last_state;
    int32_t angle;
    int redraw;
    uint32_t *data;
    QemuConsole *con;
    char *description;
    qemu_irq irq;

};
typedef struct ServoState ServoState;
DECLARE_INSTANCE_CHECKER(ServoState, SERVO, TYPE_SERVO)

/**
 * SERVO_create_simple: Create and realize a SERVO device
 * @parentobj: the parent object
 * @description: description of the SERVO (optional)
 *
 * Create the device state structure, initialize it, and
 * drop the reference to it (the device is realized).
 *
 * Returns: The newly allocated and instantiated SERVO object.
 */
ServoState *servo_create_simple(Object *parentobj,
                            const char *description);

#endif /* HW_MISC_SERVO_H */
