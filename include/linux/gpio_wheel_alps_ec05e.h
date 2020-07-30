/*
 * GPIO wheel APLS EC05E device driver module.
 *
 * Copyright (c) 2016 Pantech Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _GPIO_WHEEL_ALPS_EC05E_H
#define _GPIO_WHEEL_ALPS_EC05E_H

#define GPIO_WHEEL_ALPS_EC05E_DEV_NAME "gpio-wheel-alps-ec05e"

struct device;

struct gpio_wheel_alps_ec05e_terminal {
    const char *desc;
    int gpio;
    int wakeup;
};

struct gpio_wheel_alps_ec05e_platform_data {
    struct gpio_wheel_alps_ec05e_terminal *terminals;
    int nTerminals;
    const char *name;   /* input device name */
    int (*enable)(struct device *dev);
    void (*disable)(struct device *dev);
};

#endif /*  _GPIO_WHEEL_ALPS_EC05E_H  */
