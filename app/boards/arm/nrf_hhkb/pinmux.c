/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/sys_io.h>
#include <devicetree.h>

static int pinmux_nrf_hhkb_init(const struct device *port) {
    ARG_UNUSED(port);
    return 0;
}

SYS_INIT(pinmux_nrf_hhkb_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
