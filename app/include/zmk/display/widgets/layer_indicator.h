/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <lvgl.h>
#include <kernel.h>

struct zmk_widget_layer_indicator {
    sys_snode_t node;
    lv_obj_t *obj;
};

int zmk_widget_layer_indicator_init(struct zmk_widget_layer_indicator *widget, lv_obj_t *parent);
lv_obj_t *zmk_widget_layer_indicator_obj(struct zmk_widget_layer_indicator *widget);