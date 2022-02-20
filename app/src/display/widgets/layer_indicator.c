/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <kernel.h>
#include <logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/display.h>
#include <zmk/display/widgets/layer_indicator.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>

static sys_slist_t widgets = SYS_SLIST_STATIC_INIT(&widgets);

struct layer_indicator_state {
    uint8_t index;
    const char *label;
};

static void set_layer_symbol(lv_obj_t *label, struct layer_indicator_state state) {
    if (state.label == NULL) {
        char text[8] = {};

        sprintf(text, LV_SYMBOL_KEYBOARD " %i", state.index);

        lv_label_set_text(label, text);
    } else {
        char text[14] = {};

        snprintf(text, 14, LV_SYMBOL_KEYBOARD " %s", state.label);

        lv_label_set_text(label, text);
    }
}

static void layer_indicator_update_cb(struct layer_indicator_state state) {
    struct zmk_widget_layer_indicator *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { set_layer_symbol(widget->obj, state); }
}

static struct layer_indicator_state layer_indicator_get_state(const zmk_event_t *eh) {
    uint8_t index = zmk_keymap_highest_layer_active();
    return (struct layer_indicator_state){.index = index, .label = zmk_keymap_layer_label(index)};
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_layer_indicator, struct layer_indicator_state, layer_indicator_update_cb,
                            layer_indicator_get_state)

ZMK_SUBSCRIPTION(widget_layer_indicator, zmk_layer_state_changed);

int zmk_widget_layer_indicator_init(struct zmk_widget_layer_indicator *widget, lv_obj_t *parent) {
    widget->obj = lv_label_create(parent, NULL);

    lv_obj_set_size(widget->obj, 40, 15);

    sys_slist_append(&widgets, &widget->node);

    widget_layer_indicator_init();
    return 0;
}

lv_obj_t *zmk_widget_layer_indicator_obj(struct zmk_widget_layer_indicator *widget) {
    return widget->obj;
}