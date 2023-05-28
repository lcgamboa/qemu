/*
 * ESP32C3 UART emulation
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This implementation overrides the ESP32 UARt one, check it out first.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/char/esp32c3_uart.h"
#include "hw/misc/esp32c3_rtc_cntl.h"

static uint64_t esp32c3_uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32C3UARTClass *class = ESP32C3_UART_GET_CLASS(opaque);
    /* Nothing special to do at the moment here, but it is possible to override the
     * parent's read function behavior. */

    return class->parent_uart_read(opaque, addr, size);
}

static void esp32c3_uart_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    ESP32C3UARTClass *class = ESP32C3_UART_GET_CLASS(opaque);

    /* Same as above */
    class->parent_uart_write(opaque, addr, value, size);
}

static void esp32c3_uart_init(Object *obj)
{
    /* No need to call parent's class function, this is done automatically by the QOM, even before
     * calling the current function. */
}

static void esp32c3_uart_reset(DeviceState *dev)
{
    /* Nothing special to do at the moment here, call the parent reset */
    ESP32C3UARTClass* esp32c3_class = ESP32C3_UART_GET_CLASS(dev);

    esp32c3_class->parent_reset(dev);
}

static void esp32c3_uart_realize(DeviceState *dev, Error **errp)
{
    // ESP32C3UARTState* esp32c3 = ESP32C3_UART(dev);
    ESP32C3UARTClass* esp32c3_class = ESP32C3_UART_GET_CLASS(dev);

    /* Call the realize function of the parent class: ESP32UARTClass */
    esp32c3_class->parent_realize(dev, errp);
}

static void esp32c3_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ESP32C3UARTClass* esp32c3 = ESP32C3_UART_CLASS(klass);
    ESP32UARTClass* esp32 = ESP32_UART_CLASS(klass);

    /* Set our class' parent_realize field to the current realize function, set by the
     * parent class initializer.
     * After doing this, it will be necessary for our function, esp32c3_uart_realize,
     * to manually call the parent's one. */
    device_class_set_parent_realize(dc, esp32c3_uart_realize, &esp32c3->parent_realize);

    /* Let's do the same thing for the reset function */
    device_class_set_parent_reset(dc, esp32c3_uart_reset, &esp32c3->parent_reset);

    /* Override the UART operations functions */
    esp32c3->parent_uart_write = esp32->uart_write;
    esp32c3->parent_uart_read = esp32->uart_read;
    esp32->uart_write = esp32c3_uart_write;
    esp32->uart_read = esp32c3_uart_read;
}

static const TypeInfo esp32c3_uart_info = {
    .name = TYPE_ESP32C3_UART,
    .parent = TYPE_ESP32_UART,
    .instance_size = sizeof(ESP32C3UARTState),
    .instance_init = esp32c3_uart_init,
    .class_init = esp32c3_uart_class_init,
    .class_size = sizeof(ESP32C3UARTClass)
};

static void esp32c3_uart_register_types(void)
{
    type_register_static(&esp32c3_uart_info);
}

type_init(esp32c3_uart_register_types)
