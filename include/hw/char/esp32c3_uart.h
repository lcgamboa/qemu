#pragma once

#include "esp32_uart.h"


#define TYPE_ESP32C3_UART "esp32c3_soc.uart"
/* This macro can be used to "convert" a generic object into a more specific object, here ESP32C3UARTState.
 * For example, it can convert a DeviceState to a ESP32C3UARTState */
#define ESP32C3_UART(obj) OBJECT_CHECK(ESP32C3UARTState, (obj), TYPE_ESP32C3_UART)
/* This one can be used to get the class of a given object.
 * For example, it can give the ESP32UARTClass type structure out of a DeviceState or a
 * ESP32C3UARTState structure. */
#define ESP32C3_UART_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32C3UARTClass, obj, TYPE_ESP32C3_UART)
/* Finally, this macro is used to convert a generic class to ESP32C3UARTClass.
 * For example, it can be used to convert an ObjectClass to a ESP32C3UARTClass*/
#define ESP32C3_UART_CLASS(klass) OBJECT_CLASS_CHECK(ESP32C3UARTClass, klass, TYPE_ESP32C3_UART)

typedef struct ESP32C3UARTState {
    ESP32UARTState parent;

} ESP32C3UARTState;

typedef struct ESP32C3UARTClass {
    ESP32UARTClass parent_class;

    /* These function pointers will be during class init, they will be populated
     * by device_class_set_parent_* functions. They can then be called in the
     * respective class methods: realize and reset. */
    DeviceRealize parent_realize;
    DeviceReset parent_reset;

    /* Virtual attributes/methods overriden */
    void (*parent_uart_write)(void *opaque, hwaddr addr, uint64_t value, unsigned int size);
    uint64_t (*parent_uart_read)(void *opaque, hwaddr addr, unsigned int size);
} ESP32C3UARTClass;

