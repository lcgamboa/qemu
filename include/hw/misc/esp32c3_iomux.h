#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"
#include "esp32_iomux.h"

#define TYPE_ESP32C3_IOMUX "esp32C3.iomux"
#define ESP32C3_IOMUX(obj) OBJECT_CHECK(Esp32C3IomuxState, (obj), TYPE_ESP32C3_IOMUX)
#define ESP32C3_IOMUX_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32C3IomuxClass, obj, TYPE_ESP32C3_IOMUX)
#define ESP32C3_IOMUX_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32C3IomuxClass, klass, TYPE_ESP32C3_IOMUX)
            
typedef struct Esp32C3IomuxState {
    Esp32IomuxState parent;
} Esp32C3IomuxState;

typedef struct Esp32C3IomuxClass {
    Esp32IomuxClass parent;
} Esp32C3IomuxClass;