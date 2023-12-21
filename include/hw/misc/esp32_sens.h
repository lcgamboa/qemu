#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_reg.h"


#define TYPE_ESP32_SENS "misc.esp32.sens"
#define ESP32_SENS(obj) OBJECT_CHECK(Esp32SensState, (obj), TYPE_ESP32_SENS)

typedef struct Esp32SensState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    int touch_sensor[10];
    short ADC_values[32];
    int channel1;
    int channel2;
} Esp32SensState;

//#define ESP32_RNG_BASE (DR_REG_WDEV_BASE + 0x144)

