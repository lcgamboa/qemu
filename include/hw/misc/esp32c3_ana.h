#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"


#define TYPE_ESP32C3_ANA "misc.esp32c3.ana"
#define ESP32C3_ANA(obj) OBJECT_CHECK(Esp32C3AnaState, (obj), TYPE_ESP32C3_ANA)

typedef struct Esp32C3AnaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mem[1024];
} Esp32C3AnaState;


