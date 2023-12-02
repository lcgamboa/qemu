#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32C3_UNIMP "misc.esp32c3.unimp"
#define ESP32C3_UNIMP(obj) OBJECT_CHECK(Esp32c3UnimpState, (obj), TYPE_ESP32C3_UNIMP)


typedef struct Esp32c3UnimpState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mem[1024];
} Esp32c3UnimpState;

