#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/misc/esp32_reg.h"

#define TYPE_ESP32_UNIMP "misc.esp32.unimp"
#define ESP32_UNKNOWN(obj) OBJECT_CHECK(Esp32UnimpState, (obj), TYPE_ESP32_UNIMP)

typedef struct Esp32UnimpState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint64_t DefaultValue;
} Esp32UnimpState;

