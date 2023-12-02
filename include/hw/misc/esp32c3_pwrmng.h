#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32C3_PWR_MANAGER "misc.esp32c3.pwrmng"
#define ESP32C3_PWR_MANAGER(obj) OBJECT_CHECK(Esp32c3PwrMngState, (obj), TYPE_ESP32C3_PWR_MANAGER)


typedef struct Esp32c3PwrMngState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mem[1024];
} Esp32c3PwrMngState;

