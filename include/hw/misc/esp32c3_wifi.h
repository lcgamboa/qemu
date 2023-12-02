#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "sysemu/sysemu.h"
#include "net/net.h"
#include "esp32_wifi.h"

#define TYPE_ESP32C3_WIFI "esp32c3_wifi"
//#define ESP32C3_WIFI(obj) OBJECT_CHECK(Esp32WifiState, (obj), TYPE_ESP32C3_WIFI)


REG32(C3_WIFI_DMA_IN_STATUS, 0x84);
REG32(C3_WIFI_DMA_INLINK, 0x88);
REG32(C3_WIFI_DMA_INT_STATUS, 0xc3C);
REG32(C3_WIFI_DMA_INT_CLR, 0xc40);
REG32(C3_WIFI_STATUS, 0xcb0);
REG32(C3_WIFI_DMA_OUTLINK, 0xd08); 
REG32(C3_WIFI_DMA_OUT_STATUS, 0xd14); 
