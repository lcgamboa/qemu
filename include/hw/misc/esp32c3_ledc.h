#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/misc/esp32c3_reg.h"
#include "hw/misc/led.h"

#define TYPE_ESP32C3_LEDC "misc.esp32c3.ledc"
#define ESP32C3_LEDC(obj) OBJECT_CHECK(Esp32C3LEDCState, (obj), TYPE_ESP32C3_LEDC)
#define ESP32C3_LEDC_TIMER_CNT 4
#define ESP32C3_LEDC_CHANNEL_CNT 6

typedef struct Esp32C3LEDCState {
    SysBusDevice parent_object;
    MemoryRegion iomem;
    uint32_t conf_reg;
    uint32_t duty_res[ESP32C3_LEDC_TIMER_CNT];
    uint32_t freq[ESP32C3_LEDC_TIMER_CNT];
    uint32_t timer_conf_reg[ESP32C3_LEDC_TIMER_CNT];
    uint32_t channel_conf0_reg[ESP32C3_LEDC_CHANNEL_CNT];
    float duty[ESP32C3_LEDC_CHANNEL_CNT];
    LEDState led[ESP32C3_LEDC_CHANNEL_CNT];
    qemu_irq ledc_sync[1];
} Esp32C3LEDCState;

REG32(LEDC_CONF_REG, 0xD0)

#define ESP32C3_LEDC_SYNC "esp32c3_ledc_sync"

#define LEDC_REG_GROUP(name, base) \
    REG32(name ## _CONF0_REG, (base)) \
    REG32(name ## _CONF1_REG, ((base) + 0x00C)) \
    REG32(name ## _DUTY_REG, ((base) + 0x008)) \
    REG32(name ## _DUTY_R_REG, ((base) + 0x010))

#define LEDC_TIMER_REG_GROUP(name, base) \
    REG32(name ## _CONF_REG, (base)) \
    REG32(name ## _VALUE_REG, ((base) + 0x004))

LEDC_REG_GROUP(LEDC_CH0, 0x000)
LEDC_REG_GROUP(LEDC_CH1, 0x014)
LEDC_REG_GROUP(LEDC_CH2, 0x028)
LEDC_REG_GROUP(LEDC_CH3, 0x03C)
LEDC_REG_GROUP(LEDC_CH4, 0x050)
LEDC_REG_GROUP(LEDC_CH5, 0x064)

LEDC_TIMER_REG_GROUP(LEDC_TIMER0, 0x0A0)
LEDC_TIMER_REG_GROUP(LEDC_TIMER1, 0x0A8)
LEDC_TIMER_REG_GROUP(LEDC_TIMER2, 0x0B0)
LEDC_TIMER_REG_GROUP(LEDC_TIMER3, 0x0B8)

REG32(LEDC_INT_RAW_REG, 0x0C0)
REG32(LEDC_INT_ST_REG, 0x0C4)
REG32(LEDC_INT_ENA_REG, 0x0C8)
REG32(LEDC_INT_CLR_REG, 0x0CC)
