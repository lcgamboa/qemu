/*
 * ESP32-C3 GPIO emulation
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32c3_gpio.h"


static uint64_t esp32c3_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
   Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;
    switch (addr) {
        case 0x04: //GPIO_OUT_REG
            r = s->gpio_out;
            break;
        case 0x20: //GPIO_ENABLE_REG
            r = s->gpio_enable;
            break;
        case 0x38: //A_GPIO_STRAP:
            r = s->strap_mode;
            break;
        case 0x3C: //GPIO_IN_REG
            qemu_set_irq(s->gpios_sync[0], -1); //request sync
            r = s->gpio_in;
            break;
        case 0x44: //GPIO_STATUS_REG
            r = s->gpio_status;
            break;
        case 0x5C: //GPIO_PCPU_INT_REG
            r = s->gpio_pcpu_int;
            break;
        default:
            break;
    }
    if (addr >= 0x74 && addr < 0xCC) { //GPIO_PINXX_REG
        int n = (addr - 0x74) / 4;
        r = s->gpio_pin[n];
    }
    else if (addr >= 0x154 && addr < 0x354) { //GPIO_FUNCY_IN_SEL_CFG_REG 
        int n = (addr - 0x154) / 4;
        r = s->gpio_in_sel[n];
    }
    else if (addr >= 0x554 && addr < 0x5AC) { //GPIO_FUNCX_OUT_SEL_CFG_REG 
        int n = (addr - 0x554) / 4;
        r = s->gpio_out_sel[n];
    }
    return r;
} 

static void esp32c3_gpio_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    int clearirq;
    uint32_t oldvalue;
    uint32_t oldenable;
    oldvalue = s->gpio_out;
    oldenable = s->gpio_enable;
    switch (addr) {
        case 0x04: //GPIO_OUT_REG
            s->gpio_out = value;
            break;
        case 0x08: //GPIO_OUT_W1TS_REG
            s->gpio_out |= value;
            break;
        case 0x0C: //GPIO_OUT_W1TC_REG 
            s->gpio_out &= ~value;
            break;
        case 0x20: //GPIO_ENABLE_REG
            s->gpio_enable = value;
            break;
        case 0x24: //GPIO_ENABLE_W1TS_REG
            s->gpio_enable |= value;
            break;
        case 0x28: //GPIO_ENABLE_W1TC_REG
            s->gpio_enable &= ~value;
            break;
        case 0x38: //A_GPIO_STRAP
            s->strap_mode = value;
            break;
        case 0x44: //GPIO_STATUS_REG
            s->gpio_status = value;
            break;
        case 0x48: //GPIO_STATUS_W1TS_REG 
            s->gpio_status |= value;
            break;
        case 0x4c: //GPIO_STATUS_W1TC_REG 
            clearirq = 1;
            for (int i = 0; i < 32; i++) {
                if ((1 << i) & value) {
                    int int_type = (s->gpio_pin[i] >> 7) & 7;
                    if ((int_type == 4 && !(s->gpio_in & (1 << i))) ||
                        (int_type == 5 && (s->gpio_in & (1 << i))))
                        clearirq = 0;
                }
            }
            if (clearirq) {
                s->gpio_status &= ~value;
                s->gpio_pcpu_int &= ~value;
                s->gpio_acpu_int &= ~value;
                qemu_set_irq(s->irq, 0);
            }
            break;
    }
    if (addr >= 0x74 && addr < 0xCC) { //GPIO_PINXX_REG
        int n = (addr - 0x74) / 4;
        s->gpio_pin[n] = value;
    }
    else if (addr >= 0x154 && addr < 0x354) { //GPIO_FUNCY_IN_SEL_CFG_REG
        int n = (addr - 0x154) / 4;
        s->gpio_in_sel[n] = value;
        qemu_set_irq(s->gpios_sync[0], (0x1000 | n)); //report in sel cfg change
    }
    else if (addr >= 0x554 && addr < 0x5AC) { //GPIO_FUNCX_OUT_SEL_CFG_REG 
        int n = (addr - 0x554) / 4;
        s->gpio_out_sel[n] = value;
        qemu_set_irq(s->gpios_sync[0], (0x2000 | n)); ////report out sel cfg change
    }

    //printf("out 0x%04X in 0x%04X enable 0x%04X \n"  ,s->gpio_out, s->gpio_in, s->gpio_enable);

    if ((s->gpio_out != oldvalue)||(s->gpio_enable != oldenable)) {
        uint32_t diff = (s->gpio_out ^ oldvalue) | (s->gpio_enable ^ oldenable);
        for (int i = 0; i < 32; i++) {
            if (((1 << i) & diff)&&((1 << i) & s->gpio_enable)){
                qemu_set_irq(s->gpios[i], (s->gpio_out & (1 << i)) ? 1 : 0);
                if(s->gpio_out & (1 << i)){
                   s->gpio_in |= (1 << i);
                }else{
                   s->gpio_in &= ~(1 << i); 
                } 
            }
        }
    }
    if (s->gpio_enable != oldenable) {
        uint32_t diff =(s->gpio_enable ^ oldenable);
        for (int i = 0; i < 32; i++) {
            if ((1 << i) & diff){
                qemu_set_irq(s->gpios_dir[i], (s->gpio_enable & (1 << i)) ? 1 : 0); 
            }
        }
    }                  
}

static const MemoryRegionOps gpio_ops = {
    .read = esp32c3_gpio_read,
    .write = esp32c3_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_gpio_init(Object *obj)
{
    ESP32C3GPIOState *s = ESP32C3_GPIO(obj);
    /* Set the default value for the property */
    object_property_set_int(obj, "strap_mode", ESP32C3_STRAP_MODE_FLASH_BOOT, &error_fatal);

    memory_region_init_io(&s->parent.iomem, obj, &gpio_ops, s, TYPE_ESP32C3_GPIO,
                          0x1000);
}

static void esp32c3_gpio_reset(DeviceState *dev)
{
    /*
    static int reset_cont = 0;
    if(reset_cont > 0) {
        Esp32GpioState *s = ESP32_GPIO(dev);
        s->strap_mode  = ESP32C3_STRAP_MODE_FLASH_BOOT;
    }
    reset_cont++;
    */
}

/* If we need to override any function from the parent (reset, realize, ...), it shall be done
 * in this class_init function */
static void esp32c3_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32c3_gpio_reset;
}

static const TypeInfo esp32c3_gpio_info = {
    .name = TYPE_ESP32C3_GPIO,
    .parent = TYPE_ESP32_GPIO,
    .instance_size = sizeof(ESP32C3GPIOState),
    .instance_init = esp32c3_gpio_init,
    .class_init = esp32c3_gpio_class_init,
    .class_size = sizeof(ESP32C3GPIOClass),
};

static void esp32c3_gpio_register_types(void)
{
    type_register_static(&esp32c3_gpio_info);
}

type_init(esp32c3_gpio_register_types)
