/*
 * ESP32 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "ui/console.h"
#include "hw/hw.h"
#include "ui/input.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"
#include "sysemu/runstate.h"


static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
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
        case 0x40: //GPIO_IN1_REG 
            qemu_set_irq(s->gpios_sync[0], -1); //request sync
            r = s->gpio_in1;
            break;
        case 0x44: //GPIO_STATUS_REG
            r = s->gpio_status;
            break;
        case 0x50: //GPIO_STATUS1_REG
            r = s->gpio_status1;
            break;
        case 0x60: //GPIO_ACPU_INT_REG
            r = s->gpio_acpu_int;
            break;
        case 0x68: //GPIO_PCPU_INT_REG
            r = s->gpio_pcpu_int;
            break;
        case 0x74: //GPIO_ACPU_INT1_REG
            r = s->gpio_acpu_int1;
            break;
        case 0x7c: //GPIO_PCPU_INT1_REG
            r = s->gpio_pcpu_int1;
            break;
        default:
            break;
    }
    if (addr >= 0x88 && addr < 0x130) { //GPIO_PINXX_REG
        int n = (addr - 0x88) / 4;
        r = s->gpio_pin[n];
    }
    else if (addr >= 0x130 && addr < 0x530) { //GPIO_FUNCY_IN_SEL_CFG_REG 
        int n = (addr - 0x130) / 4;
        r = s->gpio_in_sel[n];
    }
    else if (addr >= 0x530 && addr < 0x5d0) { //GPIO_FUNCX_OUT_SEL_CFG_REG 
        int n = (addr - 0x530) / 4;
        r = s->gpio_out_sel[n];
    }
    return r;
}

//Interrupt type selection
static int get_triggering(int int_type, int oldval, int val) {
    switch (int_type) {
        case 1: //rising edge trigger
            return (val > oldval);
        case 2: //falling edge trigger
            return (val < oldval);
        case 3: //any edge trigger
            return (val != oldval);
        case 4: //low level trigger
            return (val == 0);
        case 5: //high level trigger
            return (val == 1);
    }
    return 0; //GPIO interrupt disable
}

static void set_gpio(void *opaque, int n, int val) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    if (n < 32) {
        int oldval = (s->gpio_in >> n) & 1;
        int int_type = (s->gpio_pin[n] >> 7) & 7;
        s->gpio_in &= ~(1 << n);
        s->gpio_in |= (val << n);
        int irq = get_triggering(int_type, oldval, val);
        // says bit 16 in the ref manual, is that wrong?
        if (irq && (s->gpio_pin[n] & (1 << 15))) {  // pro cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_pcpu_int |= (1 << n);
        }
        if (irq && (s->gpio_pin[n] & (1 << 13))) {  // app cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_acpu_int |= (1 << n);
        }
    } else {
        int n1 = n - 32;
        int oldval = (s->gpio_in1 >> n1) & 1;
        int int_type = (s->gpio_pin[n] >> 7) & 7;
        s->gpio_in1 &= ~(1 << n1);
        s->gpio_in1 |= (val << n1);
        int irq = get_triggering(int_type, oldval, val);
        // says bit 16 in the ref manual, is that wrong?
        if (irq && (s->gpio_pin[n] & (1 << 15))) {  // pro cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_pcpu_int1 |= (1 << n1);
        }
        if (irq && (s->gpio_pin[n] & (1 << 13))) {  // app cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_acpu_int1 |= (1 << n1);
        }
    }
}
extern const struct {
    int width;
    int height;
} ttgo_board_skin;

static void esp32_gpio_write(void *opaque, hwaddr addr, uint64_t value,
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
        case 0x50: //GPIO_STATUS1_REG 
            s->gpio_status1 = value;
            break;
        case 0x54: //GPIO_STATUS1_W1TS_REG 
            s->gpio_status1 |= value;
            break;
        case 0x58: //GPIO_STATUS1_W1TC_REG 
            clearirq = 1;
            for (int i = 0; i < 32; i++) {
                if ((1 << i) & value) {
                    int int_type = (s->gpio_pin[i + 32] >> 7) & 7;
                    if ((int_type == 4 && !(s->gpio_in1 & (1 << i))) ||
                        (int_type == 5 && (s->gpio_in1 & (1 << i))))
                        clearirq = 0;
                }
            }
            if (clearirq) {
                s->gpio_status1 &= ~value;
                s->gpio_pcpu_int1 &= ~value;
                s->gpio_acpu_int1 &= ~value;
                qemu_set_irq(s->irq, 0);
            }
            break;
    }
    if (addr >= 0x88 && addr < 0x130) { //GPIO_PINXX_REG
        int n = (addr - 0x88) / 4;
        s->gpio_pin[n] = value;
    }
    else if (addr >= 0x130 && addr < 0x530) { //GPIO_FUNCY_IN_SEL_CFG_REG
        int n = (addr - 0x130) / 4;
        s->gpio_in_sel[n] = value;
        qemu_set_irq(s->gpios_sync[0], -(0x1000 | n)); //report in sel cfg change
    }
    else if (addr >= 0x530 && addr < 0x5d0) { //GPIO_FUNCX_OUT_SEL_CFG_REG 
        int n = (addr - 0x530) / 4;
        s->gpio_out_sel[n] = value;
        qemu_set_irq(s->gpios_sync[0], -(0x2000 | n)); ////report out sel cfg change
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

static const MemoryRegionOps uart_ops = {
    .read = esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset(DeviceState *dev) {}

static void esp32_gpio_realize(DeviceState *dev, Error **errp) {}

static void esp32_gpio_init(Object *obj) {
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s, TYPE_ESP32_GPIO,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(s), &s->irq, SYSBUS_DEVICE_GPIO_IRQ, 1);
    qdev_init_gpio_out_named(DEVICE(s), s->gpios, ESP32_GPIOS, 32);
    qdev_init_gpio_out_named(DEVICE(s), s->gpios_dir, ESP32_GPIOS_DIR, 32);
    qdev_init_gpio_out_named(DEVICE(s), s->gpios_sync, ESP32_GPIOS_SYNC, 1);
    qdev_init_gpio_in_named(DEVICE(s), set_gpio, ESP32_GPIOS_IN, 40);
    s->gpio_in = 0x1;
    s->gpio_in1 = 0x8;
}

static Property esp32_gpio_properties[] = {
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode,
                       ESP32_STRAP_MODE_FLASH_BOOT),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_gpio_reset;
    dc->realize = esp32_gpio_realize;
    device_class_set_props(dc, esp32_gpio_properties);
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init};

static void esp32_gpio_register_types(void) {
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
