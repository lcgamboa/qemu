/*
 * ESP32 IOMUX emulation
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
#include "hw/misc/esp32_iomux.h"
#include "sysemu/runstate.h"

static uint64_t esp32_iomux_read(void *opaque, hwaddr addr, unsigned int size) {
    Esp32IomuxState *s = ESP32_IOMUX(opaque);
    uint64_t r = 0;
    switch (addr) {
        case 0x44: 
            r = s->muxgpios[0];
            break;
        case 0x88: 
            r = s->muxgpios[1];//U0TXD
            break;
        case 0x40: 
            r = s->muxgpios[2];
            break;
        case 0x84: 
            r = s->muxgpios[3];//U0RXD
            break;
        case 0x48: 
            r = s->muxgpios[4];
            break;
        case 0x6C: 
            r = s->muxgpios[5];
            break;
        case 0x60: 
            r = s->muxgpios[6];//SD_CLK
            break;
        case 0x64: 
            r = s->muxgpios[7];//SD_DATA0
            break;
        case 0x68: 
            r = s->muxgpios[8];//SD_DATA1
            break;
        case 0x54: 
            r = s->muxgpios[9];//SD_DATA2
            break;
        case 0x58: 
            r = s->muxgpios[10];//SD_DATA3
            break;
        case 0x5C: 
            r = s->muxgpios[11];//SD_CMD
            break;
        case 0x34: 
            r = s->muxgpios[12];//MTDI
            break;
        case 0x38: 
            r = s->muxgpios[13];//MTCK
            break;
        case 0x30: 
            r = s->muxgpios[14];//MTMS
            break;
        case 0x3C: 
            r = s->muxgpios[15];//MTDO
            break;
        case 0x4C: 
            r = s->muxgpios[16];
            break;
        case 0x50: 
            r = s->muxgpios[17];
            break;
        case 0x70: 
            r = s->muxgpios[18];
            break;
        case 0x74: 
            r = s->muxgpios[19];
            break;
        case 0x78: 
            r = s->muxgpios[20];
            break;
        case 0x7C: 
            r = s->muxgpios[21];
            break;
        case 0x80: 
            r = s->muxgpios[22];
            break;
        case 0x8C: 
            r = s->muxgpios[23];
            break;
        case 0x90: 
            r = s->muxgpios[24];
            break;
        case 0x24: 
            r = s->muxgpios[25];
            break;
        case 0x28: 
            r = s->muxgpios[26];
            break;
        case 0x2C: 
            r = s->muxgpios[27];
            break;
            /*
            //Not documented
        case 0x: 
            r = s->muxgpios[28];
            break;
        case 0x: 
            r = s->muxgpios[29];
            break;
        case 0x: 
            r = s->muxgpios[30];
            break;
        case 0x: 
            r = s->muxgpios[31];
            break;
            */
        case 0x1C: 
            r = s->muxgpios[32];
            break;
        case 0x20: 
            r = s->muxgpios[33];
            break;
        case 0x14: 
            r = s->muxgpios[34];
            break;
        case 0x18: 
            r = s->muxgpios[35];
            break;
        case 0x04: 
            r = s->muxgpios[36];
            break;
        case 0x08: 
            r = s->muxgpios[37];
            break;
        case 0x0C: 
            r = s->muxgpios[38];
            break;
        case 0x10: 
            r = s->muxgpios[39];
            break;
        default:
            break;
    }
    return r;
}


static void esp32_iomux_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    Esp32IomuxState *s = ESP32_IOMUX(opaque);
    switch (addr) {
        case 0x44: 
            s->muxgpios[0]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000));
            break;
        case 0x88: 
            s->muxgpios[1]= value;//U0TXD
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 1));
            break;
        case 0x40: 
            s->muxgpios[2]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 2));
            break;
        case 0x84: 
            s->muxgpios[3]= value;//U0RXD
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 3));
            break;
        case 0x48: 
            s->muxgpios[4]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 4));
            break;
        case 0x6C: 
            s->muxgpios[5]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 5));
            break;
        case 0x60: 
            s->muxgpios[6]= value;//SD_CLK
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 6));
            break;
        case 0x64: 
            s->muxgpios[7]= value;//SD_DATA0
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 7));
            break;
        case 0x68: 
            s->muxgpios[8]= value;//SD_DATA1
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 8));
            break;
        case 0x54: 
            s->muxgpios[9]= value;//SD_DATA2
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 9));
            break;
        case 0x58: 
            s->muxgpios[10]= value;//SD_DATA3
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 10));
            break;
        case 0x5C: 
            s->muxgpios[11]= value;//SD_CMD
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 11));
            break;
        case 0x34: 
            s->muxgpios[12]= value;//MTDI
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 12));
            break;
        case 0x38: 
            s->muxgpios[13]= value;//MTCK
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 13));
            break;
        case 0x30: 
            s->muxgpios[14]= value;//MTMS
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 14));
            break;
        case 0x3C: 
            s->muxgpios[15]= value;//MTDO
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 15));
            break;
        case 0x4C: 
            s->muxgpios[16]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 16));
            break;
        case 0x50: 
            s->muxgpios[17]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 17));
            break;
        case 0x70: 
            s->muxgpios[18]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 18));
            break;
        case 0x74: 
            s->muxgpios[19]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 19));
            break;
        case 0x78: 
            s->muxgpios[20]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 20));
            break;
        case 0x7C: 
            s->muxgpios[21]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 21));
            break;
        case 0x80: 
            s->muxgpios[22]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 |22));
            break;
        case 0x8C: 
            s->muxgpios[23]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 23));
            break;
        case 0x90: 
            s->muxgpios[24]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 24));
            break;
        case 0x24: 
            s->muxgpios[25]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 25));
            break;
        case 0x28: 
            s->muxgpios[26]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 26));
            break;
        case 0x2C: 
            s->muxgpios[27]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 27));
            break;
            /*
            //Not documented
        case 0x: 
            s->muxgpios[28]= value;
            break;
        case 0x: 
            s->muxgpios[29]= value;
            break;
        case 0x: 
            s->muxgpios[30]= value;
            break;
        case 0x: 
            s->muxgpios[31]= value;
            break;
            */
        case 0x1C: 
            s->muxgpios[32]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 32));
            break;
        case 0x20: 
            s->muxgpios[33]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 33));
            break;
        case 0x14: 
            s->muxgpios[34]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 34));
            break;
        case 0x18: 
            s->muxgpios[35]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 35));
            break;
        case 0x04: 
            s->muxgpios[36]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 36));
            break;
        case 0x08: 
            s->muxgpios[37]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 37));
            break;
        case 0x0C: 
            s->muxgpios[38]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 38));
            break;
        case 0x10: 
            s->muxgpios[39]= value;
            qemu_set_irq(s->iomux_sync[0], -(0x4000 | 39));
            break;
    } 
}

static const MemoryRegionOps iomux_ops = {
    .read = esp32_iomux_read,
    .write = esp32_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_iomux_reset(DeviceState *dev) {
    Esp32IomuxState *s = ESP32_IOMUX(dev);    
    for(int i=0; i < 40; i++){
       s->muxgpios[i]=0x00000800;
    }
}

static void esp32_iomux_realize(DeviceState *dev, Error **errp) {}

static void esp32_iomux_init(Object *obj) {
    Esp32IomuxState *s = ESP32_IOMUX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &iomux_ops, s, TYPE_ESP32_IOMUX,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_out_named(DEVICE(s), s->iomux_sync, ESP32_IOMUX_SYNC, 1);

    for(int i=0; i < 40; i++){
       s->muxgpios[i]=0x00000800;
    }
}

static void esp32_iomux_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = esp32_iomux_reset;
    dc->realize = esp32_iomux_realize;
}

static const TypeInfo esp32_iomux_info = {
    .name = TYPE_ESP32_IOMUX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32IomuxState),
    .instance_init = esp32_iomux_init,
    .class_init = esp32_iomux_class_init};

static void esp32_iomux_register_types(void) {
    type_register_static(&esp32_iomux_info);
}

type_init(esp32_iomux_register_types)
