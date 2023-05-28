/*
 * ESP32-C3 IOMUX emulation
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
#include "hw/misc/esp32c3_iomux.h"
#include "sysemu/runstate.h"

static uint64_t esp32c3_iomux_read(void *opaque, hwaddr addr, unsigned int size) {
    Esp32IomuxState *s = ESP32_IOMUX(opaque);
    
    int n = (addr - 4)/4;  

    if(n <= 21 ){
        return s->muxgpios[n];
    }
    else{
        return 0;
    }
}


static void esp32c3_iomux_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    Esp32IomuxState *s = ESP32_IOMUX(opaque);

    int n = (addr - 4)/4;

    if(n <= 21 ){
        s->muxgpios[n]= value;
        qemu_set_irq(s->iomux_sync[0], -(0x4000 | n));
    }
}

static const MemoryRegionOps iomux_ops = {
    .read = esp32c3_iomux_read,
    .write = esp32c3_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};



static void esp32c3_iomux_init(Object *obj) {
    Esp32C3IomuxState *s = ESP32C3_IOMUX(obj);
    memory_region_init_io(&s->parent.iomem, obj, &iomux_ops, s, TYPE_ESP32C3_IOMUX,
                          0x1000);
}

static void esp32c3_iomux_class_init(ObjectClass *klass, void *data) {

}

static const TypeInfo esp32c3_iomux_info = {
    .name = TYPE_ESP32C3_IOMUX,
    .parent = TYPE_ESP32_IOMUX,
    .instance_size = sizeof(Esp32C3IomuxState),
    .instance_init = esp32c3_iomux_init,
    .class_init = esp32c3_iomux_class_init};

static void esp32c3_iomux_register_types(void) {
    type_register_static(&esp32c3_iomux_info);
}

type_init(esp32c3_iomux_register_types)
