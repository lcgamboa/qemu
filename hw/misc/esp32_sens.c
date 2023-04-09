/*
 * ESP32 Random Number Generator peripheral
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
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_sens.h"

int touch_sensor[10];

unsigned short ADC_values[31]={31,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,
                               24,25,26,27,28,29,30};

static int channel1=0;
static int channel2=0;
 

static uint64_t esp32_sens_read(void *opaque, hwaddr addr, unsigned int size)
{
    //Esp32SensState *s = ESP32_SENS(opaque);
    uint32_t r = 0;
    //printf("esp32_sens_read 0x%lx\n",addr);
    switch(addr) {
    case 0x54://SENS_MEAS1_START_SAR
        return 0x10000+ADC_values[channel1];
    case 0x84://SENS_SAR_TOUCH_CTRL2_REG
	return (1<<10);
    case 0x94://SENS_MEAS2_START_SAR
	return 0x10000+ADC_values[channel2+8];
    }
// 2=gpio2, 3=gpio15, 4=gpio14(13?), 5=gpio12, 7=gpio27, 8=gpio33, 9=gpio32
// land +/- 300: 2=12166,31743 15=13618,31585 13=15277,31743 12=16798,31743 27=18388,3071 33=13791,2993 32=12166,2914
// port	       : 2=1417,12132  15=1339,13791  13=1496,15312  12=1417,16694  27=30010,18388 33=30088,13860 32=30010,12201
    if(addr>=0x70 && addr<0x84) {
	int n1=((addr-0x70)/4)*2;
        return ((1500-touch_sensor[n1]+rand()%20)<<16) | (1500-touch_sensor[n1+1]+rand()%20);
    }

//    qemu_guest_getrandom_nofail(&r, sizeof(r));
    return r;
}

static int bitpos( int value )
{
    switch( value )
    {
        case 1: return 0; 
        case 2: return 1;
        case 4: return 2;
        case 8: return 3;
        case 16: return 4;
        case 32: return 5;
        case 64: return 6;
        case 128: return 7;
        case 256: return 8;
        case 512: return 9;
        case 1024: return 10;
        case 2048: return 11;
        case 4096: return 12;
        case 8192: return 13;
        case 16384: return 14;
        case 32768: return 15;
        case 65536: return 16;
       default      : return 0;
    }
}

static void esp32_sens_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size) {
    int cval;
   // Esp32SensState *s = ESP32_SENS(opaque);
   // printf("esp32_sens_write 0x%lx 0x%lx\n",addr, value);
    switch(addr) {
    case 0x54://SENS_MEAS1_START_SAR
     cval = (value & 0x7FF80000) >> 19;
     if(cval){
       channel1 = bitpos(cval);
     } 
    break; 
    case 0x94://SENS_MEAS2_START_SAR
     cval = (value & 0x7FF80000) >> 19;
     if(cval){
       channel2 = bitpos(cval);
     }
    break;
    }
}

static const MemoryRegionOps esp32_sens_ops = {
    .read =  esp32_sens_read,
    .write = esp32_sens_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_sens_init(Object *obj)
{
    Esp32SensState *s = ESP32_SENS(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_sens_ops, s,
                          TYPE_ESP32_SENS, 0x400);
    sysbus_init_mmio(sbd, &s->iomem);
}


static const TypeInfo esp32_sens_info = {
    .name = TYPE_ESP32_SENS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SensState),
    .instance_init = esp32_sens_init,
};

static void esp32_sens_register_types(void)
{
    type_register_static(&esp32_sens_info);
}

type_init(esp32_sens_register_types)
