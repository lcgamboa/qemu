/* ESP32C3 saradc peripheral handler
*/

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/boards.h"
#include "hw/misc/esp32c3_saradc.h"
#include "hw/misc/esp32c3_reg.h"


#define DEBUG 0

static uint64_t esp32c3_saradc_read(void *opaque, hwaddr addr, unsigned int size)
{   
    Esp32c3SarAdcState *s = ESP32C3_SARADC(opaque);

    uint32_t r = s->mem[addr/4];

    switch(addr) {
        case A_APB_SARADC_INT_RAW_REG:
            r=FIELD_DP32(r, APB_SARADC_INT_RAW_REG, APB_SARADC_ADC1_DONE_INT_RAW, 1);
            r=FIELD_DP32(r, APB_SARADC_INT_RAW_REG, APB_SARADC_ADC2_DONE_INT_RAW, 1);
            break;
    }
    
    if(DEBUG) printf("esp32_saradc_read  0x%04lx= 0x%08x\n",addr,r);

    return r;
}

static void esp32c3_saradc_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32c3SarAdcState *s = ESP32C3_SARADC(opaque);
    
    if(DEBUG) printf("esp32_saradc_write 0x%04lx= 0x%08lx\n",addr, value);

    s->mem[addr/4]=value;
}

static const MemoryRegionOps esp32c3_saradc_ops = {
    .read =  esp32c3_saradc_read,
    .write = esp32c3_saradc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_saradc_init(Object *obj)
{
    Esp32c3SarAdcState *s = ESP32C3_SARADC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_saradc_ops, s, TYPE_ESP32C3_SARADC, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const TypeInfo esp32c3_saradc_info = {
    .name = TYPE_ESP32C3_SARADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32c3SarAdcState),
    .instance_init = esp32c3_saradc_init,
};

static void esp32c3_saradc_register_types(void)
{
    type_register_static(&esp32c3_saradc_info);
}

type_init(esp32c3_saradc_register_types)
