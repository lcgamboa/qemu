/* ESP32C3 unimp/dummy peripheral handler
*/

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/boards.h"
#include "hw/misc/esp32c3_unimp.h"
#include "hw/misc/esp32c3_reg.h"


#define DEBUG 1

static uint64_t esp32c3_unimp_read(void *opaque, hwaddr addr, unsigned int size)
{   
    Esp32c3UnimpState *s = ESP32C3_UNIMP(opaque);

    uint32_t r = s->mem[addr/4];
    
    if(DEBUG) printf("esp32_unimp_read  0x%04lx= 0x%08x\n",addr,r);

    return r;
}

static void esp32c3_unimp_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32c3UnimpState *s = ESP32C3_UNIMP(opaque);
    
    if(DEBUG) printf("esp32_unimp_write 0x%04lx= 0x%08lx\n",addr, value);

    s->mem[addr/4]=value;
}

static const MemoryRegionOps esp32c3_unimp_ops = {
    .read =  esp32c3_unimp_read,
    .write = esp32c3_unimp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_unimp_init(Object *obj)
{
    Esp32c3UnimpState *s = ESP32C3_UNIMP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_unimp_ops, s, TYPE_ESP32C3_UNIMP, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const TypeInfo esp32c3_unimp_info = {
    .name = TYPE_ESP32C3_UNIMP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32c3UnimpState),
    .instance_init = esp32c3_unimp_init,
};

static void esp32c3_unimp_register_types(void)
{
    type_register_static(&esp32c3_unimp_info);
}

type_init(esp32c3_unimp_register_types)
