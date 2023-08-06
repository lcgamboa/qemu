/* ESP32 unimp peripheral handler
*/

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/boards.h"
#include "hw/misc/esp32_unimp.h"
#include "hw/misc/esp32_reg.h"

static uint64_t esp32_unimp_read(void *opaque, hwaddr addr, unsigned int size)
{   
    Esp32UnimpState *s = ESP32_UNIMP(opaque);
    return s->DefaultValue;
}

static void esp32_unknown_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    // Esp32UnimpState *s = ESP32_UNIMP(opaque);
    printf("unimp write for %016lX, setting to %ld (size=%d)\n", addr, value, size);
    
}

static const MemoryRegionOps esp32_unknown_ops = {
    .read =  esp32_unimp_read,
    .write = esp32_unimp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_unknown_init(Object *obj)
{
    Esp32UnimpState *s = ESP32_UNIMP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_unimp_ops, s, TYPE_ESP32_UNIMP, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    s->DefaultValue = -1;
}

static const TypeInfo esp32_unimp_info = {
    .name = TYPE_ESP32_UNIMP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32UnimpState),
    .instance_init = esp32_unimp_init,
};

static void esp32_unimp_register_types(void)
{
    type_register_static(&esp32_unimp_info);
}

type_init(esp32_unimp_register_types)
