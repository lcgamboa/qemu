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
#include "hw/misc/esp32c3_pwrmng.h"
#include "hw/misc/esp32c3_reg.h"


#define DEBUG 0

static uint64_t esp32c3_pwrmng_read(void *opaque, hwaddr addr, unsigned int size)
{   
    static bool init = false;
    Esp32c3PwrMngState *s = ESP32C3_PWR_MANAGER(opaque);

    uint32_t r = s->mem[addr/4];

    switch(addr){
        case 0x07C:
          /* Return a random 32-bit value */
          if (!init) {
            srand(time(NULL));
            init = true;
          }
          r = rand();
          break;
        case 0x128: //DMA enabled
          r = 0 ;
          break;  
        case 0x118: //PWR int events
          r = 0 ;
          break;    
    }
    
    if(DEBUG) printf("esp32_pwrmng_read  0x%04lx= 0x%08x\n",addr,r);

    return r;
}

static void esp32c3_pwrmng_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32c3PwrMngState *s = ESP32C3_PWR_MANAGER(opaque);
    
    if(DEBUG) printf("esp32_pwrmng_write 0x%04lx= 0x%08lx\n",addr, value);

    s->mem[addr/4]=value;
}

static const MemoryRegionOps esp32c3_pwrmng_ops = {
    .read =  esp32c3_pwrmng_read,
    .write = esp32c3_pwrmng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_pwrmng_init(Object *obj)
{
    Esp32c3PwrMngState *s = ESP32C3_PWR_MANAGER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_pwrmng_ops, s, TYPE_ESP32C3_PWR_MANAGER, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const TypeInfo esp32c3_pwrmng_info = {
    .name = TYPE_ESP32C3_PWR_MANAGER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32c3PwrMngState),
    .instance_init = esp32c3_pwrmng_init,
};

static void esp32c3_pwrmng_register_types(void)
{
    type_register_static(&esp32c3_pwrmng_info);
}

type_init(esp32c3_pwrmng_register_types)
