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
#include "hw/irq.h"


#define DEBUG 0


static void esp32c3_saradc_update_irq(Esp32c3SarAdcState * s)
{
    int irq_state = !!(s->int_raw_reg & s->int_ena_reg);
    qemu_set_irq(s->irq, irq_state);
}

static uint64_t esp32c3_saradc_read(void *opaque, hwaddr addr, unsigned int size)
{   
    Esp32c3SarAdcState *s = ESP32C3_SARADC(opaque);

    uint32_t r = s->mem[addr/4];

    switch(addr) {
        case A_APB_SARADC_INT_ENA_REG:
            r= s->int_ena_reg;
            break;
        case A_APB_SARADC_INT_RAW_REG:
            r= s->int_raw_reg;
            break;
        case A_APB_SARADC_INT_ST_REG:
             r= s->int_raw_reg & s->int_ena_reg;
             break;    
        case A_APB_SARADC_1_DATA_STATUS_REG:   
            r= s->ADC_values[s->channel1];
            if(DEBUG) printf("Read ADC 1 channel %i = %i\n",s->channel1, r);
            break; 
        case A_APB_SARADC_2_DATA_STATUS_REG:   
            r= s->ADC_values[s->channel2];
            if(DEBUG) printf("Read ADC 2 channel %i = %i\n",s->channel1, r);
            break;     
    }
    
    if(DEBUG) printf("esp32c3_saradc_read  0x%04lx= 0x%08x\n",(unsigned long) addr,r);

    return r;
}

static void esp32c3_saradc_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    Esp32c3SarAdcState *s = ESP32C3_SARADC(opaque);
    
    if(DEBUG) printf("esp32c3_saradc_write 0x%04lx= 0x%08lx\n",(unsigned long) addr, (unsigned long) value);

        switch(addr) {
        case A_APB_SARADC_INT_ENA_REG:
            s->int_ena_reg = value;
            esp32c3_saradc_update_irq(s);
            break;
        case A_APB_SARADC_INT_CLR_REG:
            s->int_raw_reg &= ~value;
            esp32c3_saradc_update_irq(s);
            break;        
        case A_APB_SARADC_ONETIME_SAMPLE_REG:   
            if(FIELD_EX32(value, APB_SARADC_ONETIME_SAMPLE_REG, APB_SARADC_ONETIME_START))
            {
               if(FIELD_EX32(value, APB_SARADC_ONETIME_SAMPLE_REG, APB_SARADC1_ONETIME_SAMPLE))
               {
                    s->channel1 = FIELD_EX32(value, APB_SARADC_ONETIME_SAMPLE_REG, APB_SARADC_ONETIME_CHANNEL);
                    s->int_raw_reg = FIELD_DP32(s->int_raw_reg, APB_SARADC_INT_RAW_REG, APB_SARADC_ADC1_DONE_INT_RAW, 1);
                    esp32c3_saradc_update_irq(s);
               } 
               if(FIELD_EX32(value, APB_SARADC_ONETIME_SAMPLE_REG, APB_SARADC2_ONETIME_SAMPLE))
               {
                    s->channel2 = FIELD_EX32(value, APB_SARADC_ONETIME_SAMPLE_REG, APB_SARADC_ONETIME_CHANNEL);
                    s->int_raw_reg = FIELD_DP32(s->int_raw_reg, APB_SARADC_INT_RAW_REG, APB_SARADC_ADC2_DONE_INT_RAW, 1);
                    esp32c3_saradc_update_irq(s);
               } 
            }
            break; 
    }

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
    sysbus_init_irq(sbd, &s->irq);

    memset(s->ADC_values, 0, sizeof(s->ADC_values));
}

static void esp32c3_saradc_reset(DeviceState * dev)
{
    Esp32c3SarAdcState * s = ESP32C3_SARADC(dev);

    s->int_ena_reg = 0;
    s->int_raw_reg = 0;
    s->channel1 = 0;
    s->channel2 = 0;
    memset(s->mem, 0, sizeof(s->mem));
}

static void esp32c3_saradc_class_init(ObjectClass * klass, void * data)
{
    DeviceClass * dc = DEVICE_CLASS(klass);
    dc->reset = esp32c3_saradc_reset;
}

static const TypeInfo esp32c3_saradc_info = {
    .name = TYPE_ESP32C3_SARADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32c3SarAdcState),
    .instance_init = esp32c3_saradc_init,
    .class_init = esp32c3_saradc_class_init,
};

static void esp32c3_saradc_register_types(void)
{
    type_register_static(&esp32c3_saradc_info);
}

type_init(esp32c3_saradc_register_types)
