#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/misc/esp32_ledc.h"

#define ESP32_LEDC_REGS_SIZE (A_LEDC_CONF_REG + 4)

static uint64_t esp32_ledc_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_LEDC_HSTIMER0_CONF_REG ... A_LEDC_LSTIMER3_CONF_REG:
        r = s->timer_conf_reg[(addr - A_LEDC_HSTIMER0_CONF_REG) / 0x8];
        break;

    case A_LEDC_HSCH0_CONF0_REG:
    case A_LEDC_HSCH1_CONF0_REG:
    case A_LEDC_HSCH2_CONF0_REG:
    case A_LEDC_HSCH3_CONF0_REG:
    case A_LEDC_HSCH4_CONF0_REG:
    case A_LEDC_HSCH5_CONF0_REG:
    case A_LEDC_HSCH6_CONF0_REG:
    case A_LEDC_HSCH7_CONF0_REG:
    case A_LEDC_LSCH0_CONF0_REG:
    case A_LEDC_LSCH1_CONF0_REG:
    case A_LEDC_LSCH2_CONF0_REG:
    case A_LEDC_LSCH3_CONF0_REG:
    case A_LEDC_LSCH4_CONF0_REG:
    case A_LEDC_LSCH5_CONF0_REG:
    case A_LEDC_LSCH6_CONF0_REG:
    case A_LEDC_LSCH7_CONF0_REG:
        r = s->channel_conf0_reg[(addr - A_LEDC_HSCH0_CONF0_REG) / 0x14];
        break;
    }
    return r;
}

static uint32_t esp32_ledc_get_percent(Esp32LEDCState *s, uint32_t value, int  channel)
{
    uint32_t duty_val =  (value >> 4) & ((1 << 20) - 1);
    uint32_t duty_res = 0;
    if (channel < 8) {
        /* get duty res for the high speed channel from high speed timer */
        duty_res = s->duty_res[(s->channel_conf0_reg[channel] & ((1 << 2) - 1))];
    } else {
        /* get duty res for the low speed channel from low speed timer */
        duty_res = s->duty_res[((s->channel_conf0_reg[channel]) & ((1 << 2) - 1)) + 4];
    }
    if(duty_res){
        s->duty[channel] = (100.0 * (value & ((1 << 24) - 1))) / (16.0 * ((2 << (duty_res - 1)) - 1));
    }
    else{
        s->duty[channel] = 0;
    }
    return duty_res ? (100 * duty_val / ((2 << (duty_res - 1)) - 1)) : 0;
}

static void esp32_ledc_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned int size)
{
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    
    switch (addr) {
    case A_LEDC_CONF_REG:
        s->conf_reg = value;
        break;
        
    case A_LEDC_HSTIMER0_CONF_REG ... A_LEDC_LSTIMER3_CONF_REG:{
        int chn = (addr - A_LEDC_HSTIMER0_CONF_REG) / 0x8;
        s->timer_conf_reg[chn] = value;
        /* get duty resolution from timer config */
        int duty_res = s->timer_conf_reg[chn] & ((1 << 5) - 1);
        if (duty_res != 0) {
            s->duty_res[chn] = duty_res;

            int div = (s->timer_conf_reg[chn]  & 0x007FFFE0) >> 5;
            int freq = 0; 
            if(chn < 8){//HSTimer
                if(s->timer_conf_reg[chn]  & 0x02000000){//LEDC_TICK_SEL_HSTIMER
                    freq = 80000000L; //APB_CLK
                }else{
                    freq = 1000000L; //REF_TICK
                }
            }else{ //LSTimer
                if(s->timer_conf_reg[chn]  & 0x02000000){//LEDC_TICK_SEL_LSTIMER
                    if(s->conf_reg & 0x00000001){ //LEDC_APB_CLK_SEL 
                        freq = 80000000L; //APB_CLK
                    }else{
                        freq = 8000000L; //RC_FAST_CLK
                    }
                }else{
                    freq = 1000000L; //REF_TICK
                }
            }
            
            s->freq[chn] = freq / ((div / 256.0) * (1 << duty_res));
        }
        
        }break;

    case A_LEDC_HSCH0_CONF0_REG:
    case A_LEDC_HSCH1_CONF0_REG:
    case A_LEDC_HSCH2_CONF0_REG:
    case A_LEDC_HSCH3_CONF0_REG:
    case A_LEDC_HSCH4_CONF0_REG:
    case A_LEDC_HSCH5_CONF0_REG:
    case A_LEDC_HSCH6_CONF0_REG:
    case A_LEDC_HSCH7_CONF0_REG:
    case A_LEDC_LSCH0_CONF0_REG:
    case A_LEDC_LSCH1_CONF0_REG:
    case A_LEDC_LSCH2_CONF0_REG:
    case A_LEDC_LSCH3_CONF0_REG:
    case A_LEDC_LSCH4_CONF0_REG:
    case A_LEDC_LSCH5_CONF0_REG:
    case A_LEDC_LSCH6_CONF0_REG:
    case A_LEDC_LSCH7_CONF0_REG:
        s->channel_conf0_reg[(addr - A_LEDC_HSCH0_CONF0_REG) / 0x14] = value;
        break;

    case A_LEDC_HSCH0_DUTY_REG:
    case A_LEDC_HSCH1_DUTY_REG:
    case A_LEDC_HSCH2_DUTY_REG:
    case A_LEDC_HSCH3_DUTY_REG:
    case A_LEDC_HSCH4_DUTY_REG:
    case A_LEDC_HSCH5_DUTY_REG:
    case A_LEDC_HSCH6_DUTY_REG:
    case A_LEDC_HSCH7_DUTY_REG:
    case A_LEDC_LSCH0_DUTY_REG:
    case A_LEDC_LSCH1_DUTY_REG:
    case A_LEDC_LSCH2_DUTY_REG:
    case A_LEDC_LSCH3_DUTY_REG:
    case A_LEDC_LSCH4_DUTY_REG:
    case A_LEDC_LSCH5_DUTY_REG:
    case A_LEDC_LSCH6_DUTY_REG:
    case A_LEDC_LSCH7_DUTY_REG:{
        int ledn = (addr - A_LEDC_HSCH0_DUTY_REG) / 0x14;
        led_set_intensity(&s->led[ledn], esp32_ledc_get_percent(s, value, ledn));
        qemu_set_irq(s->ledc_sync[0], (0x5000 | (ledn << 8) | led_get_intensity(&s->led[ledn])));
        }break;
    }

}

static const MemoryRegionOps esp32_ledc_ops = {
        .read =  esp32_ledc_read,
        .write = esp32_ledc_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_ledc_realize(DeviceState *dev, Error **errp)
{
    Esp32LEDCState *s = ESP32_LEDC(dev);
    for (int i = 0; i < ESP32_LEDC_CHANNEL_CNT; i++) {
        qdev_realize(DEVICE(&s->led[i]), NULL, &error_fatal);
    }
}

static void esp32_ledc_init(Object *obj)
{
    Esp32LEDCState *s = ESP32_LEDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    qdev_init_gpio_out_named(DEVICE(s), s->ledc_sync, ESP32_LEDC_SYNC, 1);
    memory_region_init_io(&s->iomem, obj, &esp32_ledc_ops, s,
                          TYPE_ESP32_LEDC, ESP32_LEDC_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int i = 0; i < ESP32_LEDC_CHANNEL_CNT; i++) {
        object_initialize_child(obj, g_strdup_printf("led%d", i + 1), &s->led[i], TYPE_LED);
        s->led[i].color = (char *)"blue";
    }
}

static void esp32_ledc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = esp32_ledc_realize;
}

static const TypeInfo esp32_ledc_info = {
        .name = TYPE_ESP32_LEDC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Esp32LEDCState),
        .instance_init = esp32_ledc_init,
        .class_init = esp32_ledc_class_init
};

static void esp32_ledc_register_types(void)
{
    type_register_static(&esp32_ledc_info);
}

type_init(esp32_ledc_register_types)
