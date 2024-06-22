#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "hw/misc/esp32c3_ledc.h"

#define ESP32C3_LEDC_REGS_SIZE (A_LEDC_CONF_REG + 4)

static uint64_t esp32c3_ledc_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32C3LEDCState *s = ESP32C3_LEDC(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_LEDC_TIMER0_CONF_REG ... A_LEDC_TIMER3_CONF_REG:
        r = s->timer_conf_reg[(addr - A_LEDC_TIMER0_CONF_REG) / 0x8];
        break;

    case A_LEDC_CH0_CONF0_REG:
    case A_LEDC_CH1_CONF0_REG:
    case A_LEDC_CH2_CONF0_REG:
    case A_LEDC_CH3_CONF0_REG:
    case A_LEDC_CH4_CONF0_REG:
    case A_LEDC_CH5_CONF0_REG:
        r = s->channel_conf0_reg[(addr - A_LEDC_CH0_CONF0_REG) / 0x14];
        break;
    }
    return r;
}

static uint32_t esp32c3_ledc_get_percent(Esp32C3LEDCState *s, uint32_t value, int  channel)
{
    uint32_t duty_val =  (value >> 4) & ((1 << 14) - 1);
    uint32_t duty_res = 0;
    if (channel < 8) {
        /* get duty res for the high speed channel from high speed timer */
        duty_res = s->duty_res[(s->channel_conf0_reg[channel] & ((1 << 2) - 1))];
    } else {
        /* get duty res for the low speed channel from low speed timer */
        duty_res = s->duty_res[((s->channel_conf0_reg[channel]) & ((1 << 2) - 1)) + 4];
    }

    if(duty_res){
        s->duty[channel] = (100.0 * (value & ((1 << 18) - 1))) / (16.0 * ((2 << (duty_res - 1)) - 1));
    } 
    else{
        s->duty[channel] = 0;
    }

    return duty_res ? (100 * duty_val / ((2 << (duty_res - 1)) - 1)) : 0;
}

static void esp32c3_ledc_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned int size)
{
    Esp32C3LEDCState *s = ESP32C3_LEDC(opaque);
    int ledn;
    
    switch (addr) {

    case A_LEDC_CONF_REG:
        s->conf_reg = value;
        break;    

    case A_LEDC_TIMER0_CONF_REG ... A_LEDC_TIMER3_CONF_REG:{
        int chn = (addr - A_LEDC_TIMER0_CONF_REG) / 0x8;
        s->timer_conf_reg[chn] = value;
        /* get duty resolution from timer config */
        int duty_res = s->timer_conf_reg[chn] & ((1 << 4) - 1);
        if (duty_res != 0) {
            s->duty_res[chn] = duty_res;

            int div = (s->timer_conf_reg[chn]  & 0x003FFFF0) >> 4;
            int freq = 0; 

            switch (s->conf_reg & 0x00000003)
            {
            case 1: 
                freq = 80000000L; //APB_CLK
                break;
            case 2:
                freq = 17500000L; //RC_FAST_CLK
                break;
            case 3:
                freq = 40000000L; //XTAL_CLK
                break;    
            default:
                freq = 80000000L; //APB_CLK
                break;
            }
            s->freq[chn] = freq / ((div / 256.0) * (1 << duty_res));
        }

        }break;

    case A_LEDC_CH0_CONF0_REG:
    case A_LEDC_CH1_CONF0_REG:
    case A_LEDC_CH2_CONF0_REG:
    case A_LEDC_CH3_CONF0_REG:
    case A_LEDC_CH4_CONF0_REG:
    case A_LEDC_CH5_CONF0_REG:
        s->channel_conf0_reg[(addr - A_LEDC_CH0_CONF0_REG) / 0x14] = value;
        break;

    case A_LEDC_CH0_DUTY_REG:
    case A_LEDC_CH1_DUTY_REG:
    case A_LEDC_CH2_DUTY_REG:
    case A_LEDC_CH3_DUTY_REG:
    case A_LEDC_CH4_DUTY_REG:
    case A_LEDC_CH5_DUTY_REG:
        ledn = (addr - A_LEDC_CH0_DUTY_REG) / 0x14; 
        led_set_intensity(&s->led[ledn], esp32c3_ledc_get_percent(s, value, ledn));
        qemu_set_irq(s->ledc_sync[0], (0x5000 | (ledn << 8) | led_get_intensity(&s->led[ledn])));
        break;
    }

}

static const MemoryRegionOps esp32c3_ledc_ops = {
        .read =  esp32c3_ledc_read,
        .write = esp32c3_ledc_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_ledc_realize(DeviceState *dev, Error **errp)
{
    Esp32C3LEDCState *s = ESP32C3_LEDC(dev);
    for (int i = 0; i < ESP32C3_LEDC_CHANNEL_CNT; i++) {
        qdev_realize(DEVICE(&s->led[i]), NULL, &error_fatal);
    }
}

static void esp32c3_ledc_init(Object *obj)
{
    Esp32C3LEDCState *s = ESP32C3_LEDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    qdev_init_gpio_out_named(DEVICE(s), s->ledc_sync, ESP32C3_LEDC_SYNC, 1);
    memory_region_init_io(&s->iomem, obj, &esp32c3_ledc_ops, s,
                          TYPE_ESP32C3_LEDC, ESP32C3_LEDC_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int i = 0; i < ESP32C3_LEDC_CHANNEL_CNT; i++) {
        object_initialize_child(obj, g_strdup_printf("led%d", i + 1), &s->led[i], TYPE_LED);
        s->led[i].color = (char *)"blue";
    }
}

static void esp32c3_ledc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = esp32c3_ledc_realize;
}

static const TypeInfo esp32c3_ledc_info = {
        .name = TYPE_ESP32C3_LEDC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Esp32C3LEDCState),
        .instance_init = esp32c3_ledc_init,
        .class_init = esp32c3_ledc_class_init
};

static void esp32c3_ledc_register_types(void)
{
    type_register_static(&esp32c3_ledc_info);
}

type_init(esp32c3_ledc_register_types)
