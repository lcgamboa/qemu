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

static uint32_t esp32c3_ledc_get_percent(Esp32C3LEDCState *s, uint32_t value, hwaddr addr)
{
    uint32_t duty_val =  (value >> 4) & ((1 << 20) - 1);
    uint32_t duty_res;
    if (((addr - A_LEDC_CH0_DUTY_REG) / 0x14) < 8) {
        /* get duty res for the high speed channel from high speed timer */
        duty_res = s->duty_res[(s->channel_conf0_reg[(addr - A_LEDC_CH0_DUTY_REG) / 0x14] & ((1 << 2) - 1))];
    } else {
        /* get duty res for the low speed channel from low speed timer */
        duty_res = s->duty_res[((s->channel_conf0_reg[(addr - A_LEDC_CH0_DUTY_REG) / 0x14]) & ((1 << 2) - 1)) + 4];
    }
    return duty_res ? (100 * duty_val / ((2 << (duty_res - 1)) - 1)) : 0;
}

static void esp32c3_ledc_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned int size)
{
    Esp32C3LEDCState *s = ESP32C3_LEDC(opaque);
    int ledn;
    
    switch (addr) {
    case A_LEDC_TIMER0_CONF_REG ... A_LEDC_TIMER3_CONF_REG:
        /* get duty resolution from timer config */
        if (((uint32_t)value & ((1 << 4) - 1)) != 0) {
            s->duty_res[(addr - A_LEDC_TIMER0_CONF_REG) / 0x8] = (uint32_t)value & ((1 << 4) - 1);
        }
        s->timer_conf_reg[(addr - A_LEDC_TIMER0_CONF_REG) / 0x8] = value;
        break;

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
        led_set_intensity(&s->led[ledn], esp32c3_ledc_get_percent(s, value, addr));
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
