/*
 * ESP32-C3 AES emulation
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32c3_aes.h"
#include "crypto/aes.h"
#include "qemu/error-report.h"

#define AES_WARNING 0
#define AES_DEBUG   0


static void esp32c3_aes_dma_exit(ESP32C3AesState *s)
{
    /* DMA is not supported yet */
    (void) s;
}


static void esp32c3_aes_start(ESP32C3AesState *s)
{
    AES_KEY aes_key;

    /* DMA mode is not supported yet! */
    if (FIELD_EX32(s->dma_enable_reg , AES_DMA_ENA_REG, AES_DMA_ENA) != 0) {
        error_report("[AES] DMA-AES is not supported yet\n");
        return;
    }

    /* Check whether we have to encrypt or decrypt */
    const uint32_t mode = FIELD_EX32(s->mode_reg , AES_MODE_REG, AES_MODE);
    const bool encrypt = (mode == ESP32C3_AES_MODE_128_ENC) || (mode == ESP32C3_AES_MODE_256_ENC);
    const bool decrypt = (mode == ESP32C3_AES_MODE_128_DEC) || (mode == ESP32C3_AES_MODE_256_DEC);

    /* Get the length, in bits of the key */
    const int length = (mode == ESP32C3_AES_MODE_128_ENC || mode == ESP32C3_AES_MODE_128_DEC) ? 128 : 256;

    /* Cast the keys and data to byte array.
     * This can only work as-is if the host computer is has a little-endian CPU.  */
    const uint8_t* key = (uint8_t*) &s->key;
    const uint8_t* tin = (uint8_t*) &s->text_in;
    uint8_t* tout = (uint8_t*) &s->text_out;

    if (encrypt) {

        AES_set_encrypt_key(key, length, &aes_key);
        AES_encrypt(tin, tout, &aes_key);

    } else if (decrypt) {

        AES_set_decrypt_key(key, length, &aes_key);
        AES_decrypt(tin, tout, &aes_key);

    }

    s->state_reg = ESP32C3_AES_IDLE;
}

static uint64_t esp32c3_aes_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32C3AesState *s = ESP32C3_AES(opaque);
    uint64_t r = 0;

    switch (addr) {
    case A_AES_KEY_0_REG ... A_AES_KEY_7_REG:
        r = s->key[(addr - A_AES_KEY_0_REG) / sizeof(uint32_t)];
        break;

    case A_AES_TEXT_IN_0_REG ... A_AES_TEXT_IN_3_REG:
        r = s->text_in[(addr - A_AES_TEXT_IN_0_REG) / sizeof(uint32_t)];
        break;

    case A_AES_TEXT_OUT_0_REG ... A_AES_TEXT_OUT_3_REG:
        r = s->text_out[(addr - A_AES_TEXT_OUT_0_REG) / sizeof(uint32_t)];
        break;

    case A_AES_IV_MEM_0_REG ... A_AES_IV_MEM_15_REG:
        r = s->iv_mem[(addr - A_AES_IV_MEM_0_REG) / sizeof(uint32_t)];
        break;

    case A_AES_STATE_REG:
        r = s->state_reg;
        break;

    case A_AES_MODE_REG:
        r = s->mode_reg;
        break;

    case A_AES_DMA_ENA_REG:
        r = s->dma_enable_reg;
        break;

    case A_AES_BLK_MODE_REG:
        r = s->block_mode_reg;
        break;

    case A_AES_BLK_NUM_REG:
        r = s->block_num_reg;
        break;

    case A_AES_INC_SEL_REG:
        r = s->inc_sel_reg;
        break;

    case A_AES_INT_ENA_REG:
        r = s->int_ena_reg;
        break;

    default:
#if AES_WARNING
        /* Other registers are not supported yet */
        warn_report("[AES] Unsupported read to %08lx\n", addr);
#endif
        break;
    }

#if AES_DEBUG
    info_report("[AES] Reading from %08lx (%08lx)\n", addr, r);
#endif


    return r;
}


static void esp32c3_aes_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    ESP32C3AesState *s = ESP32C3_AES(opaque);

    switch (addr) {
    case A_AES_KEY_0_REG ... A_AES_KEY_7_REG:
        s->key[(addr - A_AES_KEY_0_REG) / sizeof(uint32_t)] = value;
        break;

    case A_AES_TEXT_IN_0_REG ... A_AES_TEXT_IN_3_REG:
        s->text_in[(addr - A_AES_TEXT_IN_0_REG) / sizeof(uint32_t)] = value;
        break;

    case A_AES_IV_MEM_0_REG ... A_AES_IV_MEM_15_REG:
        s->iv_mem[(addr - A_AES_IV_MEM_0_REG) / sizeof(uint32_t)] = value;
        break;

    case A_AES_MODE_REG:
        s->mode_reg = value;
        break;

    case A_AES_TRIGGER_REG:
        if (FIELD_EX32(value, AES_TRIGGER_REG, AES_TRIGGER)) {
            esp32c3_aes_start(s);
        }
        break;

    case A_AES_DMA_ENA_REG:
        s->dma_enable_reg = value;
        break;

    case A_AES_BLK_MODE_REG:
        s->block_mode_reg = value;
        break;

    case A_AES_BLK_NUM_REG:
        s->block_num_reg = value;
        break;

    case A_AES_INC_SEL_REG:
        s->inc_sel_reg = value;
        break;

    case A_AES_INT_CLR_REG:
        if (FIELD_EX32(value, AES_INT_CLR_REG, AES_INT_CLR)) {
            s->int_st = 0;
        }
        break;

    case A_AES_INT_ENA_REG:
        s->int_ena_reg = FIELD_EX32(value, AES_INT_ENA_REG, AES_INT_ENA) ? 1 : 0;
        break;

    case A_AES_DMA_EXIT_REG:
        esp32c3_aes_dma_exit(s);
        break;

    default:
#if AES_WARNING
        /* Other registers are not supported yet */
        warn_report("[AES] Unsupported write to %08lx (%08lx)\n", addr, value);
#endif
        break;
    }

#if AES_DEBUG
    info_report("[AES] Writing to %08lx (%08lx)\n", addr, value);
#endif

}

static const MemoryRegionOps esp32c3_aes_ops = {
        .read =  esp32c3_aes_read,
        .write = esp32c3_aes_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_aes_reset(DeviceState *dev)
{
    ESP32C3AesState *s = ESP32C3_AES(dev);
    memset(s->key, 0, ESP32C3_AES_KEY_REG_CNT * sizeof(uint32_t));
    memset(s->text_in, 0, ESP32C3_AES_TEXT_REG_CNT * sizeof(uint32_t));
    memset(s->text_out, 0, ESP32C3_AES_TEXT_REG_CNT * sizeof(uint32_t));

    s->state_reg = ESP32C3_AES_IDLE;
    s->mode_reg = 0;
    s->dma_enable_reg = 0;
    s->block_mode_reg = 0;
    s->block_num_reg = 0;
    s->inc_sel_reg = 0;
    s->int_ena_reg = 0;
}

static void esp32c3_aes_init(Object *obj)
{
    ESP32C3AesState *s = ESP32C3_AES(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32c3_aes_ops, s,
                          TYPE_ESP32C3_AES, ESP32C3_AES_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void esp32_aes_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32c3_aes_reset;
}

static const TypeInfo esp32c3_aes_info = {
        .name = TYPE_ESP32C3_AES,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(ESP32C3AesState),
        .instance_init = esp32c3_aes_init,
        .class_init = esp32_aes_class_init
};

static void esp32c3_aes_register_types(void)
{
    type_register_static(&esp32c3_aes_info);
}

type_init(esp32c3_aes_register_types)
